#ifndef VERIFIER_H
#define VERIFIER_H

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "safetyChecker.h"
#include "fl_reach/msg/hyper_rectangle.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include <cmath>
#include <stdexcept>
#include <cstring>
#include <chrono>
#include <fstream>
#include <sstream>

#include <filesystem>
#include <iomanip>

inline double getUnixTime() {
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    
    // Convert the time point to Unix time (seconds since epoch)
    std::chrono::duration<double> unix_time = now.time_since_epoch();
    
    // Convert the duration to a floating-point number
    double unix_time_float = unix_time.count();
    
    return unix_time_float;
}

// SystemID: unique numeric id of the agent, 0..255 (0 is also the broadcast target on the radio).
// It has no default on purpose: two agents with the same id ignore each other's messages.
inline int checkSystemId(int id) {
    if (id < 0 || id > 255) {
        throw std::runtime_error("SystemID must be a value between 0 and 255, got " + std::to_string(id));
    }
    return id;
}

inline safetyChecker reach_checker;

class agent : public rclcpp::Node {
    /* Agent class holds all state data (both dynaimic and reachable)*/
    public:
        agent();

        int SYSID_THISMAV = 0;

        //Agent's state variables
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float vx = 0.0;
        float vy = 0.0;
        float vz = 0.0;
        float ax = 0.0;
        float ay = 0.0;
        float az = 0.0;
        double timeStamp = 0.0;
        bool plotBool = false;
        bool ros_on = false;

        float reachTime; //Lookahead horizon (s), set from the reachTime parameter

        //Std of position and velocity (earth frame), from the state/pose and state/twist covariances
        float posStd[3] = {0.0, 0.0, 0.0};
        float velStd[3] = {0.0, 0.0, 0.0};
        bool havePose = false;
        bool haveTwist = false;

        //Ploting locations
        std::string plotFile;
        std::string othrFile;

        //Flight mode things
        char nextFlightMode = 'L';

        //ROS publishers and subscribers
        rclcpp::Publisher<fl_reach::msg::HyperRectangle>::SharedPtr rect_publisher; //<-- Topic where reachable state gets published
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_subscriber;

        //ROS timers
        float reachTimeout = 0.005;
        rclcpp::TimerBase::SharedPtr rs_timer;
        void compute_rs();


        //Verification-related states
        hyperRectangle myReachableState;
        double myReachTime;

        fl_reach::msg::HyperRectangle rsMessage;

        //Logging things
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rs_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr en_pub;

        std::tuple<hyperRectangle, float> computeReachableSet(float nSigma, float reachTime, float initialStepSize, float reachTimeout) {

            interval posErrorX(x - nSigma * posStd[0], x + nSigma * posStd[0]);
            interval posErrorY(y - nSigma * posStd[1], y + nSigma * posStd[1]);
            interval posErrorZ(z - nSigma * posStd[2], z + nSigma * posStd[2]);
            interval velErrorX(vx - nSigma * velStd[0], vx + nSigma * velStd[0]);
            interval velErrorY(vy - nSigma * velStd[1], vy + nSigma * velStd[1]);
            interval velErrorZ(vz - nSigma * velStd[2], vz + nSigma * velStd[2]);

            std::vector<interval> rectIntervals;

            rectIntervals.push_back(posErrorX);
            rectIntervals.push_back(posErrorY);
            rectIntervals.push_back(posErrorZ);
            rectIntervals.push_back(velErrorX);
            rectIntervals.push_back(velErrorY);
            rectIntervals.push_back(velErrorZ);

            hyperRectangle rect(rectIntervals);
            float acceleration[] = {ax, ay, az};

           // Managing the time proved to be a pain, for now we just calculate the reach time from "0.0"
           // and will just convert it to seconds and add it to a freshly updated time stamp

            auto myRs = reach_checker.faceLiftingIterativeImprovement(rect, 0.0, acceleration, reachTime, initialStepSize, reachTimeout);

            myReachableState = std::get<0>(myRs); //Set the agent's reach space

            timeStamp = getUnixTime();
            myReachTime = std::get<1>(myRs) / 1000.0 + timeStamp; //Set the agent's reach time

            return {std::get<0>(myRs), std::get<1>(myRs) };
        }

        void packRectangleMessage(hyperRectangle rect, fl_reach::msg::HyperRectangle &msg) {
            /* Pack a hyperRectangle ROS message with data from hypreRectangle object*/
            
            msg.x.min = rect.intervals[0].min;
            msg.x.max = rect.intervals[0].max;
            msg.y.min = rect.intervals[1].min;
            msg.y.max = rect.intervals[1].max;
            msg.z.min = rect.intervals[2].min;
            msg.z.max = rect.intervals[2].max;
            msg.vx.min = rect.intervals[3].min;
            msg.vx.max = rect.intervals[3].max;
            msg.vy.min = rect.intervals[4].min;
            msg.vy.max = rect.intervals[4].max;
            msg.vz.min = rect.intervals[5].min;
            msg.vz.max = rect.intervals[5].max;
        }

        void pose_cb( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg ) {
            /* px4_drone state/pose: position in the earth frame (ENU), common to all drones */
            x = msg->pose.pose.position.x;
            y = msg->pose.pose.position.y;
            z = msg->pose.pose.position.z;
            for (int i = 0; i < 3; i++) {
                posStd[i] = std::sqrt(msg->pose.covariance[7 * i]);
            }
            havePose = true;
        }

        void twist_cb( const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg ) {
            /* px4_drone state/twist: linear velocity in the earth frame, which the reachable set needs */
            vx = msg->twist.twist.linear.x;
            vy = msg->twist.twist.linear.y;
            vz = msg->twist.twist.linear.z;
            for (int i = 0; i < 3; i++) {
                velStd[i] = std::sqrt(msg->twist.covariance[7 * i]);
            }
            haveTwist = true;
        }

};

#endif
