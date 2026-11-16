#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "fl_reach/safetyChecker.h"
#include "fl_reach/msg/hyper_rectangle.hpp"
#include "fl_reach/verifier.h"

#include "std_msgs/msg/float32.hpp"


//This ros node is to store "my_rs" and compare
//it to the other agents' reachable sets

class collision_checker : public rclcpp::Node {
    public:
        collision_checker();

        hyperRectangle my_rs;
        hyperRectangle rcvd_rs;

        rclcpp::Subscription<fl_reach::msg::HyperRectangle>::SharedPtr my_rs_subscriber;
        rclcpp::Subscription<fl_reach::msg::HyperRectangle>::SharedPtr rcvd_rs_subscriber;

        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr collision_pub; //<-- SystemID of the colliding agent
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dc_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr in_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ttc_pub;

        void my_rs_cb(const fl_reach::msg::HyperRectangle::SharedPtr msg);
        void rcvd_rs_cb(const fl_reach::msg::HyperRectangle::SharedPtr msg);
};

void collision_checker::my_rs_cb(const fl_reach::msg::HyperRectangle::SharedPtr msg) {

    

    my_rs.intervals[0].min = msg->x.min;
    my_rs.intervals[0].max = msg->x.max;
    my_rs.intervals[1].min = msg->y.min;
    my_rs.intervals[1].max = msg->y.max;
    my_rs.intervals[2].min = msg->z.min;
    my_rs.intervals[2].max = msg->z.max;
    //my_rs.disp();

}

void collision_checker::rcvd_rs_cb(const fl_reach::msg::HyperRectangle::SharedPtr msg) {
    double decode_start = getUnixTime();

    int sending_agent = msg->id;
    rcvd_rs.intervals[0].min = msg->x.min;
    rcvd_rs.intervals[0].max = msg->x.max;
    rcvd_rs.intervals[1].min = msg->y.min;
    rcvd_rs.intervals[1].max = msg->y.max;
    rcvd_rs.intervals[2].min = msg->z.min;
    rcvd_rs.intervals[2].max = msg->z.max;
    //rcvd_rs.disp();

    double decode_end = getUnixTime(); //compute decode time
    
    std_msgs::msg::Float32 dc_time;
    dc_time.data = decode_end - decode_start;
    dc_pub->publish(dc_time);

    if (my_rs.intersects(rcvd_rs)) {


        RCLCPP_WARN(get_logger(), "COLLISION DETECTED with agent %d", sending_agent);

        // px4_drone's random_walk aborts its target, brakes and retreats on this event
        std_msgs::msg::UInt8 collision_msg;
        collision_msg.data = sending_agent;
        collision_pub->publish(collision_msg);

    }

    double intersect_end = getUnixTime();
    std_msgs::msg::Float32 in_time;
    in_time.data = intersect_end - decode_end;
    in_pub->publish(in_time);

    double ttc = rclcpp::Time(msg->rs_time).seconds() - decode_start;
    std_msgs::msg::Float32 ttc_msg;
    ttc_msg.data = ttc;
    ttc_pub->publish(ttc_msg);
}

collision_checker::collision_checker() : rclcpp::Node("collision_checker") {

    using std::placeholders::_1;

    interval x(0.0,0.0);

    std::vector<interval> intervals;

    intervals.push_back(x);
    intervals.push_back(x);
    intervals.push_back(x);
    intervals.push_back(x);
    intervals.push_back(x);
    intervals.push_back(x);

    my_rs = hyperRectangle(intervals);
    rcvd_rs = hyperRectangle(intervals);

    my_rs_subscriber = create_subscription<fl_reach::msg::HyperRectangle>(
        "my_rs", 5, std::bind(&collision_checker::my_rs_cb, this, _1));
    rcvd_rs_subscriber = create_subscription<fl_reach::msg::HyperRectangle>(
        "received_rs", 30, std::bind(&collision_checker::rcvd_rs_cb, this, _1));
    collision_pub = create_publisher<std_msgs::msg::UInt8>("collision_detected", 10);

    //Latched in ROS 1 -> transient_local in ROS 2
    auto latched = rclcpp::QoS(1).transient_local();
    dc_pub = create_publisher<std_msgs::msg::Float32>("dc_time", latched);
    in_pub = create_publisher<std_msgs::msg::Float32>("in_time", latched);
    ttc_pub= create_publisher<std_msgs::msg::Float32>("ttc", latched);

}

int main (int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<collision_checker>());
    rclcpp::shutdown();
    return 0;
}