#include "fl_reach/verifier.h"

#include <functional>

using namespace std::chrono_literals;

/* This node will subscribe to the agent's xy position topic and calculate it's reachable space */

agent::agent() : rclcpp::Node("verifier") {

    using std::placeholders::_1;

    //State from the as2 state estimator (resolved in the drone namespace)
    pose_subscriber = create_subscription<geometry_msgs::msg::PoseStamped>(
        "self_localization/pose", rclcpp::SensorDataQoS(), std::bind(&agent::pose_cb, this, _1));
    twist_subscriber = create_subscription<geometry_msgs::msg::TwistStamped>(
        "self_localization/twist", rclcpp::SensorDataQoS(), std::bind(&agent::twist_cb, this, _1));
    odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
        "sensor_measurements/odom", rclcpp::SensorDataQoS(), std::bind(&agent::odom_cb, this, _1));

    //Latched in ROS 1 -> transient_local in ROS 2
    auto latched = rclcpp::QoS(1).transient_local();
    rect_publisher = create_publisher<fl_reach::msg::HyperRectangle>("my_rs", latched);

    //Switching to a timer based intterupt for reachable set computation
    rs_timer = create_wall_timer(100ms, std::bind(&agent::compute_rs, this));

    declare_parameter<int>("SystemID", 0);
    SYSID_THISMAV = checkSystemId(get_parameter("SystemID").as_int());
    RCLCPP_WARN(get_logger(), "Set SYSID_THISMAV to %d", SYSID_THISMAV);

    declare_parameter<double>("reachTimout", 0.005);
    reachTimeout = static_cast<float>(get_parameter("reachTimout").as_double());
    RCLCPP_WARN(get_logger(), "Set reachTimeout to %f", reachTimeout);

    declare_parameter<double>("reachTime", 1.0);
    reachTime = static_cast<float>(get_parameter("reachTime").as_double());
    RCLCPP_WARN(get_logger(), "Set reachTime to %f", reachTime);

    //Logging Publishers
    rs_pub = create_publisher<std_msgs::msg::Float32>("rs_time", latched); //For publishing the rs_time
    en_pub = create_publisher<std_msgs::msg::Float32>("en_time", latched); //For publishing the en_time

    rsMessage.id = SYSID_THISMAV;

}

void agent::compute_rs() {

    if (!haveStd) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for sensor_measurements/odom");
        return;
    }

    std_msgs::msg::Float32 rs_time;
    std_msgs::msg::Float32 en_time;

    //std::cout << "computing a reach set for " << reachTimeout << " seconds" << std::endl;
    float compute_time = reachTimeout * 0.4;

    //The actual time it takes to compute the reachable set
    double start_compute = getUnixTime();

    computeReachableSet(3.0, reachTime, 2.0, compute_time);

    /*
    Because I often forget compute reachable set arguments:
    1) n sigma (initial box is mean +- n * std, same truncation as the ProbStar)
    2) Reach time (how far ahead to look
    3) Initial Step Size (how big )
    4) Reach timeout
    */

    //finish timing the computation
    double end_compute = getUnixTime();

    rs_time.data = end_compute - start_compute;

    //Populate the ROS message with the reachable set
    packRectangleMessage(myReachableState, rsMessage);

    rclcpp::Time tempTime(static_cast<int64_t>((start_compute + reachTime) * 1e9));

    //Find the encode time
    double end_encode = getUnixTime();
    en_time.data = end_encode - end_compute;

    //Add the timestamp and the actual time
    rsMessage.header.stamp = now();
    rsMessage.rs_time = tempTime;

    rect_publisher->publish(rsMessage);
    rs_pub->publish(rs_time);
    en_pub->publish(en_time);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<agent>());
    rclcpp::shutdown();
    return 0;
}
