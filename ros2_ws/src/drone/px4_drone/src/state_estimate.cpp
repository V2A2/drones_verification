// state_estimate: publishes the drone's PX4 EKF2 estimate, with its uncertainty, in the shared earth frame.
//
// Subscribes (relative to the node namespace, plus the optional px4_namespace prefix):
//   fmu/out/vehicle_odometry          position, attitude, velocity, body rates and their variances
//   fmu/out/vehicle_local_position    EKF origin (ref_lat / ref_lon / ref_alt), for the earth frame offset
// Publishes, both in frame "earth" (ENU around the shared origin) and with the same stamp for each odometry message:
//   state/pose   PoseWithCovarianceStamped: position; attitude of the body FLU frame;
//                covariance diag = position variance (x, y, z) and attitude variance (roll, pitch, yaw)
//   state/twist  TwistWithCovarianceStamped: linear velocity in the earth frame, angular rates in body FLU;
//                covariance diag = velocity variance (x, y, z) and the attitude variance as a proxy for the rates
//
// The stamp is the node clock on receipt, the same for both topics, so that subscribers can pair them exactly.

#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
// After rclcpp: it uses rmw_get_implementation_identifier() without including rmw/rmw.h
#include <px4_ros2/utils/message_version.hpp>

#include "px4_drone/earth_frame.hpp"

namespace px4_drone
{

namespace
{

// Rotation from NED to ENU (roll pi, then yaw pi/2), as px4_ros_com's NED_ENU_Q
const Eigen::Quaterniond kNedToEnu =
  Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
// Rotation from the PX4 aircraft body frame (FRD) to the ROS base_link frame (FLU), as AIRCRAFT_BASELINK_Q
const Eigen::Quaterniond kAircraftToBaselink(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

constexpr char kEarthFrame[] = "earth";

bool allFinite(const std::array<float, 3> & v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// Diagonal of R diag(variance) R^T: the per-axis variance after rotating a vector with independent per-axis errors
Eigen::Vector3d rotateVariance(const Eigen::Matrix3d & rot, const std::array<float, 3> & variance)
{
  const Eigen::Vector3d var(variance[0], variance[1], variance[2]);
  return (rot * var.asDiagonal() * rot.transpose()).diagonal();
}

}  // namespace

class StateEstimate : public rclcpp::Node
{
public:
  StateEstimate()
  : Node("state_estimate"), earth_(declareOrigin(*this))
  {
    const std::string prefix = declare_parameter<std::string>("px4_namespace", "");

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "state/pose", rclcpp::SensorDataQoS());
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "state/twist", rclcpp::SensorDataQoS());

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      prefix + "fmu/out/vehicle_local_position" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(),
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLocalPosition & msg) {
        const bool was_ready = earth_.ready();
        if (earth_.update(msg) && !was_ready) {
          const auto & o = earth_.offsetEnu();
          RCLCPP_INFO(
            get_logger(), "EKF origin at earth (%.2f, %.2f, %.2f) m, publishing state", o.x(), o.y(), o.z());
        }
      });

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      prefix + "fmu/out/vehicle_odometry" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleOdometry>(),
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry & msg) {odometryCallback(msg);});
  }

private:
  void odometryCallback(const px4_msgs::msg::VehicleOdometry & msg)
  {
    if (!earth_.ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for the PX4 global reference (GPS fix) before publishing state");
      return;
    }
    if (msg.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED &&
      msg.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported pose_frame %u", msg.pose_frame);
      return;
    }
    if (!allFinite(msg.position) || !std::isfinite(msg.q[0]) || !allFinite(msg.velocity)) {
      return;  // PX4 marks invalid fields with NaN, e.g. right after an EKF reset
    }

    const rclcpp::Time stamp = get_clock()->now();

    // Pose. FRD (arbitrary heading) is treated as NED: PX4 reports FRD only without a heading reference.
    const Eigen::Vector3d position_ned(msg.position[0], msg.position[1], msg.position[2]);
    const Eigen::Vector3d position = earth_.localNedToEarthEnu(position_ned);
    const Eigen::Quaterniond q_ned_frd(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
    const Eigen::Quaterniond q_enu_flu = (kNedToEnu * q_ned_frd * kAircraftToBaselink).normalized();

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = kEarthFrame;
    pose.pose.pose.position.x = position.x();
    pose.pose.pose.position.y = position.y();
    pose.pose.pose.position.z = position.z();
    pose.pose.pose.orientation.w = q_enu_flu.w();
    pose.pose.pose.orientation.x = q_enu_flu.x();
    pose.pose.pose.orientation.y = q_enu_flu.y();
    pose.pose.pose.orientation.z = q_enu_flu.z();
    // NED -> ENU swaps x and y; the z sign flip does not change a variance
    pose.pose.covariance[0] = msg.position_variance[1];
    pose.pose.covariance[7] = msg.position_variance[0];
    pose.pose.covariance[14] = msg.position_variance[2];
    // Body-frame roll / pitch / yaw variance; FRD -> FLU only flips signs
    pose.pose.covariance[21] = msg.orientation_variance[0];
    pose.pose.covariance[28] = msg.orientation_variance[1];
    pose.pose.covariance[35] = msg.orientation_variance[2];

    // Twist: linear velocity in the earth frame (PX4 normally reports it in NED already)
    Eigen::Vector3d velocity_ned(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    Eigen::Vector3d velocity_variance_ned(
      msg.velocity_variance[0], msg.velocity_variance[1], msg.velocity_variance[2]);
    if (msg.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      const Eigen::Matrix3d rot = q_ned_frd.normalized().toRotationMatrix();
      velocity_ned = rot * velocity_ned;
      velocity_variance_ned = rotateVariance(rot, msg.velocity_variance);
    } else if (msg.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED &&
      msg.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported velocity_frame %u", msg.velocity_frame);
      return;
    }
    const Eigen::Vector3d velocity = nedToEnu(velocity_ned);

    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    twist.header.stamp = stamp;
    twist.header.frame_id = kEarthFrame;
    twist.twist.twist.linear.x = velocity.x();
    twist.twist.twist.linear.y = velocity.y();
    twist.twist.twist.linear.z = velocity.z();
    twist.twist.covariance[0] = velocity_variance_ned.y();
    twist.twist.covariance[7] = velocity_variance_ned.x();
    twist.twist.covariance[14] = velocity_variance_ned.z();
    // Body rates, FRD -> FLU. PX4 has no rate variance: the attitude variance stands in for it, so that it is not 0.
    if (allFinite(msg.angular_velocity)) {
      twist.twist.twist.angular.x = msg.angular_velocity[0];
      twist.twist.twist.angular.y = -msg.angular_velocity[1];
      twist.twist.twist.angular.z = -msg.angular_velocity[2];
    }
    twist.twist.covariance[21] = msg.orientation_variance[0];
    twist.twist.covariance[28] = msg.orientation_variance[1];
    twist.twist.covariance[35] = msg.orientation_variance[2];

    pose_pub_->publish(pose);
    twist_pub_->publish(twist);
  }

  EarthFrame earth_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
};

}  // namespace px4_drone

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_drone::StateEstimate>());
  rclcpp::shutdown();
  return 0;
}
