// random_walk: random walk mission of one drone, run as a PX4 external mode (px4-ros2-interface-lib).
//
// RandomWalkExecutor takes over as soon as it is registered: it waits until PX4 is ready to arm, arms, takes off and
// runs RandomWalkMode, then switches to Hold when the walk is done. RandomWalkMode flies straight segments between
// random targets in the shared earth frame (see earth_frame.hpp) with PX4's go-to setpoint, which limits the speed and
// smooths the motion.
//
// Collision handling: the verifier publishes collision_detected (the SystemID of the other drone) when the reachable
// sets intersect. A collision reported while flying to a target aborts that target: the drone brakes to a stop, retreats
// `retreat` m along the line back from the target, holds for `collision_wait` s, and goes on with the next target.
//
// Braking uses a raw velocity setpoint (0 m/s), not the go-to setpoint: PX4's go-to smoother is jerk-limited
// (MPC_JERK_AUTO, MPC_ACC_HOR) and needs ~1.4 s and ~1.4 m to stop from 2 m/s, longer than the verifier's 1 s lookahead.
// A velocity setpoint is tracked directly by the velocity controller and stops the drone several times faster.
//
// Logged for bags/analysis_scripts/extract2.py (do not reword): "GoTo to position: x, y, z", "Retreat to position:
// x, y, z", "Collision with agent N".

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "px4_drone/earth_frame.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace px4_drone
{

namespace
{
constexpr char kModeName[] = "Random Walk";
constexpr auto kRegistrationRetryDelay = 1s;
}  // namespace

struct WalkParams
{
  int64_t steps;          // number of targets, 0 = unlimited
  double segment;         // m, length of each straight segment (< half the area diagonal)
  double turn;            // rad, max heading change at the end of a segment
  std::vector<double> area;  // x_min, x_max, y_min, y_max in the earth frame (m)
  double height;          // m above the origin altitude
  double speed;           // m/s, max horizontal speed
  double takeoff_speed;   // m/s, max vertical speed while climbing to height
  double threshold;       // m, distance at which a target counts as reached
  double sleep_time;      // s, pause at each reached target
  double collision_wait;  // s, hold after a collision before the next target
  double retreat;         // m, distance flown back from the target after a collision
  double brake_speed;     // m/s, horizontal speed below which braking is done
  double brake_timeout;   // s, max braking time

  static WalkParams declare(rclcpp::Node & node)
  {
    WalkParams p{
      node.declare_parameter<int64_t>("steps", 0),
      node.declare_parameter("segment", 10.0),
      node.declare_parameter("turn", 90.0) * M_PI / 180.0,
      node.declare_parameter("area", std::vector<double>{-30.0, 5.0, 0.0, 30.0}),
      node.declare_parameter("height", 30.0),
      node.declare_parameter("speed", 2.0),
      node.declare_parameter("takeoff_speed", 1.0),
      node.declare_parameter("threshold", 0.2),
      node.declare_parameter("sleep_time", 0.5),
      node.declare_parameter("collision_wait", 3.0),
      node.declare_parameter("retreat", 1.0),
      node.declare_parameter("brake_speed", 0.2),
      node.declare_parameter("brake_timeout", 2.0)};
    if (p.area.size() != 4 || p.area[0] >= p.area[1] || p.area[2] >= p.area[3]) {
      throw std::invalid_argument("area must be [x_min, x_max, y_min, y_max]");
    }
    if (2.0 * p.segment >= std::hypot(p.area[1] - p.area[0], p.area[3] - p.area[2])) {
      throw std::invalid_argument("segment must be shorter than half the area diagonal");
    }
    return p;
  }
};

// Straight segments of a fixed length, with a new heading only at the end of a segment (within +-turn of the current
// one). Each target starts from the previous target, not the measured position, so the targets depend only on the
// random draws and the start. If the next segment would leave the area, a new heading is drawn at random until it doesn't.
class RandomWalk
{
public:
  RandomWalk(const WalkParams & params, std::mt19937 & rng, double start_x, double start_y)
  : params_(params), rng_(rng), x_(std::round(start_x)), y_(std::round(start_y)),
    heading_(uniform(0.0, 2.0 * M_PI)) {}

  // Next (x, y) target in the earth frame, or nothing when all steps are done
  std::optional<Eigen::Vector2d> next()
  {
    if (params_.steps > 0 && step_ >= params_.steps) {
      return std::nullopt;
    }
    ++step_;
    const auto & a = params_.area;
    heading_ += uniform(-params_.turn, params_.turn);
    while (true) {
      // heading from north (y) towards east (x)
      const double end_x = x_ + params_.segment * std::sin(heading_);
      const double end_y = y_ + params_.segment * std::cos(heading_);
      if (a[0] <= end_x && end_x <= a[1] && a[2] <= end_y && end_y <= a[3]) {
        x_ = end_x;
        y_ = end_y;
        return Eigen::Vector2d(x_, y_);
      }
      heading_ = uniform(0.0, 2.0 * M_PI);
    }
  }

  int64_t step() const {return step_;}

private:
  double uniform(double lo, double hi) {return std::uniform_real_distribution<double>(lo, hi)(rng_);}

  const WalkParams & params_;
  std::mt19937 & rng_;
  double x_;
  double y_;
  double heading_;
  int64_t step_{0};
};

class RandomWalkMode : public px4_ros2::ModeBase
{
public:
  RandomWalkMode(rclcpp::Node & node, const WalkParams & params, const GeoPoint & origin,
    const std::string & topic_namespace_prefix)
  : ModeBase(node, Settings{kModeName}, topic_namespace_prefix),
    params_(params), earth_(origin), rng_(std::random_device{}())
  {
    goto_ = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    brake_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    collision_sub_ = node.create_subscription<std_msgs::msg::UInt8>(
      "collision_detected", 10,
      [this](const std_msgs::msg::UInt8 & msg) {
        colliding_agent_ = msg.data;
        collision_detected_ = true;
      });
  }

  void onActivate() override
  {
    state_ = State::Climb;
    target_.reset();
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;
    if (!local_position_->lastValid() || !local_position_->positionXYValid() ||
      !local_position_->positionZValid())
    {
      return;
    }
    earth_.update(local_position_->last());
    if (!earth_.ready()) {
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000, "Waiting for the PX4 global reference");
      return;
    }
    const Eigen::Vector3d position = earth_.localNedToEarthEnu(local_position_->positionNed().cast<double>());
    const float heading = local_position_->heading();
    const rclcpp::Time now = node().get_clock()->now();

    switch (state_) {
      case State::Climb:
        if (!target_) {
          if (!walk_) {
            walk_.emplace(params_, rng_, position.x(), position.y());
          }
          setTarget(Eigen::Vector3d(position.x(), position.y(), params_.height), heading);
        }
        if (reached(position)) {
          RCLCPP_INFO(node().get_logger(), "At height %.1f m, starting the random walk", params_.height);
          nextTarget(position);
        }
        break;

      case State::GoTo:
        if (collision_detected_) {
          onCollision(position);
        } else if (reached(position)) {
          RCLCPP_INFO(node().get_logger(), "Go to done");
          enterWait(State::Pause, now);
        }
        break;

      case State::Pause:
        if (now >= wait_until_) {
          nextTarget(position);
        }
        break;

      case State::Brake:
        if (local_position_->velocityNed().head<2>().norm() < params_.brake_speed || now >= wait_until_) {
          startRetreat(position, heading);
        }
        break;

      case State::Retreat:
        if (reached(position)) {
          setTarget(position, heading);
          RCLCPP_INFO(node().get_logger(), "Collision, holding for %.1f s", params_.collision_wait);
          enterWait(State::Hold, now);
        }
        break;

      case State::Hold:
        if (now >= wait_until_) {
          nextTarget(position);
        }
        break;

      case State::Done:
        break;
    }

    if (state_ == State::Done) {
      return;
    }
    if (state_ == State::Brake) {
      // Zero velocity in all axes, heading held; no position setpoint, so nothing pulls the drone back meanwhile
      brake_->update(
        Eigen::Vector3f::Zero(), std::nullopt,
        std::isfinite(heading) ? std::optional<float>(heading) : std::nullopt);
      return;
    }
    // Climbing: only the vertical speed is limited; otherwise the horizontal speed (vertical moves are short)
    const bool climbing = state_ == State::Climb;
    goto_->update(
      earth_.earthEnuToLocalNed(*target_).cast<float>(),
      std::isfinite(target_heading_) ? std::optional<float>(target_heading_) : std::nullopt,
      climbing ? std::nullopt : std::optional<float>(params_.speed),
      climbing ? std::optional<float>(params_.takeoff_speed) : std::optional<float>(params_.speed));
  }

private:
  enum class State
  {
    Climb,    // to height above the takeoff point
    GoTo,     // to the current target, aborted on a collision
    Pause,    // at a reached target for sleep_time
    Brake,    // after a collision, to a stop with a velocity setpoint
    Retreat,  // after braking, back from the target
    Hold,     // after a retreat, for collision_wait
    Done,
  };

  bool reached(const Eigen::Vector3d & position) const
  {
    return (position - *target_).norm() < params_.threshold;
  }

  void setTarget(const Eigen::Vector3d & target, float heading)
  {
    target_ = target;
    target_heading_ = heading;
  }

  void enterWait(State state, const rclcpp::Time & now)
  {
    state_ = state;
    const double wait = state == State::Pause ? params_.sleep_time : params_.collision_wait;
    wait_until_ = now + rclcpp::Duration::from_seconds(wait);
  }

  void nextTarget(const Eigen::Vector3d & position)
  {
    const auto xy = walk_->next();
    if (!xy) {
      RCLCPP_INFO(node().get_logger(), "Random walk done after %ld steps", walk_->step());
      state_ = State::Done;
      completed(px4_ros2::Result::Success);
      return;
    }
    const Eigen::Vector3d target(xy->x(), xy->y(), params_.height);
    // Path facing: towards the target, from north (NED heading)
    const float heading = static_cast<float>(std::atan2(target.x() - position.x(), target.y() - position.y()));
    setTarget(target, heading);
    // Only collisions reported after the target is set abort it
    collision_detected_ = false;
    state_ = State::GoTo;
    if (params_.steps > 0) {
      RCLCPP_INFO(node().get_logger(), "Step %ld/%ld", walk_->step(), params_.steps);
    } else {
      RCLCPP_INFO(node().get_logger(), "Step %ld", walk_->step());
    }
    RCLCPP_INFO(node().get_logger(), "GoTo to position: %.4f, %.4f, %.4f", target.x(), target.y(), target.z());
  }

  void onCollision(const Eigen::Vector3d & position)
  {
    RCLCPP_WARN(
      node().get_logger(), "Collision with agent %d detected, aborting GoTo and holding position",
      colliding_agent_);
    // Away from the target, as seen from where the alert came
    const Eigen::Vector3d to_target = *target_ - position;
    const double dist = to_target.norm();
    retreat_dir_ = dist >= params_.threshold ? std::optional<Eigen::Vector3d>(-to_target / dist) : std::nullopt;
    alert_time_ = node().get_clock()->now();
    state_ = State::Brake;
    wait_until_ = alert_time_ + rclcpp::Duration::from_seconds(params_.brake_timeout);
  }

  void startRetreat(const Eigen::Vector3d & position, float heading)
  {
    const double brake_s = (node().get_clock()->now() - alert_time_).seconds();
    if (!retreat_dir_) {
      // The alert came at the target: no direction to retreat in
      setTarget(position, heading);
      RCLCPP_INFO(
        node().get_logger(), "Stopped in %.2f s, collision, holding for %.1f s", brake_s, params_.collision_wait);
      enterWait(State::Hold, node().get_clock()->now());
      return;
    }
    // Back from where the drone stopped, keeping the heading
    const Eigen::Vector3d retreat = position + params_.retreat * *retreat_dir_;
    setTarget(retreat, heading);
    state_ = State::Retreat;
    RCLCPP_INFO(node().get_logger(), "Stopped in %.2f s", brake_s);
    RCLCPP_INFO(
      node().get_logger(), "Retreat to position: %.4f, %.4f, %.4f", retreat.x(), retreat.y(), retreat.z());
  }

  const WalkParams & params_;
  EarthFrame earth_;
  std::mt19937 rng_;
  std::optional<RandomWalk> walk_;

  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> goto_;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> brake_;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> local_position_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr collision_sub_;

  State state_{State::Climb};
  std::optional<Eigen::Vector3d> target_;  // earth frame
  float target_heading_{NAN};
  rclcpp::Time wait_until_;
  rclcpp::Time alert_time_;
  std::optional<Eigen::Vector3d> retreat_dir_;  // unit vector away from the aborted target
  bool collision_detected_{false};
  uint8_t colliding_agent_{0};
};

class RandomWalkExecutor : public px4_ros2::ModeExecutorBase
{
public:
  // takeoff_altitude: AMSL (m), what NAV_TAKEOFF takes
  RandomWalkExecutor(RandomWalkMode & mode, float takeoff_altitude)
  : ModeExecutorBase(
      ModeExecutorBase::Settings{}.activate(ModeExecutorBase::Settings::Activation::ActivateImmediately), mode),
    node_(mode.node()), takeoff_altitude_(takeoff_altitude) {}

  void onActivate() override {runState(State::WaitReadyToArm, px4_ros2::Result::Success);}

  void onDeactivate(DeactivateReason reason) override
  {
    RCLCPP_WARN(
      node_.get_logger(), "Executor deactivated (%s)",
      reason == DeactivateReason::FailsafeActivated ? "failsafe" : "mode switched outside the mission");
  }

private:
  enum class State
  {
    WaitReadyToArm,
    Arm,
    Takeoff,
    Walk,
    Hold,
  };

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        node_.get_logger(), "Mission step %d failed: %s", static_cast<int>(state),
        px4_ros2::resultToString(previous_result));
      if (state != State::Hold && state > State::Takeoff) {
        runState(State::Hold, px4_ros2::Result::Success);
      }
      return;
    }

    switch (state) {
      case State::WaitReadyToArm:
        RCLCPP_INFO(node_.get_logger(), "Waiting until ready to arm");
        waitReadyToArm([this](px4_ros2::Result result) {runState(State::Arm, result);});
        break;

      case State::Arm:
        RCLCPP_INFO(node_.get_logger(), "Arm");
        arm([this](px4_ros2::Result result) {runState(State::Takeoff, result);});
        break;

      case State::Takeoff:
        RCLCPP_INFO(node_.get_logger(), "Take off to %.1f m AMSL", takeoff_altitude_);
        takeoff([this](px4_ros2::Result result) {runState(State::Walk, result);}, takeoff_altitude_);
        break;

      case State::Walk:
        RCLCPP_INFO(node_.get_logger(), "Run mission");
        scheduleMode(ownedMode().id(), [this](px4_ros2::Result result) {runState(State::Hold, result);});
        break;

      case State::Hold:
        RCLCPP_INFO(node_.get_logger(), "End mission, holding position");
        scheduleMode(px4_ros2::ModeBase::kModeIDLoiter, [](px4_ros2::Result) {});
        break;
    }
  }

  rclcpp::Node & node_;
  float takeoff_altitude_;
};

}  // namespace px4_drone

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("random_walk");

  // "" = the fmu topics relative to the node namespace (/<ns>/fmu/...), which is where PX4_UXRCE_DDS_NS puts them
  const std::string prefix = node->declare_parameter<std::string>("px4_namespace", "");
  const px4_drone::GeoPoint origin = px4_drone::declareOrigin(*node);
  const px4_drone::WalkParams params = px4_drone::WalkParams::declare(*node);

  auto mode = std::make_shared<px4_drone::RandomWalkMode>(*node, params, origin, prefix);
  auto executor = std::make_shared<px4_drone::RandomWalkExecutor>(
    *mode, static_cast<float>(origin.alt_m + params.height));

  // The FMU topics may not be up yet (PX4 / the DDS agent still starting): retry instead of giving up
  while (rclcpp::ok() && !executor->doRegister()) {
    RCLCPP_WARN(node->get_logger(), "Registration failed, retrying in 1 s");
    rclcpp::sleep_for(px4_drone::kRegistrationRetryDelay);
  }
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
