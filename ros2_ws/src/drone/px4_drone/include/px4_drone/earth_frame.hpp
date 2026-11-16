#pragma once

// Shared earth frame of all drones: ENU around one fixed geodetic origin.
//
// PX4 runs one EKF per drone, each with its own local NED frame whose origin (ref_lat, ref_lon, ref_alt in
// vehicle_local_position) is where that drone first got a GPS fix, roughly its spawn point. To compare the
// reachable sets of different drones, every drone's state is moved into the same earth frame. Over the few
// hundred meters of the simulation, the two frames differ only by a translation: the offset of the drone's
// EKF origin from the shared origin.
//
// Both state_estimate (what the verifiers see) and random_walk (where the drone is sent) use this class, so the
// two always agree on the frame.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Eigen/Core>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_drone
{

struct GeoPoint
{
  double lat_deg;
  double lon_deg;
  double alt_m;  // AMSL
};

// spherical_coordinates of the PX4 baylands world
inline constexpr GeoPoint kDefaultOrigin{37.412173071650805, -121.998878727967, 38.0};

// Reads the shared origin from the origin.lat / origin.lon / origin.alt parameters (config/earth_frame.yaml)
inline GeoPoint declareOrigin(rclcpp::Node & node)
{
  return {
    node.declare_parameter("origin.lat", kDefaultOrigin.lat_deg),
    node.declare_parameter("origin.lon", kDefaultOrigin.lon_deg),
    node.declare_parameter("origin.alt", kDefaultOrigin.alt_m)};
}

// NED <-> ENU swaps x/y and flips z; the same permutation works in both directions
inline Eigen::Vector3d nedToEnu(const Eigen::Vector3d & v) {return {v.y(), v.x(), -v.z()};}
inline Eigen::Vector3d enuToNed(const Eigen::Vector3d & v) {return {v.y(), v.x(), -v.z()};}

// North / east (m) of (lat, lon) in the azimuthal equidistant projection around ref, the projection PX4 uses for
// its local frame (PX4-Autopilot src/lib/geo/geo.cpp, MapProjection::project)
inline Eigen::Vector2d projectNorthEast(const GeoPoint & ref, double lat_deg, double lon_deg)
{
  constexpr double kEarthRadius = 6371000.0;  // CONSTANTS_RADIUS_OF_EARTH
  const double ref_lat = ref.lat_deg * M_PI / 180.0;
  const double lat = lat_deg * M_PI / 180.0;
  const double d_lon = (lon_deg - ref.lon_deg) * M_PI / 180.0;

  const double cos_d_lon = std::cos(d_lon);
  const double arg = std::clamp(
    std::sin(ref_lat) * std::sin(lat) + std::cos(ref_lat) * std::cos(lat) * cos_d_lon, -1.0, 1.0);
  const double c = std::acos(arg);
  const double k = std::abs(c) > 0.0 ? c / std::sin(c) : 1.0;

  return {
    k * (std::cos(ref_lat) * std::sin(lat) - std::sin(ref_lat) * std::cos(lat) * cos_d_lon) * kEarthRadius,
    k * std::cos(lat) * std::sin(d_lon) * kEarthRadius};
}

class EarthFrame
{
public:
  explicit EarthFrame(const GeoPoint & origin)
  : origin_(origin) {}

  // Takes the EKF origin from a vehicle_local_position message. Returns ready(). The offset is recomputed only when
  // PX4 moves its reference (ref_timestamp changes), e.g. on an EKF global origin reset.
  bool update(const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    if (!msg.xy_global || !msg.z_global) {
      return ready_;
    }
    if (ready_ && msg.ref_timestamp == ref_timestamp_) {
      return true;
    }
    const Eigen::Vector2d ne = projectNorthEast(origin_, msg.ref_lat, msg.ref_lon);
    offset_enu_ = {ne.y(), ne.x(), static_cast<double>(msg.ref_alt) - origin_.alt_m};
    ref_timestamp_ = msg.ref_timestamp;
    ready_ = true;
    return true;
  }

  // False until PX4 has a global reference (GPS fix); the conversions below are meaningless before that
  bool ready() const {return ready_;}

  const GeoPoint & origin() const {return origin_;}

  // Position of the drone's EKF origin in the earth frame
  const Eigen::Vector3d & offsetEnu() const {return offset_enu_;}

  Eigen::Vector3d localNedToEarthEnu(const Eigen::Vector3d & ned) const {return nedToEnu(ned) + offset_enu_;}

  Eigen::Vector3d earthEnuToLocalNed(const Eigen::Vector3d & enu) const {return enuToNed(enu - offset_enu_);}

private:
  GeoPoint origin_;
  Eigen::Vector3d offset_enu_{Eigen::Vector3d::Zero()};
  uint64_t ref_timestamp_{0};
  bool ready_{false};
};

}  // namespace px4_drone
