// simulated_vehicle_node.cpp
//
// Straight-line ground-vehicle simulator that publishes:
//  - /gv/odom   (nav_msgs/Odometry) in a local ENU "map" frame
//  - /gv/navsat (sensor_msgs/NavSatFix) as global lat/lon/alt using GeographicLib
//
// ENU convention used internally:
//   x = East (m), y = North (m), z = Up (m)
//
// Run example:
//   ros2 run vehicle_tracking simulated_vehicle_node \
//     --ros-args -p speed_mps:=5.0 -p heading_deg:=90.0 -p rate_hz:=50.0 \
//                -p origin_lat_deg:=44.2253 -p origin_lon_deg:=-76.4951 -p origin_alt_m:=80.0

#include <chrono>
#include <cmath>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;

static geometry_msgs::msg::Quaternion yawToQuat(double yaw_rad)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw_rad * 0.5);
  q.w = std::cos(yaw_rad * 0.5);
  return q;
}

class SimulatedVehicleNode : public rclcpp::Node
{
public:
  SimulatedVehicleNode()
  : Node("simulated_vehicle_node")
  {
    // --- Params ---
    speed_mps_ = declare_parameter<double>("speed_mps", 5.0);
    heading_deg_ = declare_parameter<double>("heading_deg", 0.0);   // 0 deg = +East, 90 deg = +North
    rate_hz_ = declare_parameter<double>("rate_hz", rate_hz_);

    // WGS84 origin for LocalCartesian (defines your "map" frame)
    origin_lat_deg_ = declare_parameter<double>("origin_lat_deg", 44.2253);
    origin_lon_deg_ = declare_parameter<double>("origin_lon_deg", -76.4951);
    origin_alt_m_ = declare_parameter<double>("origin_alt_m", 0.0);

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "ground_vehicle");
    gps_frame_id_ = declare_parameter<std::string>("gps_frame_id", "gps");

    // Reset local tangent plane at origin
    local_cart_.Reset(origin_lat_deg_, origin_lon_deg_, origin_alt_m_);

    // --- Publishers ---
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gv/odom", 10);
    navsat_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gv/navsat", 10);

    // --- Timer ---
    last_time_ = now();
    setTimerFromRate(rate_hz_);
  }

private:
  void setTimerFromRate(double rate_hz)
  {
    const double safe_rate = std::max(1e-3, rate_hz);
    auto period = std::chrono::duration<double>(1.0 / safe_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimulatedVehicleNode::tick, this));
  }

  void tick()
  {
    // (Optional) re-read params each tick so you can tweak via ros2 param set
    speed_mps_ = get_parameter("speed_mps").as_double();
    heading_deg_ = get_parameter("heading_deg").as_double();
    const double new_rate = get_parameter("rate_hz").as_double();

    // If rate changed, recreate timer (simple approach)
    if (std::abs(new_rate - rate_hz_) > 1e-6) {
      rate_hz_ = new_rate;
      setTimerFromRate(rate_hz_);
      // reset timing so dt doesn't spike
      last_time_ = now();
      return;
    }

    const rclcpp::Time t = now();
    const double dt = (t - last_time_).seconds();
    last_time_ = t;

    // Guard against weird dt (pause/resume)
    const double dt_safe = std::clamp(dt, 0.0, 0.2);

    // Heading: 0 deg = +East, 90 deg = +North (ENU)
    const double psi = heading_deg_ * M_PI / 180.0;
    const double v = speed_mps_;

    // Integrate straight-line motion in ENU
    x_e_m_ += v * std::cos(psi) * dt_safe;
    y_n_m_ += v * std::sin(psi) * dt_safe;
    z_u_m_ = 0.0;

    publishOdom(t, psi, v);
    publishNavSat(t);
  }

  void publishOdom(const rclcpp::Time& stamp, double yaw_rad, double speed_mps)
  {
    nav_msgs::msg::Odometry odom{};
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    // Pose in map (ENU meters)
    odom.pose.pose.position.x = x_e_m_;
    odom.pose.pose.position.y = y_n_m_;
    odom.pose.pose.position.z = z_u_m_;
    odom.pose.pose.orientation = yawToQuat(yaw_rad);

    // Twist (simple): forward speed reported on x
    odom.twist.twist.linear.x = speed_mps;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub_->publish(odom);
  }

  void publishNavSat(const rclcpp::Time& stamp)
  {
    // Convert local ENU meters -> WGS84 lat/lon/alt
    double lat_deg = 0.0, lon_deg = 0.0, alt_m = 0.0;
    local_cart_.Reverse(x_e_m_, y_n_m_, z_u_m_, lat_deg, lon_deg, alt_m);

    sensor_msgs::msg::NavSatFix fix{};
    fix.header.stamp = stamp;
    fix.header.frame_id = gps_frame_id_;

    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    fix.latitude = lat_deg;
    fix.longitude = lon_deg;
    fix.altitude = alt_m;

    // Unknown covariance (you can set a realistic value later)
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    navsat_pub_->publish(fix);
  }

  // Publishers/timer
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Time + state
  rclcpp::Time last_time_;
  double x_e_m_{0.0};  // East
  double y_n_m_{0.0};  // North
  double z_u_m_{0.0};  // Up

  // Params
  double speed_mps_{5.0};
  double heading_deg_{0.0};
  double rate_hz_{5.0};

  double origin_lat_deg_{0.0};
  double origin_lon_deg_{0.0};
  double origin_alt_m_{0.0};

  std::string frame_id_{"map"};
  std::string child_frame_id_{"ground_vehicle"};
  std::string gps_frame_id_{"gps"};

  // GeographicLib converter (WGS84 <-> local ENU)
  GeographicLib::LocalCartesian local_cart_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedVehicleNode>());
  rclcpp::shutdown();
  return 0;
}
