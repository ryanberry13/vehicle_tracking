// simulated_vehicle_node.cpp
//
// Ground-vehicle simulator that publishes:
//  - /gv/odom   (nav_msgs/Odometry) in a local ENU "map" frame
//  - /gv/navsat (sensor_msgs/NavSatFix) as global lat/lon/alt using GeographicLib
//
// ENU convention used internally:
//   x = East (m), y = North (m), z = Up (m)
//
// path_mode options:
//   1: straight, constant speed
//   2: straight, variable speed profile (minute-scale timing)
//   3: constant speed, gentle alternating curvature (after 20s)
//   4: variable speed + gentle alternating curvature
//
// Run example:
//   ros2 run vehicle_tracking simulated_vehicle_node --ros-args
//     -p speed_mps:=5.0 -p heading_deg:=90.0 -p rate_hz:=50.0
//     -p origin_lat_deg:=44.2253 -p origin_lon_deg:=-76.4951 -p origin_alt_m:=80.0
//     -p path_mode:=4

#include <algorithm>
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
    speed_mps_ = declare_parameter<double>("speed_mps", speed_mps_);
    heading_deg_ = declare_parameter<double>("heading_deg", heading_deg_);  // 0 deg = +East, 90 deg = +North
    rate_hz_ = declare_parameter<double>("rate_hz", rate_hz_);
    path_mode_ = declare_parameter<int64_t>("path_mode", path_mode_);

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
    sim_start_time_ = last_time_;
    heading_rad_ = heading_deg_ * M_PI / 180.0;
    setTimerFromRate(rate_hz_);
  }

private:
  static double lerp(double a, double b, double u)
  {
    return a + (b - a) * std::clamp(u, 0.0, 1.0);
  }

  double speedProfileMps(int mode, double t_since_start_s, double base_speed_mps) const
  {
    const double base = std::max(0.0, base_speed_mps);
    if (mode == 1 || mode == 3) {
      return base;
    }

    // Modes 2 and 4: speed schedule with holds and ramps.
    // Mode 2 uses much longer (minute-scale) timing.
    const bool minute_scale = (mode == 2);
    const double startup_delay_s = 20.0;
    const double ramp_up_first_s = 20.0;
    const double hold_high_s = minute_scale ? 180.0 : 15.0;
    const double ramp_down_s = 20.0;
    const double hold_low_s = minute_scale ? 180.0 : 15.0;
    const double ramp_up_s = 20.0;

    if (t_since_start_s < startup_delay_s) {
      return base;
    }

    const double v_high = 8.0;
    const double v_low = 3.0;

    double tau = t_since_start_s - startup_delay_s;
    if (tau < ramp_up_first_s) {
      return lerp(base, v_high, tau / ramp_up_first_s);
    }

    tau -= ramp_up_first_s;
    const double cycle_s = hold_high_s + ramp_down_s + hold_low_s + ramp_up_s;
    double phase = std::fmod(tau, cycle_s);

    if (phase < hold_high_s) {
      return v_high;
    }
    phase -= hold_high_s;

    if (phase < ramp_down_s) {
      return lerp(v_high, v_low, phase / ramp_down_s);
    }
    phase -= ramp_down_s;

    if (phase < hold_low_s) {
      return v_low;
    }
    phase -= hold_low_s;

    return lerp(v_low, v_high, phase / ramp_up_s);
  }

  double yawRateProfileRadS(int mode, double t_since_start_s) const
  {
    if (mode == 1 || mode == 2) {
      return 0.0;
    }

    // Modes 3 and 4: start gentle alternating curvature after 20s.
    if (t_since_start_s < 20.0) {
      return 0.0;
    }

    const double curve_rate_rad_s = 2.0 * M_PI / 180.0;  // gentle turn rate
    const double turn_angle_rad = M_PI / 2.0;            // ~90 deg per turn segment
    const double curve_duration_s = turn_angle_rad / curve_rate_rad_s;
    const double straight_duration_s = 20.0;
    const double cycle_s = 2.0 * (curve_duration_s + straight_duration_s);

    double phase = std::fmod(t_since_start_s - 20.0, cycle_s);
    if (phase < curve_duration_s) {
      return curve_rate_rad_s;
    }
    phase -= curve_duration_s;
    if (phase < straight_duration_s) {
      return 0.0;
    }
    phase -= straight_duration_s;
    if (phase < curve_duration_s) {
      return -curve_rate_rad_s;
    }
    return 0.0;
  }

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
    path_mode_ = get_parameter("path_mode").as_int();
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
    const double t_since_start_s = (t - sim_start_time_).seconds();

    // Guard against weird dt (pause/resume)
    const double dt_safe = std::clamp(dt, 0.0, 0.2);

    const int mode = static_cast<int>(std::clamp<int64_t>(path_mode_, 1, 4));
    if (mode != last_path_mode_) {
      RCLCPP_INFO(get_logger(), "Using path_mode=%d", mode);
      last_path_mode_ = mode;
    }

    const double yaw_rate_rad_s = yawRateProfileRadS(mode, t_since_start_s);
    const double v = speedProfileMps(mode, t_since_start_s, speed_mps_);
    const double dv = v - prev_commanded_speed_mps_;
    constexpr double kSpeedChangeEps = 1e-3;
    int speed_trend = 0;  // 1: speeding up, -1: slowing down, 0: steady
    if (dv > kSpeedChangeEps) {
      speed_trend = 1;
    } else if (dv < -kSpeedChangeEps) {
      speed_trend = -1;
    }

    if (!have_prev_commanded_speed_ || speed_trend != last_speed_trend_) {
      if (speed_trend == 1) {
        RCLCPP_INFO(get_logger(), "Speed profile: speeding up (mode=%d, v=%.2f m/s)", mode, v);
      } else if (speed_trend == -1) {
        RCLCPP_INFO(get_logger(), "Speed profile: slowing down (mode=%d, v=%.2f m/s)", mode, v);
      } else {
        RCLCPP_INFO(get_logger(), "Speed profile: holding speed (mode=%d, v=%.2f m/s)", mode, v);
      }
      last_speed_trend_ = speed_trend;
    }
    prev_commanded_speed_mps_ = v;
    have_prev_commanded_speed_ = true;

    // Modes 1/2 keep configured heading; modes 3/4 integrate yaw rate.
    if (mode == 1 || mode == 2) {
      heading_rad_ = heading_deg_ * M_PI / 180.0;
    } else {
      heading_rad_ += yaw_rate_rad_s * dt_safe;
    }
    heading_rad_ = std::atan2(std::sin(heading_rad_), std::cos(heading_rad_));
    const double psi = heading_rad_;

    // Integrate motion in ENU
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
  rclcpp::Time sim_start_time_;
  double x_e_m_{0.0};  // East
  double y_n_m_{0.0};  // North
  double z_u_m_{0.0};  // Up
  double heading_rad_{0.0};

  // Params
  double speed_mps_{5.0};
  double heading_deg_{0.0};
  double rate_hz_{5.0};
  int64_t path_mode_{1};
  int last_path_mode_{-1};
  bool have_prev_commanded_speed_{false};
  double prev_commanded_speed_mps_{0.0};
  int last_speed_trend_{0};

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
