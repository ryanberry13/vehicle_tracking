#include <chrono>
#include <cmath>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <vehicle_tracking/path_math.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std::chrono_literals;


class PathGeneration : public rclcpp::Node
{
public:
  PathGeneration()
  : Node("path_generation")
  {
    // Parameters
    rate_hz_ = declare_parameter<double>("rate_hz", rate_hz_);
    period_s_ = declare_parameter<double>("period_s", period_s_);
    cruise_speed_mps_ = declare_parameter<double>("cruise_speed_mps", cruise_speed_mps_);
    lookahead_dist_m_ = declare_parameter<double>("lookahead_dist_m", lookahead_dist_m_);


    // Subscribers
    gv_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/gv/odom", 10, std::bind(&PathGeneration::gvOdomCallback, this, std::placeholders::_1));
    gv_navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gv/navsat", 10, std::bind(&PathGeneration::gvNavSatCallback, this, std::placeholders::_1));
    plane_global_pos_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/plane/global_position", 10, std::bind(&PathGeneration::planeGlobalPositionCallback, this, std::placeholders::_1));
    plane_local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/plane/local_position", 10, std::bind(&PathGeneration::planeLocalPositionCallback, this, std::placeholders::_1));
    // Publishers
    lookahead_navsat_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/path/lookahead_navsat", 10);

    // Timer to update path at specified rate (independent of incoming vehicle updates)
    setTimerFromRate(rate_hz_);

  }
  
private:
    void setTimerFromRate(double rate_hz)
    {
      const double safe_rate = std::max(1e-3, rate_hz);
      auto period = std::chrono::duration<double>(1.0 / safe_rate);
      timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PathGeneration::updatePath, this));
    }

    void updatePath(){
        double gv_x, gv_y, gv_z;
        local_cart_.Forward(ground_vehicle_state_.latitude, ground_vehicle_state_.longitude, 0.0, gv_x, gv_y, gv_z);
        RCLCPP_INFO(get_logger(), "GV local position ENU: x=%.2f m, y=%.2f m", gv_x, gv_y);

        double plane_x, plane_y, plane_z;
        local_cart_.Forward(plane_state_.latitude, plane_state_.longitude, plane_state_.altitude, plane_x, plane_y, plane_z);
        RCLCPP_INFO(get_logger(), "Plane local position ENU: x=%.2f m, y=%.2f m", plane_x, plane_y);
        Vec2 plane_pos_global(plane_x, plane_y);

        GvFrame gv_frame = buildGvFrame(
          gv_x,
          gv_y,
          ground_vehicle_state_.heading,
          lead_dist_m_);

        Vec2 plane_pos_local = globalToGvLocal(plane_pos_global, gv_frame);
        RCLCPP_INFO(get_logger(), "Plane local position GV frame: x=%.2f m, y=%.2f m", plane_pos_local.x, plane_pos_local.y);

        // plane_state_.cog: 0=N, 90=E, 180=S, 270=W. Convert to ENU yaw (0=E, +CCW to N).
        const double plane_cog_enu_rad = (M_PI / 2.0) - plane_state_.cog;
        const double rel_course_rad = plane_cog_enu_rad - ground_vehicle_state_.heading;
        const double plane_vy_local_mps = plane_state_.groundspeed * std::sin(rel_course_rad);

        const double gv_speed_safe_mps = std::max(1e-3, ground_vehicle_state_.speed);
        const double spatial_wavelength_m = gv_speed_safe_mps * period_s_;
        const double spatial_angular_freq_rad_m = 2.0 * M_PI / spatial_wavelength_m;
        double amplitude_m = generatePathAmplitude(ground_vehicle_state_.speed, cruise_speed_mps_, period_s_);
        RCLCPP_INFO(get_logger(), "Generated path amplitude: %.2f m", amplitude_m);
    
        // Spatial sine wave e = A*sin(k * s + phi0)
        // where s is along-track distance, A is amplitude, k is spatial angular frequency, e is lateral offset, phi is phase offset

        // Solve phase so y_plane = A*sin(k*x_plane + phase) while using lateral velocity sign to
        // disambiguate branch and clamping to 90/270 deg when |y| >= A.
        double phase_offset_rad = 0.0;
        if (amplitude_m > 1e-6) {
          const double y = plane_pos_local.y;
          const double x = plane_pos_local.x;
          const auto wrap_0_2pi = [](double a) {
            double w = std::fmod(a, 2.0 * M_PI);
            if (w < 0.0) {
              w += 2.0 * M_PI;
            }
            return w;
          };

          if (std::abs(y) >= amplitude_m) {
            phase_offset_rad = (y >= 0.0) ? (M_PI / 2.0) : (3.0 * M_PI / 2.0);
          } else {
            const double sin_arg = y / amplitude_m;
            const double theta_1 = std::asin(sin_arg);
            const double theta_2 = M_PI - theta_1;
            const double phase_1 = wrap_0_2pi(theta_1 - spatial_angular_freq_rad_m * x);
            const double phase_2 = wrap_0_2pi(theta_2 - spatial_angular_freq_rad_m * x);

            if (std::abs(plane_vy_local_mps) < 1e-3) {
              if (have_prev_phase_) {
                const auto angle_distance = [&](double a, double b) {
                  const double d = std::abs(a - b);
                  return std::min(d, 2.0 * M_PI - d);
                };
                phase_offset_rad =
                  (angle_distance(phase_1, prev_phase_offset_rad_) <= angle_distance(phase_2, prev_phase_offset_rad_))
                    ? phase_1
                    : phase_2;
              } else {
                phase_offset_rad = phase_1;
              }
            } else {
              const double velocity_sign = (plane_vy_local_mps >= 0.0) ? 1.0 : -1.0;
              phase_offset_rad = (std::cos(theta_1) * velocity_sign >= 0.0) ? phase_1 : phase_2;
            }
          }

          phase_offset_rad = wrap_0_2pi(phase_offset_rad);
        }

        prev_phase_offset_rad_ = phase_offset_rad;
        have_prev_phase_ = true;

        const double lookahead_x_local_m = plane_pos_local.x + lookahead_dist_m_;
        const double lookahead_y_local_m = amplitude_m * std::sin(
          spatial_angular_freq_rad_m * lookahead_x_local_m + phase_offset_rad);
        const Vec2 lookahead_local(lookahead_x_local_m, lookahead_y_local_m);
        const Vec2 lookahead_global = gvLocalToGlobal(lookahead_local, gv_frame);

        double lookahead_lat = 0.0;
        double lookahead_lon = 0.0;
        double lookahead_alt = 0.0;
        local_cart_.Reverse(
          lookahead_global.x,
          lookahead_global.y,
          0.0,
          lookahead_lat,
          lookahead_lon,
          lookahead_alt);

        sensor_msgs::msg::NavSatFix lookahead_fix{};
        lookahead_fix.header.stamp = now();
        lookahead_fix.header.frame_id = "lookahead";
        lookahead_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        lookahead_fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        lookahead_fix.latitude = lookahead_lat;
        lookahead_fix.longitude = lookahead_lon;
        lookahead_fix.altitude = lookahead_alt;
        lookahead_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        lookahead_navsat_pub_->publish(lookahead_fix);

        RCLCPP_INFO(
          get_logger(),
          "Phase estimate: %.1f deg (y=%.2f m, vy_local=%.2f m/s, A=%.2f m)",
          radToDeg(phase_offset_rad),
          plane_pos_local.y,
          plane_vy_local_mps,
          amplitude_m);
        RCLCPP_INFO(
          get_logger(),
          "Lookahead point: local(x=%.2f, y=%.2f) globalENU(x=%.2f, y=%.2f) lat=%.7f lon=%.7f",
          lookahead_local.x,
          lookahead_local.y,
          lookahead_global.x,
          lookahead_global.y,
          lookahead_lat,
          lookahead_lon);


      }

    void gvOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        // Process vehicle odometry data
        const auto& odom = *msg;
        ground_vehicle_state_.speed = std::hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y); 
        geometry_msgs::msg::Quaternion q = odom.pose.pose.orientation;
        ground_vehicle_state_.heading = quatToYaw(q); // radians
        // RCLCPP_INFO(get_logger(), "GV heading: %.2f deg", ground_vehicle_state_.heading);
    }

    void gvNavSatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
        // Process vehicle GNSS data
        const auto& fix  = *msg;
        if (fix.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
          if (gv_lat_origin_ == 0.0 && gv_lon_origin_ == 0.0){
              gv_lat_origin_ = fix.latitude;
              gv_lon_origin_ = fix.longitude;
              RCLCPP_INFO(get_logger(), "Set GV origin to lat: %.6f deg, lon: %.6f deg", gv_lat_origin_, gv_lon_origin_);
              local_cart_.Reset(gv_lat_origin_, gv_lon_origin_);
          }
          ground_vehicle_state_.latitude = fix.latitude;
          ground_vehicle_state_.longitude = fix.longitude;
        }
        else{
          RCLCPP_WARN(get_logger(), "No GPS fix available, skipping update...");
        }
    }

    void planeGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg){
        // Process plane global position data if needed for path generation
        const auto& global_pos = *msg;
        plane_state_.latitude = global_pos.lat;
        plane_state_.longitude = global_pos.lon;
        plane_state_.altitude = global_pos.alt;
      }

    void planeLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){
        // Process plane local position data if needed for path generation
        const auto& local_pos = *msg;
        plane_state_.groundspeed = std::hypot(local_pos.vx, local_pos.vy);
        plane_state_.cog = std::atan2(local_pos.vy, local_pos.vx); // radians
        plane_state_.heading = local_pos.heading; //radians
    }

    struct GroundVehicleState {
      double latitude;
      double longitude;
      double speed;
      double heading;
    } ground_vehicle_state_;

    struct PlaneState {
      double latitude;
      double longitude;
      double altitude;
      double groundspeed;
      double cog;
      double airspeed;
      double heading;
    } plane_state_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gv_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gv_navsat_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr plane_global_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr plane_local_pos_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr lookahead_navsat_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    double rate_hz_{5.0};


    double period_s_{10.0};
    double cruise_speed_mps_{20.0};
    double lookahead_dist_m_{30.0};
    double gv_lat_origin_{0.0};
    double gv_lon_origin_{0.0};
    double lead_dist_m_{0.0};
    bool have_prev_phase_{false};
    double prev_phase_offset_rad_{0.0};

    GeographicLib::LocalCartesian local_cart_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathGeneration>());
  rclcpp::shutdown();
  return 0;
}
