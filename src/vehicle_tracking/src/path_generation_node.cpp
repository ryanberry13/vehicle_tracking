#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>
#include <iomanip>
#include <sstream>

#include <GeographicLib/LocalCartesian.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

#include <vehicle_tracking/path_math.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#ifdef VEHICLE_TRACKING_HAS_FOXGLOVE_MSGS
#include <foxglove_msgs/msg/geo_json.hpp>
#endif

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
    lookahead_integration_dx_m_ = declare_parameter<double>("lookahead_integration_dx_m", lookahead_integration_dx_m_);
    sine_geojson_half_span_m_ = declare_parameter<double>("sine_geojson_half_span_m", sine_geojson_half_span_m_);
    sine_geojson_step_m_ = declare_parameter<double>("sine_geojson_step_m", sine_geojson_step_m_);
    amplitude_scale_deadband_m_ = declare_parameter<double>("amplitude_scale_deadband_m", amplitude_scale_deadband_m_);
    amplitude_scale_double_at_m_ = declare_parameter<double>("amplitude_scale_double_at_m", amplitude_scale_double_at_m_);
    amplitude_scale_cap_at_m_ = declare_parameter<double>("amplitude_scale_cap_at_m", amplitude_scale_cap_at_m_);
    amplitude_scale_cap_factor_ = declare_parameter<double>("amplitude_scale_cap_factor", amplitude_scale_cap_factor_);


    // Subscribers
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    gv_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/gv/odom", 10, std::bind(&PathGeneration::gvOdomCallback, this, std::placeholders::_1));
    gv_navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gv/navsat", 10, std::bind(&PathGeneration::gvNavSatCallback, this, std::placeholders::_1));
    plane_global_pos_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", px4_qos, std::bind(&PathGeneration::planeGlobalPositionCallback, this, std::placeholders::_1));
    plane_local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos, std::bind(&PathGeneration::planeLocalPositionCallback, this, std::placeholders::_1));
    // Publishers
    lookahead_navsat_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/path/lookahead_navsat", 10);
    plane_navsat_debug_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/path/debug/plane_navsat", 10);
    base_amplitude_debug_pub_ = create_publisher<std_msgs::msg::Float64>("/path/debug/base_amplitude_m", 10);
    scaled_amplitude_debug_pub_ = create_publisher<std_msgs::msg::Float64>("/path/debug/scaled_amplitude_m", 10);
    gv_local_x_error_debug_pub_ = create_publisher<std_msgs::msg::Float64>("/path/debug/gv_local_x_error_m", 10);
    gv_local_y_error_debug_pub_ = create_publisher<std_msgs::msg::Float64>("/path/debug/gv_local_y_error_m", 10);
    phase_deg_debug_pub_ = create_publisher<std_msgs::msg::Float64>("/path/debug/phase_estimate_deg", 10);
#ifdef VEHICLE_TRACKING_HAS_FOXGLOVE_MSGS
    sine_wave_geojson_pub_ = create_publisher<foxglove_msgs::msg::GeoJSON>("/path/debug/sine_wave_geojson", 10);
#endif

    // Timer to update path at specified rate (independent of incoming vehicle updates)
    setTimerFromRate(rate_hz_);

  }
  
private:
    Vec2 advanceAlongSineByArcLength(
      double x0,
      double amplitude_m,
      double wavenumber_rad_m,
      double phase_rad,
      double lookahead_arc_m,
      double dx_m) const
    {
      const double q = std::max(0.0, lookahead_arc_m);
      const double dx = std::max(1e-3, dx_m);
      double x = x0;
      double y = amplitude_m * std::sin(wavenumber_rad_m * x + phase_rad);

      if (q <= 1e-6) {
        return {x, y};
      }

      double s = 0.0;
      const int kMaxSteps = 200000;
      for (int i = 0; i < kMaxSteps; ++i) {
        const double x_next = x + dx;
        const double y_next = amplitude_m * std::sin(wavenumber_rad_m * x_next + phase_rad);
        const double ds = std::hypot(x_next - x, y_next - y);

        if (s + ds >= q && ds > 1e-9) {
          const double segment_fraction = (q - s) / ds;
          const double x_interp = x + (x_next - x) * segment_fraction;
          const double y_interp = y + (y_next - y) * segment_fraction;
          return {x_interp, y_interp};
        }

        s += ds;
        x = x_next;
        y = y_next;
      }

      return {x, y};
    }

    double amplitudeScaleFromAlongTrackError(double along_track_error_m) const
    {
      const double deadband = std::max(0.0, amplitude_scale_deadband_m_);
      const double double_at = std::max(deadband + 1e-3, amplitude_scale_double_at_m_);
      const double cap_at = std::max(double_at + 1e-3, amplitude_scale_cap_at_m_);
      const double cap_factor = std::max(2.0, amplitude_scale_cap_factor_);

      const double abs_err = std::abs(along_track_error_m);
      const bool ahead = along_track_error_m >= 0.0;

      if (abs_err <= deadband) {
        return 1.0;
      }

      if (abs_err <= double_at) {
        const double u = (abs_err - deadband) / (double_at - deadband);
        return ahead ? (1.0 + u) : (1.0 - 0.5 * u);  // 1->2 (ahead), 1->0.5 (behind)
      }

      const double u = std::clamp((abs_err - double_at) / (cap_at - double_at), 0.0, 1.0);
      return ahead
        ? (2.0 + (cap_factor - 2.0) * u)
        : (0.5 + ((1.0 / cap_factor) - 0.5) * u);
    }

    void publishSineWaveGeoJson(
      const GvFrame& gv_frame,
      const Vec2& plane_pos_local,
      const Vec2& lookahead_local,
      double amplitude_m,
      double wavenumber_rad_m,
      double phase_offset_rad)
    {
#ifdef VEHICLE_TRACKING_HAS_FOXGLOVE_MSGS
      if (!sine_wave_geojson_pub_) {
        return;
      }

      const double half_span = std::max(20.0, sine_geojson_half_span_m_);
      const double sample_step = std::max(0.5, sine_geojson_step_m_);
      const double x_start = plane_pos_local.x - half_span;
      const double x_end = plane_pos_local.x + half_span;

      std::ostringstream json;
      json << std::fixed << std::setprecision(7);
      json << "{\"type\":\"FeatureCollection\",\"features\":[";
      json << "{\"type\":\"Feature\",\"properties\":{";
      json << "\"name\":\"desired_sine_path\",";
      json << "\"amplitude_m\":" << amplitude_m << ",";
      json << "\"wavenumber_rad_m\":" << wavenumber_rad_m << ",";
      json << "\"phase_rad\":" << phase_offset_rad;
      json << "},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[";

      bool first_point = true;
      for (double x = x_start; x <= x_end + 1e-9; x += sample_step) {
        const double y = amplitude_m * std::sin(wavenumber_rad_m * x + phase_offset_rad);
        const Vec2 p_global = gvLocalToGlobal({x, y}, gv_frame);

        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;
        local_cart_.Reverse(p_global.x, p_global.y, 0.0, lat, lon, alt);
        (void)alt;

        if (!first_point) {
          json << ",";
        }
        first_point = false;
        json << "[" << lon << "," << lat << "]";
      }

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
      (void)lookahead_alt;

      json << "]}},";
      json << "{\"type\":\"Feature\",\"properties\":{\"name\":\"lookahead_point\"},";
      json << "\"geometry\":{\"type\":\"Point\",\"coordinates\":[";
      json << lookahead_lon << "," << lookahead_lat;
      json << "]}}]}";

      foxglove_msgs::msg::GeoJSON msg{};
      msg.geojson = json.str();
      sine_wave_geojson_pub_->publish(msg);
#else
      (void)gv_frame;
      (void)plane_pos_local;
      (void)lookahead_local;
      (void)amplitude_m;
      (void)wavenumber_rad_m;
      (void)phase_offset_rad;
#endif
    }

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
        const double base_amplitude_m = generatePathAmplitude(ground_vehicle_state_.speed, cruise_speed_mps_, period_s_);
        const double amplitude_scale = amplitudeScaleFromAlongTrackError(plane_pos_local.x);
        // const double amplitude_m = base_amplitude_m * amplitude_scale;
        const double amplitude_m = base_amplitude_m;
      
        RCLCPP_INFO(
          get_logger(),
          "Generated path amplitude: base=%.2f m scale=%.2f result=%.2f m (x_err=%.2f m)",
          base_amplitude_m,
          amplitude_scale,
          amplitude_m,
          plane_pos_local.x);
    
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

        std_msgs::msg::Float64 base_amp_msg{};
        base_amp_msg.data = base_amplitude_m;
        base_amplitude_debug_pub_->publish(base_amp_msg);

        std_msgs::msg::Float64 scaled_amp_msg{};
        scaled_amp_msg.data = amplitude_m;
        scaled_amplitude_debug_pub_->publish(scaled_amp_msg);

        std_msgs::msg::Float64 x_err_msg{};
        x_err_msg.data = plane_pos_local.x;
        gv_local_x_error_debug_pub_->publish(x_err_msg);

        std_msgs::msg::Float64 y_err_msg{};
        y_err_msg.data = plane_pos_local.y;
        gv_local_y_error_debug_pub_->publish(y_err_msg);

        const double phase_deg = radToDeg(phase_offset_rad);
        std_msgs::msg::Float64 phase_deg_msg{};
        phase_deg_msg.data = phase_deg;
        phase_deg_debug_pub_->publish(phase_deg_msg);

        const Vec2 lookahead_local = advanceAlongSineByArcLength(
          plane_pos_local.x,
          amplitude_m,
          spatial_angular_freq_rad_m,
          phase_offset_rad,
          lookahead_dist_m_,
          lookahead_integration_dx_m_);
        publishSineWaveGeoJson(
          gv_frame,
          plane_pos_local,
          lookahead_local,
          amplitude_m,
          spatial_angular_freq_rad_m,
          phase_offset_rad);
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
          phase_deg,
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

        sensor_msgs::msg::NavSatFix plane_fix{};
        plane_fix.header.stamp = now();
        plane_fix.header.frame_id = "plane_global_position";
        plane_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        plane_fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        plane_fix.latitude = plane_state_.latitude;
        plane_fix.longitude = plane_state_.longitude;
        plane_fix.altitude = plane_state_.altitude;
        plane_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        plane_navsat_debug_pub_->publish(plane_fix);
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
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr plane_navsat_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr base_amplitude_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr scaled_amplitude_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gv_local_x_error_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gv_local_y_error_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr phase_deg_debug_pub_;
#ifdef VEHICLE_TRACKING_HAS_FOXGLOVE_MSGS
    rclcpp::Publisher<foxglove_msgs::msg::GeoJSON>::SharedPtr sine_wave_geojson_pub_;
#endif

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    double rate_hz_{5.0};


    double period_s_{10.0};
    double cruise_speed_mps_{20.0};
    double lookahead_dist_m_{30.0};
    double lookahead_integration_dx_m_{0.5};
    double sine_geojson_half_span_m_{200.0};
    double sine_geojson_step_m_{5.0};
    double amplitude_scale_deadband_m_{5.0};
    double amplitude_scale_double_at_m_{15.0};
    double amplitude_scale_cap_at_m_{40.0};
    double amplitude_scale_cap_factor_{4.0};
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
