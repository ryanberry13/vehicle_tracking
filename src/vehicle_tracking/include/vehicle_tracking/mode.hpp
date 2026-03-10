/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cmath>
#include <functional>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

static const std::string kName = "Vehicle Tracking";

class VehicleTracking : public px4_ros2::ModeBase {
 public:
  explicit VehicleTracking(rclcpp::Node& node)
  : ModeBase(node, Settings(kName).preventArming(true)), node_(node)
  {
    _fw_ll_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
    desired_alt_m_ = node_.declare_parameter<double>("desired_alt_m", desired_alt_m_);

    lookahead_sub_ = node_.create_subscription<sensor_msgs::msg::NavSatFix>(
      "/path/lookahead_navsat", 10, std::bind(&VehicleTracking::lookaheadCallback, this, std::placeholders::_1));
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    plane_global_pos_sub_ = node_.create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", px4_qos,
      std::bind(&VehicleTracking::planeGlobalPositionCallback, this, std::placeholders::_1));
  }

  void onActivate() override {}

  void onDeactivate() override {}

  // Update setpoint at 30Hz
  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;

    updateDesiredCourseFromPosition();
    _fw_ll_setpoint->updateWithAltitude(static_cast<float>(desired_alt_m_), desired_course_rad_);
  }

 private:
  void lookaheadCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    desired_position_.lat_deg = msg->latitude;
    desired_position_.lon_deg = msg->longitude;
    desired_position_.alt_m = msg->altitude;
    desired_position_.valid = true;
  }

  void planeGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
  {
    plane_position_.lat_deg = msg->lat;
    plane_position_.lon_deg = msg->lon;
    plane_position_.valid = true;
  }

  void updateDesiredCourseFromPosition()
  {
    if (!desired_position_.valid || !plane_position_.valid) {
      return;
    }

    const double bearing_ned = bearingNorthClockwiseRad(
      plane_position_.lat_deg,
      plane_position_.lon_deg,
      desired_position_.lat_deg,
      desired_position_.lon_deg);

    desired_course_rad_ = static_cast<float>(wrapMinusPiToPi(bearing_ned));
  }

  static double bearingNorthClockwiseRad(
    double lat1_deg,
    double lon1_deg,
    double lat2_deg,
    double lon2_deg)
  {
    const double lat1 = lat1_deg * M_PI / 180.0;
    const double lon1 = lon1_deg * M_PI / 180.0;
    const double lat2 = lat2_deg * M_PI / 180.0;
    const double lon2 = lon2_deg * M_PI / 180.0;
    const double dlon = lon2 - lon1;

    const double y = std::sin(dlon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) -
      std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    return std::atan2(y, x);
  }

  static double wrapMinusPiToPi(double angle_rad)
  {
    double wrapped = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
    if (wrapped < 0.0) {
      wrapped += 2.0 * M_PI;
    }
    return wrapped - M_PI;
  }

  struct GeoPoint {
    double lat_deg{0.0};
    double lon_deg{0.0};
    double alt_m{0.0};
    bool valid{false};
  };

  rclcpp::Node& node_;
  std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_ll_setpoint;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr lookahead_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr plane_global_pos_sub_;
  GeoPoint desired_position_;
  GeoPoint plane_position_;
  double desired_alt_m_{100.0};
  float desired_course_rad_{0.0F};
};
