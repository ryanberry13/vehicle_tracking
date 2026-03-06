#pragma once

#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>


double generatePathAmplitude(double gv_speed, double plane_speed, double period_s);
double solveAmplitude(double speed_ratio);
double quatToYaw(const geometry_msgs::msg::Quaternion& q);
double radToDeg(double radians);

struct Vec2 {
  double x{0.0};
  double y{0.0};

  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double s) const { return {x * s, y * s}; }
};

struct Mat2 {
  // Row-major:
  // [ a  b ]
  // [ c  d ]
  double a{1.0}, b{0.0};
  double c{0.0}, d{1.0};

  Vec2 operator*(const Vec2& v) const {
    return {a * v.x + b * v.y,
            c * v.x + d * v.y};
  }

  Mat2 transpose() const {
    return {a, c,
            b, d};
  }
};

struct GvFrame {
  Mat2 R_LG;     // local → global
  Mat2 R_GL;     // global → local (transpose)
  Vec2 p_anchor; // origin of GV local frame in global
};

inline GvFrame buildGvFrame(
    double x_gv,
    double y_gv,
    double psi_gv,
    double lead_dist)
{
  GvFrame f;

  const double c = std::cos(psi_gv);
  const double s = std::sin(psi_gv);

  // Local → Global rotation
  f.R_LG = {
    c, -s,
    s,  c
  };

  // Global → Local rotation
  f.R_GL = f.R_LG.transpose();

  // Anchor point (GV position + optional forward lead)
  Vec2 p_gv(x_gv, y_gv);
  f.p_anchor = p_gv + f.R_LG * Vec2(lead_dist, 0.0);

  return f;
}

inline Vec2 globalToGvLocal(
    const Vec2& p_global,
    const GvFrame& gv)
{
  Vec2 r = p_global - gv.p_anchor;
  return gv.R_GL * r;
}

inline Vec2 gvLocalToGlobal(
    const Vec2& p_local,
    const GvFrame& gv)
{
  return gv.p_anchor + gv.R_LG * p_local;
}
