#include "vehicle_tracking/path_math.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <geometry_msgs/msg/quaternion.hpp>

#include <boost/math/special_functions/ellint_2.hpp>
#include <boost/math/tools/roots.hpp>

double quatToYaw(const geometry_msgs::msg::Quaternion& q) {
    // Standard ZYX yaw extraction; robust when roll/pitch are zero or small
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double radToDeg(double radians) {
    return radians * (180.0 / M_PI);
}

double solveAmplitude(double speed_ratio){
    if (speed_ratio < 1.0) {
        // throw std::invalid_argument("No real solution: need sigma >= 1 (vP >= vB).");
        return 0.0;
    }

    auto f = [speed_ratio](double Ap) {
        double Ap2 = Ap * Ap;
        double m = Ap2 / (1.0 + Ap2);  // parameter m (not modulus)
        double k = std::sqrt(m);       // Boost takes modulus k where m = k^2
        double E = boost::math::ellint_2(k);
        double lhs = (2.0 / M_PI) * std::sqrt(1.0 + Ap2) * E;
        return lhs - speed_ratio;
    };

    double a = 0.0;                                 // f(0) = 1 - sigma <= 0
    double b = std::max(5.0, 2.0 * M_PI * speed_ratio);

    // Expand upper bound until sign change occurs
    while (f(b) < 0.0) {
        b *= 2.0;
        if (!std::isfinite(b) || b > 1e6) {
            throw std::runtime_error("Failed to bracket amplitude root.");
        }
    }

    // Robust root solve (toms748) once bracketed
    std::uintmax_t max_iter = 64;
    auto tol = boost::math::tools::eps_tolerance<double>(48);
    auto result = boost::math::tools::toms748_solve(f, a, b, tol, max_iter);

    return 0.5 * (result.first + result.second);
}
