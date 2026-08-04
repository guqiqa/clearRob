#include "odometry.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Odometry::Odometry(OdometryConfig config) : config_(config) {}

void Odometry::reset(double x_m, double y_m, double yaw_deg) {
    pose_.x_m = x_m;
    pose_.y_m = y_m;
    pose_.yaw_deg = wrapDegrees(yaw_deg);
    linear_speed_mps_ = 0.0;
    angular_speed_dps_ = 0.0;
    initialized_ = true;
}

void Odometry::update(const std::array<double, 4>& wheel_rpm,
                      double dt_s,
                      bool imu_valid,
                      double imu_yaw_deg) {
    if (dt_s <= 0.0) return;

    const double fl = wheel_rpm[0];
    const double rl = wheel_rpm[1];
    const double fr = wheel_rpm[2];
    const double rr = wheel_rpm[3];

    const double left_rpm = (fl + rl) * 0.5;
    const double right_rpm = (fr + rr) * 0.5;

    const double left_wheel_rps = left_rpm / 60.0;
    const double right_wheel_rps = right_rpm / 60.0;

    const double left_mps = left_wheel_rps * 2.0 * kPi * config_.wheel_radius_m;
    const double right_mps = right_wheel_rps * 2.0 * kPi * config_.wheel_radius_m;

    linear_speed_mps_ = (left_mps + right_mps) * 0.5;
    angular_speed_dps_ = ((right_mps - left_mps) / config_.track_width_m) * (180.0 / kPi);

    if (!initialized_) {
        pose_.yaw_deg = imu_valid && config_.use_imu_yaw ? wrapDegrees(imu_yaw_deg) : 0.0;
        initialized_ = true;
    }

    double yaw_for_motion = pose_.yaw_deg;
    if (imu_valid && config_.use_imu_yaw) {
        const double blended = shortestAngleDeg(imu_yaw_deg, pose_.yaw_deg);
        yaw_for_motion = wrapDegrees(pose_.yaw_deg + config_.imu_yaw_blend * blended);
        pose_.yaw_deg = yaw_for_motion;
    } else {
        pose_.yaw_deg = wrapDegrees(pose_.yaw_deg + angular_speed_dps_ * dt_s);
        yaw_for_motion = pose_.yaw_deg;
    }

    const double yaw_rad = yaw_for_motion * (kPi / 180.0);
    pose_.x_m += linear_speed_mps_ * std::cos(yaw_rad) * dt_s;
    pose_.y_m += linear_speed_mps_ * std::sin(yaw_rad) * dt_s;
}

Pose2D Odometry::pose() const {
    return pose_;
}

double Odometry::wrapDegrees(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

double Odometry::shortestAngleDeg(double target, double current) {
    return wrapDegrees(target - current);
}
