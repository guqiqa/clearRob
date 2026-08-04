#pragma once

#include <array>

struct Pose2D {
    double x_m{0.0};
    double y_m{0.0};
    double yaw_deg{0.0};
};

struct OdometryConfig {
    double wheel_radius_m{0.10};
    double track_width_m{0.45};
    double gear_ratio{5.2};
    bool use_imu_yaw{true};
    double imu_yaw_blend{0.85};
};

class Odometry {
public:
    explicit Odometry(OdometryConfig config);

    void reset(double x_m = 0.0, double y_m = 0.0, double yaw_deg = 0.0);
    void update(const std::array<double, 4>& wheel_rpm,
                double dt_s,
                bool imu_valid,
                double imu_yaw_deg);

    Pose2D pose() const;
    double linearSpeedMps() const { return linear_speed_mps_; }
    double angularSpeedDps() const { return angular_speed_dps_; }

private:
    static double wrapDegrees(double deg);
    static double shortestAngleDeg(double target, double current);

    OdometryConfig config_;
    Pose2D pose_{};
    double linear_speed_mps_{0.0};
    double angular_speed_dps_{0.0};
    bool initialized_{false};
};
