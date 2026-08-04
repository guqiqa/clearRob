// chassis_core.hpp — Field-tuned chassis control core extracted from the
// colleague's WheelPID V2 C++ program (main.cpp), adapted for embedding into
// ROS2 via pybind11.
//
// Faithful extraction: the Controller logic (steering model, in-place
// rotation, arc-turn blend, heading-hold PID, direction-change hold, gear
// selection, per-wheel speed loop) is preserved as-is.  Two adaptations:
//   1. CanBus   → stub that RECORDS the per-motor CAN currents/speeds instead
//                 of sending them.  The Python shell reads the recorded
//                 commands after tick() and sends them over python-can.
//   2. ImuYaw   → external-value holder.  The Python shell integrates yaw
//                 from /imu/data and sets it before each tick.
//   3. apply()  → throttle/steering changed from int to double so continuous
//                 cmd_vel can be mapped to proportional speed.
//
// No threads, no locks — strictly synchronous single-threaded, matching the
// original program's simplicity.

#ifndef CHASSIS_CORE_HPP
#define CHASSIS_CORE_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace chassis_core {

// =========================================================================
// Config — exact mirror of the colleague's Config struct (all 90+ params).
// Defaults match the colleague's main.cpp; ROS2 YAML overrides them via
// configFromDict / setParam.
// =========================================================================
struct Config {
    std::string sbus_port{"/dev/ttyS1"};
    std::string can_port{"can0"};
    int run_rpm{100};            // physical wheel RPM (base, pre-gear)
    int rotate_rpm{50};          // physical wheel RPM for in-place rotate
    int turn_max_rpm{0};         // max outer wheel RPM while arc-turning; 0 disables
    int arc_turn_blend_ms{700};  // ramp straight-to-arc differential over this time
    bool rotate_in_place_enable{true};
    int rotate_direction_sign{1};
    bool rotate_factory_current_control_enable{true};
    int rotate_neutral_hold_ms{150};
    bool rotate_speed_guard_enable{true};
    double rotate_start_max_rpm{25.0};
    int rotate_start_guard_timeout_ms{1200};
    int speed_accel_rpm_s{100};  // software target ramp
    int speed_decel_rpm_s{200};
    double wheel_speed_kp{0.08};
    double wheel_speed_ki{0.03};
    std::string wheel_control_mode{"current"};
    int wheel_feedforward_current{18};
    int wheel_max_current{40};        // running limit: 0.40A, 10mA/unit
    int rotate_max_current{500};      // factory in-place rotate torque: 5.00A
    int wheel_integral_max_current{15};
    int wheel_start_initial_current{20};
    int wheel_start_step_current{2};
    int wheel_start_step_ms{100};
    int wheel_start_max_current{70};
    int wheel_start_threshold_rpm{5};
    int wheel_start_confirm_samples{2};
    int wheel_stall_threshold_rpm{2};
    int wheel_stall_confirm_samples{3};
    int wheel_start_timeout_ms{6000};
    int wheel_overspeed_rpm{160};
    int speed_feedback_timeout_ms{300};
    int cut_current{1000};       // mower current, 10mA/unit
    double turn_inner_ratio{0.50};
    int low_threshold{800};
    int high_threshold{1199};
    int failsafe_ms{300};
    int heartbeat_ms{200};
    int control_ms{100};
    int speed_query_ms{200};
    int speed_log_ms{500};
    double wheel_radius_m{0.10};
    double track_width_m{0.45};
    bool odom_log_enable{true};
    std::string odom_log_path{"odometry.csv"};
    int pole_pairs{4};
    double gear_ratio{1.0};
    int throttle_channel{3};
    int steering_channel{4};
    int mower_channel{2};
    int gear_channel{6};
    bool gear_select_enable{true};
    int gear_low_run_rpm{70};
    int gear_low_turn_max_rpm{70};
    int gear_mid_run_rpm{120};
    int gear_mid_turn_max_rpm{100};
    int gear_high_run_rpm{140};
    int gear_high_turn_max_rpm{100};
    bool app_control_enable{true};
    int app_udp_port{8091};
    int app_timeout_ms{300};
    std::array<uint16_t, 4> motor_ids{2, 1, 4, 3};   // FL, RL, FR, RR
    std::array<int, 4> motor_dirs{1, 1, -1, -1};     // physical direction correction
    uint16_t mower_id{5};
    bool show_log{true};
    bool imu_heading_enable{true};
    std::string imu_device{"/sys/bus/iio/devices/iio:device0"};
    std::string imu_axis{"z"};
    double imu_yaw_sign{-1.0};
    int imu_calibrate_ms{3000};
    double imu_deadband_dps{0.30};
    double heading_kp{4.0};
    double heading_ki{0.0};
    double heading_kd{1.0};
    int heading_integral_max_correction{30};
    int heading_max_correction{60};
    int heading_correction_sign{1};
    int heading_forward_trim{0};
    int heading_reverse_trim{0};
    int heading_reset_steering_hold_ms{500};
    bool direction_change_hold_enable{true};
    double direction_change_stop_rpm{5.0};
    int direction_change_hold_timeout_ms{1500};
    int direction_change_post_hold_ms{1000};
    int direction_change_post_max_diff_rpm{4};
    bool lateral_hold_enable{false};
    double lateral_kp_deg_per_m{0.0};
    double lateral_max_heading_deg{0.0};
    // Post-start anti-stutter (formal config.xml ROS2 section)
    int wheel_post_start_hold_ms{500};
    int wheel_post_start_current{48};
    int wheel_min_run_current{42};
    // Unified direction sign
    int motion_forward_rpm_sign{-1};
    // ROS2 autonomous profile (formal config.xml §13)
    bool ros2_enable{true};
    int ros2_cmd_timeout_ms{300};
    bool ros2_publish_tf{true};
    std::string ros2_odom_frame{"odom"};
    std::string ros2_base_frame{"base_link"};
    std::string ros2_wheel_control_mode{"current"};
    double ros2_wheel_speed_kp{0.80};
    double ros2_wheel_speed_ki{0.010};
    int ros2_wheel_feedforward_current{30};
    int ros2_wheel_max_current{130};
    int ros2_wheel_integral_max_current{10};
    int ros2_wheel_start_initial_current{130};
    int ros2_wheel_start_max_current{170};
    // Velocity / accel limits (formal config.xml §8, 0=unlimited)
    double max_velocity_mps{0.0};
    double max_angular_radps{0.0};
    double max_accel_mps2{0.40};
    double max_decel_mps2{0.60};
    double max_angular_accel_radps2{1.00};
};

// A parameter value coming from ROS2 YAML.  All four types are supported so
// numeric, boolean, string (wheel_control_mode, gear names) and array
// (motor_dirs, motor_ids) params map cleanly.
using ParamValue = std::variant<bool, int, double, std::string, std::vector<int>>;

// Build a Config from a string-keyed dict (ROS2 YAML params → Config).
// Unknown keys are ignored; missing keys keep the struct defaults.
Config configFromDict(const std::unordered_map<std::string, ParamValue>& params);

// =========================================================================
// CanBus stub — records the commands the Controller would have sent.
// The Python shell reads the recorded currents/speeds after tick() and
// transmits them over python-can (the shell owns the CAN socket).
// =========================================================================
class CanBus {
public:
    void heartbeat(uint16_t /*id*/) {}

    void current(uint16_t id, int value_10ma) {
        value_10ma = std::clamp(value_10ma, -32768, 32767);
        last_current_[id] = value_10ma;
    }

    void speed(uint16_t id, int32_t erpm) { last_speed_[id] = erpm; }

    void setAccel(uint16_t /*id*/, int32_t /*erpm_per_s*/) {}
    void setDecel(uint16_t /*id*/, int32_t /*erpm_per_s*/) {}

    void setClosedLoopMaxCurrent(uint16_t id, int value_10ma) {
        value_10ma = std::clamp(value_10ma, 0, 32767);
        last_max_current_[id] = value_10ma;
    }

    void querySpeed(uint16_t /*id*/) {}

    bool readSpeed(uint16_t& /*id*/, int32_t& /*erpm*/) { return false; }

    // Recorded per-motor commands for the current tick (dir already applied).
    const std::unordered_map<uint16_t, int>& currents() const { return last_current_; }
    const std::unordered_map<uint16_t, int>& speeds() const { return last_speed_; }

private:
    std::unordered_map<uint16_t, int> last_current_;
    std::unordered_map<uint16_t, int> last_speed_;
    std::unordered_map<uint16_t, int> last_max_current_;
};

// =========================================================================
// ImuYaw stub — the Python shell integrates yaw from /imu/data and pushes it
// in before each tick.  The Controller reads yawDeg()/rateDps()/valid().
// =========================================================================
class ImuYaw {
public:
    void setYawDeg(double yaw) { yaw_deg_ = yaw; }
    void setRateDps(double rate) { rate_dps_ = rate; }
    void setValid(bool valid) { valid_ = valid; }

    bool valid() const { return valid_; }
    double yawDeg() const { return yaw_deg_; }
    double rateDps() const { return rate_dps_; }

private:
    double yaw_deg_{0.0};
    double rate_dps_{0.0};
    bool valid_{false};
};

// =========================================================================
// GearProfile
// =========================================================================
struct GearProfile {
    const char* name;
    int run_rpm;
    int turn_max_rpm;
};

inline GearProfile gearProfileFromName(const std::string& name, const Config& c) {
    if (name == "LOW" || name == "low") {
        return {"LOW", c.gear_low_run_rpm, c.gear_low_turn_max_rpm};
    }
    if (name == "HIGH" || name == "high") {
        return {"HIGH", c.gear_high_run_rpm, c.gear_high_turn_max_rpm};
    }
    return {"MID", c.gear_mid_run_rpm, c.gear_mid_turn_max_rpm};
}

// =========================================================================
// Controller — the field-tuned core.  Transcribed from the colleague's
// main.cpp lines 773-1510 with three adaptations (see file header).
// =========================================================================
class Controller {
public:
    Controller(Config config, CanBus& can, ImuYaw* imu);

    void heartbeatAll();

    void setDriveMaxCurrent(int value_10ma, const char* reason);

    void setWheelFeedback(const std::array<double, 4>& rpm,
                          const std::array<bool, 4>& valid,
                          const std::array<uint64_t, 4>& generation);

    void setOdometryY(double y_m);

    const std::array<double, 4>& wheelTargets() const { return ramped_target_rpm_; }
    const std::array<int, 4>& wheelCurrents() const { return wheel_current_10ma_; }
    bool safetyLatched() const { return wheel_safety_latched_; }
    std::string safetyReason() const { return wheel_safety_reason_; }

    void setGear(const GearProfile& gear);

    void resetHeadingTarget(const char* reason);

    void stopAll(bool reset_heading_target = false);

    // throttle/steering are -1..1 (discrete ±1 for RC, continuous for cmd_vel).
    void apply(double throttle, double steering, bool mower_on, bool failsafe);

private:
    void maybeResetHeadingForSteering();
    void logStraightness(double throttle, double steering, bool heading_holding,
                         double heading_error, int heading_correction);
    void applyWheelSpeedControl(const std::array<double, 4>& desired_rpm);
    void logState(double throttle, double steering, bool mower, bool failsafe,
                  int left_rpm, int right_rpm, bool heading_holding,
                  double yaw_deg, double heading_error, int correction);

    Config c_;
    CanBus& can_;
    ImuYaw* imu_{nullptr};
    int active_closed_loop_max_current_{-1};
    bool heading_hold_active_{false};
    bool heading_target_valid_{false};
    double heading_throttle_{0.0};
    double target_yaw_deg_{0.0};
    double heading_integral_forward_{0.0};
    double heading_integral_reverse_{0.0};
    std::array<double, 4> measured_rpm_{};
    std::array<bool, 4> feedback_valid_{};
    std::array<double, 4> ramped_target_rpm_{};
    std::array<double, 4> wheel_integral_{};
    std::array<int, 4> wheel_current_10ma_{};
    std::array<bool, 4> wheel_started_{};
    std::array<int, 4> wheel_start_elapsed_ms_{};
    std::array<int, 4> wheel_start_confirm_count_{};
    std::array<int, 4> wheel_stall_count_{};
    std::array<int, 4> wheel_post_start_hold_ms_{};
    std::array<uint64_t, 4> feedback_generation_{};
    std::array<uint64_t, 4> processed_feedback_generation_{};
    bool wheel_safety_latched_{false};
    bool wheel_safety_reported_{false};
    std::string wheel_safety_reason_;
    std::array<int, 5> last_state_{99, 99, 99, 99, 999};
    bool straight_check_active_{false};
    std::chrono::steady_clock::time_point straight_check_start_{};
    std::chrono::steady_clock::time_point straight_check_last_log_{};
    double straight_start_yaw_deg_{0.0};
    double straight_max_abs_error_deg_{0.0};
    double odom_y_m_{0.0};
    bool odom_y_valid_{false};
    double lateral_target_y_m_{0.0};
    bool lateral_target_valid_{false};
    bool steering_reset_candidate_active_{false};
    bool steering_reset_done_for_hold_{false};
    std::chrono::steady_clock::time_point steering_reset_start_{};
    bool direction_change_hold_active_{false};
    std::chrono::steady_clock::time_point direction_change_hold_start_{};
    std::chrono::steady_clock::time_point direction_change_post_hold_start_{};
    double direction_change_pending_throttle_{0.0};
    bool direction_change_post_hold_logged_{false};
    bool rotate_neutral_hold_active_{false};
    bool rotate_neutral_hold_logged_{false};
    bool rotate_speed_guard_logged_{false};
    bool rotate_speed_guard_timeout_logged_{false};
    std::chrono::steady_clock::time_point rotate_neutral_hold_start_{};
    bool arc_turn_blend_active_{false};
    bool arc_turn_blend_logged_{false};
    int arc_turn_blend_key_{0};
    std::chrono::steady_clock::time_point arc_turn_blend_start_{};
    std::string current_gear_name_{"CONFIG"};
    double last_motion_throttle_{0.0};
};

}  // namespace chassis_core

#endif  // CHASSIS_CORE_HPP
