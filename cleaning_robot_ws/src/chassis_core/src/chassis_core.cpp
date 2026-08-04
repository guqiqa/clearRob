// chassis_core.cpp — Implementation of the field-tuned chassis Controller.
// Transcribed from the colleague's main.cpp (lines 773-1510) with:
//   - throttle/steering widened int → double (continuous cmd_vel support)
//   - CAN sends become recordings (CanBus stub)
//   - IMU yaw/rate pushed in externally (ImuYaw stub)
// All control logic is preserved exactly.

#include "chassis_core/chassis_core.hpp"

#include <cmath>
#include <iostream>

namespace chassis_core {

// =========================================================================
// configFromDict — map a string-keyed param dict onto the Config struct.
// =========================================================================

namespace {
template <typename T>
T getNum(const std::unordered_map<std::string, ParamValue>& p, const std::string& key, T fallback) {
    auto it = p.find(key);
    if (it == p.end()) return fallback;
    const ParamValue& v = it->second;
    if (auto* d = std::get_if<double>(&v)) return static_cast<T>(*d);
    if (auto* i = std::get_if<int>(&v)) return static_cast<T>(*i);
    if (auto* b = std::get_if<bool>(&v)) return static_cast<T>(*b);
    return fallback;
}
bool getBool(const std::unordered_map<std::string, ParamValue>& p, const std::string& key, bool fallback) {
    auto it = p.find(key);
    if (it == p.end()) return fallback;
    if (auto* b = std::get_if<bool>(&it->second)) return *b;
    if (auto* i = std::get_if<int>(&it->second)) return *i != 0;
    if (auto* d = std::get_if<double>(&it->second)) return *d != 0.0;
    return fallback;
}
std::string getStr(const std::unordered_map<std::string, ParamValue>& p, const std::string& key, const std::string& fallback) {
    auto it = p.find(key);
    if (it == p.end()) return fallback;
    if (auto* s = std::get_if<std::string>(&it->second)) return *s;
    return fallback;
}
std::vector<int> getArr(const std::unordered_map<std::string, ParamValue>& p, const std::string& key) {
    auto it = p.find(key);
    if (it == p.end()) return {};
    if (auto* v = std::get_if<std::vector<int>>(&it->second)) return *v;
    return {};
}
}  // namespace

Config configFromDict(const std::unordered_map<std::string, ParamValue>& params) {
    Config c;
    c.sbus_port                 = getStr(params, "sbus_port", c.sbus_port);
    c.can_port                  = getStr(params, "can_port", c.can_port);
    c.run_rpm                   = getNum(params, "run_rpm", c.run_rpm);
    c.rotate_rpm                = getNum(params, "rotate_rpm", c.rotate_rpm);
    c.turn_max_rpm              = getNum(params, "turn_max_rpm", c.turn_max_rpm);
    c.arc_turn_blend_ms         = getNum(params, "arc_turn_blend_ms", c.arc_turn_blend_ms);
    c.rotate_in_place_enable    = getBool(params, "rotate_in_place_enable", c.rotate_in_place_enable);
    c.rotate_direction_sign     = getNum(params, "rotate_direction_sign", c.rotate_direction_sign);
    c.rotate_factory_current_control_enable =
        getBool(params, "rotate_factory_current_control_enable", c.rotate_factory_current_control_enable);
    c.rotate_neutral_hold_ms    = getNum(params, "rotate_neutral_hold_ms", c.rotate_neutral_hold_ms);
    c.rotate_speed_guard_enable = getBool(params, "rotate_speed_guard_enable", c.rotate_speed_guard_enable);
    c.rotate_start_max_rpm      = getNum(params, "rotate_start_max_rpm", c.rotate_start_max_rpm);
    c.rotate_start_guard_timeout_ms =
        getNum(params, "rotate_start_guard_timeout_ms", c.rotate_start_guard_timeout_ms);
    c.speed_accel_rpm_s         = getNum(params, "speed_accel_rpm_s", c.speed_accel_rpm_s);
    c.speed_decel_rpm_s         = getNum(params, "speed_decel_rpm_s", c.speed_decel_rpm_s);
    c.wheel_speed_kp            = getNum(params, "wheel_speed_kp", c.wheel_speed_kp);
    c.wheel_speed_ki            = getNum(params, "wheel_speed_ki", c.wheel_speed_ki);
    c.wheel_control_mode        = getStr(params, "wheel_control_mode", c.wheel_control_mode);
    c.wheel_feedforward_current = getNum(params, "wheel_feedforward_current", c.wheel_feedforward_current);
    c.wheel_max_current         = getNum(params, "wheel_max_current", c.wheel_max_current);
    c.rotate_max_current        = getNum(params, "rotate_max_current", c.rotate_max_current);
    c.wheel_integral_max_current = getNum(params, "wheel_integral_max_current", c.wheel_integral_max_current);
    c.wheel_start_initial_current = getNum(params, "wheel_start_initial_current", c.wheel_start_initial_current);
    c.wheel_start_step_current  = getNum(params, "wheel_start_step_current", c.wheel_start_step_current);
    c.wheel_start_step_ms       = getNum(params, "wheel_start_step_ms", c.wheel_start_step_ms);
    c.wheel_start_max_current   = getNum(params, "wheel_start_max_current", c.wheel_start_max_current);
    c.wheel_start_threshold_rpm = getNum(params, "wheel_start_threshold_rpm", c.wheel_start_threshold_rpm);
    c.wheel_start_confirm_samples = getNum(params, "wheel_start_confirm_samples", c.wheel_start_confirm_samples);
    c.wheel_stall_threshold_rpm = getNum(params, "wheel_stall_threshold_rpm", c.wheel_stall_threshold_rpm);
    c.wheel_stall_confirm_samples = getNum(params, "wheel_stall_confirm_samples", c.wheel_stall_confirm_samples);
    c.wheel_start_timeout_ms    = getNum(params, "wheel_start_timeout_ms", c.wheel_start_timeout_ms);
    c.wheel_overspeed_rpm       = getNum(params, "wheel_overspeed_rpm", c.wheel_overspeed_rpm);
    c.speed_feedback_timeout_ms = getNum(params, "speed_feedback_timeout_ms", c.speed_feedback_timeout_ms);
    c.cut_current               = getNum(params, "cut_current", c.cut_current);
    c.turn_inner_ratio          = getNum(params, "turn_inner_ratio", c.turn_inner_ratio);
    c.low_threshold             = getNum(params, "low_threshold", c.low_threshold);
    c.high_threshold            = getNum(params, "high_threshold", c.high_threshold);
    c.failsafe_ms               = getNum(params, "failsafe_ms", c.failsafe_ms);
    c.heartbeat_ms              = getNum(params, "heartbeat_ms", c.heartbeat_ms);
    c.control_ms                = getNum(params, "control_ms", c.control_ms);
    c.speed_query_ms            = getNum(params, "speed_query_ms", c.speed_query_ms);
    c.speed_log_ms              = getNum(params, "speed_log_ms", c.speed_log_ms);
    c.wheel_radius_m            = getNum(params, "wheel_radius", c.wheel_radius_m);
    c.track_width_m             = getNum(params, "track_width", c.track_width_m);
    c.odom_log_enable           = getBool(params, "odom_log_enable", c.odom_log_enable);
    c.odom_log_path             = getStr(params, "odom_log_path", c.odom_log_path);
    c.pole_pairs                = getNum(params, "pole_pairs", c.pole_pairs);
    c.gear_ratio                = getNum(params, "gear_ratio", c.gear_ratio);
    c.throttle_channel          = getNum(params, "throttle_channel", c.throttle_channel);
    c.steering_channel          = getNum(params, "steering_channel", c.steering_channel);
    c.mower_channel             = getNum(params, "mower_channel", c.mower_channel);
    c.gear_channel              = getNum(params, "gear_channel", c.gear_channel);
    c.gear_select_enable        = getBool(params, "gear_select_enable", c.gear_select_enable);
    c.gear_low_run_rpm          = getNum(params, "gear_low_run_rpm", c.gear_low_run_rpm);
    c.gear_low_turn_max_rpm     = getNum(params, "gear_low_turn_max_rpm", c.gear_low_turn_max_rpm);
    c.gear_mid_run_rpm          = getNum(params, "gear_mid_run_rpm", c.gear_mid_run_rpm);
    c.gear_mid_turn_max_rpm     = getNum(params, "gear_mid_turn_max_rpm", c.gear_mid_turn_max_rpm);
    c.gear_high_run_rpm         = getNum(params, "gear_high_run_rpm", c.gear_high_run_rpm);
    c.gear_high_turn_max_rpm    = getNum(params, "gear_high_turn_max_rpm", c.gear_high_turn_max_rpm);
    c.app_control_enable        = getBool(params, "app_control_enable", c.app_control_enable);
    c.app_udp_port              = getNum(params, "app_udp_port", c.app_udp_port);
    c.app_timeout_ms            = getNum(params, "app_timeout_ms", c.app_timeout_ms);
    c.mower_id                  = getNum(params, "mower_id", c.mower_id);
    c.show_log                  = getBool(params, "show_log", c.show_log);
    c.imu_heading_enable        = getBool(params, "imu_heading_enable", c.imu_heading_enable);
    c.imu_device                = getStr(params, "imu_device", c.imu_device);
    c.imu_axis                  = getStr(params, "imu_axis", c.imu_axis);
    c.imu_yaw_sign              = getNum(params, "imu_yaw_sign", c.imu_yaw_sign);
    c.imu_calibrate_ms          = getNum(params, "imu_calibrate_ms", c.imu_calibrate_ms);
    c.imu_deadband_dps          = getNum(params, "imu_deadband_dps", c.imu_deadband_dps);
    c.heading_kp                = getNum(params, "heading_kp", c.heading_kp);
    c.heading_ki                = getNum(params, "heading_ki", c.heading_ki);
    c.heading_kd                = getNum(params, "heading_kd", c.heading_kd);
    c.heading_integral_max_correction =
        getNum(params, "heading_integral_max_correction", c.heading_integral_max_correction);
    c.heading_max_correction    = getNum(params, "heading_max_correction", c.heading_max_correction);
    c.heading_correction_sign   = getNum(params, "heading_correction_sign", c.heading_correction_sign);
    c.heading_forward_trim      = getNum(params, "heading_forward_trim", c.heading_forward_trim);
    c.heading_reverse_trim      = getNum(params, "heading_reverse_trim", c.heading_reverse_trim);
    c.heading_reset_steering_hold_ms =
        getNum(params, "heading_reset_steering_hold_ms", c.heading_reset_steering_hold_ms);
    c.direction_change_hold_enable =
        getBool(params, "direction_change_hold_enable", c.direction_change_hold_enable);
    c.direction_change_stop_rpm = getNum(params, "direction_change_stop_rpm", c.direction_change_stop_rpm);
    c.direction_change_hold_timeout_ms =
        getNum(params, "direction_change_hold_timeout_ms", c.direction_change_hold_timeout_ms);
    c.direction_change_post_hold_ms =
        getNum(params, "direction_change_post_hold_ms", c.direction_change_post_hold_ms);
    c.direction_change_post_max_diff_rpm =
        getNum(params, "direction_change_post_max_diff_rpm", c.direction_change_post_max_diff_rpm);
    c.lateral_hold_enable       = getBool(params, "lateral_hold_enable", c.lateral_hold_enable);
    c.lateral_kp_deg_per_m      = getNum(params, "lateral_kp_deg_per_m", c.lateral_kp_deg_per_m);
    c.lateral_max_heading_deg   = getNum(params, "lateral_max_heading_deg", c.lateral_max_heading_deg);

    // Post-start anti-stutter + direction sign (formal config.xml additions)
    c.wheel_post_start_hold_ms  = getNum(params, "wheel_post_start_hold_ms", c.wheel_post_start_hold_ms);
    c.wheel_post_start_current  = getNum(params, "wheel_post_start_current", c.wheel_post_start_current);
    c.wheel_min_run_current     = getNum(params, "wheel_min_run_current", c.wheel_min_run_current);
    c.motion_forward_rpm_sign   = getNum(params, "motion_forward_rpm_sign", c.motion_forward_rpm_sign);
    c.ros2_enable               = getBool(params, "ros2_enable", c.ros2_enable);
    c.ros2_cmd_timeout_ms       = getNum(params, "ros2_cmd_timeout_ms", c.ros2_cmd_timeout_ms);
    c.ros2_publish_tf           = getBool(params, "ros2_publish_tf", c.ros2_publish_tf);
    c.ros2_odom_frame           = getStr(params, "ros2_odom_frame", c.ros2_odom_frame);
    c.ros2_base_frame           = getStr(params, "ros2_base_frame", c.ros2_base_frame);
    c.ros2_wheel_control_mode   = getStr(params, "ros2_wheel_control_mode", c.ros2_wheel_control_mode);
    c.ros2_wheel_speed_kp       = getNum(params, "ros2_wheel_speed_kp", c.ros2_wheel_speed_kp);
    c.ros2_wheel_speed_ki       = getNum(params, "ros2_wheel_speed_ki", c.ros2_wheel_speed_ki);
    c.ros2_wheel_feedforward_current = getNum(params, "ros2_wheel_feedforward_current", c.ros2_wheel_feedforward_current);
    c.ros2_wheel_max_current    = getNum(params, "ros2_wheel_max_current", c.ros2_wheel_max_current);
    c.ros2_wheel_integral_max_current = getNum(params, "ros2_wheel_integral_max_current", c.ros2_wheel_integral_max_current);
    c.ros2_wheel_start_initial_current = getNum(params, "ros2_wheel_start_initial_current", c.ros2_wheel_start_initial_current);
    c.ros2_wheel_start_max_current = getNum(params, "ros2_wheel_start_max_current", c.ros2_wheel_start_max_current);
    c.max_velocity_mps          = getNum(params, "max_velocity_mps", c.max_velocity_mps);
    c.max_angular_radps         = getNum(params, "max_angular_radps", c.max_angular_radps);
    c.max_accel_mps2            = getNum(params, "max_accel_mps2", c.max_accel_mps2);
    c.max_decel_mps2            = getNum(params, "max_decel_mps2", c.max_decel_mps2);
    c.max_angular_accel_radps2  = getNum(params, "max_angular_accel_radps2", c.max_angular_accel_radps2);

    auto ids = getArr(params, "motor_ids");
    if (ids.size() == 4) {
        for (int i = 0; i < 4; ++i) c.motor_ids[i] = static_cast<uint16_t>(ids[i]);
    }
    auto dirs = getArr(params, "motor_dirs");
    if (dirs.size() == 4) {
        for (int i = 0; i < 4; ++i) c.motor_dirs[i] = dirs[i];
    }
    return c;
}

// =========================================================================
// Controller — faithful transcription (double throttle/steering).
// =========================================================================

Controller::Controller(Config config, CanBus& can, ImuYaw* imu)
    : c_(std::move(config)), can_(can), imu_(imu) {}

void Controller::heartbeatAll() {
    for (auto id : c_.motor_ids) can_.heartbeat(id);
    can_.heartbeat(c_.mower_id);
}

void Controller::setDriveMaxCurrent(int value_10ma, const char* reason) {
    if (c_.wheel_control_mode != "speed") return;
    value_10ma = std::clamp(value_10ma, 1, 1000);
    if (active_closed_loop_max_current_ == value_10ma) return;
    active_closed_loop_max_current_ = value_10ma;
    for (auto id : c_.motor_ids) {
        can_.setClosedLoopMaxCurrent(id, value_10ma);
    }
    if (c_.show_log) {
        std::cout << "DRIVER_MAX_CURRENT_SET reason=" << reason
                  << " value_10mA=" << value_10ma
                  << " amp=" << value_10ma * 0.01
                  << std::endl;
    }
}

void Controller::setWheelFeedback(const std::array<double, 4>& rpm,
                                  const std::array<bool, 4>& valid,
                                  const std::array<uint64_t, 4>& generation) {
    measured_rpm_ = rpm;
    feedback_valid_ = valid;
    feedback_generation_ = generation;
}

void Controller::setOdometryY(double y_m) {
    odom_y_m_ = y_m;
    odom_y_valid_ = true;
}

void Controller::setGear(const GearProfile& gear) {
    const bool changed = gear.name != current_gear_name_ ||
                         gear.run_rpm != c_.run_rpm ||
                         gear.turn_max_rpm != c_.turn_max_rpm;
    c_.run_rpm = gear.run_rpm;
    c_.turn_max_rpm = gear.turn_max_rpm;
    current_gear_name_ = gear.name;
    if (changed && c_.show_log) {
        std::cout << "GEAR_SELECT name=" << current_gear_name_
                  << " run_rpm=" << c_.run_rpm
                  << " turn_max_rpm=" << c_.turn_max_rpm
                  << " inner_ratio=" << c_.turn_inner_ratio
                  << std::endl;
    }
}

void Controller::resetHeadingTarget(const char* reason) {
    if (heading_target_valid_ && c_.show_log) {
        std::cout << "HEADING_TARGET_RESET reason=" << reason
                  << " yaw=" << (imu_ ? imu_->yawDeg() : 0.0)
                  << " old_target=" << target_yaw_deg_
                  << " odom_y=" << (odom_y_valid_ ? odom_y_m_ : 0.0)
                  << std::endl;
    }
    heading_target_valid_ = false;
    lateral_target_valid_ = false;
    heading_integral_forward_ = 0.0;
    heading_integral_reverse_ = 0.0;
}

void Controller::stopAll(bool reset_heading_target) {
    heading_hold_active_ = false;
    direction_change_hold_active_ = false;
    direction_change_pending_throttle_ = 0.0;
    direction_change_post_hold_start_ = {};
    direction_change_post_hold_logged_ = false;
    last_motion_throttle_ = 0.0;
    if (reset_heading_target) resetHeadingTarget("stop_all");
    heading_throttle_ = 0.0;
    heading_integral_forward_ = 0.0;
    heading_integral_reverse_ = 0.0;
    ramped_target_rpm_.fill(0.0);
    wheel_integral_.fill(0.0);
    wheel_current_10ma_.fill(0);
    for (auto id : c_.motor_ids) can_.current(id, 0);
    can_.current(c_.mower_id, 0);
}

void Controller::apply(double throttle, double steering, bool mower_on, bool failsafe) {
    if (failsafe) {
        stopAll(false);
        logState(0, 0, false, true, 0, 0, false, 0.0, 0.0, 0);
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool straight_command = throttle != 0 && steering == 0;
    const bool rotate_request = throttle == 0 && steering != 0;
    if (rotate_request) {
        if (!rotate_neutral_hold_active_) {
            rotate_neutral_hold_active_ = true;
            rotate_neutral_hold_start_ = now;
            rotate_neutral_hold_logged_ = false;
        }
    } else {
        rotate_neutral_hold_active_ = false;
        rotate_neutral_hold_logged_ = false;
        rotate_speed_guard_logged_ = false;
        rotate_speed_guard_timeout_logged_ = false;
    }
    const bool rotate_command = rotate_request &&
        now - rotate_neutral_hold_start_ >= std::chrono::milliseconds(c_.rotate_neutral_hold_ms);
    if (rotate_request && !rotate_command && c_.show_log && !rotate_neutral_hold_logged_) {
        std::cout << "ROTATE_NEUTRAL_HOLD wait_ms=" << c_.rotate_neutral_hold_ms << std::endl;
        rotate_neutral_hold_logged_ = true;
    }
    if (c_.direction_change_hold_enable && straight_command &&
        !direction_change_hold_active_ && last_motion_throttle_ != 0 &&
        throttle != last_motion_throttle_) {
        direction_change_hold_active_ = true;
        direction_change_hold_start_ = now;
        direction_change_pending_throttle_ = throttle;
        heading_hold_active_ = false;
        heading_integral_forward_ = 0.0;
        heading_integral_reverse_ = 0.0;
        if (c_.show_log) {
            std::cout << "DIRECTION_CHANGE_HOLD start old="
                      << last_motion_throttle_
                      << " new=" << throttle
                      << " stop_rpm<=" << c_.direction_change_stop_rpm
                      << " timeout_ms=" << c_.direction_change_hold_timeout_ms
                      << std::endl;
        }
    }

    if (direction_change_hold_active_) {
        const bool feedback_stopped =
            std::all_of(feedback_valid_.begin(), feedback_valid_.end(),
                        [](bool valid) { return valid; }) &&
            std::all_of(measured_rpm_.begin(), measured_rpm_.end(),
                        [this](double rpm) {
                            return std::abs(rpm) <= c_.direction_change_stop_rpm;
                        });
        const bool timed_out =
            now - direction_change_hold_start_ >=
            std::chrono::milliseconds(c_.direction_change_hold_timeout_ms);
        if (!feedback_stopped && !timed_out) {
            heading_hold_active_ = false;
            heading_throttle_ = 0.0;
            applyWheelSpeedControl({0.0, 0.0, 0.0, 0.0});
            can_.current(c_.mower_id, mower_on ? c_.cut_current : 0);
            if (c_.show_log) {
                std::cout << "DIRECTION_CHANGE_HOLD waiting"
                          << " rpm=" << measured_rpm_[0] << "/"
                          << measured_rpm_[1] << "/"
                          << measured_rpm_[2] << "/"
                          << measured_rpm_[3]
                          << std::endl;
            }
            return;
        }

        direction_change_hold_active_ = false;
        last_motion_throttle_ = direction_change_pending_throttle_;
        direction_change_post_hold_start_ = now;
        resetHeadingTarget(timed_out ? "direction_change_timeout"
                                     : "direction_change_stopped");
        if (c_.show_log) {
            std::cout << "DIRECTION_CHANGE_HOLD release reason="
                      << (timed_out ? "timeout" : "stopped")
                      << " rpm=" << measured_rpm_[0] << "/"
                      << measured_rpm_[1] << "/"
                      << measured_rpm_[2] << "/"
                      << measured_rpm_[3] << std::endl;
        }
    } else if (straight_command) {
        last_motion_throttle_ = throttle;
    }

    double left = 0.0;
    double right = 0.0;
    int rpm_limit = c_.run_rpm;

    if (rotate_command) {
        heading_hold_active_ = false;
        maybeResetHeadingForSteering();
        heading_throttle_ = 0.0;
        direction_change_hold_active_ = false;
        direction_change_pending_throttle_ = 0.0;
        direction_change_post_hold_start_ = {};
        direction_change_post_hold_logged_ = false;
        arc_turn_blend_active_ = false;
        arc_turn_blend_key_ = 0;
        arc_turn_blend_logged_ = false;
        last_motion_throttle_ = 0.0;
        if (c_.rotate_in_place_enable) {
            rpm_limit = c_.rotate_rpm;
            // steering > 0 is ROTATE LEFT, steering < 0 is ROTATE RIGHT.
            // In-place rotation: left and right wheels run opposite
            // directions with equal magnitude, so the chassis turns
            // around its center instead of driving forward/backward.
            const double rotate = static_cast<double>(
                steering * c_.rotate_direction_sign);
            left = -rotate;
            right = rotate;
        }
    } else if (throttle != 0) {
        double arc_blend = 1.0;
        if (steering != 0 && c_.arc_turn_blend_ms > 0) {
            const int arc_key = (throttle > 0 ? 1 : -1) * (steering > 0 ? 1 : -1);
            if (!arc_turn_blend_active_ || arc_key != arc_turn_blend_key_) {
                arc_turn_blend_active_ = true;
                arc_turn_blend_key_ = arc_key;
                arc_turn_blend_start_ = now;
                arc_turn_blend_logged_ = false;
            }
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(now - arc_turn_blend_start_).count();
            arc_blend = std::clamp(elapsed_ms / static_cast<double>(c_.arc_turn_blend_ms),
                                   0.0, 1.0);
            if (c_.show_log && !arc_turn_blend_logged_) {
                std::cout << "ARC_TURN_BLEND start_ms=" << c_.arc_turn_blend_ms
                          << " from=straight_to_arc" << std::endl;
                arc_turn_blend_logged_ = true;
            }
        } else {
            arc_turn_blend_active_ = false;
            arc_turn_blend_key_ = 0;
            arc_turn_blend_logged_ = false;
        }

        if (steering != 0 && c_.turn_max_rpm > 0) {
            const int turn_limit = std::min(c_.run_rpm, c_.turn_max_rpm);
            rpm_limit = static_cast<int>(std::lround(
                static_cast<double>(c_.run_rpm) +
                (static_cast<double>(turn_limit - c_.run_rpm) * arc_blend)));
        }
        const double gain = (1.0 - c_.turn_inner_ratio) / (1.0 + c_.turn_inner_ratio);
        const double effective_steering = static_cast<double>(steering) * arc_blend;
        left = static_cast<double>(throttle) - effective_steering * gain;
        right = static_cast<double>(throttle) + effective_steering * gain;
        const double peak = std::max({1.0, std::abs(left), std::abs(right)});
        left /= peak;
        right /= peak;
    } else {
        // Neutral throttle keeps the previous straight-line target.
        heading_hold_active_ = false;
        heading_throttle_ = 0.0;
        arc_turn_blend_active_ = false;
        arc_turn_blend_key_ = 0;
        arc_turn_blend_logged_ = false;
    }

    int left_rpm = static_cast<int>(std::lround(left * rpm_limit));
    int right_rpm = static_cast<int>(std::lround(right * rpm_limit));
    bool heading_holding = false;
    double heading_error = 0.0;
    int heading_correction = 0;

    if (throttle != 0 && steering == 0 && imu_ && imu_->valid()) {
        // Keep one straight-line heading target across forward/reverse
        // changes and short neutral pauses.  Otherwise a small yaw error
        // at the end of a forward run becomes the new reverse target, so
        // repeated back-and-forth motion walks away from the start line.
        // The target is reset only by an intentional steering command.
        steering_reset_candidate_active_ = false;
        if (!heading_target_valid_) {
            target_yaw_deg_ = imu_->yawDeg();
            lateral_target_y_m_ = odom_y_valid_ ? odom_y_m_ : 0.0;
            lateral_target_valid_ = odom_y_valid_;
            heading_target_valid_ = true;
            if (c_.show_log) {
                std::cout << "HEADING_TARGET_SET reason=first_straight"
                          << " yaw=" << target_yaw_deg_
                          << " odom_y=" << (odom_y_valid_ ? odom_y_m_ : 0.0)
                          << std::endl;
            }
        }
        heading_hold_active_ = true;
        heading_throttle_ = throttle;
        double effective_target_yaw_deg = target_yaw_deg_;
        if (c_.lateral_hold_enable && lateral_target_valid_ && odom_y_valid_) {
            const double lateral_error_m = odom_y_m_ - lateral_target_y_m_;
            // Positive odom y needs opposite yaw bias for forward vs reverse,
            // because physical forward is negative v in the odometry frame.
            const double direction = throttle < 0 ? 1.0 : -1.0;
            const double lateral_heading_deg = std::clamp(
                direction * c_.lateral_kp_deg_per_m * lateral_error_m,
                -c_.lateral_max_heading_deg, c_.lateral_max_heading_deg);
            effective_target_yaw_deg += lateral_heading_deg;
            while (effective_target_yaw_deg > 180.0) effective_target_yaw_deg -= 360.0;
            while (effective_target_yaw_deg < -180.0) effective_target_yaw_deg += 360.0;
        }
        heading_error = effective_target_yaw_deg - imu_->yawDeg();
        while (heading_error > 180.0) heading_error -= 360.0;
        while (heading_error < -180.0) heading_error += 360.0;
        double& heading_integral = throttle > 0 ?
                                   heading_integral_forward_ : heading_integral_reverse_;
        heading_integral += heading_error * (static_cast<double>(c_.control_ms) / 1000.0);
        if (c_.heading_ki > 0.0) {
            const double integral_state_limit =
                static_cast<double>(c_.heading_integral_max_correction) / c_.heading_ki;
            heading_integral = std::clamp(heading_integral,
                                          -integral_state_limit,
                                          integral_state_limit);
        } else {
            heading_integral = 0.0;
        }
        // Physical forward is negative logical throttle on this robot.
        const int direction_trim = throttle < 0 ?
                                   c_.heading_forward_trim : c_.heading_reverse_trim;
        const double correction = c_.heading_kp * heading_error +
                                  c_.heading_ki * heading_integral -
                                  c_.heading_kd * imu_->rateDps() +
                                  direction_trim;
        heading_correction = static_cast<int>(std::lround(std::clamp(
            correction, -static_cast<double>(c_.heading_max_correction),
            static_cast<double>(c_.heading_max_correction))));
        int applied_correction = heading_correction * c_.heading_correction_sign;
        const bool post_change_hold_active =
            c_.direction_change_post_hold_ms > 0 &&
            direction_change_post_hold_start_ !=
                std::chrono::steady_clock::time_point{} &&
            now - direction_change_post_hold_start_ <
                std::chrono::milliseconds(c_.direction_change_post_hold_ms);
        if (post_change_hold_active) {
            const int max_correction_from_diff =
                c_.direction_change_post_max_diff_rpm / 2;
            applied_correction = std::clamp(
                applied_correction, -max_correction_from_diff,
                max_correction_from_diff);
            if (c_.show_log && !direction_change_post_hold_logged_) {
                std::cout << "DIRECTION_CHANGE_POST_HOLD active_ms="
                          << c_.direction_change_post_hold_ms
                          << " max_diff_rpm="
                          << c_.direction_change_post_max_diff_rpm
                          << std::endl;
                direction_change_post_hold_logged_ = true;
            }
        } else {
            direction_change_post_hold_logged_ = false;
        }
        left_rpm = static_cast<int>(std::lround(throttle * c_.run_rpm)) - applied_correction;
        right_rpm = static_cast<int>(std::lround(throttle * c_.run_rpm)) + applied_correction;
        // Do not ask either side to exceed Run_RPM.  The driver/motor may
        // not be able to track the over-speed side (for example 35/65 when
        // Run_RPM is 50), which makes the correction ineffective and can
        // worsen reverse drift.  Scale both sides down while preserving the
        // requested left/right ratio so the differential correction stays
        // achievable.
        const int peak_rpm = std::max(std::abs(left_rpm), std::abs(right_rpm));
        if (peak_rpm > c_.run_rpm) {
            const double scale = static_cast<double>(c_.run_rpm) /
                                 static_cast<double>(peak_rpm);
            left_rpm = static_cast<int>(std::lround(left_rpm * scale));
            right_rpm = static_cast<int>(std::lround(right_rpm * scale));
        }
        heading_holding = true;
    } else if (steering != 0) {
        heading_hold_active_ = false;
        maybeResetHeadingForSteering();
        heading_throttle_ = 0.0;
    } else if (throttle == 0) {
        heading_hold_active_ = false;
        steering_reset_candidate_active_ = false;
        heading_throttle_ = 0.0;
    }

    if (rotate_command && c_.rotate_factory_current_control_enable) {
        if (c_.rotate_speed_guard_enable) {
            const double max_abs_rpm = std::max({
                std::abs(measured_rpm_[0]), std::abs(measured_rpm_[1]),
                std::abs(measured_rpm_[2]), std::abs(measured_rpm_[3])});
            const auto guard_deadline = rotate_neutral_hold_start_ +
                std::chrono::milliseconds(c_.rotate_neutral_hold_ms +
                                          c_.rotate_start_guard_timeout_ms);
            if (max_abs_rpm > c_.rotate_start_max_rpm && now < guard_deadline) {
                applyWheelSpeedControl({0.0, 0.0, 0.0, 0.0});
                can_.current(c_.mower_id, mower_on ? c_.cut_current : 0);
                if (c_.show_log && !rotate_speed_guard_logged_) {
                    std::cout << "ROTATE_SPEED_GUARD wait max_rpm=" << max_abs_rpm
                              << " allow_below=" << c_.rotate_start_max_rpm
                              << " timeout_ms=" << c_.rotate_start_guard_timeout_ms
                              << std::endl;
                    rotate_speed_guard_logged_ = true;
                }
                logState(throttle, steering, mower_on, false, 0, 0,
                         false, imu_ ? imu_->yawDeg() : 0.0, 0.0, 0);
                return;
            }
            if (max_abs_rpm > c_.rotate_start_max_rpm && c_.show_log &&
                !rotate_speed_guard_timeout_logged_) {
                std::cout << "ROTATE_SPEED_GUARD timeout allow max_rpm=" << max_abs_rpm
                          << " allow_below=" << c_.rotate_start_max_rpm
                          << std::endl;
                rotate_speed_guard_timeout_logged_ = true;
            }
        }
        setDriveMaxCurrent(c_.rotate_max_current, "rotate_factory_current");
        // Factory behavior for in-place rotation: direct OID 0x01 current
        // command.  The value is in 10mA units; 500 means 5.00A.
        // This bypasses speed-loop start torque limits so the robot can
        // break static friction while turning in place.
        ramped_target_rpm_.fill(0.0);
        wheel_integral_.fill(0.0);
        wheel_current_10ma_ = {
            left_rpm >= 0 ? c_.rotate_max_current : -c_.rotate_max_current,
            left_rpm >= 0 ? c_.rotate_max_current : -c_.rotate_max_current,
            right_rpm >= 0 ? c_.rotate_max_current : -c_.rotate_max_current,
            right_rpm >= 0 ? c_.rotate_max_current : -c_.rotate_max_current
        };
        can_.current(c_.motor_ids[0], wheel_current_10ma_[0] * c_.motor_dirs[0]);
        can_.current(c_.motor_ids[1], wheel_current_10ma_[1] * c_.motor_dirs[1]);
        can_.current(c_.motor_ids[2], wheel_current_10ma_[2] * c_.motor_dirs[2]);
        can_.current(c_.motor_ids[3], wheel_current_10ma_[3] * c_.motor_dirs[3]);
        can_.current(c_.mower_id, mower_on ? c_.cut_current : 0);
        if (c_.show_log) {
            std::cout << "ROTATE_FACTORY_CURRENT value_10mA="
                      << c_.rotate_max_current
                      << " amp=" << c_.rotate_max_current * 0.01
                      << std::endl;
        }
        logState(throttle, steering, mower_on, false, left_rpm, right_rpm,
                 false, imu_ ? imu_->yawDeg() : 0.0, 0.0, 0);
        return;
    }

    setDriveMaxCurrent(rotate_command ? c_.rotate_max_current : c_.wheel_max_current,
                       rotate_command ? "rotate_factory_torque" : "normal_run");
    applyWheelSpeedControl({static_cast<double>(left_rpm), static_cast<double>(left_rpm),
                            static_cast<double>(right_rpm), static_cast<double>(right_rpm)});
    can_.current(c_.mower_id, mower_on ? c_.cut_current : 0);
    logState(throttle, steering, mower_on, false, left_rpm, right_rpm,
             heading_holding, imu_ ? imu_->yawDeg() : 0.0,
             heading_error, heading_correction);
    logStraightness(throttle, steering, heading_holding, heading_error, heading_correction);
}

void Controller::maybeResetHeadingForSteering() {
    const auto now = std::chrono::steady_clock::now();
    if (c_.heading_reset_steering_hold_ms == 0) {
        resetHeadingTarget("steering");
        return;
    }
    if (!steering_reset_candidate_active_) {
        steering_reset_candidate_active_ = true;
        steering_reset_start_ = now;
        steering_reset_done_for_hold_ = false;
        return;
    }
    if (!steering_reset_done_for_hold_ &&
        now - steering_reset_start_ >= std::chrono::milliseconds(c_.heading_reset_steering_hold_ms)) {
        resetHeadingTarget("steering_hold");
        steering_reset_done_for_hold_ = true;
    }
}

void Controller::logStraightness(double throttle, double steering, bool heading_holding,
                                 double heading_error, int heading_correction) {
    const auto now = std::chrono::steady_clock::now();
    if (throttle == 0 || steering != 0 || !heading_holding || !imu_ || !imu_->valid()) {
        straight_check_active_ = false;
        straight_max_abs_error_deg_ = 0.0;
        return;
    }
    if (!straight_check_active_) {
        straight_check_active_ = true;
        straight_check_start_ = now;
        straight_check_last_log_ = now;
        straight_start_yaw_deg_ = imu_->yawDeg();
        straight_max_abs_error_deg_ = 0.0;
    }
    straight_max_abs_error_deg_ = std::max(straight_max_abs_error_deg_,
                                           std::abs(heading_error));
    if (now - straight_check_last_log_ < std::chrono::milliseconds(1000)) return;

    const double current_yaw = imu_->yawDeg();
    double yaw_delta = current_yaw - straight_start_yaw_deg_;
    while (yaw_delta > 180.0) yaw_delta -= 360.0;
    while (yaw_delta < -180.0) yaw_delta += 360.0;
    const double elapsed_s = std::chrono::duration<double>(now - straight_check_start_).count();
    const double abs_delta = std::abs(yaw_delta);
    const char* verdict = abs_delta <= 2.0 ? "OK" : (abs_delta <= 5.0 ? "WARN" : "BAD");
    const char* drift = yaw_delta > 0.5 ? "LEFT" : (yaw_delta < -0.5 ? "RIGHT" : "NONE");
    std::cout << "STRAIGHT_CHECK verdict=" << verdict
              << " drift=" << drift
              << " elapsed_s=" << elapsed_s
              << " yaw_start=" << straight_start_yaw_deg_
              << " yaw_now=" << current_yaw
              << " yaw_delta_deg=" << yaw_delta
              << " heading_error_deg=" << heading_error
              << " max_abs_error_deg=" << straight_max_abs_error_deg_
              << " correction=" << heading_correction
              << std::endl;
    straight_check_last_log_ = now;
}

void Controller::applyWheelSpeedControl(const std::array<double, 4>& desired_rpm) {
    const double dt = static_cast<double>(c_.control_ms) / 1000.0;
    const bool ros2_profile = false;   // RC/intent mode; set true for autonomous nav

    // Profile selection (RC vs ROS2 — formal config.xml dual-profile system)
    const double wheel_speed_kp       = ros2_profile ? c_.ros2_wheel_speed_kp : c_.wheel_speed_kp;
    const double wheel_speed_ki       = ros2_profile ? c_.ros2_wheel_speed_ki : c_.wheel_speed_ki;
    const int wheel_feedforward_current = ros2_profile ? c_.ros2_wheel_feedforward_current : c_.wheel_feedforward_current;
    const int wheel_max_current       = ros2_profile ? c_.ros2_wheel_max_current : c_.wheel_max_current;
    const int wheel_integral_max_current = ros2_profile ? c_.ros2_wheel_integral_max_current : c_.wheel_integral_max_current;
    const int wheel_start_initial_current = ros2_profile ? c_.ros2_wheel_start_initial_current : c_.wheel_start_initial_current;
    const int wheel_start_max_current = ros2_profile ? c_.ros2_wheel_start_max_current : c_.wheel_start_max_current;
    const std::string& wheel_control_mode = ros2_profile ? c_.ros2_wheel_control_mode : c_.wheel_control_mode;
    const int wheel_min_run_current   = ros2_profile ? c_.wheel_min_run_current : 0;
    const int wheel_post_start_hold_ms_val = ros2_profile ? c_.wheel_post_start_hold_ms : 0;
    const int wheel_post_start_current = ros2_profile ? c_.wheel_post_start_current : 0;

    const bool stopped = std::all_of(desired_rpm.begin(), desired_rpm.end(),
                                     [](double v) { return std::abs(v) < 0.5; });
    if (stopped) {
        wheel_safety_latched_ = false;
        wheel_safety_reason_.clear();
        wheel_started_.fill(false);
        wheel_start_elapsed_ms_.fill(0);
        wheel_start_confirm_count_.fill(0);
        wheel_stall_count_.fill(0);
        wheel_post_start_hold_ms_.fill(0);
    }

    const bool any_feedback_invalid = std::any_of(feedback_valid_.begin(), feedback_valid_.end(),
                                                  [](bool v) { return !v; });
    if (!stopped && any_feedback_invalid) {
        if (ros2_profile) {
            // ROS2 nodes may receive commands while feedback is still warming up.
            // Don't latch a permanent safety stop — just command zero until valid.
            ramped_target_rpm_.fill(0.0);
            wheel_integral_.fill(0.0);
            wheel_current_10ma_.fill(0);
            wheel_started_.fill(false);
            wheel_start_elapsed_ms_.fill(0);
            wheel_start_confirm_count_.fill(0);
            wheel_stall_count_.fill(0);
            wheel_post_start_hold_ms_.fill(0);
            for (auto id : c_.motor_ids) can_.current(id, 0);
            return;
        }
        wheel_safety_latched_ = true;
        wheel_safety_reason_ = "speed feedback lost";
    }

    for (std::size_t i = 0; i < 4; ++i) {
        if (feedback_valid_[i] && std::abs(measured_rpm_[i]) > c_.wheel_overspeed_rpm) {
            wheel_safety_latched_ = true;
            wheel_safety_reason_ = "wheel overspeed";
        }
    }

    if (wheel_safety_latched_) {
        ramped_target_rpm_.fill(0.0);
        wheel_integral_.fill(0.0);
        wheel_current_10ma_.fill(0);
        for (auto id : c_.motor_ids) can_.current(id, 0);
        if (!wheel_safety_reported_) {
            std::cout << "WHEEL SAFETY STOP: "
                      << (wheel_safety_reason_.empty() ? "startup timeout" : wheel_safety_reason_)
                      << "; return throttle to neutral" << std::endl;
            wheel_safety_reported_ = true;
        }
        return;
    }
    wheel_safety_reported_ = false;

    // ---- speed mode ----
    if (wheel_control_mode == "speed") {
        for (std::size_t i = 0; i < 4; ++i) {
            if (std::abs(desired_rpm[i]) < 0.5) {
                ramped_target_rpm_[i] = 0.0; wheel_integral_[i] = 0.0;
                wheel_current_10ma_[i] = 0; wheel_started_[i] = false;
                can_.speed(c_.motor_ids[i], 0); continue;
            }
            if (!feedback_valid_[i]) {
                ramped_target_rpm_[i] = 0.0; wheel_integral_[i] = 0.0;
                wheel_current_10ma_[i] = 0; can_.speed(c_.motor_ids[i], 0); continue;
            }
            const bool accel = std::abs(desired_rpm[i]) > std::abs(ramped_target_rpm_[i]);
            double acc_r = c_.speed_accel_rpm_s, dec_r = c_.speed_decel_rpm_s;
            if (ros2_profile) { acc_r = std::max(acc_r, 60.0); dec_r = std::max(dec_r, 90.0); }
            const double slew = (accel ? acc_r : dec_r) * dt;
            ramped_target_rpm_[i] += std::clamp(desired_rpm[i] - ramped_target_rpm_[i], -slew, slew);
            const int32_t erpm = static_cast<int32_t>(std::lround(
                ramped_target_rpm_[i] * c_.pole_pairs * c_.gear_ratio * c_.motor_dirs[i]));
            wheel_current_10ma_[i] = 0; wheel_started_[i] = true;
            can_.speed(c_.motor_ids[i], erpm);
        }
        return;
    }

    // ---- current mode (software PI) ----
    bool startup_failed = false;
    for (std::size_t i = 0; i < 4; ++i) {
        if (std::abs(desired_rpm[i]) < 0.5) {
            ramped_target_rpm_[i] = 0.0; wheel_integral_[i] = 0.0;
            wheel_current_10ma_[i] = 0; wheel_started_[i] = false;
            wheel_start_elapsed_ms_[i] = 0; wheel_post_start_hold_ms_[i] = 0;
            can_.current(c_.motor_ids[i], 0); continue;
        }
        if (!feedback_valid_[i]) {
            ramped_target_rpm_[i] = 0.0; wheel_integral_[i] = 0.0;
            wheel_current_10ma_[i] = 0; can_.current(c_.motor_ids[i], 0); continue;
        }

        const bool accelerating = std::abs(desired_rpm[i]) > std::abs(ramped_target_rpm_[i]);
        const double slew = (accelerating ? c_.speed_accel_rpm_s : c_.speed_decel_rpm_s) * dt;
        ramped_target_rpm_[i] += std::clamp(desired_rpm[i] - ramped_target_rpm_[i], -slew, slew);

        // Dynamic start-release: require a meaningful fraction of desired RPM before
        // leaving startup current.  Prevents a tiny encoder twitch from switching to
        // the weak PI current → "start—sag—stall—restart" stutter.
        const double start_command_rpm = std::clamp(std::abs(desired_rpm[i]) * 0.67, 2.0, 10.0);
        double start_release_rpm = std::max(
            static_cast<double>(c_.wheel_start_threshold_rpm),
            std::clamp(std::abs(desired_rpm[i]) * 0.45, 5.0, 15.0));
        if (ros2_profile) {
            start_release_rpm = std::max(
                static_cast<double>(c_.wheel_start_threshold_rpm),
                std::clamp(std::abs(desired_rpm[i]) * 0.15, 3.0, 5.0));
        }

        const bool new_feedback = feedback_generation_[i] != processed_feedback_generation_[i];
        if (new_feedback) {
            processed_feedback_generation_[i] = feedback_generation_[i];
            if (wheel_started_[i]) {
                if (std::abs(measured_rpm_[i]) < c_.wheel_stall_threshold_rpm)
                    ++wheel_stall_count_[i];
                else
                    wheel_stall_count_[i] = 0;
                if (wheel_stall_count_[i] >= c_.wheel_stall_confirm_samples) {
                    wheel_started_[i] = false;
                    wheel_start_elapsed_ms_[i] = 0;
                    wheel_start_confirm_count_[i] = 0;
                    wheel_stall_count_[i] = 0;
                    wheel_post_start_hold_ms_[i] = 0;
                    wheel_integral_[i] = 0.0;
                }
            } else if (wheel_start_elapsed_ms_[i] > 0) {
                if (std::abs(measured_rpm_[i]) >= start_release_rpm)
                    ++wheel_start_confirm_count_[i];
                else
                    wheel_start_confirm_count_[i] = 0;
                if (wheel_start_confirm_count_[i] >= c_.wheel_start_confirm_samples) {
                    wheel_started_[i] = true;
                    // Jump the ramp target to the command target at handoff.
                    // The startup current (55–70 mA) far exceeds the PI run limit
                    // (35 mA), so the wheel is already near target speed.  If the
                    // ramp is still crawling from zero, the PI sees a huge negative
                    // error → slams reverse current → stall → stutter.  Jumping
                    // avoids that gap.
                    ramped_target_rpm_[i] = desired_rpm[i];
                    wheel_integral_[i] = 0.0;
                    wheel_stall_count_[i] = 0;
                    wheel_post_start_hold_ms_[i] = wheel_post_start_hold_ms_val;
                }
            }
        }

        if (!wheel_started_[i]) {
            if (std::abs(desired_rpm[i]) < start_command_rpm) {
                processed_feedback_generation_[i] = feedback_generation_[i];
                wheel_start_elapsed_ms_[i] = 0;
                wheel_start_confirm_count_[i] = 0;
                wheel_current_10ma_[i] = 0;
                can_.current(c_.motor_ids[i], 0); continue;
            }
            wheel_start_elapsed_ms_[i] += c_.control_ms;
            if (wheel_start_elapsed_ms_[i] > c_.wheel_start_timeout_ms) startup_failed = true;
            const int steps = wheel_start_elapsed_ms_[i] / c_.wheel_start_step_ms;
            const int start_current = std::min(wheel_start_max_current,
                wheel_start_initial_current + steps * c_.wheel_start_step_current);
            wheel_current_10ma_[i] = desired_rpm[i] > 0 ? start_current : -start_current;
            can_.current(c_.motor_ids[i], wheel_current_10ma_[i] * c_.motor_dirs[i]);
            continue;
        }

        // ---- PI + feed-forward ----
        const double error = ramped_target_rpm_[i] - measured_rpm_[i];
        double next_integral = wheel_integral_[i] + error * dt;
        if (wheel_speed_ki > 0.0) {
            const double lim = static_cast<double>(wheel_integral_max_current) / wheel_speed_ki;
            next_integral = std::clamp(next_integral, -lim, lim);
        } else next_integral = 0.0;

        const double desired_magnitude = std::max(1.0, std::abs(desired_rpm[i]));
        const double feedforward_scale = std::clamp(
            std::abs(ramped_target_rpm_[i]) / desired_magnitude, 0.0, 1.0);
        const bool same_direction = measured_rpm_[i] * ramped_target_rpm_[i] > 0.0;
        const bool overspeed_same_direction = same_direction &&
            std::abs(measured_rpm_[i]) > std::abs(ramped_target_rpm_[i]);
        double feedforward = wheel_feedforward_current * feedforward_scale;
        // ROS2: when wheel is already faster than ramp, stop adding drive FF.
        // Prevents push-through overspeed → stick-slip.
        if (ros2_profile && overspeed_same_direction) feedforward = 0.0;
        const double ff = ramped_target_rpm_[i] >= 0 ? feedforward : -feedforward;

        const double raw = ff + wheel_speed_kp * error + wheel_speed_ki * next_integral;
        double limited = std::clamp(raw, -static_cast<double>(wheel_max_current),
                                    static_cast<double>(wheel_max_current));
        if (std::abs(raw) <= wheel_max_current) wheel_integral_[i] = next_integral;

        // ROS2 anti-brake: don't actively brake with opposite-sign current when the
        // wheel is already faster than target.  Braking drops it into static friction
        // → startup current kicks again → mid-run stick-slip.  Coasting is smoother.
        if (ros2_profile && overspeed_same_direction && limited * ramped_target_rpm_[i] < 0.0) {
            limited = 0.0;
            wheel_integral_[i] = 0.0;
        }

        // Post-start minimum current: hold a floor current after startup so the PI
        // doesn't sag below the stall threshold.  Decrements the hold timer each tick.
        int min_current = wheel_min_run_current;
        if (wheel_post_start_hold_ms_[i] > 0) {
            min_current = std::max(min_current, wheel_post_start_current);
            wheel_post_start_hold_ms_[i] = std::max(0, wheel_post_start_hold_ms_[i] - c_.control_ms);
        }
        // Only apply anti-stall minimum when wheel is slower than target.
        // If the wheel is already overshooting, a minimum drive current would
        // keep accelerating it and could trip wheel overspeed.
        const bool wheel_too_slow = desired_rpm[i] * error > 0.0;
        if (min_current > 0 && wheel_too_slow && std::abs(desired_rpm[i]) >= 0.5 &&
            std::abs(limited) < min_current) {
            const double sign_ref = std::abs(ramped_target_rpm_[i]) >= 0.5
                                        ? ramped_target_rpm_[i] : desired_rpm[i];
            limited = sign_ref >= 0.0 ? static_cast<double>(min_current) : -static_cast<double>(min_current);
        }

        wheel_current_10ma_[i] = static_cast<int>(std::lround(limited));
        can_.current(c_.motor_ids[i], wheel_current_10ma_[i] * c_.motor_dirs[i]);
    }
    if (startup_failed) {
        wheel_safety_latched_ = true;
        wheel_safety_reason_ = "startup timeout";
        wheel_current_10ma_.fill(0);
        for (auto id : c_.motor_ids) can_.current(id, 0);
    }
}

void Controller::logState(double throttle, double steering, bool mower, bool failsafe,
                          int left_rpm, int right_rpm, bool heading_holding,
                          double yaw_deg, double heading_error, int correction) {
    const int correction_bin = heading_holding ? correction / 5 : 999;
    const std::array<int, 5> state{static_cast<int>(throttle), static_cast<int>(steering),
                                   mower ? 1 : 0, failsafe ? 1 : 0, correction_bin};
    if (!c_.show_log || state == last_state_) return;
    last_state_ = state;
    const char* motion = "STOP";
    if (failsafe) motion = "FAILSAFE STOP";
    // On this robot, negative logical throttle is physical forward.
    else if (throttle < 0 && steering > 0) motion = "FORWARD LEFT";
    else if (throttle < 0 && steering < 0) motion = "FORWARD RIGHT";
    else if (throttle > 0 && steering > 0) motion = "REVERSE LEFT";
    else if (throttle > 0 && steering < 0) motion = "REVERSE RIGHT";
    else if (throttle < 0) motion = "FORWARD";
    else if (throttle > 0) motion = "REVERSE";
    else if (steering > 0) motion = "ROTATE LEFT";
    else if (steering < 0) motion = "ROTATE RIGHT";
    std::cout << motion << " left=" << left_rpm
              << " right=" << right_rpm
              << " mower=" << (mower ? "ON" : "OFF");
    if (heading_holding) {
        std::cout << " HEADING yaw=" << yaw_deg
                  << " target=" << target_yaw_deg_
                  << " error=" << heading_error
                  << " correction=" << correction;
    }
    std::cout << std::endl;
}

}  // namespace chassis_core
