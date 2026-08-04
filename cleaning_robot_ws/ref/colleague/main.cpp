#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <asm/termbits.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "odometry.h"

namespace {
std::atomic<bool> running{true};

void signalHandler(int) { running = false; }

std::string readTextFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open config: " + path);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::string xmlValue(const std::string& xml, const std::string& tag,
                     const std::string& fallback) {
    const std::string begin = "<" + tag + ">";
    const std::string end = "</" + tag + ">";
    const auto p1 = xml.find(begin);
    if (p1 == std::string::npos) return fallback;
    const auto p2 = xml.find(end, p1 + begin.size());
    if (p2 == std::string::npos) return fallback;
    return xml.substr(p1 + begin.size(), p2 - p1 - begin.size());
}

int xmlInt(const std::string& xml, const std::string& tag, int fallback) {
    try { return std::stoi(xmlValue(xml, tag, std::to_string(fallback))); }
    catch (...) { return fallback; }
}

double xmlDouble(const std::string& xml, const std::string& tag, double fallback) {
    try { return std::stod(xmlValue(xml, tag, std::to_string(fallback))); }
    catch (...) { return fallback; }
}

struct Config {
    std::string sbus_port{"/dev/ttyS1"};
    std::string can_port{"can0"};
    int run_rpm{100};           // physical wheel RPM
    int rotate_rpm{50};        // physical wheel RPM
    int turn_max_rpm{0};       // max outer wheel RPM while arc-turning; 0 disables
    int arc_turn_blend_ms{700}; // ramp straight-to-arc differential over this time
    bool rotate_in_place_enable{true};
    int rotate_direction_sign{1};
    bool rotate_factory_current_control_enable{true};
    int rotate_neutral_hold_ms{150}; // delay before in-place rotate, filters stick jitter
    bool rotate_speed_guard_enable{true};
    double rotate_start_max_rpm{25.0}; // allow factory rotate current only below this wheel rpm
    int rotate_start_guard_timeout_ms{1200}; // then allow anyway to avoid getting stuck
    int speed_accel_rpm_s{100};       // software target ramp
    int speed_decel_rpm_s{200};       // software target ramp
    double wheel_speed_kp{0.08};
    double wheel_speed_ki{0.03};
    std::string wheel_control_mode{"current"};
    int wheel_feedforward_current{18};
    int wheel_max_current{40};        // running limit: 0.40A
    int rotate_max_current{500};      // factory in-place rotate torque: 5.00A, 10mA/unit
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
    int cut_current{1000};     // mower remains current controlled, 10 mA/unit
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
    int throttle_channel{3};    // 1-based SBUS channel
    int steering_channel{4};    // 1-based SBUS channel
    int mower_channel{2};       // 1-based SBUS channel
    int gear_channel{6};        // 1-based SBUS channel, 3-position gear switch; 0 disables
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
    std::array<uint16_t, 4> motor_ids{2, 1, 4, 3}; // FL, RL, FR, RR
    std::array<int, 4> motor_dirs{1, 1, -1, -1};   // physical direction correction
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
};

Config loadConfig(const std::string& path) {
    const auto xml = readTextFile(path);
    Config c;
    c.sbus_port = xmlValue(xml, "SBus_Port", c.sbus_port);
    c.can_port = xmlValue(xml, "Can_Port", c.can_port);
    c.run_rpm = xmlInt(xml, "Run_RPM", c.run_rpm);
    c.rotate_rpm = xmlInt(xml, "Rotate_RPM", c.rotate_rpm);
    c.turn_max_rpm = xmlInt(xml, "Turn_Max_RPM", c.turn_max_rpm);
    c.arc_turn_blend_ms = xmlInt(xml, "Arc_Turn_Blend_ms", c.arc_turn_blend_ms);
    c.rotate_in_place_enable =
        xmlInt(xml, "Rotate_In_Place_Enable",
               c.rotate_in_place_enable ? 1 : 0) != 0;
    c.rotate_direction_sign =
        xmlInt(xml, "Rotate_Direction_Sign", c.rotate_direction_sign);
    c.rotate_factory_current_control_enable =
        xmlInt(xml, "Rotate_Factory_Current_Control_Enable",
               c.rotate_factory_current_control_enable ? 1 : 0) != 0;
    c.rotate_neutral_hold_ms = xmlInt(xml, "Rotate_Neutral_Hold_ms", c.rotate_neutral_hold_ms);
    c.rotate_speed_guard_enable =
        xmlInt(xml, "Rotate_Speed_Guard_Enable", c.rotate_speed_guard_enable ? 1 : 0) != 0;
    c.rotate_start_max_rpm = xmlDouble(xml, "Rotate_Start_Max_RPM", c.rotate_start_max_rpm);
    c.rotate_start_guard_timeout_ms =
        xmlInt(xml, "Rotate_Start_Guard_Timeout_ms", c.rotate_start_guard_timeout_ms);
    c.speed_accel_rpm_s = xmlInt(xml, "Speed_Accel_RPM_s", c.speed_accel_rpm_s);
    c.speed_decel_rpm_s = xmlInt(xml, "Speed_Decel_RPM_s", c.speed_decel_rpm_s);
    c.wheel_speed_kp = xmlDouble(xml, "Wheel_Speed_Kp", c.wheel_speed_kp);
    c.wheel_speed_ki = xmlDouble(xml, "Wheel_Speed_Ki", c.wheel_speed_ki);
    c.wheel_control_mode = xmlValue(xml, "Wheel_Control_Mode", c.wheel_control_mode);
    c.wheel_feedforward_current = xmlInt(xml, "Wheel_Feedforward_Current", c.wheel_feedforward_current);
    c.wheel_max_current = xmlInt(xml, "Wheel_Max_Current", c.wheel_max_current);
    c.rotate_max_current = xmlInt(xml, "Rotate_Max_Current", c.rotate_max_current);
    c.wheel_integral_max_current = xmlInt(xml, "Wheel_Integral_Max_Current", c.wheel_integral_max_current);
    c.wheel_start_initial_current = xmlInt(xml, "Wheel_Start_Initial_Current", c.wheel_start_initial_current);
    c.wheel_start_step_current = xmlInt(xml, "Wheel_Start_Step_Current", c.wheel_start_step_current);
    c.wheel_start_step_ms = xmlInt(xml, "Wheel_Start_Step_ms", c.wheel_start_step_ms);
    c.wheel_start_max_current = xmlInt(xml, "Wheel_Start_Max_Current", c.wheel_start_max_current);
    c.wheel_start_threshold_rpm = xmlInt(xml, "Wheel_Start_Threshold_RPM", c.wheel_start_threshold_rpm);
    c.wheel_start_confirm_samples = xmlInt(xml, "Wheel_Start_Confirm_Samples", c.wheel_start_confirm_samples);
    c.wheel_stall_threshold_rpm = xmlInt(xml, "Wheel_Stall_Threshold_RPM", c.wheel_stall_threshold_rpm);
    c.wheel_stall_confirm_samples = xmlInt(xml, "Wheel_Stall_Confirm_Samples", c.wheel_stall_confirm_samples);
    c.wheel_start_timeout_ms = xmlInt(xml, "Wheel_Start_Timeout_ms", c.wheel_start_timeout_ms);
    c.wheel_overspeed_rpm = xmlInt(xml, "Wheel_Overspeed_RPM", c.wheel_overspeed_rpm);
    c.speed_feedback_timeout_ms = xmlInt(xml, "Speed_Feedback_Timeout_ms", c.speed_feedback_timeout_ms);
    c.cut_current = xmlInt(xml, "Cut_Speed", c.cut_current);
    c.turn_inner_ratio = xmlDouble(xml, "Turn_Inner_Ratio", c.turn_inner_ratio);
    c.low_threshold = xmlInt(xml, "Low_Threshold", c.low_threshold);
    c.high_threshold = xmlInt(xml, "High_Threshold", c.high_threshold);
    c.failsafe_ms = xmlInt(xml, "Failsafe_ms", c.failsafe_ms);
    c.heartbeat_ms = xmlInt(xml, "Heartbeat_ms", c.heartbeat_ms);
    c.control_ms = xmlInt(xml, "Control_ms", c.control_ms);
    c.speed_query_ms = xmlInt(xml, "Speed_Query_ms", c.speed_query_ms);
    c.speed_log_ms = xmlInt(xml, "Speed_Log_ms", c.speed_log_ms);
    c.wheel_radius_m = xmlDouble(xml, "Wheel_Radius_m", c.wheel_radius_m);
    c.track_width_m = xmlDouble(xml, "Track_Width_m", c.track_width_m);
    c.odom_log_enable = xmlInt(xml, "Odom_Log_Enable", c.odom_log_enable ? 1 : 0) != 0;
    c.odom_log_path = xmlValue(xml, "Odom_Log_Path", c.odom_log_path);
    c.pole_pairs = xmlInt(xml, "Pole_Pairs", c.pole_pairs);
    c.gear_ratio = xmlDouble(xml, "Gear_Ratio", c.gear_ratio);
    c.throttle_channel = xmlInt(xml, "Throttle_Channel", c.throttle_channel);
    c.steering_channel = xmlInt(xml, "Steering_Channel", c.steering_channel);
    c.mower_channel = xmlInt(xml, "Mower_Channel", c.mower_channel);
    c.gear_channel = xmlInt(xml, "Gear_Channel", c.gear_channel);
    c.gear_select_enable = xmlInt(xml, "Gear_Select_Enable", c.gear_select_enable ? 1 : 0) != 0;
    c.gear_low_run_rpm = xmlInt(xml, "Gear_Low_Run_RPM", c.gear_low_run_rpm);
    c.gear_low_turn_max_rpm = xmlInt(xml, "Gear_Low_Turn_Max_RPM", c.gear_low_turn_max_rpm);
    c.gear_mid_run_rpm = xmlInt(xml, "Gear_Mid_Run_RPM", c.gear_mid_run_rpm);
    c.gear_mid_turn_max_rpm = xmlInt(xml, "Gear_Mid_Turn_Max_RPM", c.gear_mid_turn_max_rpm);
    c.gear_high_run_rpm = xmlInt(xml, "Gear_High_Run_RPM", c.gear_high_run_rpm);
    c.gear_high_turn_max_rpm = xmlInt(xml, "Gear_High_Turn_Max_RPM", c.gear_high_turn_max_rpm);
    c.app_control_enable = xmlInt(xml, "App_Control_Enable", c.app_control_enable ? 1 : 0) != 0;
    c.app_udp_port = xmlInt(xml, "App_UDP_Port", c.app_udp_port);
    c.app_timeout_ms = xmlInt(xml, "App_Timeout_ms", c.app_timeout_ms);
    c.motor_ids = {
        static_cast<uint16_t>(xmlInt(xml, "FL_ID", c.motor_ids[0])),
        static_cast<uint16_t>(xmlInt(xml, "RL_ID", c.motor_ids[1])),
        static_cast<uint16_t>(xmlInt(xml, "FR_ID", c.motor_ids[2])),
        static_cast<uint16_t>(xmlInt(xml, "RR_ID", c.motor_ids[3]))
    };
    c.motor_dirs = {
        xmlInt(xml, "FL_Dir", c.motor_dirs[0]),
        xmlInt(xml, "RL_Dir", c.motor_dirs[1]),
        xmlInt(xml, "FR_Dir", c.motor_dirs[2]),
        xmlInt(xml, "RR_Dir", c.motor_dirs[3])
    };
    c.mower_id = static_cast<uint16_t>(xmlInt(xml, "Mower_ID", c.mower_id));
    c.show_log = xmlInt(xml, "ShowLog", c.show_log ? 1 : 0) != 0;
    c.imu_heading_enable = xmlInt(xml, "IMU_Heading_Enable", c.imu_heading_enable ? 1 : 0) != 0;
    c.imu_device = xmlValue(xml, "IMU_Device", c.imu_device);
    c.imu_axis = xmlValue(xml, "IMU_Axis", c.imu_axis);
    c.imu_yaw_sign = xmlDouble(xml, "IMU_Yaw_Sign", c.imu_yaw_sign);
    c.imu_calibrate_ms = xmlInt(xml, "IMU_Calibrate_ms", c.imu_calibrate_ms);
    c.imu_deadband_dps = xmlDouble(xml, "IMU_Deadband_dps", c.imu_deadband_dps);
    c.heading_kp = xmlDouble(xml, "Heading_Kp", c.heading_kp);
    c.heading_ki = xmlDouble(xml, "Heading_Ki", c.heading_ki);
    c.heading_kd = xmlDouble(xml, "Heading_Kd", c.heading_kd);
    c.heading_integral_max_correction = xmlInt(xml, "Heading_Integral_Max_Correction", c.heading_integral_max_correction);
    c.heading_max_correction = xmlInt(xml, "Heading_Max_Correction", c.heading_max_correction);
    c.heading_correction_sign = xmlInt(xml, "Heading_Correction_Sign", c.heading_correction_sign);
    c.heading_forward_trim = xmlInt(xml, "Heading_Forward_Trim", c.heading_forward_trim);
    c.heading_reverse_trim = xmlInt(xml, "Heading_Reverse_Trim", c.heading_reverse_trim);
    c.heading_reset_steering_hold_ms = xmlInt(xml, "Heading_Reset_Steering_Hold_ms", c.heading_reset_steering_hold_ms);
    c.direction_change_hold_enable =
        xmlInt(xml, "Direction_Change_Hold_Enable",
               c.direction_change_hold_enable ? 1 : 0) != 0;
    c.direction_change_stop_rpm =
        xmlDouble(xml, "Direction_Change_Stop_RPM", c.direction_change_stop_rpm);
    c.direction_change_hold_timeout_ms =
        xmlInt(xml, "Direction_Change_Hold_Timeout_ms",
               c.direction_change_hold_timeout_ms);
    c.direction_change_post_hold_ms =
        xmlInt(xml, "Direction_Change_Post_Hold_ms",
               c.direction_change_post_hold_ms);
    c.direction_change_post_max_diff_rpm =
        xmlInt(xml, "Direction_Change_Post_Max_Diff_RPM",
               c.direction_change_post_max_diff_rpm);
    c.lateral_hold_enable = xmlInt(xml, "Lateral_Hold_Enable", c.lateral_hold_enable ? 1 : 0) != 0;
    c.lateral_kp_deg_per_m = xmlDouble(xml, "Lateral_Kp_Deg_Per_M", c.lateral_kp_deg_per_m);
    c.lateral_max_heading_deg = xmlDouble(xml, "Lateral_Max_Heading_Deg", c.lateral_max_heading_deg);

    if (c.run_rpm <= 0 || c.rotate_rpm <= 0 || c.speed_accel_rpm_s <= 0 ||
        c.speed_decel_rpm_s <= 0 || c.wheel_speed_kp < 0.0 || c.wheel_speed_ki < 0.0 ||
        c.wheel_feedforward_current < 0 || c.wheel_feedforward_current > c.wheel_max_current ||
        c.wheel_max_current <= 0 || c.wheel_max_current > 500 ||
        c.rotate_max_current <= 0 || c.rotate_max_current > 1000 ||
        c.wheel_integral_max_current < 0 || c.wheel_integral_max_current > c.wheel_max_current ||
        c.wheel_start_initial_current < 1 || c.wheel_start_initial_current > c.wheel_start_max_current ||
        c.wheel_start_step_current < 1 || c.wheel_start_step_ms < c.control_ms ||
        c.wheel_start_max_current < c.wheel_max_current || c.wheel_start_max_current > 500 ||
        c.wheel_start_threshold_rpm < 1 || c.wheel_start_confirm_samples < 1 ||
        c.wheel_stall_threshold_rpm < 0 || c.wheel_stall_threshold_rpm >= c.wheel_start_threshold_rpm ||
        c.wheel_stall_confirm_samples < 1 || c.wheel_start_timeout_ms < c.wheel_start_step_ms ||
        c.wheel_overspeed_rpm <= c.run_rpm ||
        c.speed_feedback_timeout_ms < c.speed_query_ms || c.cut_current < 0 ||
        (c.wheel_control_mode != "current" && c.wheel_control_mode != "speed"))
        throw std::runtime_error("invalid software wheel speed settings");
    if (c.speed_query_ms < 10 || c.speed_log_ms < 100 ||
        c.wheel_radius_m <= 0.0 || c.track_width_m <= 0.0 ||
        c.pole_pairs <= 0 || c.gear_ratio <= 0.0)
        throw std::runtime_error("invalid speed feedback settings");
    if (c.odom_log_path.empty()) c.odom_log_enable = false;
    if (c.turn_inner_ratio < -0.50 || c.turn_inner_ratio > 1.0)
        throw std::runtime_error("Turn_Inner_Ratio must be between -0.50 and 1");
    if (c.turn_max_rpm < 0 || c.turn_max_rpm > c.run_rpm)
        throw std::runtime_error("Turn_Max_RPM must be 0..Run_RPM");
    if (c.arc_turn_blend_ms < 0 || c.arc_turn_blend_ms > 3000)
        throw std::runtime_error("Arc_Turn_Blend_ms must be 0..3000");
    if (c.rotate_neutral_hold_ms < 0 || c.rotate_neutral_hold_ms > 1000)
        throw std::runtime_error("Rotate_Neutral_Hold_ms must be 0..1000");
    if (c.rotate_start_max_rpm < 0.0 || c.rotate_start_max_rpm > c.wheel_overspeed_rpm)
        throw std::runtime_error("Rotate_Start_Max_RPM must be 0..Wheel_Overspeed_RPM");
    if (c.rotate_start_guard_timeout_ms < 0 || c.rotate_start_guard_timeout_ms > 5000)
        throw std::runtime_error("Rotate_Start_Guard_Timeout_ms must be 0..5000");
    for (int channel : {c.throttle_channel, c.steering_channel, c.mower_channel})
        if (channel < 1 || channel > 18) throw std::runtime_error("invalid SBUS channel");
    if (c.gear_select_enable && (c.gear_channel < 1 || c.gear_channel > 18))
        throw std::runtime_error("invalid Gear_Channel");
    auto valid_gear = [](int run, int turn) { return run > 0 && turn >= 0 && turn <= run; };
    if (!valid_gear(c.gear_low_run_rpm, c.gear_low_turn_max_rpm) ||
        !valid_gear(c.gear_mid_run_rpm, c.gear_mid_turn_max_rpm) ||
        !valid_gear(c.gear_high_run_rpm, c.gear_high_turn_max_rpm))
        throw std::runtime_error("invalid gear RPM settings");
    if (c.app_udp_port < 1 || c.app_udp_port > 65535 ||
        c.app_timeout_ms < 100 || c.app_timeout_ms > 5000)
        throw std::runtime_error("invalid APP control settings");
    for (int dir : c.motor_dirs)
        if (dir != 1 && dir != -1) throw std::runtime_error("motor direction must be 1 or -1");
    if (c.imu_axis != "x" && c.imu_axis != "y" && c.imu_axis != "z")
        throw std::runtime_error("IMU_Axis must be x, y or z");
    if (c.imu_yaw_sign != 1.0 && c.imu_yaw_sign != -1.0)
        throw std::runtime_error("IMU_Yaw_Sign must be 1 or -1");
    if (c.imu_calibrate_ms < 500 || c.heading_kp < 0 || c.heading_ki < 0 || c.heading_kd < 0 ||
        c.heading_integral_max_correction < 0 ||
        c.heading_integral_max_correction > c.heading_max_correction ||
        c.heading_max_correction < 0 || c.heading_max_correction > c.run_rpm ||
        (c.heading_correction_sign != 1 && c.heading_correction_sign != -1) ||
        std::abs(c.heading_forward_trim) > c.heading_max_correction ||
        std::abs(c.heading_reverse_trim) > c.heading_max_correction ||
        c.heading_reset_steering_hold_ms < 0 || c.heading_reset_steering_hold_ms > 5000 ||
        c.direction_change_stop_rpm < 0.0 ||
        c.direction_change_hold_timeout_ms < c.control_ms ||
        c.direction_change_post_hold_ms < 0 ||
        c.direction_change_post_max_diff_rpm < 0 ||
        c.direction_change_post_max_diff_rpm > 2 * c.run_rpm ||
        c.lateral_kp_deg_per_m < 0.0 || c.lateral_max_heading_deg < 0.0 ||
        c.lateral_max_heading_deg > 15.0)
        throw std::runtime_error("invalid IMU heading settings");
    if (c.rotate_direction_sign != 1 && c.rotate_direction_sign != -1)
        throw std::runtime_error("Rotate_Direction_Sign must be 1 or -1");
    return c;
}

class CanBus {
public:
    explicit CanBus(const std::string& interface_name) {
        fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) throw std::runtime_error("cannot create CAN socket");
        ifreq ifr{};
        std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) throw std::runtime_error("CAN interface not found");
        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            throw std::runtime_error("cannot bind CAN interface");
    }
    ~CanBus() { if (fd_ >= 0) ::close(fd_); }

    void heartbeat(uint16_t id) {
        uint8_t data[1]{0x00};
        send(id, data, 1);
    }

    void current(uint16_t id, int value_10ma) {
        value_10ma = std::clamp(value_10ma, -32768, 32767);
        const uint16_t raw = static_cast<uint16_t>(static_cast<int16_t>(value_10ma));
        uint8_t data[3]{0x01,
                        static_cast<uint8_t>((raw >> 8) & 0xff),
                        static_cast<uint8_t>(raw & 0xff)};
        send(id, data, 3);
    }

    void speed(uint16_t id, int32_t erpm) {
        const uint32_t raw = static_cast<uint32_t>(erpm);
        uint8_t data[5]{0x02,
                        static_cast<uint8_t>((raw >> 24) & 0xff),
                        static_cast<uint8_t>((raw >> 16) & 0xff),
                        static_cast<uint8_t>((raw >> 8) & 0xff),
                        static_cast<uint8_t>(raw & 0xff)};
        send(id, data, 5);
    }

    void setAccel(uint16_t id, int32_t erpm_per_s) { sendInt32Command(id, 0x0a, erpm_per_s); }
    void setDecel(uint16_t id, int32_t erpm_per_s) { sendInt32Command(id, 0x10, erpm_per_s); }

    void setClosedLoopMaxCurrent(uint16_t id, int value_10ma) {
        value_10ma = std::clamp(value_10ma, 0, 32767);
        const uint16_t raw = static_cast<uint16_t>(value_10ma);
        uint8_t data[3]{0x14,
                        static_cast<uint8_t>((raw >> 8) & 0xff),
                        static_cast<uint8_t>(raw & 0xff)};
        send(id, data, 3);
    }

    void querySpeed(uint16_t id) {
        uint8_t data[2]{0x0f, 0x01};
        send(id, data, 2);
    }

    bool readSpeed(uint16_t& id, int32_t& erpm) {
        can_frame frame{};
        const auto n = ::recv(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
        if (n != static_cast<ssize_t>(sizeof(frame))) return false;
        id = 0;
        erpm = 0;
        if (frame.can_dlc >= 6 && frame.data[0] == 0x0f && frame.data[1] == 0x01) {
            id = static_cast<uint16_t>(frame.can_id & CAN_SFF_MASK);
            const uint32_t raw = (static_cast<uint32_t>(frame.data[2]) << 24) |
                                 (static_cast<uint32_t>(frame.data[3]) << 16) |
                                 (static_cast<uint32_t>(frame.data[4]) << 8) |
                                  static_cast<uint32_t>(frame.data[5]);
            erpm = static_cast<int32_t>(raw);
        }
        return true;
    }

private:
    void sendInt32Command(uint16_t id, uint8_t command, int32_t value) {
        const uint32_t raw = static_cast<uint32_t>(value);
        uint8_t data[5]{command,
                        static_cast<uint8_t>((raw >> 24) & 0xff),
                        static_cast<uint8_t>((raw >> 16) & 0xff),
                        static_cast<uint8_t>((raw >> 8) & 0xff),
                        static_cast<uint8_t>(raw & 0xff)};
        send(id, data, 5);
    }

    void send(uint16_t id, const uint8_t* data, uint8_t length) {
        can_frame frame{};
        frame.can_id = id & CAN_SFF_MASK;
        frame.can_dlc = length;
        std::memcpy(frame.data, data, length);
        for (int attempt = 0; attempt < 20; ++attempt) {
            if (::write(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame)))
                return;
            const int error = errno;
            if (error == ENOBUFS || error == EAGAIN || error == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            throw std::runtime_error("CAN write failed: id=" + std::to_string(id) +
                                     " command=" + std::to_string(data[0]) +
                                     " errno=" + std::to_string(error) +
                                     " (" + std::strerror(error) + ")");
        }
        throw std::runtime_error("CAN transmit queue remained full: id=" +
                                 std::to_string(id) + " command=" +
                                 std::to_string(data[0]));
    }
    int fd_{-1};
};

struct SbusFrame {
    std::array<uint16_t, 18> channel{};
    bool frame_lost{false};
    bool failsafe{false};
};

class SbusReader {
public:
    explicit SbusReader(const std::string& device) {
        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("cannot open SBUS device: " + device);
        termios2 tio{};
        if (::ioctl(fd_, TCGETS2, &tio) < 0) throw std::runtime_error("TCGETS2 failed");
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cflag &= ~(CBAUD | CSIZE | PARODD);
        tio.c_cflag |= BOTHER | CS8 | PARENB | CSTOPB | CLOCAL | CREAD;
        tio.c_ispeed = 100000;
        tio.c_ospeed = 100000;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (::ioctl(fd_, TCSETS2, &tio) < 0) throw std::runtime_error("TCSETS2 failed");
    }
    ~SbusReader() { if (fd_ >= 0) ::close(fd_); }

    bool poll(SbusFrame& output) {
        uint8_t input[128];
        bool got_frame = false;
        for (;;) {
            const auto n = ::read(fd_, input, sizeof(input));
            if (n <= 0) break;
            for (ssize_t i = 0; i < n; ++i) {
                if (position_ == 0) {
                    if (input[i] != 0x0f) continue;
                    buffer_[position_++] = input[i];
                } else {
                    buffer_[position_++] = input[i];
                    if (position_ == buffer_.size()) {
                        if (decode(output)) got_frame = true;
                        position_ = 0;
                    }
                }
            }
        }
        return got_frame;
    }

private:
    bool decode(SbusFrame& out) const {
        if (buffer_[0] != 0x0f) return false;
        for (int ch = 0; ch < 16; ++ch) {
            const int bit = ch * 11;
            const int byte_index = 1 + bit / 8;
            const int shift = bit % 8;
            uint32_t value = buffer_[byte_index];
            if (byte_index + 1 < 23) value |= static_cast<uint32_t>(buffer_[byte_index + 1]) << 8;
            if (byte_index + 2 < 23) value |= static_cast<uint32_t>(buffer_[byte_index + 2]) << 16;
            out.channel[ch] = static_cast<uint16_t>((value >> shift) & 0x07ff);
        }
        out.channel[16] = (buffer_[23] & 0x01) ? 2047 : 0;
        out.channel[17] = (buffer_[23] & 0x02) ? 2047 : 0;
        out.frame_lost = (buffer_[23] & 0x04) != 0;
        out.failsafe = (buffer_[23] & 0x08) != 0;
        return true;
    }

    int fd_{-1};
    std::array<uint8_t, 25> buffer_{};
    std::size_t position_{0};
};

int throttleDirection(uint16_t value, const Config& c) {
    if (value <= c.low_threshold) return 1;      // preserves original forward direction
    if (value > c.high_threshold) return -1;
    return 0;
}

int steeringDirection(uint16_t value, const Config& c) {
    if (value <= c.low_threshold) return -1;     // right
    if (value > c.high_threshold) return 1;      // left
    return 0;
}

class ImuYaw {
public:
    explicit ImuYaw(const Config& c)
        : raw_path_(c.imu_device + "/in_anglvel_" + c.imu_axis + "_raw"),
          scale_path_(c.imu_device + "/in_anglvel_scale"),
          yaw_sign_(c.imu_yaw_sign), deadband_dps_(c.imu_deadband_dps),
          calibrate_ms_(c.imu_calibrate_ms) {
        scale_ = readNumber(scale_path_);
    }

    void calibrate() {
        double sum = 0.0;
        int count = 0;
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(calibrate_ms_);
        while (std::chrono::steady_clock::now() < end) {
            sum += readNumber(raw_path_) * scale_;
            ++count;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (count == 0) throw std::runtime_error("IMU calibration failed");
        bias_rad_s_ = sum / count;
        yaw_deg_ = 0.0;
        rate_dps_ = 0.0;
        last_update_ = std::chrono::steady_clock::now();
        valid_ = true;
    }

    void update() {
        if (!valid_) return;
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_update_).count();
        last_update_ = now;
        double rate = (readNumber(raw_path_) * scale_ - bias_rad_s_) *
                      (180.0 / 3.14159265358979323846) * yaw_sign_;
        if (std::abs(rate) < deadband_dps_) rate = 0.0;
        rate_dps_ = rate;
        if (dt > 0.0 && dt < 0.5) yaw_deg_ += rate_dps_ * dt;
    }

    bool valid() const { return valid_; }
    double yawDeg() const { return yaw_deg_; }
    double rateDps() const { return rate_dps_; }
    double biasDps() const {
        return bias_rad_s_ * (180.0 / 3.14159265358979323846) * yaw_sign_;
    }

private:
    static double readNumber(const std::string& path) {
        std::ifstream input(path);
        double value = 0.0;
        if (!(input >> value)) throw std::runtime_error("cannot read IMU: " + path);
        return value;
    }

    std::string raw_path_;
    std::string scale_path_;
    double yaw_sign_{-1.0};
    double deadband_dps_{0.30};
    int calibrate_ms_{3000};
    double scale_{0.0};
    double bias_rad_s_{0.0};
    double yaw_deg_{0.0};
    double rate_dps_{0.0};
    bool valid_{false};
    std::chrono::steady_clock::time_point last_update_{};
};

class OdometryCsvLogger {
public:
    OdometryCsvLogger() = default;

    explicit OdometryCsvLogger(const std::string& path) {
        open(path);
    }

    void open(const std::string& path) {
        if (path.empty()) return;
        file_.open(path, std::ios::out | std::ios::trunc);
        if (!file_) throw std::runtime_error("cannot open odometry log: " + path);
        file_ << "time_s,x_m,y_m,yaw_deg,v_mps,w_dps,fl_rpm,rl_rpm,fr_rpm,rr_rpm\n";
        enabled_ = true;
    }

    void write(double time_s, const Pose2D& pose, const Odometry& odom,
               const std::array<double, 4>& wheel_rpm) {
        if (!enabled_) return;
        file_ << time_s << ','
              << pose.x_m << ','
              << pose.y_m << ','
              << pose.yaw_deg << ','
              << odom.linearSpeedMps() << ','
              << odom.angularSpeedDps() << ','
              << wheel_rpm[0] << ','
              << wheel_rpm[1] << ','
              << wheel_rpm[2] << ','
              << wheel_rpm[3] << '\n';
        if (++pending_flush_ >= flush_every_) {
            file_.flush();
            pending_flush_ = 0;
        }
    }

    void close() {
        if (file_.is_open()) {
            file_.flush();
            file_.close();
        }
        enabled_ = false;
    }

    ~OdometryCsvLogger() { close(); }

private:
    std::ofstream file_;
    bool enabled_{false};
    int pending_flush_{0};
    const int flush_every_{20};
};

struct GearProfile {
    const char* name;
    int run_rpm;
    int turn_max_rpm;
};

GearProfile gearProfileFromName(const std::string& name, const Config& c) {
    if (name == "LOW" || name == "low") {
        return {"LOW", c.gear_low_run_rpm, c.gear_low_turn_max_rpm};
    }
    if (name == "HIGH" || name == "high") {
        return {"HIGH", c.gear_high_run_rpm, c.gear_high_turn_max_rpm};
    }
    return {"MID", c.gear_mid_run_rpm, c.gear_mid_turn_max_rpm};
}

struct AppCommand {
    bool valid{false};
    bool enable{false};
    bool estop{false};
    bool mower_on{false};
    int throttle{0};
    int steering{0};
    std::string gear{"MID"};
    std::chrono::steady_clock::time_point last_update{};
};

class AppControlReceiver {
public:
    AppControlReceiver(bool enabled, int port) : enabled_(enabled) {
        if (!enabled_) return;
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error("cannot create APP UDP socket");
        int opt = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("cannot bind APP UDP port " + std::to_string(port));
        }
        const int flags = ::fcntl(fd_, F_GETFL, 0);
        ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        std::cout << "APP control UDP enabled port=" << port << std::endl;
    }
    ~AppControlReceiver() { if (fd_ >= 0) ::close(fd_); }

    void poll() {
        if (!enabled_ || fd_ < 0) return;
        char buf[512];
        for (;;) {
            const ssize_t n = ::recv(fd_, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break;
            buf[n] = '\0';
            parse(std::string(buf, static_cast<std::size_t>(n)));
        }
    }

    const AppCommand& command() const { return cmd_; }

private:
    static int extractInt(const std::string& msg, const std::string& key, int fallback) {
        auto pos = msg.find(key);
        if (pos == std::string::npos) return fallback;
        pos = msg.find_first_of(":=", pos + key.size());
        if (pos == std::string::npos) return fallback;
        ++pos;
        while (pos < msg.size() && (msg[pos] == ' ' || msg[pos] == '\"')) ++pos;
        try { return std::stoi(msg.substr(pos)); } catch (...) { return fallback; }
    }

    static std::string extractString(const std::string& msg, const std::string& key, const std::string& fallback) {
        auto pos = msg.find(key);
        if (pos == std::string::npos) return fallback;
        pos = msg.find_first_of(":=", pos + key.size());
        if (pos == std::string::npos) return fallback;
        ++pos;
        while (pos < msg.size() && (msg[pos] == ' ' || msg[pos] == '\"')) ++pos;
        std::string out;
        while (pos < msg.size()) {
            const char ch = msg[pos++];
            if (!(std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '-')) break;
            out.push_back(ch);
        }
        return out.empty() ? fallback : out;
    }

    void parse(const std::string& msg) {
        cmd_.enable = extractInt(msg, "enable", cmd_.enable ? 1 : 0) != 0;
        cmd_.estop = extractInt(msg, "estop", cmd_.estop ? 1 : 0) != 0;
        cmd_.mower_on = extractInt(msg, "mower", cmd_.mower_on ? 1 : 0) != 0;
        cmd_.throttle = std::clamp(extractInt(msg, "throttle", cmd_.throttle), -1, 1);
        cmd_.steering = std::clamp(extractInt(msg, "steering", cmd_.steering), -1, 1);
        cmd_.gear = extractString(msg, "gear", cmd_.gear);
        cmd_.valid = true;
        cmd_.last_update = std::chrono::steady_clock::now();
    }

    bool enabled_{false};
    int fd_{-1};
    AppCommand cmd_{};
};

GearProfile gearProfileFromSbus(uint16_t value, const Config& c) {
    if (!c.gear_select_enable) return {"CONFIG", c.run_rpm, c.turn_max_rpm};
    if (value <= c.low_threshold) {
        return {"LOW", c.gear_low_run_rpm, c.gear_low_turn_max_rpm};
    }
    if (value >= c.high_threshold) {
        return {"HIGH", c.gear_high_run_rpm, c.gear_high_turn_max_rpm};
    }
    return {"MID", c.gear_mid_run_rpm, c.gear_mid_turn_max_rpm};
}

class Controller {
public:
    Controller(Config config, CanBus& can, ImuYaw* imu)
        : c_(std::move(config)), can_(can), imu_(imu) {}

    void heartbeatAll() {
        for (auto id : c_.motor_ids) can_.heartbeat(id);
        can_.heartbeat(c_.mower_id);
    }

    void setDriveMaxCurrent(int value_10ma, const char* reason) {
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

    void setWheelFeedback(const std::array<double, 4>& rpm,
                          const std::array<bool, 4>& valid,
                          const std::array<uint64_t, 4>& generation) {
        measured_rpm_ = rpm;
        feedback_valid_ = valid;
        feedback_generation_ = generation;
    }

    void setOdometryY(double y_m) {
        odom_y_m_ = y_m;
        odom_y_valid_ = true;
    }

    const std::array<double, 4>& wheelTargets() const { return ramped_target_rpm_; }
    const std::array<int, 4>& wheelCurrents() const { return wheel_current_10ma_; }

    void setGear(const GearProfile& gear) {
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

    void resetHeadingTarget(const char* reason) {
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

    void stopAll(bool reset_heading_target = false) {
        heading_hold_active_ = false;
        direction_change_hold_active_ = false;
        direction_change_pending_throttle_ = 0;
        direction_change_post_hold_start_ = {};
        direction_change_post_hold_logged_ = false;
        last_motion_throttle_ = 0;
        if (reset_heading_target) resetHeadingTarget("stop_all");
        heading_throttle_ = 0;
        heading_integral_forward_ = 0.0;
        heading_integral_reverse_ = 0.0;
        ramped_target_rpm_.fill(0.0);
        wheel_integral_.fill(0.0);
        wheel_current_10ma_.fill(0);
        for (auto id : c_.motor_ids) can_.current(id, 0);
        can_.current(c_.mower_id, 0);
    }

    void apply(int throttle, int steering, bool mower_on, bool failsafe) {
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
                heading_throttle_ = 0;
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
            heading_throttle_ = 0;
            direction_change_hold_active_ = false;
            direction_change_pending_throttle_ = 0;
            direction_change_post_hold_start_ = {};
            direction_change_post_hold_logged_ = false;
            arc_turn_blend_active_ = false;
            arc_turn_blend_key_ = 0;
            arc_turn_blend_logged_ = false;
            last_motion_throttle_ = 0;
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
            heading_throttle_ = 0;
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
            left_rpm = throttle * c_.run_rpm - applied_correction;
            right_rpm = throttle * c_.run_rpm + applied_correction;
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
            heading_throttle_ = 0;
        } else if (throttle == 0) {
            heading_hold_active_ = false;
            steering_reset_candidate_active_ = false;
            heading_throttle_ = 0;
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

private:

    void maybeResetHeadingForSteering() {
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

    void logStraightness(int throttle, int steering, bool heading_holding,
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

    void applyWheelSpeedControl(const std::array<double, 4>& desired_rpm) {
        const double dt = static_cast<double>(c_.control_ms) / 1000.0;
        const bool stopped = std::all_of(desired_rpm.begin(), desired_rpm.end(),
                                         [](double value) { return std::abs(value) < 0.5; });
        if (stopped) {
            wheel_safety_latched_ = false;
            wheel_safety_reason_.clear();
            wheel_started_.fill(false);
            wheel_start_elapsed_ms_.fill(0);
            wheel_start_confirm_count_.fill(0);
            wheel_stall_count_.fill(0);
        }
        if (!stopped && std::any_of(feedback_valid_.begin(), feedback_valid_.end(),
                                   [](bool valid) { return !valid; })) {
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
                          << (wheel_safety_reason_.empty() ? "startup timeout"
                                                         : wheel_safety_reason_)
                          << "; return throttle to neutral" << std::endl;
                wheel_safety_reported_ = true;
            }
            return;
        }
        wheel_safety_reported_ = false;
        if (c_.wheel_control_mode == "speed") {
            for (std::size_t i = 0; i < 4; ++i) {
                if (std::abs(desired_rpm[i]) < 0.5) {
                    ramped_target_rpm_[i] = 0.0;
                    wheel_integral_[i] = 0.0;
                    wheel_current_10ma_[i] = 0;
                    wheel_started_[i] = false;
                    can_.speed(c_.motor_ids[i], 0);
                    continue;
                }
                if (!feedback_valid_[i]) {
                    ramped_target_rpm_[i] = 0.0;
                    wheel_integral_[i] = 0.0;
                    wheel_current_10ma_[i] = 0;
                    can_.speed(c_.motor_ids[i], 0);
                    continue;
                }
                const bool accelerating = std::abs(desired_rpm[i]) > std::abs(ramped_target_rpm_[i]);
                const double slew = (accelerating ? c_.speed_accel_rpm_s : c_.speed_decel_rpm_s) * dt;
                ramped_target_rpm_[i] += std::clamp(desired_rpm[i] - ramped_target_rpm_[i], -slew, slew);
                const int32_t erpm = static_cast<int32_t>(std::lround(
                    ramped_target_rpm_[i] * c_.pole_pairs * c_.gear_ratio * c_.motor_dirs[i]));
                wheel_current_10ma_[i] = 0;
                wheel_started_[i] = true;
                can_.speed(c_.motor_ids[i], erpm);
            }
            return;
        }
        bool startup_failed = false;
        for (std::size_t i = 0; i < 4; ++i) {
            if (std::abs(desired_rpm[i]) < 0.5) {
                ramped_target_rpm_[i] = 0.0; wheel_integral_[i] = 0.0;
                wheel_current_10ma_[i] = 0; wheel_started_[i] = false;
                wheel_start_elapsed_ms_[i] = 0; can_.current(c_.motor_ids[i], 0); continue;
            }
            if (!feedback_valid_[i]) {
                ramped_target_rpm_[i] = 0.0; wheel_integral_[i] = 0.0;
                wheel_current_10ma_[i] = 0; can_.current(c_.motor_ids[i], 0); continue;
            }
            const bool accelerating = std::abs(desired_rpm[i]) > std::abs(ramped_target_rpm_[i]);
            const double slew = (accelerating ? c_.speed_accel_rpm_s : c_.speed_decel_rpm_s) * dt;
            ramped_target_rpm_[i] += std::clamp(desired_rpm[i] - ramped_target_rpm_[i], -slew, slew);
            const double start_command_rpm = std::min(
                10.0, std::max(2.0, std::abs(desired_rpm[i]) * 0.67));
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
                        wheel_integral_[i] = 0.0;
                    }
                } else if (wheel_start_elapsed_ms_[i] > 0) {
                    if (std::abs(measured_rpm_[i]) >= c_.wheel_start_threshold_rpm)
                        ++wheel_start_confirm_count_[i];
                    else
                        wheel_start_confirm_count_[i] = 0;
                    if (wheel_start_confirm_count_[i] >= c_.wheel_start_confirm_samples) {
                        wheel_started_[i] = true;
                        wheel_integral_[i] = 0.0;
                        wheel_stall_count_[i] = 0;
                    }
                }
            }
            if (!wheel_started_[i]) {
                if (std::abs(desired_rpm[i]) < start_command_rpm) {
                    processed_feedback_generation_[i] = feedback_generation_[i];
                    wheel_start_elapsed_ms_[i] = 0;
                    wheel_start_confirm_count_[i] = 0;
                    wheel_current_10ma_[i] = 0;
                    can_.current(c_.motor_ids[i], 0);
                    continue;
                }
                wheel_start_elapsed_ms_[i] += c_.control_ms;
                if (wheel_start_elapsed_ms_[i] > c_.wheel_start_timeout_ms) startup_failed = true;
                const int steps = wheel_start_elapsed_ms_[i] / c_.wheel_start_step_ms;
                const int start_current = std::min(c_.wheel_start_max_current,
                    c_.wheel_start_initial_current + steps * c_.wheel_start_step_current);
                wheel_current_10ma_[i] = desired_rpm[i] > 0 ? start_current : -start_current;
                can_.current(c_.motor_ids[i], wheel_current_10ma_[i] * c_.motor_dirs[i]);
                continue;
            }
            const double error = ramped_target_rpm_[i] - measured_rpm_[i];
            double next_integral = wheel_integral_[i] + error * dt;
            if (c_.wheel_speed_ki > 0.0) {
                const double lim = static_cast<double>(c_.wheel_integral_max_current) / c_.wheel_speed_ki;
                next_integral = std::clamp(next_integral, -lim, lim);
            } else next_integral = 0.0;
            const double desired_magnitude = std::max(1.0, std::abs(desired_rpm[i]));
            const double feedforward_scale = std::clamp(
                std::abs(ramped_target_rpm_[i]) / desired_magnitude, 0.0, 1.0);
            const double feedforward = c_.wheel_feedforward_current * feedforward_scale;
            const double ff = ramped_target_rpm_[i] >= 0 ? feedforward : -feedforward;
            const double raw = ff + c_.wheel_speed_kp * error + c_.wheel_speed_ki * next_integral;
            const double limited = std::clamp(raw, -static_cast<double>(c_.wheel_max_current), static_cast<double>(c_.wheel_max_current));
            if (std::abs(raw) <= c_.wheel_max_current) wheel_integral_[i] = next_integral;
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

    void logState(int throttle, int steering, bool mower, bool failsafe,
                  int left_rpm, int right_rpm, bool heading_holding,
                  double yaw_deg, double heading_error, int correction) {
        const int correction_bin = heading_holding ? correction / 5 : 999;
        const std::array<int, 5> state{throttle, steering, mower ? 1 : 0,
                                       failsafe ? 1 : 0, correction_bin};
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

    Config c_;
    CanBus& can_;
    ImuYaw* imu_{nullptr};
    int active_closed_loop_max_current_{-1};
    bool heading_hold_active_{false};
    bool heading_target_valid_{false};
    int heading_throttle_{0};
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
    int direction_change_pending_throttle_{0};
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
    int last_motion_throttle_{0};
};
}

int main(int argc, char** argv) {
    try {
        const std::string config_path = argc > 1 ? argv[1] : "config.xml";
        const Config config = loadConfig(config_path);
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        std::cout << "RemoteControl_WheelPID_V2 2.1\n"
                  << "Software four-wheel PI speed loop using OID current control; original binaries are not modified.\n"
                  << "SBUS=" << config.sbus_port << " CAN=" << config.can_port << "\n"
                  << "IDs FL/RL/FR/RR=" << config.motor_ids[0] << "/"
                  << config.motor_ids[1] << "/" << config.motor_ids[2] << "/"
                  << config.motor_ids[3] << "\n"
                  << "Run/rotate wheel RPM=" << config.run_rpm << "/" << config.rotate_rpm << "\n"
                  << "Pole pairs/gear ratio=" << config.pole_pairs << "/"
                  << config.gear_ratio << "\n"
                  << "Accel/decel RPM/s=" << config.speed_accel_rpm_s << "/" << config.speed_decel_rpm_s << "\n"
                  << "Rotate in place=" << (config.rotate_in_place_enable ? 1 : 0)
                  << ", rotate_direction_sign=" << config.rotate_direction_sign
                  << ", neutral_hold_ms=" << config.rotate_neutral_hold_ms
                  << ", speed_guard=" << (config.rotate_speed_guard_enable ? 1 : 0)
                  << ", start_max_rpm=" << config.rotate_start_max_rpm
                  << ", guard_timeout_ms=" << config.rotate_start_guard_timeout_ms << "\n"
                  << "Wheel control mode=" << config.wheel_control_mode << "\n"
                  << "Software wheel PI Kp/Ki=" << config.wheel_speed_kp << "/" << config.wheel_speed_ki << "\n"
                  << "Wheel run/start-max current=" << config.wheel_max_current * 0.01
                  << "/" << config.wheel_start_max_current * 0.01 << " A\n"
                  << "Rotate max current(factory torque)="
                  << config.rotate_max_current * 0.01 << " A\n"
                  << "Turn inner ratio=" << config.turn_inner_ratio << "\n"
                  << "Turn max RPM while arc-turning=" << config.turn_max_rpm << "\n"
                  << "Gear select=" << (config.gear_select_enable ? 1 : 0)
                  << ", channel=" << config.gear_channel
                  << ", low=" << config.gear_low_run_rpm << "/" << config.gear_low_turn_max_rpm
                  << ", mid=" << config.gear_mid_run_rpm << "/" << config.gear_mid_turn_max_rpm
                  << ", high=" << config.gear_high_run_rpm << "/" << config.gear_high_turn_max_rpm << "\n"
                  << "APP control=" << (config.app_control_enable ? 1 : 0)
                  << ", udp_port=" << config.app_udp_port
                  << ", timeout_ms=" << config.app_timeout_ms << "\n"
                  << "Arc turn blend ms=" << config.arc_turn_blend_ms << "\n"
                  << "Wheel radius/track width=" << config.wheel_radius_m
                  << "/" << config.track_width_m << " m" << std::endl;

        CanBus can(config.can_port);
        SbusReader sbus(config.sbus_port);
        ImuYaw imu(config);
        Odometry odom({config.wheel_radius_m, config.track_width_m, config.gear_ratio,
                       config.imu_heading_enable, 0.85});
        OdometryCsvLogger odom_logger;
        if (config.odom_log_enable) {
            odom_logger.open(config.odom_log_path);
            std::cout << "Odometry log=" << config.odom_log_path << std::endl;
        }
        Controller controller(config, can, config.imu_heading_enable ? &imu : nullptr);
        AppControlReceiver app_control(config.app_control_enable, config.app_udp_port);
        controller.stopAll(true);
        controller.heartbeatAll();
        if (config.wheel_control_mode == "speed") {
            const int32_t accel_erpm_s = static_cast<int32_t>(std::lround(
                config.speed_accel_rpm_s * config.pole_pairs * config.gear_ratio));
            const int32_t decel_erpm_s = static_cast<int32_t>(std::lround(
                config.speed_decel_rpm_s * config.pole_pairs * config.gear_ratio));
            for (auto id : config.motor_ids) {
                can.setClosedLoopMaxCurrent(id, config.wheel_max_current);
                can.setAccel(id, accel_erpm_s);
                can.setDecel(id, decel_erpm_s);
                can.speed(id, 0);
            }
            std::cout << "Driver speed-loop mode enabled, accel/decel erpm/s="
                      << accel_erpm_s << "/" << decel_erpm_s << std::endl;
        }
        if (config.imu_heading_enable) {
            std::cout << "IMU calibration: keep robot still for "
                      << config.imu_calibrate_ms << " ms..." << std::endl;
            imu.calibrate();
            std::cout << "IMU heading enabled, bias=" << imu.biasDps()
                      << " deg/s, yaw_sign=" << config.imu_yaw_sign
                      << ", Kp=" << config.heading_kp
                      << ", Ki=" << config.heading_ki
                      << ", Kd=" << config.heading_kd
                      << ", max_correction=" << config.heading_max_correction
                      << ", correction_sign=" << config.heading_correction_sign
                      << ", forward_trim=" << config.heading_forward_trim
                      << ", reverse_trim=" << config.heading_reverse_trim
                      << ", heading_reset_steering_hold_ms=" << config.heading_reset_steering_hold_ms
                      << ", lateral_hold=" << (config.lateral_hold_enable ? 1 : 0)
                      << ", lateral_kp=" << config.lateral_kp_deg_per_m
                      << ", lateral_max=" << config.lateral_max_heading_deg
                      << std::endl;
            odom.reset(0.0, 0.0, imu.yawDeg());
        } else {
            odom.reset(0.0, 0.0, 0.0);
        }
        SbusFrame frame;
        bool have_frame = false;
        std::array<int32_t, 4> speed_erpm{};
        std::array<bool, 4> speed_seen{};
        std::array<double, 4> latest_measured_rpm{};
        std::array<uint64_t, 4> speed_generation{};
        std::array<std::chrono::steady_clock::time_point, 4> speed_update_time{};
        auto last_frame = std::chrono::steady_clock::now();
        auto last_control = last_frame - std::chrono::milliseconds(config.control_ms);
        auto last_heartbeat = last_frame - std::chrono::milliseconds(config.heartbeat_ms);
        auto last_speed_query = last_frame - std::chrono::milliseconds(config.speed_query_ms);
        std::size_t speed_query_index = 0;
        auto last_speed_log = last_frame - std::chrono::milliseconds(config.speed_log_ms);
        auto last_odom_update = last_frame;
        const auto program_start = last_frame;
        bool last_logged_failsafe = false;
        std::string last_logged_failsafe_reason;

        while (running) {
            app_control.poll();
            if (sbus.poll(frame)) {
                have_frame = true;
                last_frame = std::chrono::steady_clock::now();
            }

            uint16_t feedback_id = 0;
            int32_t feedback_erpm = 0;
            while (can.readSpeed(feedback_id, feedback_erpm)) {
                if (feedback_id == 0) continue;
                for (std::size_t i = 0; i < config.motor_ids.size(); ++i) {
                    if (config.motor_ids[i] == feedback_id) {
                        speed_erpm[i] = feedback_erpm;
                        speed_seen[i] = true;
                        ++speed_generation[i];
                        speed_update_time[i] = std::chrono::steady_clock::now();
                    }
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_heartbeat >= std::chrono::milliseconds(config.heartbeat_ms)) {
                controller.heartbeatAll();
                last_heartbeat = now;
            }

            if (now - last_speed_query >= std::chrono::milliseconds(config.speed_query_ms)) {
                can.querySpeed(config.motor_ids[speed_query_index]);
                speed_query_index = (speed_query_index + 1) % config.motor_ids.size();
                last_speed_query = now;
            }

            if (now - last_speed_log >= std::chrono::milliseconds(config.speed_log_ms)) {
                std::cout << "SPEED_RPM";
                const char* names[4]{" FL=", " RL=", " FR=", " RR="};
                for (std::size_t i = 0; i < 4; ++i) {
                    std::cout << names[i];
                    if (speed_seen[i]) {
                        const double rpm = static_cast<double>(speed_erpm[i]) *
                                           config.motor_dirs[i] /
                                           (config.pole_pairs * config.gear_ratio);
                        std::cout << rpm;
                    } else {
                        std::cout << "N/A";
                    }
                }
                const auto& targets = controller.wheelTargets();
                const auto& currents = controller.wheelCurrents();
                std::cout << " TARGET=" << targets[0] << "/" << targets[1] << "/"
                          << targets[2] << "/" << targets[3]
                          << " CURRENT_10mA=" << currents[0] << "/" << currents[1] << "/"
                          << currents[2] << "/" << currents[3] << std::endl;
                const auto pose = odom.pose();
                std::cout << "ODOM x=" << pose.x_m
                          << " y=" << pose.y_m
                          << " yaw=" << pose.yaw_deg
                          << " v=" << odom.linearSpeedMps()
                          << " w=" << odom.angularSpeedDps()
                          << std::endl;
                if (config.odom_log_enable) {
                    const double time_s = std::chrono::duration<double>(now - program_start).count();
                    odom_logger.write(time_s, pose, odom, latest_measured_rpm);
                }
                last_speed_log = now;
            }

            if (now - last_control >= std::chrono::milliseconds(config.control_ms)) {
                if (config.imu_heading_enable) imu.update();
                std::array<double, 4> measured_rpm{};
                std::array<bool, 4> feedback_fresh{};
                for (std::size_t i = 0; i < 4; ++i) {
                    measured_rpm[i] = static_cast<double>(speed_erpm[i]) *
                                      config.motor_dirs[i] /
                                      (config.pole_pairs * config.gear_ratio);
                    feedback_fresh[i] = speed_seen[i] &&
                        now - speed_update_time[i] <=
                            std::chrono::milliseconds(config.speed_feedback_timeout_ms);
                }
                latest_measured_rpm = measured_rpm;
                controller.setWheelFeedback(measured_rpm, feedback_fresh, speed_generation);
                const double odom_dt = std::chrono::duration<double>(now - last_odom_update).count();
                odom.update(measured_rpm, odom_dt, config.imu_heading_enable && imu.valid(), imu.yawDeg());
                last_odom_update = now;
                controller.setOdometryY(odom.pose().y_m);
                const bool timed_out = !have_frame ||
                    now - last_frame > std::chrono::milliseconds(config.failsafe_ms);
                const bool failsafe = timed_out || frame.frame_lost || frame.failsafe;
                std::string failsafe_reason;
                if (timed_out) failsafe_reason += "timeout ";
                if (frame.frame_lost) failsafe_reason += "frame_lost ";
                if (frame.failsafe) failsafe_reason += "receiver_failsafe ";
                if (failsafe_reason.empty()) failsafe_reason = "none";
                if (failsafe && (!last_logged_failsafe || failsafe_reason != last_logged_failsafe_reason)) {
                    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame).count();
                    std::cout << "SBUS_FAILSAFE_REASON " << failsafe_reason
                              << "have_frame=" << (have_frame ? 1 : 0)
                              << " last_frame_age_ms=" << age_ms
                              << " limit_ms=" << config.failsafe_ms
                              << " frame_lost=" << (frame.frame_lost ? 1 : 0)
                              << " receiver_failsafe=" << (frame.failsafe ? 1 : 0)
                              << std::endl;
                    last_logged_failsafe = true;
                    last_logged_failsafe_reason = failsafe_reason;
                } else if (!failsafe) {
                    last_logged_failsafe = false;
                    last_logged_failsafe_reason.clear();
                }
                const auto throttle_value = frame.channel[config.throttle_channel - 1];
                const auto steering_value = frame.channel[config.steering_channel - 1];
                const auto mower_value = frame.channel[config.mower_channel - 1];
                const auto gear_value = config.gear_select_enable ?
                    frame.channel[config.gear_channel - 1] : 0;
                const int rc_throttle = throttleDirection(throttle_value, config);
                const int rc_steering = steeringDirection(steering_value, config);
                const bool rc_mower_on = mower_value > config.high_threshold;
                const bool rc_active = !failsafe && (rc_throttle != 0 || rc_steering != 0 || rc_mower_on);
                const auto& app = app_control.command();
                const bool app_fresh = config.app_control_enable && app.valid &&
                    now - app.last_update <= std::chrono::milliseconds(config.app_timeout_ms);

                if (config.app_control_enable && app.valid && app.estop) {
                    controller.setGear(gearProfileFromName(app.gear, config));
                    controller.apply(0, 0, false, true);
                } else if (rc_active) {
                    controller.setGear(gearProfileFromSbus(gear_value, config));
                    controller.apply(rc_throttle, rc_steering, rc_mower_on, false);
                } else if (app_fresh && app.enable) {
                    controller.setGear(gearProfileFromName(app.gear, config));
                    controller.apply(app.throttle, app.steering, app.mower_on, false);
                } else {
                    controller.setGear(gearProfileFromSbus(gear_value, config));
                    controller.apply(0, 0, false, failsafe && !app_fresh);
                }
                last_control = now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        controller.stopAll(true);
        controller.heartbeatAll();
        std::cout << "Stopped safely." << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
