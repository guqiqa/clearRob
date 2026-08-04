// pybind_module.cpp — pybind11 binding exposing the C++ chassis core to the
// Python ROS2 shell (chassis_driver).
//
// Exposes a single `ChassisCore` class:
//   core = ChassisCore(config_dict)
//   currents, safety, reason = core.tick(throttle, steering, gear, mower,
//                                        estop, wheel_rpm, wheel_valid,
//                                        wheel_gen, imu_yaw_deg, imu_rate_dps)
//   core.set_gear("MID")          /  core.stop_all()  /  core.reset_safety()
//   core.set_imu(yaw_deg, rate_dps, valid)
//   core.set_wheel_feedback(rpm, valid, gen)
//
// tick() returns the per-motor CAN current commands (10mA units, motor_dir
// already applied) in motor order FL,RL,FR,RR — the Python shell sends these
// over python-can.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "chassis_core/chassis_core.hpp"

namespace py = pybind11;

namespace {

chassis_core::ParamValue toParamValue(const py::object& v) {
    if (py::isinstance<py::bool_>(v)) return chassis_core::ParamValue(v.cast<bool>());
    if (py::isinstance<py::int_>(v)) return chassis_core::ParamValue(v.cast<int>());
    if (py::isinstance<py::float_>(v)) return chassis_core::ParamValue(v.cast<double>());
    if (py::isinstance<py::str>(v)) return chassis_core::ParamValue(v.cast<std::string>());
    if (py::isinstance<py::list>(v) || py::isinstance<py::tuple>(v)) {
        std::vector<int> arr;
        for (py::handle item : v) arr.push_back(py::cast<int>(item));
        return chassis_core::ParamValue(arr);
    }
    throw std::runtime_error("chassis_core: unsupported config value type");
}

}  // namespace

class ChassisCore {
public:
    explicit ChassisCore(const py::dict& params) {
        std::unordered_map<std::string, chassis_core::ParamValue> cfg;
        for (auto item : params) {
            const std::string key = py::cast<std::string>(item.first);
            cfg[key] = toParamValue(py::reinterpret_borrow<py::object>(item.second));
        }
        config_ = chassis_core::configFromDict(cfg);
        imu_ = std::make_unique<chassis_core::ImuYaw>();
        ctrl_ = std::make_unique<chassis_core::Controller>(
            config_, can_, config_.imu_heading_enable ? imu_.get() : nullptr);
    }

    // One control tick (20 Hz).  Returns (currents[4], safety_latched, reason).
    py::tuple tick(double throttle, double steering, const std::string& gear,
                   bool mower_on, bool estop,
                   const std::vector<double>& wheel_rpm,
                   const std::vector<bool>& wheel_valid,
                   const std::vector<uint64_t>& wheel_gen,
                   double imu_yaw_deg, double imu_yaw_rate_dps) {
        imu_->setYawDeg(imu_yaw_deg);
        imu_->setRateDps(imu_yaw_rate_dps);
        if (config_.imu_heading_enable) imu_->setValid(true);

        std::array<double, 4> rpm{};
        std::array<bool, 4> valid{};
        std::array<uint64_t, 4> gen{};
        for (int i = 0; i < 4 && i < static_cast<int>(wheel_rpm.size()); ++i) rpm[i] = wheel_rpm[i];
        for (int i = 0; i < 4 && i < static_cast<int>(wheel_valid.size()); ++i) valid[i] = wheel_valid[i];
        for (int i = 0; i < 4 && i < static_cast<int>(wheel_gen.size()); ++i) gen[i] = wheel_gen[i];
        ctrl_->setWheelFeedback(rpm, valid, gen);

        ctrl_->setGear(chassis_core::gearProfileFromName(gear, config_));
        ctrl_->apply(throttle, steering, mower_on, estop);

        return py::make_tuple(currentsList(), ctrl_->safetyLatched(),
                              ctrl_->safetyReason());
    }

    void set_gear(const std::string& gear) {
        ctrl_->setGear(chassis_core::gearProfileFromName(gear, config_));
    }

    // Hard stop: zero all state, returns zeroed current commands.
    py::list stop_all() {
        ctrl_->stopAll(true);
        return currentsList();
    }

    // Clear the wheel safety latch by bringing the wheels to a stopped state.
    void reset_safety() {
        imu_->setYawDeg(0.0);
        imu_->setRateDps(0.0);
        ctrl_->apply(0.0, 0.0, false, false);
    }

    void set_imu(double yaw_deg, double rate_dps, bool valid) {
        imu_->setYawDeg(yaw_deg);
        imu_->setRateDps(rate_dps);
        imu_->setValid(valid);
    }

    void set_wheel_feedback(const std::vector<double>& wheel_rpm,
                            const std::vector<bool>& wheel_valid,
                            const std::vector<uint64_t>& wheel_gen) {
        std::array<double, 4> rpm{};
        std::array<bool, 4> valid{};
        std::array<uint64_t, 4> gen{};
        for (int i = 0; i < 4 && i < static_cast<int>(wheel_rpm.size()); ++i) rpm[i] = wheel_rpm[i];
        for (int i = 0; i < 4 && i < static_cast<int>(wheel_valid.size()); ++i) valid[i] = wheel_valid[i];
        for (int i = 0; i < 4 && i < static_cast<int>(wheel_gen.size()); ++i) gen[i] = wheel_gen[i];
        ctrl_->setWheelFeedback(rpm, valid, gen);
    }

    py::list wheel_currents() const { return currentsList(); }
    py::list wheel_targets() const {
        const auto& t = ctrl_->wheelTargets();
        py::list out;
        for (double v : t) out.append(v);
        return out;
    }
    py::dict last_currents() const {
        py::dict d;
        for (auto& kv : can_.currents()) d[py::cast(kv.first)] = kv.second;
        return d;
    }

private:
    py::list currentsList() const {
        py::list out;
        for (auto id : config_.motor_ids) {
            const auto& m = can_.currents();
            out.append(m.count(id) ? m.at(id) : 0);
        }
        return out;
    }

    chassis_core::Config config_;
    chassis_core::CanBus can_;
    std::unique_ptr<chassis_core::ImuYaw> imu_;
    std::unique_ptr<chassis_core::Controller> ctrl_;
};

PYBIND11_MODULE(chassis_core, m) {
    m.doc() = "Field-tuned C++ chassis control core (colleague WheelPID V2) "
              "embedded in ROS2 via pybind11.";
    py::class_<ChassisCore>(m, "ChassisCore")
        .def(py::init<const py::dict&>(), py::arg("params"))
        .def("tick", &ChassisCore::tick,
             py::arg("throttle"), py::arg("steering"), py::arg("gear"),
             py::arg("mower_on"), py::arg("estop"),
             py::arg("wheel_rpm"), py::arg("wheel_valid"), py::arg("wheel_gen"),
             py::arg("imu_yaw_deg"), py::arg("imu_yaw_rate_dps"))
        .def("set_gear", &ChassisCore::set_gear, py::arg("gear"))
        .def("stop_all", &ChassisCore::stop_all)
        .def("reset_safety", &ChassisCore::reset_safety)
        .def("set_imu", &ChassisCore::set_imu,
             py::arg("yaw_deg"), py::arg("rate_dps"), py::arg("valid"))
        .def("set_wheel_feedback", &ChassisCore::set_wheel_feedback,
             py::arg("wheel_rpm"), py::arg("wheel_valid"), py::arg("wheel_gen"))
        .def("wheel_currents", &ChassisCore::wheel_currents)
        .def("wheel_targets", &ChassisCore::wheel_targets)
        .def("last_currents", &ChassisCore::last_currents);
}
