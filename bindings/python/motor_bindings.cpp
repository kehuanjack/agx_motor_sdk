#include "motor_bindings.hpp"

#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "agx_motor_sdk/can/can_port.hpp"
#include "can_bindings.hpp"
#include "agx_motor_sdk/motor/arm_driver.hpp"

namespace py = pybind11;

namespace agx::motor {
namespace {

double NsToSeconds(uint64_t ns) { return static_cast<double>(ns) * 1e-9; }

/// Keeps the wrapped CanPort alive for Python-bound drivers (non-owning :class:`ArmDriver`).
class BoundArmDriver : public ArmDriver {
 public:
  explicit BoundArmDriver(std::shared_ptr<CanPort> can)
      : ArmDriver(can ? can.get() : nullptr), can_keepalive_(std::move(can)) {}

     ~BoundArmDriver() {
          if (PyGILState_Check()) {
               py::gil_scoped_release release;
               Close();
          } else {
               Close();
          }
     }

  void AttachCanPortBinding(std::shared_ptr<CanPort> can) {
          auto previous = can_keepalive_;
          {
               py::gil_scoped_release release;
               if (previous) previous->SetReceiveHandler(nullptr);
          }
    can_keepalive_ = std::move(can);
          {
               py::gil_scoped_release release;
               AttachCanPort(can_keepalive_.get());
          }
  }

 private:
  std::shared_ptr<CanPort> can_keepalive_;
};

}  // namespace

void BindMotor(py::module& m) {
  py::class_<DriverStatusBits>(m, "DriverStatus")
      .def_readonly("undervoltage", &DriverStatusBits::undervoltage)
      .def_readonly("motor_overtemp", &DriverStatusBits::motor_overtemp)
      .def_readonly("driver_overcurrent", &DriverStatusBits::driver_overcurrent)
      .def_readonly("driver_overtemp", &DriverStatusBits::driver_overtemp)
      .def_readonly("collision_tripped", &DriverStatusBits::collision_tripped)
      .def_readonly("driver_error", &DriverStatusBits::driver_error)
      .def_readonly("enabled", &DriverStatusBits::enabled)
      .def_readonly("stall_tripped", &DriverStatusBits::stall_tripped);

  py::class_<HighSpeedState>(m, "HighSpeedFeedback")
      .def_readonly("position", &HighSpeedState::position)
      .def_readonly("velocity", &HighSpeedState::velocity)
      .def_readonly("current", &HighSpeedState::current)
      .def_property_readonly("timestamp",
                             [](const HighSpeedState& self) { return NsToSeconds(self.timestamp_ns); })
      .def_readonly("timestamp_ns", &HighSpeedState::timestamp_ns);

  py::class_<LowSpeedState>(m, "LowSpeedFeedback")
      .def_readonly("bus_voltage_v", &LowSpeedState::bus_voltage_v)
      .def_readonly("driver_temp_deg", &LowSpeedState::driver_temp_deg)
      .def_readonly("motor_temp_deg", &LowSpeedState::motor_temp_deg)
      .def_readonly("bus_current", &LowSpeedState::bus_current)
      .def_readonly("status_raw", &LowSpeedState::status_raw)
      .def_readonly("status", &LowSpeedState::status)
      .def_property_readonly("timestamp",
                             [](const LowSpeedState& self) { return NsToSeconds(self.timestamp_ns); })
      .def_readonly("timestamp_ns", &LowSpeedState::timestamp_ns);

  py::class_<VersionInfo>(m, "VersionInfo")
      .def_readonly("software", &VersionInfo::software)
      .def_readonly("hardware", &VersionInfo::hardware)
      .def_readonly("motor", &VersionInfo::motor)
      .def_property_readonly("timestamp",
                             [](const VersionInfo& self) { return NsToSeconds(self.timestamp_ns); })
      .def_readonly("timestamp_ns", &VersionInfo::timestamp_ns);

     py::class_<ArmDriver, std::shared_ptr<ArmDriver>> arm_driver(m, "ArmDriver");
  py::enum_<ArmDriver::MotorParam>(arm_driver, "MotorParam")
      .value("ID", ArmDriver::MotorParam::kId)
      .value("AC", ArmDriver::MotorParam::kAc)
      .value("DC", ArmDriver::MotorParam::kDc)
      .value("VV", ArmDriver::MotorParam::kVv)
      .value("IQ", ArmDriver::MotorParam::kIq)
      .value("OI", ArmDriver::MotorParam::kOi)
      .value("OT", ArmDriver::MotorParam::kOt)
      .value("TX", ArmDriver::MotorParam::kTx)
      .value("TC", ArmDriver::MotorParam::kTc)
      .value("TF", ArmDriver::MotorParam::kTf)
      .value("SO", ArmDriver::MotorParam::kSo)
      .value("PP", ArmDriver::MotorParam::kPp)
      .value("KP", ArmDriver::MotorParam::kKp)
      .value("KI", ArmDriver::MotorParam::kKi);

  arm_driver
      .def(py::init([](py::object can_port) {
             return std::make_shared<BoundArmDriver>(WrapCanPortObject(std::move(can_port)));
           }),
           py::arg("can_port") = py::none())
     .def("close", &ArmDriver::Close, py::call_guard<py::gil_scoped_release>())
      .def("attach_can_port",
                          [](ArmDriver& driver, py::object can_port) {
                               static_cast<BoundArmDriver&>(driver).AttachCanPortBinding(
                                         WrapCanPortObject(std::move(can_port)));
           },
           py::arg("can_port"))
      .def(
          "handle_rx_once",
          [](ArmDriver& driver, uint32_t can_id, py::bytes payload, uint64_t timestamp_ns) {
            CanFrame frame;
            frame.id = can_id;
            const std::string data = payload;
            frame.data.assign(data.begin(), data.end());
            frame.timestamp_ns = timestamp_ns;
            return driver.HandleRxOnce(frame);
          },
          py::arg("can_id"), py::arg("data"), py::arg("timestamp_ns") = 0)
      .def("get_high_speed_feedback", &ArmDriver::GetHighSpeedState, py::arg("node_id"))
      .def("get_low_speed_feedback", &ArmDriver::GetLowSpeedState, py::arg("node_id"))
      .def("get_version", &ArmDriver::GetVersion, py::arg("node_id"), py::arg("timeout") = 1.0f)
      .def("set_enable", &ArmDriver::SetEnable, py::arg("node_id"), py::arg("enable") = true,
           py::arg("has_brake") = false, py::arg("brake_on") = false)
      .def("set_target", &ArmDriver::SetTarget, py::arg("node_id"), py::arg("value"),
           py::arg("mode"), py::arg("timeout") = 0.0f,
           "Generic target command; mode is protocol-defined (e.g. 0=vel, 1=pos, 0xFF=unchanged).")
      .def("set_target_velocity", &ArmDriver::SetTargetVelocity, py::arg("node_id"),
           py::arg("velocity"), py::arg("timeout") = 0.0f)
      .def("set_target_position", &ArmDriver::SetTargetPosition, py::arg("node_id"),
           py::arg("position"), py::arg("timeout") = 0.0f)
      .def("set_target_current", &ArmDriver::SetTargetCurrent, py::arg("node_id"),
           py::arg("current"), py::arg("timeout") = 0.0f)
      .def("set_reset", &ArmDriver::SetReset, py::arg("node_id"), py::arg("mode") = 0,
           py::arg("timeout") = 1.0f)
      .def("set_clear_error", &ArmDriver::SetClearError, py::arg("node_id"),
           py::arg("timeout") = 1.0f)
      .def("set_zero_offset", &ArmDriver::SetZeroOffset, py::arg("node_id"),
           py::arg("zero_offset") = 0.0f, py::arg("save_to_flash") = false,
           py::arg("timeout") = 1.0f)
      .def("set_profile_acc_dec", &ArmDriver::SetProfileAccDec, py::arg("node_id"),
           py::arg("acceleration"), py::arg("deceleration"))
      .def("set_profile_vel", &ArmDriver::SetProfileVelocity, py::arg("node_id"),
           py::arg("velocity"))
      .def("set_current_limit", &ArmDriver::SetCurrentLimit, py::arg("node_id"),
           py::arg("current"))
      .def("set_mit_control", &ArmDriver::SetMitControl, py::arg("node_id"), py::arg("p_des"),
           py::arg("v_des"), py::arg("kp"), py::arg("kd"), py::arg("t_ff"),
           py::arg("t_am") = 16.0f)
      .def("set_mit_control_crc", &ArmDriver::SetMitControlCrc, py::arg("node_id"),
           py::arg("p_des"), py::arg("v_des"), py::arg("kp"), py::arg("kd"), py::arg("t_ff"),
           py::arg("t_am") = 8.0f)
      .def("set_collision_threshold", &ArmDriver::SetCollisionThreshold, py::arg("node_id"),
           py::arg("current_threshold"), py::arg("time_threshold"))
      .def("get_param", &ArmDriver::GetParam, py::arg("node_id"), py::arg("param"),
           py::arg("timeout") = 1.0f)
      .def("set_param", &ArmDriver::SetParam, py::arg("node_id"), py::arg("param"),
           py::arg("value"), py::arg("timeout") = 1.0f);
}

}  // namespace agx::motor
