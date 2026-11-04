#include <memory>
#include <unordered_map>
#include <utility>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "agx_motor_sdk/mechanism/arm/arm_motor.hpp"
#include "agx_motor_sdk/mechanism/config.hpp"
#include "agx_motor_sdk/mechanism/joint_group.hpp"
#include "agx_motor_sdk/mechanism/motor_registry.hpp"
#include "agx_motor_sdk/mechanism/types.hpp"
#include "agx_motor_sdk/mechanism/motor_base.hpp"
#include "agx_motor_sdk/mechanism/rb200/rb200_motor.hpp"
#include "agx_motor_sdk/motor/arm/arm_driver.hpp"
#include "agx_motor_sdk/motor/motor_driver.hpp"
#include "can_bindings.hpp"

namespace py = pybind11;

namespace agx::motor::mechanism {
namespace {

void DeleteJointGroup(JointGroup* group) {
  if (PyGILState_Check()) {
    {
      py::gil_scoped_release release;
      group->Close();
    }
    delete group;
  } else {
    group->Close();
    delete group;
  }
}

template <typename... Args>
std::shared_ptr<JointGroup> MakeJointGroup(Args&&... args) {
  return std::shared_ptr<JointGroup>(new JointGroup(std::forward<Args>(args)...),
                                     DeleteJointGroup);
}

template <typename Holder>
void BindJointGroupCommon(py::class_<JointGroup, Holder>& cls) {
  using JointGroupT = JointGroup;
    cls.def("connect", &JointGroupT::Connect, py::call_guard<py::gil_scoped_release>(),
      "Acquire shared CAN backend(s).")
      .def("close", &JointGroupT::Close, py::call_guard<py::gil_scoped_release>(),
         "Release module references to shared CAN backend(s).")
      .def("is_connected", &JointGroupT::IsConnected)
      .def("enable", &JointGroupT::Enable, py::arg("interval") = 0.0,
           py::call_guard<py::gil_scoped_release>(),
           "Enable all joints in this module, waiting ``interval`` seconds between joints.")
      .def("disable", &JointGroupT::Disable, py::arg("interval") = 0.0,
           py::call_guard<py::gil_scoped_release>(),
           "Disable all joints in this module, waiting ``interval`` seconds between joints.")
      .def("configure", &JointGroupT::Configure,
          "Apply YAML profile / MIT parameters to all joints.")
      .def_property_readonly("joint_count", &JointGroupT::JointCount)
      .def_property_readonly("joint_names", &JointGroupT::JointNames)
      .def_property_readonly("protocol",
                             [](const JointGroupT& self) {
                               return ProtocolToString(self.protocol());
                             })
      .def_property_readonly("can_channel", &JointGroupT::can_channel)
      .def_property_readonly("can_channels", &JointGroupT::CanChannels)
      .def_property_readonly("config",
                             [](const JointGroupT& self) -> const GroupConfig& { return self.config(); },
                             py::return_value_policy::reference_internal)
      .def(
          "index_of",
          &JointGroupT::IndexOf,
          py::arg("joint_name"),
          "Return joint index, or JointGroup.INVALID_JOINT_INDEX (-1) if unknown.")
      .def("is_valid_joint_index", &JointGroupT::IsValidJointIndex, py::arg("index"))
      .def("node_id", &JointGroupT::NodeId, py::arg("index"))
      .def("joint_can_channel", &JointGroupT::JointCanChannel, py::arg("index"))
      .def(
          "joint_config",
          [](const JointGroupT& group, std::size_t index) -> const JointConfig& {
            return group.GetJointConfig(index);
          },
          py::arg("index"), py::return_value_policy::reference_internal)
      .def(
          "set_target_position",
          [](JointGroupT& group, int index, float position_rad, float timeout) {
            return group.SetTargetPosition(static_cast<std::size_t>(index), position_rad, timeout);
          },
          py::arg("index"), py::arg("position_rad"), py::arg("timeout") = 0.0f,
          "Joint-space position command [rad] with limits and polarity applied.")
      .def(
          "get_state",
          [](const JointGroupT& group, int index) -> std::optional<JointState> {
            return group.GetState(static_cast<std::size_t>(index));
          },
          py::arg("index"))
      .def(
          "attach_can_port",
          [](JointGroupT& group, py::object can_port, const py::object& channel) {
            auto wrapped = WrapCanPortObject(std::move(can_port));
            if (channel.is_none()) {
              group.AttachCanPort(wrapped);
              return;
            }
            group.AttachCanPort(channel.cast<std::string>(), wrapped);
          },
          py::arg("can_port"), py::arg("channel") = py::none());
}

void BindMotorBase(py::class_<MotorBase>& cls) {
  cls.def_property_readonly("is_valid", &MotorBase::IsValid)
      .def_property_readonly("protocol",
                             [](const MotorBase& self) { return ProtocolToString(self.protocol()); })
      .def_property_readonly("node_id", &MotorBase::NodeId)
      .def_property_readonly("can_channel", &MotorBase::CanChannel)
      .def_property_readonly("name", &MotorBase::Name)
      .def_property_readonly("config", &MotorBase::Config, py::return_value_policy::reference_internal)
      .def("set_enable", &MotorBase::SetEnable, py::arg("enable") = true,
           py::arg("has_brake") = false, py::arg("brake_on") = false)
      .def("get_state", &MotorBase::GetState, "Read joint-space feedback.")
      .def("configure", &MotorBase::Configure, "Apply YAML profile (arm: also mit_tf).")
      .def("get_high_speed_feedback", &MotorBase::GetHighSpeedState,
           "Cached joint-space position [rad], velocity [rad/s], current [A].")
      .def("get_version", &MotorBase::GetVersion, py::arg("timeout") = 1.0f)
      .def("set_target", &MotorBase::SetTarget, py::arg("value"), py::arg("mode"),
           py::arg("timeout") = 0.0f,
           "Joint-space target; mode is protocol-defined (e.g. 1=pos, 0xFF=unchanged).")
      .def("set_target_position", &MotorBase::SetTargetPosition, py::arg("position"),
           py::arg("timeout") = 0.0f,
           "Joint-space position command [rad] with limits and polarity applied.")
      .def("set_target_velocity", &MotorBase::SetTargetVelocity, py::arg("velocity"),
           py::arg("timeout") = 0.0f,
           "Joint-space velocity command [rad/s] with polarity applied.")
      .def("set_reset", &MotorBase::SetReset, py::arg("mode") = 0, py::arg("timeout") = 1.0f)
      .def("set_clear_error", &MotorBase::SetClearError, py::arg("timeout") = 1.0f)
      .def("set_zero_offset", &MotorBase::SetZeroOffset, py::arg("zero_offset") = 0.0f,
           py::arg("save_to_flash") = false, py::arg("timeout") = 1.0f)
      .def("set_profile_acc_dec", &MotorBase::SetProfileAccDec, py::arg("acceleration"),
           py::arg("deceleration"))
      .def("set_profile_vel", &MotorBase::SetProfileVelocity, py::arg("velocity"))
      .def("set_current_limit", &MotorBase::SetCurrentLimit, py::arg("current"));
}

void BindArmMotor(py::class_<ArmMotor, MotorBase>& cls) {
  cls.def("set_mit", &ArmMotor::SetMit, py::arg("position_rad"), py::arg("velocity_rad_s"),
          py::arg("torque_nm"), "MIT command in joint space using YAML mit_* gains.")
      .def("get_low_speed_feedback", &ArmMotor::GetLowSpeedState)
      .def("set_target_current", &ArmMotor::SetTargetCurrent, py::arg("current"),
           py::arg("timeout") = 0.0f)
      .def("set_mit_control", &ArmMotor::SetMitControl, py::arg("p_des"), py::arg("v_des"),
           py::arg("kp"), py::arg("kd"), py::arg("t_ff"), py::arg("t_am") = 16.0f,
           "Low-level MIT in joint space (limits + polarity applied before transmit).")
      .def("set_mit_control_crc", &ArmMotor::SetMitControlCrc, py::arg("p_des"), py::arg("v_des"),
           py::arg("kp"), py::arg("kd"), py::arg("t_ff"), py::arg("t_am") = 8.0f,
           "Low-level MIT with CRC in joint space (limits + polarity applied before transmit).")
      .def("set_collision_threshold", &ArmMotor::SetCollisionThreshold,
           py::arg("current_threshold"), py::arg("time_threshold"))
      .def("get_param", &ArmMotor::GetParam, py::arg("param"), py::arg("timeout") = 1.0f)
      .def("set_param", &ArmMotor::SetParam, py::arg("param"), py::arg("value"),
           py::arg("timeout") = 1.0f);
}

}  // namespace

void BindMechanism(py::module& m) {
  py::class_<JointState>(m, "JointState", "Joint-space feedback (polarity applied).")
      .def(py::init<>())
      .def_readwrite("position_rad", &JointState::position_rad)
      .def_readwrite("velocity_rad_s", &JointState::velocity_rad_s)
      .def_readwrite("torque_nm", &JointState::torque_nm);

  py::class_<JointConfig>(m, "JointConfig", "Per-joint YAML entry (see YAML_CONFIG.md).")
      .def_property_readonly("name", [](const JointConfig& self) { return self.name; })
      .def_property_readonly("node_id", [](const JointConfig& self) { return self.node_id; })
      .def_property_readonly("polarity", [](const JointConfig& self) { return self.polarity; })
      .def_property_readonly("profile_vel_rad_s",
                             [](const JointConfig& self) { return self.profile_vel_rad_s; })
      .def_property_readonly("profile_accel_rad_s2",
                             [](const JointConfig& self) { return self.profile_accel_rad_s2; })
      .def_property_readonly("profile_decel_rad_s2",
                             [](const JointConfig& self) { return self.profile_decel_rad_s2; })
      .def_property_readonly("mit_tf", [](const JointConfig& self) { return self.mit_tf; })
      .def_property_readonly("mit_kp", [](const JointConfig& self) { return self.mit_kp; })
      .def_property_readonly("mit_kd", [](const JointConfig& self) { return self.mit_kd; })
      .def_property_readonly("torque_k", [](const JointConfig& self) { return self.torque_k; })
      .def_property_readonly("torque_b", [](const JointConfig& self) { return self.torque_b; })
      .def_property_readonly("lower_limit_rad",
                             [](const JointConfig& self) { return self.lower_limit_rad; })
      .def_property_readonly("upper_limit_rad",
                             [](const JointConfig& self) { return self.upper_limit_rad; })
      .def_property_readonly("has_lower_limit",
                             [](const JointConfig& self) { return self.has_lower_limit; })
      .def_property_readonly("has_upper_limit",
                             [](const JointConfig& self) { return self.has_upper_limit; })
      .def_property_readonly("can_channel",
                             [](const JointConfig& self) { return self.can_channel; })
      .def_property_readonly("protocol",
                             [](const JointConfig& self) { return ProtocolToString(self.protocol); });

  py::class_<GroupConfig>(m, "GroupConfig", "One YAML module block (see YAML_CONFIG.md).")
      .def_property_readonly("can_channel", [](const GroupConfig& self) { return self.can_channel; })
      .def_property_readonly("protocol",
                             [](const GroupConfig& self) { return ProtocolToString(self.protocol); })
      .def_property_readonly("default_profile_vel_rad_s",
                             [](const GroupConfig& self) { return self.default_profile_vel_rad_s; })
      .def_property_readonly("profile_accel_rad_s2",
                             [](const GroupConfig& self) { return self.profile_accel_rad_s2; })
      .def_property_readonly("profile_decel_rad_s2",
                             [](const GroupConfig& self) { return self.profile_decel_rad_s2; })
      .def_property_readonly("mit_tf", [](const GroupConfig& self) { return self.mit_tf; })
      .def_property_readonly("mit_kp", [](const GroupConfig& self) { return self.mit_kp; })
      .def_property_readonly("mit_kd", [](const GroupConfig& self) { return self.mit_kd; })
      .def_property_readonly("torque_k", [](const GroupConfig& self) { return self.torque_k; })
      .def_property_readonly("torque_b", [](const GroupConfig& self) { return self.torque_b; })
      .def_property_readonly("joints", [](const GroupConfig& self) { return self.joints; });

  auto motor_base = py::class_<MotorBase>(m, "MotorBase",
                                          "Common per-joint motor interface (real or no-op).");
  BindMotorBase(motor_base);
  auto arm_motor = py::class_<ArmMotor, MotorBase>(m, "ArmMotor", "Arm-protocol joint motor.");
  BindArmMotor(arm_motor);
  py::class_<Rb200Motor, MotorBase>(m, "Rb200Motor", "RB200-protocol joint motor.")
      .def("get_low_speed_feedback", &Rb200Motor::GetLowSpeedState);

  auto driver_owners = std::make_shared<
      std::unordered_map<MotorBase*, std::shared_ptr<agx::motor::MotorDriver>>>();
  py::class_<MotorRegistry>(m, "MotorRegistry", "Global shared joint-motor pool.")
      .def_static(
          "acquire",
          [driver_owners](const std::string& channel,
                          std::shared_ptr<agx::motor::MotorDriver> driver, uint8_t node_id,
                          const JointConfig& config) -> MotorBase& {
            if (!driver) throw std::invalid_argument("driver must not be None");
            auto& motor = MotorRegistry::Default().Acquire(channel, driver, node_id, config);
            (*driver_owners)[&motor] = std::move(driver);
            return motor;
          },
          py::arg("can_channel"), py::arg("driver"), py::arg("node_id"), py::arg("config"),
          py::return_value_policy::reference)
      .def_static("clear", [driver_owners]() {
        {
          py::gil_scoped_release release;
          MotorRegistry::Default().Clear();
        }
        driver_owners->clear();
      });

  auto group = py::class_<JointGroup, std::shared_ptr<JointGroup>>(m, "JointGroup");
  group.attr("INVALID_JOINT_INDEX") = kInvalidJointIndex;
  BindJointGroupCommon(group);
  group
      .def(
          "motor",
          [](JointGroup& self, int index) -> MotorBase& { return self.Motor(index); },
          py::arg("index"), py::return_value_policy::reference,
          "Return per-joint motor (ArmMotor / Rb200Motor); invalid index yields a no-op "
          "MotorBase (is_valid=False).")
      .def("set_mit", &JointGroup::SetMit, py::arg("index"), py::arg("position_rad"),
           py::arg("velocity_rad_s"), py::arg("torque_nm"))
      .def(py::init([](const GroupConfig& config) { return MakeJointGroup(config); }),
           py::arg("config"))
      .def(py::init([](const GroupConfig& config, std::string can_channel) {
             return MakeJointGroup(config, std::move(can_channel));
           }),
           py::arg("config"), py::arg("can_channel"))
      .def_static("from_yaml",
                  [](const std::string& yaml_path, const std::string& module_key,
                     const py::object& can_channel) {
                    if (can_channel.is_none()) {
                      return MakeJointGroup(JointGroup::FromYaml(yaml_path, module_key));
                    }
                    return MakeJointGroup(JointGroup::FromYaml(
                        yaml_path, module_key, can_channel.cast<std::string>()));
                  },
                  py::arg("yaml_path"), py::arg("module_key"),
                  py::arg("can_channel") = py::none());

  m.def("create_joint_group",
        [](const GroupConfig& config) { return MakeJointGroup(config); }, py::arg("config"));
  m.def("create_joint_group",
        [](const GroupConfig& config, std::string can_channel) {
          return MakeJointGroup(config, std::move(can_channel));
        },
        py::arg("config"), py::arg("can_channel"));
  m.def("load_module_config", &LoadModuleConfig, py::arg("yaml_path"), py::arg("module_key"));
}

}  // namespace agx::motor::mechanism
