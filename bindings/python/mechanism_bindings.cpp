#include <memory>
#include <unordered_map>
#include <utility>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "agx_motor_sdk/mechanism/arm_motor.hpp"
#include "agx_motor_sdk/mechanism/config.hpp"
#include "agx_motor_sdk/mechanism/joint_group.hpp"
#include "agx_motor_sdk/mechanism/motor_registry.hpp"
#include "agx_motor_sdk/mechanism/types.hpp"
#include "agx_motor_sdk/motor/arm_driver.hpp"
#include "can_bindings.hpp"

namespace py = pybind11;

namespace agx::motor::mechanism {
namespace {

void DeleteArmJointGroup(ArmJointGroup* group) {
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
std::shared_ptr<ArmJointGroup> MakeArmJointGroup(Args&&... args) {
  return std::shared_ptr<ArmJointGroup>(
      new ArmJointGroup(std::forward<Args>(args)...), DeleteArmJointGroup);
}

template <typename JointGroupT, typename Holder>
void BindJointGroupCommon(py::class_<JointGroupT, JointGroupBase, Holder>& cls) {
    cls.def("connect", &JointGroupT::Connect, py::call_guard<py::gil_scoped_release>(),
      "Acquire shared CAN backend(s).")
      .def("close", &JointGroupT::Close, py::call_guard<py::gil_scoped_release>(),
         "Release module references to shared CAN backend(s).")
      .def("is_connected", &JointGroupT::IsConnected)
      .def("enable", &JointGroupT::Enable, "Enable all joints in this module.")
      .def("disable", &JointGroupT::Disable, "Disable all joints in this module.")
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
          "Return joint index, or JointGroupBase.INVALID_JOINT_INDEX (-1) if unknown.")
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

void BindArmMotorBase(py::class_<ArmMotorBase>& cls) {
  cls.def_property_readonly("is_valid", &ArmMotorBase::IsValid)
      .def_property_readonly("protocol",
                            [](const ArmMotorBase& self) { return ProtocolToString(self.protocol()); })
      .def_property_readonly("node_id", &ArmMotorBase::NodeId)
      .def_property_readonly("can_channel", &ArmMotorBase::CanChannel)
      .def_property_readonly("name", &ArmMotorBase::Name)
      .def_property_readonly("config", &ArmMotorBase::Config, py::return_value_policy::reference_internal)
      .def("set_enable", &ArmMotorBase::SetEnable, py::arg("enable") = true,
           py::arg("has_brake") = false, py::arg("brake_on") = false)
      .def("get_state", &ArmMotorBase::GetState, "Read joint-space feedback.")
      .def("set_mit", &ArmMotorBase::SetMit, py::arg("position_rad"), py::arg("velocity_rad_s"),
           py::arg("torque_nm"), "MIT command in joint space using YAML mit_* gains.")
      .def("configure", &ArmMotorBase::Configure, "Apply YAML profile / mit_tf to this motor.")
      .def("get_high_speed_feedback", &ArmMotorBase::GetHighSpeedState,
           "Cached joint-space position [rad], velocity [rad/s], current [A].")
      .def("get_low_speed_feedback", &ArmMotorBase::GetLowSpeedState)
      .def("get_version", &ArmMotorBase::GetVersion, py::arg("timeout") = 1.0f)
      .def("set_target", &ArmMotorBase::SetTarget, py::arg("value"), py::arg("mode"),
           py::arg("timeout") = 0.0f,
           "Joint-space target; mode is protocol-defined (e.g. 1=pos, 0xFF=unchanged).")
      .def("set_target_position", &ArmMotorBase::SetTargetPosition, py::arg("position"),
           py::arg("timeout") = 0.0f,
           "Joint-space position command [rad] with limits and polarity applied.")
      .def("set_target_velocity", &ArmMotorBase::SetTargetVelocity, py::arg("velocity"),
           py::arg("timeout") = 0.0f,
           "Joint-space velocity command [rad/s] with polarity applied.")
      .def("set_target_current", &ArmMotorBase::SetTargetCurrent, py::arg("current"),
           py::arg("timeout") = 0.0f)
      .def("set_reset", &ArmMotorBase::SetReset, py::arg("mode") = 0, py::arg("timeout") = 1.0f)
      .def("set_clear_error", &ArmMotorBase::SetClearError, py::arg("timeout") = 1.0f)
      .def("set_zero_offset", &ArmMotorBase::SetZeroOffset, py::arg("zero_offset") = 0.0f,
           py::arg("save_to_flash") = false, py::arg("timeout") = 1.0f)
      .def("set_profile_acc_dec", &ArmMotorBase::SetProfileAccDec, py::arg("acceleration"),
           py::arg("deceleration"))
      .def("set_profile_vel", &ArmMotorBase::SetProfileVelocity, py::arg("velocity"))
      .def("set_current_limit", &ArmMotorBase::SetCurrentLimit, py::arg("current"))
      .def("set_mit_control", &ArmMotorBase::SetMitControl, py::arg("p_des"), py::arg("v_des"),
           py::arg("kp"), py::arg("kd"), py::arg("t_ff"), py::arg("t_am") = 16.0f,
           "Low-level MIT in joint space (limits + polarity applied before transmit).")
      .def("set_mit_control_crc", &ArmMotorBase::SetMitControlCrc, py::arg("p_des"), py::arg("v_des"),
           py::arg("kp"), py::arg("kd"), py::arg("t_ff"), py::arg("t_am") = 8.0f,
           "Low-level MIT with CRC in joint space (limits + polarity applied before transmit).")
      .def("set_collision_threshold", &ArmMotorBase::SetCollisionThreshold,
           py::arg("current_threshold"), py::arg("time_threshold"))
      .def("get_param", &ArmMotorBase::GetParam, py::arg("param"), py::arg("timeout") = 1.0f)
      .def("set_param", &ArmMotorBase::SetParam, py::arg("param"), py::arg("value"),
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
                             [](const JointConfig& self) { return self.can_channel; });

  py::class_<GroupConfig>(m, "GroupConfig", "One YAML module block (see YAML_CONFIG.md).")
      .def_property_readonly("can_channel", [](const GroupConfig& self) { return self.can_channel; })
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

  py::class_<JointGroupBase, std::shared_ptr<JointGroupBase>>(m, "JointGroupBase")
      .attr("INVALID_JOINT_INDEX") = kInvalidJointIndex;

  auto arm_motor_base = py::class_<ArmMotorBase>(m, "ArmMotorBase",
                                               "Arm motor interface (real or no-op).");
  BindArmMotorBase(arm_motor_base);
  py::class_<ArmMotor, ArmMotorBase> arm_motor(m, "ArmMotor", "Bound arm-protocol motor.");

  auto driver_owners = std::make_shared<
      std::unordered_map<ArmMotor*, std::shared_ptr<agx::motor::ArmDriver>>>();
  py::class_<MotorRegistry>(m, "MotorRegistry", "Global shared ArmMotor pool.")
      .def_static(
          "acquire_arm",
          [driver_owners](const std::string& channel, std::shared_ptr<agx::motor::ArmDriver> driver, uint8_t node_id,
             const JointConfig& config) -> ArmMotor& {
            if (!driver) throw std::invalid_argument("driver must not be None");
            auto& motor = MotorRegistry::Default().AcquireArm(channel, driver, node_id, config);
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

  auto arm_group = py::class_<ArmJointGroup, JointGroupBase, std::shared_ptr<ArmJointGroup>>(
      m, "ArmJointGroup");
  BindJointGroupCommon(arm_group);
  arm_group
      .def(
          "motor",
          [](ArmJointGroup& group, int index) -> ArmMotorBase& {
            return group.MotorArm(index);
          },
          py::arg("index"),
          py::return_value_policy::reference,
          "Return per-joint arm motor; invalid index yields a no-op motor (is_valid=False).")
      .def("set_mit", &ArmJointGroup::SetMit, py::arg("index"),
           py::arg("position_rad"), py::arg("velocity_rad_s"), py::arg("torque_nm"))
       .def(py::init([](const GroupConfig& config) { return MakeArmJointGroup(config); }),
         py::arg("config"))
       .def(py::init([](const GroupConfig& config, std::string can_channel) {
           return MakeArmJointGroup(config, std::move(can_channel));
         }), py::arg("config"),
           py::arg("can_channel"))
      .def_static("from_yaml",
                  [](const std::string& yaml_path, const std::string& module_key,
                     const py::object& can_channel) {
                    if (can_channel.is_none()) {
                      return MakeArmJointGroup(
                          ArmJointGroup::FromYaml(yaml_path, module_key));
                    }
                    return MakeArmJointGroup(ArmJointGroup::FromYaml(
                        yaml_path, module_key, can_channel.cast<std::string>()));
                  },
                  py::arg("yaml_path"), py::arg("module_key"),
                  py::arg("can_channel") = py::none());

  m.def("create_joint_group",
      [](const GroupConfig& config) { return MakeArmJointGroup(config); },
        py::arg("config"));
  m.def("create_joint_group",
      [](const GroupConfig& config, std::string can_channel) {
        return MakeArmJointGroup(config, std::move(can_channel));
      },
        py::arg("config"), py::arg("can_channel"));
  m.def("load_module_config", &LoadModuleConfig, py::arg("yaml_path"), py::arg("module_key"));
}

}  // namespace agx::motor::mechanism
