#pragma once

#include "agx_motor_sdk/mechanism/arm_motor.hpp"
#include "agx_motor_sdk/mechanism/can_bus.hpp"
#include "agx_motor_sdk/mechanism/types.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace agx::motor {
class ArmDriver;
class CanPort;
}  // namespace agx::motor

namespace agx::motor::mechanism {

/// Common module / joint-group base (not constructible directly).
///
/// One YAML module maps to one :class:`ArmJointGroup`. Joints may use different CAN
/// channels within the same module; each channel reuses a shared ``CanBus`` from
/// ``CanBusRegistry``. Multiple modules on the same channel also share one ``CanBus``.
///
/// Per-joint motor access: ``ArmJointGroup::MotorArm``.
class JointGroupBase {
 public:
  virtual ~JointGroupBase();

  JointGroupBase(const JointGroupBase&) = delete;
  JointGroupBase& operator=(const JointGroupBase&) = delete;
  JointGroupBase(JointGroupBase&&) noexcept;
  JointGroupBase& operator=(JointGroupBase&&) noexcept;

  /// Inject an external ``CanPort`` for the module default channel before ``Connect``.
  /// The port must remain valid until ``Close``; the group does not take ownership.
  void AttachCanPort(const std::shared_ptr<agx::motor::CanPort>& can);
  /// Inject an external ``CanPort`` for a specific channel (multi-bus modules).
  void AttachCanPort(const std::string& channel,
                     const std::shared_ptr<agx::motor::CanPort>& can);

  /// Acquire/open all CAN backends and bind shared motors for this module.
  /// @return false if any bus fails to open.
  bool Connect();
  /// Detach from buses and release group-held bus references (does not clear global registries).
  void Close();
  /// true after a successful ``Connect`` that has not been ``Close``d.
  bool IsConnected() const;

  /// Enable all joints in the module.
  bool Enable();
  /// Disable all joints in the module.
  bool Disable();
  /// Run ``Configure`` on every joint (profile / MIT from YAML).
  bool Configure();

  std::size_t JointCount() const;
  std::vector<std::string> JointNames() const;
  /// Joint index by name, or ``kInvalidJointIndex`` (-1) if unknown.
  int IndexOf(const std::string& joint_name) const;
  /// @return true if ``index`` is in ``[0, JointCount())``.
  bool IsValidJointIndex(int index) const;

  const JointConfig& GetJointConfig(std::size_t index) const;
  uint8_t NodeId(std::size_t index) const;
  /// Resolved CAN channel for a joint (per-joint override or module default).
  std::string JointCanChannel(std::size_t index) const;
  /// Distinct CAN channels used by joints in this module.
  std::vector<std::string> CanChannels() const;

  /// Joint-space position command on one joint (limits + polarity applied).
  bool SetTargetPosition(std::size_t index, float position_rad, float timeout_s = 0.f);
  /// Joint-space feedback for one joint.
  std::optional<JointState> GetState(std::size_t index) const;

  Protocol protocol() const { return Protocol::kArm; }
  /// Module default CAN channel (joints may override per entry).
  const std::string& can_channel() const { return can_channel_; }
  const GroupConfig& config() const { return config_; }

  /// ``CanBus`` for ``channel``, or nullptr if not connected.
  CanBus* Bus(const std::string& channel);
  const CanBus* Bus(const std::string& channel) const;
  CanBus* BusForJoint(std::size_t index);
  const CanBus* BusForJoint(std::size_t index) const;

 protected:
  JointGroupBase(GroupConfig config, std::string can_channel);

  agx::motor::ArmDriver* ArmDriverPtr(const std::string& channel);
  const agx::motor::ArmDriver* ArmDriverPtr(const std::string& channel) const;
  agx::motor::ArmDriver* ArmDriverForJoint(std::size_t index);
  const agx::motor::ArmDriver* ArmDriverForJoint(std::size_t index) const;

  const JointConfig& JointCfg(std::size_t index) const;

  void ApplyDefaultChannelOverride(const std::string& can_channel);

  /// Internal shared ``ArmMotor`` for a valid joint index.
  ArmMotor& ArmMotorAt(std::size_t index);
  const ArmMotor& ArmMotorAt(std::size_t index) const;

  GroupConfig config_;
  std::string can_channel_;
  std::unordered_map<std::string, std::shared_ptr<agx::motor::CanPort>> injected_cans_;
  std::unordered_map<std::string, std::shared_ptr<CanBus>> buses_;
};

/// Arm-protocol module (1 Mbps). Supports MIT and motor parameters.
class ArmJointGroup : public JointGroupBase {
 public:
  explicit ArmJointGroup(const GroupConfig& config);
  ArmJointGroup(const GroupConfig& config, std::string can_channel);

  static ArmJointGroup FromYaml(const std::string& yaml_path, const std::string& module_key);
  static ArmJointGroup FromYaml(const std::string& yaml_path, const std::string& module_key,
                                const std::string& can_channel);

  /// Public arm motor view for joint ``index``; invalid index returns ``NullArmMotor``.
  ArmMotorBase& MotorArm(int index);
  const ArmMotorBase& MotorArm(int index) const;

  /// Arm driver for ``channel`` (empty string = module default channel).
  agx::motor::ArmDriver* GetArmDriver(const std::string& channel = "");
  const agx::motor::ArmDriver* GetArmDriver(const std::string& channel = "") const;
  agx::motor::ArmDriver* GetArmDriverForJoint(std::size_t index);
  const agx::motor::ArmDriver* GetArmDriverForJoint(std::size_t index) const;

  /// Joint-space MIT on one joint (requires YAML ``mit_tf`` / ``mit_kp`` / ``mit_kd``).
  bool SetMit(std::size_t index, float position_rad, float velocity_rad_s, float torque_nm);
};

/// Factory: always returns ``ArmJointGroup``.
std::unique_ptr<JointGroupBase> CreateJointGroup(const GroupConfig& config);
/// Factory with module default ``can_channel`` override.
std::unique_ptr<JointGroupBase> CreateJointGroup(const GroupConfig& config,
                                                 std::string can_channel);

std::unique_ptr<ArmJointGroup> CreateArmJointGroup(const GroupConfig& config);
std::unique_ptr<ArmJointGroup> CreateArmJointGroup(const GroupConfig& config,
                                                   std::string can_channel);

/// Load YAML and construct a joint group.
std::unique_ptr<JointGroupBase> CreateJointGroupFromYaml(const std::string& yaml_path,
                                                         const std::string& module_key);
/// Load YAML with module default ``can_channel`` override.
std::unique_ptr<JointGroupBase> CreateJointGroupFromYaml(const std::string& yaml_path,
                                                           const std::string& module_key,
                                                           const std::string& can_channel);

}  // namespace agx::motor::mechanism
