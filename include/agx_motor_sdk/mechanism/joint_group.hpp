#pragma once

#include "agx_motor_sdk/mechanism/motor_base.hpp"
#include "agx_motor_sdk/mechanism/can_bus.hpp"
#include "agx_motor_sdk/mechanism/types.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace agx::motor {
class CanPort;
class MotorDriver;
}  // namespace agx::motor

namespace agx::motor::mechanism {

/// One YAML module (joint group).
///
/// Joints may use different protocols (``arm`` / ``rb200``) and CAN channels
/// within the same module; each channel reuses a shared ``CanBus`` from
/// ``CanBusRegistry``. Multiple modules on the same channel also share one ``CanBus``.
///
/// Per-joint motor access: ``JointGroup::Motor`` returns ``MotorBase&``; cast to
/// ``ArmMotor`` / ``Rb200Motor`` for protocol-specific commands.
class JointGroup {
 public:
  explicit JointGroup(const GroupConfig& config);
  /// @param can_channel Module default CAN channel override.
  JointGroup(const GroupConfig& config, std::string can_channel);
  ~JointGroup();

  static JointGroup FromYaml(const std::string& yaml_path, const std::string& module_key);
  static JointGroup FromYaml(const std::string& yaml_path, const std::string& module_key,
                             const std::string& can_channel);

  JointGroup(const JointGroup&) = delete;
  JointGroup& operator=(const JointGroup&) = delete;
  JointGroup(JointGroup&&) noexcept;
  JointGroup& operator=(JointGroup&&) noexcept;

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
  /// @param interval_s Delay in seconds between consecutive joint commands (default 0).
  ///        Use a small value if a board drops frames sent back-to-back.
  /// @throws std::invalid_argument if ``interval_s`` is negative or not finite.
  bool Enable(double interval_s = 0.0);
  /// Disable all joints in the module.
  /// @param interval_s Delay in seconds between consecutive joint commands (default 0).
  /// @throws std::invalid_argument if ``interval_s`` is negative or not finite.
  bool Disable(double interval_s = 0.0);
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

  /// Module default protocol (joints may override per entry).
  Protocol protocol() const { return config_.protocol; }
  /// Module default CAN channel (joints may override per entry).
  const std::string& can_channel() const { return can_channel_; }
  const GroupConfig& config() const { return config_; }

  /// ``CanBus`` for ``channel``, or nullptr if not connected.
  CanBus* Bus(const std::string& channel);
  const CanBus* Bus(const std::string& channel) const;
  CanBus* BusForJoint(std::size_t index);
  const CanBus* BusForJoint(std::size_t index) const;

  /// Motor for joint ``index`` (``ArmMotor`` or ``Rb200Motor``); an invalid index
  /// returns ``MotorBase::Null()``. Connects on first use.
  MotorBase& Motor(int index);
  const MotorBase& Motor(int index) const;

  /// Default-protocol driver for ``channel`` (empty string = module default channel).
  agx::motor::MotorDriver* GetDriver(const std::string& channel = "") const;
  /// Driver used by joint ``index`` (its protocol on its channel).
  agx::motor::MotorDriver* GetDriverForJoint(std::size_t index) const;

  /// Joint-space MIT on one arm joint (requires YAML ``mit_tf`` / ``mit_kp`` / ``mit_kd``).
  /// @return false for RB200 joints.
  bool SetMit(std::size_t index, float position_rad, float velocity_rad_s, float torque_nm);

 private:
  const JointConfig& JointCfg(std::size_t index) const;
  void ApplyDefaultChannelOverride(const std::string& can_channel);
  /// Shared motor for a valid joint index.
  MotorBase& MotorAt(std::size_t index);

  GroupConfig config_;
  std::string can_channel_;
  std::unordered_map<std::string, std::shared_ptr<agx::motor::CanPort>> injected_cans_;
  std::unordered_map<std::string, std::shared_ptr<CanBus>> buses_;
};

std::unique_ptr<JointGroup> CreateJointGroup(const GroupConfig& config);
/// Factory with module default ``can_channel`` override.
std::unique_ptr<JointGroup> CreateJointGroup(const GroupConfig& config,
                                             std::string can_channel);

/// Load YAML and construct a joint group.
std::unique_ptr<JointGroup> CreateJointGroupFromYaml(const std::string& yaml_path,
                                                     const std::string& module_key);
/// Load YAML with module default ``can_channel`` override.
std::unique_ptr<JointGroup> CreateJointGroupFromYaml(const std::string& yaml_path,
                                                     const std::string& module_key,
                                                     const std::string& can_channel);

}  // namespace agx::motor::mechanism
