#pragma once

#include "agx_motor_sdk/mechanism/types.hpp"
#include "agx_motor_sdk/motor/motor_driver.hpp"
#include "agx_motor_sdk/motor/types.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace agx::motor::mechanism {

class MotorRegistry;

/// Per-joint motor with the commands every protocol supports (public mechanism API).
///
/// Obtain references via ``JointGroup::Motor``; do not construct directly.
/// Protocol-specific motors derive from this class: ``ArmMotor``
/// (arm/arm_motor.hpp) and ``Rb200Motor`` (rb200/rb200_motor.hpp); use
/// ``dynamic_cast`` to reach their extras. Invalid indices return
/// ``MotorBase::Null()`` with ``IsValid() == false``.
/// Units: position [rad], velocity [rad/s], current [A].
class MotorBase {
 public:
  virtual ~MotorBase() = default;
  MotorBase(const MotorBase&) = delete;
  MotorBase& operator=(const MotorBase&) = delete;

  /// Shared no-op motor; all calls return false/nullopt.
  static MotorBase& Null();

  /// false for the invalid-index no-op motor.
  bool IsValid() const { return valid_; }
  /// Joint protocol from YAML (``kArm`` or ``kRb200``).
  Protocol protocol() const { return config_.protocol; }
  /// CAN node ID (1–15); 0 on the no-op motor.
  uint8_t NodeId() const { return node_id_; }
  /// Resolved CAN channel for this joint.
  const std::string& CanChannel() const { return can_channel_; }
  /// Static per-joint YAML configuration.
  const JointConfig& Config() const { return config_; }
  /// Joint name from YAML ``joint_name``.
  const std::string& Name() const { return config_.name; }

  /// Enable or disable driver output.
  bool SetEnable(bool enable = true, bool has_brake = false, bool brake_on = false);
  /// Read joint-space position, velocity, and estimated torque.
  std::optional<JointState> GetState() const;
  /// Apply YAML profile parameters (and protocol extras) to the driver.
  virtual bool Configure();

  /// Cached position [rad], velocity [rad/s], current [A] in joint space (non-blocking).
  std::optional<agx::motor::HighSpeedState> GetHighSpeedState() const;
  /// Query firmware version (blocking).
  std::optional<agx::motor::VersionInfo> GetVersion(float timeout_s = 1.f);
  /// Generic target in joint space; ``mode`` is protocol-defined (0=vel, 1=pos, 0xFF=unchanged).
  bool SetTarget(float value, uint8_t mode, float timeout_s = 0.f);
  /// Position mode target [rad] in joint space (limits + polarity applied).
  bool SetTargetPosition(float rad, float timeout_s = 0.f);
  /// Velocity mode target [rad/s] in joint space (polarity applied).
  bool SetTargetVelocity(float rad_s, float timeout_s = 0.f);
  /// Reset or calibration (mode 0 = reset, 1 = enter cal, 2 = save cal).
  bool SetReset(int mode = 0, float timeout_s = 1.f);
  /// Clear fault and alarm states.
  bool SetClearError(float timeout_s = 1.f);
  /// Set mechanical zero offset [rad].
  bool SetZeroOffset(float zero_offset = 0.f, bool save_to_flash = false,
                     float timeout_s = 1.f);
  /// Set motion profile acceleration and deceleration [rad/s²].
  bool SetProfileAccDec(float acceleration, float deceleration);
  /// Set profile velocity cap [rad/s].
  bool SetProfileVelocity(float rad_s);
  /// Set phase current limit [A].
  bool SetCurrentLimit(float current_a);

 protected:
  MotorBase() = default;
  MotorBase(const std::shared_ptr<agx::motor::MotorDriver>& driver, uint8_t node_id,
            std::string can_channel, JointConfig config);

  /// Locked driver (null if the bus is gone or this is the no-op motor).
  std::shared_ptr<agx::motor::MotorDriver> LockDriver() const { return driver_.lock(); }

 private:
  friend class MotorRegistry;

  void BindDriver(const std::shared_ptr<agx::motor::MotorDriver>& driver) { driver_ = driver; }
  void BindConfig(const JointConfig& config) { config_ = config; }

  bool valid_{false};
  std::weak_ptr<agx::motor::MotorDriver> driver_;
  uint8_t node_id_{0};
  std::string can_channel_;
  JointConfig config_;
};

}  // namespace agx::motor::mechanism
