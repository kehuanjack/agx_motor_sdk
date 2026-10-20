#pragma once

#include "agx_motor_sdk/mechanism/types.hpp"
#include "agx_motor_sdk/motor/arm_driver.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace agx::motor {
struct HighSpeedState;
struct LowSpeedState;
struct VersionInfo;
}  // namespace agx::motor

namespace agx::motor::mechanism {

/// Per-joint arm-protocol motor interface (public mechanism API).
///
/// Obtain references via ``ArmJointGroup::MotorArm``; do not construct directly.
/// Invalid indices return ``NullArmMotor`` with ``IsValid() == false``.
/// Units: position [rad], velocity [rad/s], current [A].
class ArmMotorBase {
 public:
  virtual ~ArmMotorBase() = default;

  /// false for invalid-index no-op motors.
  virtual bool IsValid() const = 0;
  /// Always ``Protocol::kArm``.
  virtual Protocol protocol() const = 0;
  /// CAN node ID (1–15); 0 on no-op motors.
  virtual uint8_t NodeId() const = 0;
  /// Resolved CAN channel for this joint.
  virtual const std::string& CanChannel() const = 0;
  /// Static per-joint YAML configuration.
  virtual const JointConfig& Config() const = 0;
  /// Joint name from YAML ``joint_name``.
  virtual const std::string& Name() const = 0;

  /// Enable or disable driver output.
  virtual bool SetEnable(bool enable = true, bool has_brake = false, bool brake_on = false) = 0;
  /// Read joint-space position, velocity, and estimated torque.
  virtual std::optional<JointState> GetState() const = 0;
  /// High-level MIT using YAML ``mit_kp`` / ``mit_kd`` / ``mit_tf``.
  virtual bool SetMit(float position_rad, float velocity_rad_s, float torque_nm) = 0;
  /// Apply YAML profile and MIT parameters to the driver.
  virtual bool Configure() = 0;

  /// Cached position [rad], velocity [rad/s], current [A] in joint space (non-blocking).
  virtual std::optional<agx::motor::HighSpeedState> GetHighSpeedState() const = 0;
  /// Cached bus voltage, temperatures, and status (non-blocking).
  virtual std::optional<agx::motor::LowSpeedState> GetLowSpeedState() const = 0;
  /// Query firmware version (blocking).
  virtual std::optional<agx::motor::VersionInfo> GetVersion(float timeout_s = 1.f) = 0;
  /// Generic target in joint space; ``mode`` is protocol-defined (e.g. 1=pos, 0xFF=unchanged).
  virtual bool SetTarget(float value, uint8_t mode, float timeout_s = 0.f) = 0;
  /// Position mode target [rad] in joint space (limits + polarity applied).
  virtual bool SetTargetPosition(float rad, float timeout_s = 0.f) = 0;
  /// Velocity mode target [rad/s] in joint space (polarity applied).
  virtual bool SetTargetVelocity(float rad_s, float timeout_s = 0.f) = 0;
  /// Current / torque mode target [A].
  virtual bool SetTargetCurrent(float current_a, float timeout_s = 0.f) = 0;
  /// Reset or calibration (mode 0 = reset, 1 = enter cal, 2 = save cal).
  virtual bool SetReset(int mode = 0, float timeout_s = 1.f) = 0;
  /// Clear fault and alarm states.
  virtual bool SetClearError(float timeout_s = 1.f) = 0;
  /// Set mechanical zero offset [rad].
  virtual bool SetZeroOffset(float zero_offset = 0.f, bool save_to_flash = false,
                             float timeout_s = 1.f) = 0;
  /// Set motion profile acceleration and deceleration [rad/s²].
  virtual bool SetProfileAccDec(float acceleration, float deceleration) = 0;
  /// Set profile velocity cap [rad/s].
  virtual bool SetProfileVelocity(float rad_s) = 0;
  /// Set phase current limit [A].
  virtual bool SetCurrentLimit(float current_a) = 0;
  /// Low-level MIT impedance control (frame without CRC); joint-space inputs (limit + polarity).
  virtual bool SetMitControl(float p_des, float v_des, float kp, float kd, float t_ff,
                             float t_am = 16.f) = 0;
  /// Low-level MIT impedance control (frame with CRC); joint-space inputs (limit + polarity).
  virtual bool SetMitControlCrc(float p_des, float v_des, float kp, float kd, float t_ff,
                                float t_am = 8.f) = 0;
  /// Configure collision protection current [A] and time [s].
  virtual bool SetCollisionThreshold(float current_threshold, float time_threshold) = 0;
  /// Read a motor parameter (blocking).
  virtual std::optional<float> GetParam(agx::motor::ArmDriver::MotorParam param,
                                        float timeout_s = 1.f) = 0;
  /// Write a motor parameter (blocking).
  virtual bool SetParam(agx::motor::ArmDriver::MotorParam param, float value,
                        float timeout_s = 1.f) = 0;
};

/// No-op arm motor returned for invalid joint indices; all calls return false/nullopt.
class NullArmMotor : public ArmMotorBase {
 public:
  static NullArmMotor& Instance();

  bool IsValid() const override;
  Protocol protocol() const override;
  uint8_t NodeId() const override;
  const std::string& CanChannel() const override;
  const JointConfig& Config() const override;
  const std::string& Name() const override;

  bool SetEnable(bool enable = true, bool has_brake = false, bool brake_on = false) override;
  std::optional<JointState> GetState() const override;
  bool SetMit(float position_rad, float velocity_rad_s, float torque_nm) override;
  bool Configure() override;

  std::optional<agx::motor::HighSpeedState> GetHighSpeedState() const override;
  std::optional<agx::motor::LowSpeedState> GetLowSpeedState() const override;
  std::optional<agx::motor::VersionInfo> GetVersion(float timeout_s = 1.f) override;
  bool SetTarget(float value, uint8_t mode, float timeout_s = 0.f) override;
  bool SetTargetPosition(float rad, float timeout_s = 0.f) override;
  bool SetTargetVelocity(float rad_s, float timeout_s = 0.f) override;
  bool SetTargetCurrent(float current_a, float timeout_s = 0.f) override;
  bool SetReset(int mode = 0, float timeout_s = 1.f) override;
  bool SetClearError(float timeout_s = 1.f) override;
  bool SetZeroOffset(float zero_offset = 0.f, bool save_to_flash = false,
                     float timeout_s = 1.f) override;
  bool SetProfileAccDec(float acceleration, float deceleration) override;
  bool SetProfileVelocity(float rad_s) override;
  bool SetCurrentLimit(float current_a) override;
  bool SetMitControl(float p_des, float v_des, float kp, float kd, float t_ff,
                     float t_am = 16.f) override;
  bool SetMitControlCrc(float p_des, float v_des, float kp, float kd, float t_ff,
                        float t_am = 8.f) override;
  bool SetCollisionThreshold(float current_threshold, float time_threshold) override;
  std::optional<float> GetParam(agx::motor::ArmDriver::MotorParam param,
                                  float timeout_s = 1.f) override;
  bool SetParam(agx::motor::ArmDriver::MotorParam param, float value,
                 float timeout_s = 1.f) override;

 private:
  NullArmMotor() = default;
};

}  // namespace agx::motor::mechanism
