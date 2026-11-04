#pragma once

#include "agx_motor_sdk/mechanism/motor_base.hpp"
#include "agx_motor_sdk/motor/arm/arm_driver.hpp"
#include "agx_motor_sdk/motor/arm/types.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace agx::motor::mechanism {

/// Arm-protocol joint motor: common commands plus arm low-speed feedback,
/// current mode, MIT, collision protection and motor parameters.
///
/// Created and shared by ``MotorRegistry``; reach it from
/// ``JointGroup::Motor`` via ``dynamic_cast<ArmMotor*>``.
class ArmMotor : public MotorBase {
 public:
  /// High-level MIT using YAML ``mit_kp`` / ``mit_kd`` / ``mit_tf``.
  bool SetMit(float position_rad, float velocity_rad_s, float torque_nm);
  /// Profile parameters plus YAML ``mit_tf`` sync (motor parameter ``tf``).
  bool Configure() override;

  /// Cached bus voltage, temperatures, and status (non-blocking).
  std::optional<agx::motor::LowSpeedState> GetLowSpeedState() const;
  /// Current / torque mode target [A].
  bool SetTargetCurrent(float current_a, float timeout_s = 0.f);
  /// Low-level MIT impedance control (frame without CRC); joint-space inputs (limit + polarity).
  bool SetMitControl(float p_des, float v_des, float kp, float kd, float t_ff,
                     float t_am = 16.f);
  /// Low-level MIT impedance control (frame with CRC); joint-space inputs (limit + polarity).
  bool SetMitControlCrc(float p_des, float v_des, float kp, float kd, float t_ff,
                        float t_am = 8.f);
  /// Configure collision protection current [A] and time [s].
  bool SetCollisionThreshold(float current_threshold, float time_threshold);
  /// Read a motor parameter (blocking).
  std::optional<float> GetParam(agx::motor::ArmDriver::MotorParam param,
                                float timeout_s = 1.f);
  /// Write a motor parameter (blocking).
  bool SetParam(agx::motor::ArmDriver::MotorParam param, float value, float timeout_s = 1.f);

 private:
  friend class MotorRegistry;

  using MotorBase::MotorBase;

  std::shared_ptr<agx::motor::ArmDriver> LockArmDriver() const;
};

}  // namespace agx::motor::mechanism
