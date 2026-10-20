#pragma once

#include "agx_motor_sdk/mechanism/arm_motor_base.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace agx::motor::mechanism {

class MotorRegistry;

/// Internal bound arm motor (one per ``can_channel`` + ``node_id``).
///
/// Created and shared by ``MotorRegistry``; not part of the public Python API.
/// Use ``ArmMotorBase&`` from ``ArmJointGroup::MotorArm`` instead.
class ArmMotor : public ArmMotorBase {
 public:
  bool IsValid() const override { return true; }
  Protocol protocol() const override { return Protocol::kArm; }
  uint8_t NodeId() const override { return node_id_; }
  const std::string& CanChannel() const override { return can_channel_; }
  const JointConfig& Config() const override { return config_; }
  const std::string& Name() const override { return config_.name; }

  /// Rebind the weak driver handle after bus (re)connect.
  void BindDriver(const std::shared_ptr<agx::motor::ArmDriver>& driver);
  /// Update cached YAML configuration (e.g. after config reload).
  void BindConfig(const JointConfig& config);

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
  friend class MotorRegistry;

  ArmMotor(const std::shared_ptr<agx::motor::ArmDriver>& driver, uint8_t node_id,
           std::string can_channel, JointConfig config);

  std::weak_ptr<agx::motor::ArmDriver> driver_;
  uint8_t node_id_{0};
  std::string can_channel_;
  JointConfig config_;
};

}  // namespace agx::motor::mechanism
