#include "agx_motor_sdk/mechanism/arm/arm_motor.hpp"

#include "agx_motor_sdk/mechanism/joint_space.hpp"

#include <cmath>

namespace agx::motor::mechanism {
namespace {

float JointPositionToMotor(const JointConfig& cfg, float position_rad) {
  return joint_space::JointToMotor(cfg, joint_space::ClampJoint(cfg, position_rad));
}

}  // namespace

std::shared_ptr<agx::motor::ArmDriver> ArmMotor::LockArmDriver() const {
  return std::dynamic_pointer_cast<agx::motor::ArmDriver>(LockDriver());
}

bool ArmMotor::SetMit(float position_rad, float velocity_rad_s, float torque_nm) {
  const auto& cfg = Config();
  if (!cfg.mit_tf.has_value() || !cfg.mit_kp.has_value() || !cfg.mit_kd.has_value()) {
    return false;
  }
  return SetMitControl(position_rad, velocity_rad_s, *cfg.mit_kp, *cfg.mit_kd, torque_nm,
                       *cfg.mit_tf);
}

bool ArmMotor::Configure() {
  const auto driver = LockArmDriver();
  if (!driver) {
    return false;
  }
  bool ok = MotorBase::Configure();
  const auto& cfg = Config();
  if (cfg.mit_tf.has_value()) {
    const auto tf = driver->GetParam(NodeId(), agx::motor::ArmDriver::MotorParam::kTf, 1.f);
    if (tf.has_value() && std::abs(*tf - *cfg.mit_tf) > 1e-3f) {
      ok = driver->SetParam(NodeId(), agx::motor::ArmDriver::MotorParam::kTf, *cfg.mit_tf,
                            1.f) &&
           ok;
    }
  }
  return ok;
}

std::optional<agx::motor::LowSpeedState> ArmMotor::GetLowSpeedState() const {
  const auto driver = LockArmDriver();
  return driver ? driver->GetLowSpeedState(NodeId()) : std::nullopt;
}

bool ArmMotor::SetTargetCurrent(float current_a, float timeout_s) {
  return SetTarget(current_a, 2, timeout_s);
}

bool ArmMotor::SetMitControl(float p_des, float v_des, float kp, float kd, float t_ff,
                             float t_am) {
  const auto driver = LockArmDriver();
  if (!driver) {
    return false;
  }
  const auto& cfg = Config();
  return driver->SetMitControl(NodeId(), JointPositionToMotor(cfg, p_des),
                               joint_space::JointToMotor(cfg, v_des), kp, kd,
                               joint_space::TorqueNmToDriver(cfg, t_ff), t_am);
}

bool ArmMotor::SetMitControlCrc(float p_des, float v_des, float kp, float kd, float t_ff,
                                float t_am) {
  const auto driver = LockArmDriver();
  if (!driver) {
    return false;
  }
  const auto& cfg = Config();
  return driver->SetMitControlCrc(NodeId(), JointPositionToMotor(cfg, p_des),
                                  joint_space::JointToMotor(cfg, v_des), kp, kd,
                                  joint_space::TorqueNmToDriver(cfg, t_ff), t_am);
}

bool ArmMotor::SetCollisionThreshold(float current_threshold, float time_threshold) {
  const auto driver = LockArmDriver();
  return driver && driver->SetCollisionThreshold(NodeId(), current_threshold, time_threshold);
}

std::optional<float> ArmMotor::GetParam(agx::motor::ArmDriver::MotorParam param,
                                        float timeout_s) {
  const auto driver = LockArmDriver();
  return driver ? driver->GetParam(NodeId(), param, timeout_s) : std::nullopt;
}

bool ArmMotor::SetParam(agx::motor::ArmDriver::MotorParam param, float value,
                        float timeout_s) {
  const auto driver = LockArmDriver();
  return driver && driver->SetParam(NodeId(), param, value, timeout_s);
}

}  // namespace agx::motor::mechanism
