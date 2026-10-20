#include "agx_motor_sdk/mechanism/arm_motor.hpp"

#include "agx_motor_sdk/mechanism/joint_space.hpp"

#include <cmath>

namespace agx::motor::mechanism {
namespace {

float JointPositionToMotor(const JointConfig& cfg, float position_rad) {
  return joint_space::JointToMotor(cfg, joint_space::ClampJoint(cfg, position_rad));
}

}  // namespace

ArmMotor::ArmMotor(const std::shared_ptr<agx::motor::ArmDriver>& driver,
                   uint8_t node_id, std::string can_channel,
                   JointConfig config)
    : driver_(driver),
      node_id_(node_id),
      can_channel_(std::move(can_channel)),
      config_(std::move(config)) {}

void ArmMotor::BindDriver(const std::shared_ptr<agx::motor::ArmDriver>& driver) {
  driver_ = driver;
}

void ArmMotor::BindConfig(const JointConfig& config) { config_ = config; }

bool ArmMotor::SetEnable(bool enable, bool has_brake, bool brake_on) {
  const auto driver = driver_.lock();
  return driver && driver->SetEnable(node_id_, enable, has_brake, brake_on);
}

std::optional<JointState> ArmMotor::GetState() const {
  const auto hs = GetHighSpeedState();
  if (!hs.has_value()) {
    return std::nullopt;
  }
  JointState state;
  state.position_rad = hs->position;
  state.velocity_rad_s = hs->velocity;
  state.torque_nm = joint_space::JointCurrentToTorqueNm(config_, hs->current);
  return state;
}

bool ArmMotor::SetMit(float position_rad, float velocity_rad_s, float torque_nm) {
  if (!config_.mit_tf.has_value() || !config_.mit_kp.has_value() || !config_.mit_kd.has_value()) {
    return false;
  }
  return SetMitControl(position_rad, velocity_rad_s, *config_.mit_kp, *config_.mit_kd, torque_nm,
                       *config_.mit_tf);
}

bool ArmMotor::Configure() {
  const auto driver = driver_.lock();
  if (!driver) {
    return false;
  }
  bool ok = true;
  if (config_.profile_vel_rad_s.has_value() && *config_.profile_vel_rad_s > 0.f) {
    ok = driver->SetProfileVelocity(node_id_, *config_.profile_vel_rad_s) && ok;
  }
  if (config_.profile_accel_rad_s2.has_value() && config_.profile_decel_rad_s2.has_value() &&
      *config_.profile_accel_rad_s2 > 0.f && *config_.profile_decel_rad_s2 > 0.f) {
    ok = driver->SetProfileAccDec(node_id_, *config_.profile_accel_rad_s2,
                                   *config_.profile_decel_rad_s2) &&
         ok;
  }
  if (config_.mit_tf.has_value()) {
    const auto tf =
        driver->GetParam(node_id_, agx::motor::ArmDriver::MotorParam::kTf, 1.f);
    if (tf.has_value() && std::abs(*tf - *config_.mit_tf) > 1e-3f) {
      ok = driver->SetParam(node_id_, agx::motor::ArmDriver::MotorParam::kTf,
                             *config_.mit_tf, 1.f) &&
           ok;
    }
  }
  return ok;
}

std::optional<agx::motor::HighSpeedState> ArmMotor::GetHighSpeedState() const {
  const auto driver = driver_.lock();
  if (!driver) {
    return std::nullopt;
  }
  const auto raw = driver->GetHighSpeedState(node_id_);
  if (!raw.has_value()) {
    return std::nullopt;
  }
  agx::motor::HighSpeedState joint = *raw;
  joint.position = joint_space::MotorToJoint(config_, raw->position);
  joint.velocity = joint_space::MotorToJoint(config_, raw->velocity);
  joint.current = joint_space::MotorToJoint(config_, raw->current);
  return joint;
}

std::optional<agx::motor::LowSpeedState> ArmMotor::GetLowSpeedState() const {
  const auto driver = driver_.lock();
  return driver ? driver->GetLowSpeedState(node_id_) : std::nullopt;
}

std::optional<agx::motor::VersionInfo> ArmMotor::GetVersion(float timeout_s) {
  const auto driver = driver_.lock();
  return driver ? driver->GetVersion(node_id_, timeout_s) : std::nullopt;
}

namespace {

float JointValueToMotor(const JointConfig& cfg, float value, uint8_t mode) {
  if (mode == 0) {
    return joint_space::JointToMotor(cfg, value);
  }
  if (mode == 1 || mode == 0xFF) {
    return JointPositionToMotor(cfg, value);
  }
  return value;
}

}  // namespace

bool ArmMotor::SetTarget(float value, uint8_t mode, float timeout_s) {
  const auto driver = driver_.lock();
  if (!driver) {
    return false;
  }
  return driver->SetTarget(node_id_, JointValueToMotor(config_, value, mode), mode,
                            timeout_s);
}

bool ArmMotor::SetTargetPosition(float rad, float timeout_s) {
  return SetTarget(rad, 1, timeout_s);
}

bool ArmMotor::SetTargetVelocity(float rad_s, float timeout_s) {
  return SetTarget(rad_s, 0, timeout_s);
}

bool ArmMotor::SetTargetCurrent(float current_a, float timeout_s) {
  return SetTarget(current_a, 2, timeout_s);
}

bool ArmMotor::SetReset(int mode, float timeout_s) {
  const auto driver = driver_.lock();
  return driver && driver->SetReset(node_id_, mode, timeout_s);
}

bool ArmMotor::SetClearError(float timeout_s) {
  const auto driver = driver_.lock();
  return driver && driver->SetClearError(node_id_, timeout_s);
}

bool ArmMotor::SetZeroOffset(float zero_offset, bool save_to_flash, float timeout_s) {
  const auto driver = driver_.lock();
  return driver &&
         driver->SetZeroOffset(node_id_, zero_offset, save_to_flash, timeout_s);
}

bool ArmMotor::SetProfileAccDec(float acceleration, float deceleration) {
  const auto driver = driver_.lock();
  return driver && driver->SetProfileAccDec(node_id_, acceleration, deceleration);
}

bool ArmMotor::SetProfileVelocity(float rad_s) {
  const auto driver = driver_.lock();
  return driver && driver->SetProfileVelocity(node_id_, rad_s);
}

bool ArmMotor::SetCurrentLimit(float current_a) {
  const auto driver = driver_.lock();
  return driver && driver->SetCurrentLimit(node_id_, current_a);
}

bool ArmMotor::SetMitControl(float p_des, float v_des, float kp, float kd, float t_ff,
                             float t_am) {
  const auto driver = driver_.lock();
  if (!driver) {
    return false;
  }
  const float p_motor = JointPositionToMotor(config_, p_des);
  const float v_motor = joint_space::JointToMotor(config_, v_des);
  const float t_motor = joint_space::TorqueNmToDriver(config_, t_ff);
  return driver->SetMitControl(node_id_, p_motor, v_motor, kp, kd, t_motor, t_am);
}

bool ArmMotor::SetMitControlCrc(float p_des, float v_des, float kp, float kd, float t_ff,
                                float t_am) {
  const auto driver = driver_.lock();
  if (!driver) {
    return false;
  }
  const float p_motor = JointPositionToMotor(config_, p_des);
  const float v_motor = joint_space::JointToMotor(config_, v_des);
  const float t_motor = joint_space::TorqueNmToDriver(config_, t_ff);
  return driver->SetMitControlCrc(node_id_, p_motor, v_motor, kp, kd, t_motor, t_am);
}

bool ArmMotor::SetCollisionThreshold(float current_threshold, float time_threshold) {
  const auto driver = driver_.lock();
  return driver &&
         driver->SetCollisionThreshold(node_id_, current_threshold, time_threshold);
}

std::optional<float> ArmMotor::GetParam(agx::motor::ArmDriver::MotorParam param,
                                        float timeout_s) {
  const auto driver = driver_.lock();
  return driver ? driver->GetParam(node_id_, param, timeout_s) : std::nullopt;
}

bool ArmMotor::SetParam(agx::motor::ArmDriver::MotorParam param, float value,
                        float timeout_s) {
  const auto driver = driver_.lock();
  return driver && driver->SetParam(node_id_, param, value, timeout_s);
}

}  // namespace agx::motor::mechanism
