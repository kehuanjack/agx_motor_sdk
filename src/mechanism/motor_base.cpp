#include "agx_motor_sdk/mechanism/motor_base.hpp"

#include "agx_motor_sdk/mechanism/joint_space.hpp"

#include <utility>

namespace agx::motor::mechanism {
namespace {

float JointValueToMotor(const JointConfig& cfg, float value, uint8_t mode) {
  if (mode == 0) {
    return joint_space::JointToMotor(cfg, value);
  }
  if (mode == 1 || mode == 0xFF) {
    return joint_space::JointToMotor(cfg, joint_space::ClampJoint(cfg, value));
  }
  return value;
}

}  // namespace

MotorBase& MotorBase::Null() {
  struct NullMotor : MotorBase {};
  static NullMotor instance;
  return instance;
}

MotorBase::MotorBase(const std::shared_ptr<agx::motor::MotorDriver>& driver,
                     uint8_t node_id, std::string can_channel, JointConfig config)
    : valid_(true),
      driver_(driver),
      node_id_(node_id),
      can_channel_(std::move(can_channel)),
      config_(std::move(config)) {}

bool MotorBase::SetEnable(bool enable, bool has_brake, bool brake_on) {
  const auto driver = LockDriver();
  return driver && driver->SetEnable(node_id_, enable, has_brake, brake_on);
}

std::optional<JointState> MotorBase::GetState() const {
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

bool MotorBase::Configure() {
  const auto driver = LockDriver();
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
  return ok;
}

std::optional<agx::motor::HighSpeedState> MotorBase::GetHighSpeedState() const {
  const auto driver = LockDriver();
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

std::optional<agx::motor::VersionInfo> MotorBase::GetVersion(float timeout_s) {
  const auto driver = LockDriver();
  return driver ? driver->GetVersion(node_id_, timeout_s) : std::nullopt;
}

bool MotorBase::SetTarget(float value, uint8_t mode, float timeout_s) {
  const auto driver = LockDriver();
  return driver &&
         driver->SetTarget(node_id_, JointValueToMotor(config_, value, mode), mode, timeout_s);
}

bool MotorBase::SetTargetPosition(float rad, float timeout_s) {
  return SetTarget(rad, 1, timeout_s);
}

bool MotorBase::SetTargetVelocity(float rad_s, float timeout_s) {
  return SetTarget(rad_s, 0, timeout_s);
}

bool MotorBase::SetReset(int mode, float timeout_s) {
  const auto driver = LockDriver();
  return driver && driver->SetReset(node_id_, mode, timeout_s);
}

bool MotorBase::SetClearError(float timeout_s) {
  const auto driver = LockDriver();
  return driver && driver->SetClearError(node_id_, timeout_s);
}

bool MotorBase::SetZeroOffset(float zero_offset, bool save_to_flash, float timeout_s) {
  const auto driver = LockDriver();
  return driver && driver->SetZeroOffset(node_id_, zero_offset, save_to_flash, timeout_s);
}

bool MotorBase::SetProfileAccDec(float acceleration, float deceleration) {
  const auto driver = LockDriver();
  return driver && driver->SetProfileAccDec(node_id_, acceleration, deceleration);
}

bool MotorBase::SetProfileVelocity(float rad_s) {
  const auto driver = LockDriver();
  return driver && driver->SetProfileVelocity(node_id_, rad_s);
}

bool MotorBase::SetCurrentLimit(float current_a) {
  const auto driver = LockDriver();
  return driver && driver->SetCurrentLimit(node_id_, current_a);
}

}  // namespace agx::motor::mechanism
