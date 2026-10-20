#include "agx_motor_sdk/mechanism/joint_space.hpp"

#include <algorithm>

namespace agx::motor::mechanism::joint_space {

float ClampJoint(const JointConfig& cfg, float joint_value) {
  float value = joint_value;
  if (cfg.has_lower_limit) {
    value = std::max(value, cfg.lower_limit_rad);
  }
  if (cfg.has_upper_limit) {
    value = std::min(value, cfg.upper_limit_rad);
  }
  return value;
}

float JointToMotor(const JointConfig& cfg, float joint_value) {
  if (cfg.polarity == 0.f) {
    return joint_value;
  }
  return joint_value / cfg.polarity;
}

float MotorToJoint(const JointConfig& cfg, float motor_value) {
  if (cfg.polarity == 0.f) {
    return motor_value;
  }
  return motor_value * cfg.polarity;
}

float JointCurrentToTorqueNm(const JointConfig& cfg, float joint_current_a) {
  return joint_current_a * cfg.torque_k * cfg.torque_b;
}

float CurrentToTorqueNm(const JointConfig& cfg, float motor_current_a) {
  const float motor_torque_nm = motor_current_a * cfg.torque_k * cfg.torque_b;
  return MotorToJoint(cfg, motor_torque_nm);
}

float TorqueNmToDriver(const JointConfig& cfg, float torque_nm) {
  if (cfg.torque_b == 0.f) {
    return 0.f;
  }
  const float motor_torque_nm = JointToMotor(cfg, torque_nm);
  return motor_torque_nm / cfg.torque_b;
}

}  // namespace agx::motor::mechanism::joint_space
