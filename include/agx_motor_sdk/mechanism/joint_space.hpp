#pragma once

#include "agx_motor_sdk/mechanism/types.hpp"

namespace agx::motor::mechanism::joint_space {

/// Clamp a joint-space command to YAML ``lower_limit`` / ``upper_limit`` when set.
/// @param cfg Per-joint configuration with limit flags and values.
/// @param joint_value Desired position in joint space [rad].
/// @return Clamped joint-space value.
float ClampJoint(const JointConfig& cfg, float joint_value);

/// Convert joint-space value to motor-space command (``joint / polarity``).
/// When ``polarity == 0``, returns ``joint_value`` unchanged (pass-through).
float JointToMotor(const JointConfig& cfg, float joint_value);

/// Convert motor-space feedback to joint space (``motor * polarity``).
/// When ``polarity == 0``, returns ``motor_value`` unchanged (pass-through).
float MotorToJoint(const JointConfig& cfg, float motor_value);

/// Estimate joint-space torque from joint-space phase current [A].
/// @return ``joint_current_a × torque_k × torque_b`` [N·m].
float JointCurrentToTorqueNm(const JointConfig& cfg, float joint_current_a);

/// Estimate joint-space torque from motor-space phase current [A] (polarity applied).
/// @return ``MotorToJoint(current_a × torque_k × torque_b)`` [N·m].
float CurrentToTorqueNm(const JointConfig& cfg, float motor_current_a);

/// Map joint-space feedforward torque [N·m] to driver MIT scale (polarity applied).
/// @return ``JointToMotor(torque_nm) / torque_b``; ``0`` when ``torque_b == 0``.
float TorqueNmToDriver(const JointConfig& cfg, float torque_nm);

}  // namespace agx::motor::mechanism::joint_space
