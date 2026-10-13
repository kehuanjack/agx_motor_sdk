#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace agx::motor::mechanism {

/// Motor firmware protocol (arm only).
enum class Protocol {
  kArm,  ///< 1 Mbps CAN; joint position [rad], velocity [rad/s].
};

/// @return ``"arm"`` for logging / Python ``protocol`` property.
inline const char* ProtocolToString(Protocol protocol) {
  (void)protocol;
  return "arm";
}

/// @return true if ``index`` is a valid zero-based joint index for ``joint_count``.
inline bool IsJointIndexInRange(int index, std::size_t joint_count) {
  return index >= 0 && static_cast<std::size_t>(index) < joint_count;
}

/// Sentinel returned by ``JointGroupBase::IndexOf`` when a joint name is unknown.
constexpr int kInvalidJointIndex = -1;

/// Cached joint-space feedback returned by motor ``GetState`` [rad, rad/s, N·m].
struct JointState {
  float position_rad{0.f};   ///< Joint position [rad].
  float velocity_rad_s{0.f}; ///< Joint velocity [rad/s].
  float torque_nm{0.f};      ///< Estimated torque [N·m] from phase current.
};

/// Per-joint static configuration (one YAML ``joints`` list entry).
struct JointConfig {
  std::string name;  ///< Logical joint name (`joint_name` in YAML).
  uint8_t node_id{0};  ///< CAN node ID (1–15).
  /// Resolved CAN channel (per-joint override or module ``can_channel`` default).
  std::string can_channel;
  /// Per-joint linear coefficient for position, velocity, and torque:
  /// command ``motor = joint / polarity``, feedback ``joint = motor * polarity``.
  /// Default ``1.0``. ``0`` = pass-through.
  float polarity{1.f};
  /// Override profile speed [rad/s].
  std::optional<float> profile_vel_rad_s;
  /// Override profile acceleration [rad/s²].
  std::optional<float> profile_accel_rad_s2;
  /// Override profile deceleration [rad/s²].
  std::optional<float> profile_decel_rad_s2;
  /// MIT torque mapping half-range [N·m] (``MotorParam.TF``).
  std::optional<float> mit_tf;
  /// MIT position gain for joint-space ``SetMit``.
  std::optional<float> mit_kp;
  /// MIT damping gain for joint-space ``SetMit``.
  std::optional<float> mit_kd;
  /// Torque estimate scale: ``torque_nm = current × torque_k × torque_b``.
  float torque_k{1.f};
  /// Second torque scale; also used in MIT feedforward mapping.
  float torque_b{1.f};
  /// Soft lower limit in joint space [rad] when ``has_lower_limit``.
  float lower_limit_rad{0.f};
  /// Soft upper limit in joint space [rad] when ``has_upper_limit``.
  float upper_limit_rad{0.f};
  bool has_lower_limit{false};
  bool has_upper_limit{false};
};

/// Static configuration for one YAML module block (e.g. ``head``, ``fold``).
struct GroupConfig {
  /// Default CAN channel for joints without per-joint ``can_channel``.
  std::string can_channel;
  /// Module default profile speed [rad/s].
  std::optional<float> default_profile_vel_rad_s;
  /// Module default profile acceleration [rad/s²].
  std::optional<float> profile_accel_rad_s2;
  /// Module default profile deceleration [rad/s²].
  std::optional<float> profile_decel_rad_s2;
  /// Default MIT torque limit [N·m].
  std::optional<float> mit_tf;
  /// Default MIT Kp for ``SetMit``.
  std::optional<float> mit_kp;
  /// Default MIT Kd for ``SetMit``.
  std::optional<float> mit_kd;
  /// Module default ``torque_k`` for estimated torque.
  std::optional<float> torque_k;
  /// Module default ``torque_b`` for estimated torque / MIT feedforward.
  std::optional<float> torque_b;
  std::vector<JointConfig> joints;
};

}  // namespace agx::motor::mechanism
