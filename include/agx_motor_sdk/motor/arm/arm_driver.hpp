#pragma once

// Arm-protocol motor driver (1 Mbps CAN). Units: rad, rad/s, A.

#include <cstdint>
#include <memory>
#include <optional>

#include "agx_motor_sdk/motor/arm/arm_protocol_library.hpp"
#include "agx_motor_sdk/motor/arm/types.hpp"
#include "agx_motor_sdk/motor/motor_driver.hpp"

namespace agx {
namespace motor {

/// Arm-protocol motor driver (libagx_motor_protocol_arm.so).
///
/// Adds arm low-speed feedback, current mode, MIT, collision protection and
/// motor parameters to the common :class:`MotorDriver` commands.
class ArmDriver : public MotorDriver {
 public:
  /// Motor parameter IDs for GetParam / SetParam (two ASCII bytes each).
  enum class MotorParam {
    kId,  ///< Node ID (integer, 1–15).
    kAc,  ///< Profile acceleration [rad/s²], max 12.56.
    kDc,  ///< Profile deceleration [rad/s²], max 12.56.
    kVv,  ///< Profile velocity [rad/s], max 20.
    kIq,  ///< Torque-loop max current limit [A], max 12.
    kOi,  ///< Collision protection current threshold [A], max 12.
    kOt,  ///< Collision protection time threshold [s], max 2; OT=30 and OI=30 disables protection.
    kTx,  ///< Fast feedback mode: 0=active push, 1=response (default 0, stored in flash).
    kTc,  ///< Joint motor torque constant [N·m/A] (float32).
    kTf,  ///< MIT feedforward torque limit [N·m] (t_am), default ±8.
    kSo,  ///< Joint zero offset [rad].
    kPp,  ///< Position-loop proportional gain Kp.
    kKp,  ///< Velocity-loop proportional gain Kp.
    kKi,  ///< Velocity-loop integral gain Ki.
  };

  /// Construct driver and load the arm protocol library.
  /// @param can Optional opened CanPort; if non-null, AttachCanPort is called.
  ///        Non-owning: the caller must keep ``can`` alive until :meth:`Close`.
  /// @throws std::runtime_error if the library cannot be loaded.
  explicit ArmDriver(CanPort* can = nullptr);

  /// Read cached low-speed feedback.
  /// @param node_id Motor node ID, 1–15.
  /// @return bus voltage, temperatures, status bits; nullopt if no data.
  std::optional<LowSpeedState> GetLowSpeedState(uint8_t node_id) const;

  /// Current / torque mode target.
  /// @param node_id Motor node ID, 1–15.
  /// @param current_a Target phase current [A].
  /// @param timeout_s Command timeout [s]; 0 = no timeout monitoring.
  /// @return true if sent successfully.
  bool SetTargetCurrent(uint8_t node_id, float current_a, float timeout_s = 0.f);

  /// MIT impedance control (frame without CRC).
  /// Values are saturated by the protocol library before transmission.
  /// @param node_id Motor node ID, 1–15.
  /// @param p_des Desired position [rad], clamped to [-12.5, 12.5].
  /// @param v_des Desired velocity [rad/s], clamped to [-45, 45].
  /// @param kp Position gain, clamped to [0, 500].
  /// @param kd Derivative gain, clamped to [-5, 5].
  /// @param t_ff Feedforward torque [N·m], mapped to [-t_am, +t_am].
  /// @param t_am Torque mapping half-range [N·m]; default 16; values <= 0 use 16 internally.
  /// @return true if sent successfully. Use SetMitControlCrc if firmware requires CRC.
  bool SetMitControl(uint8_t node_id, float p_des, float v_des, float kp,
                     float kd, float t_ff, float t_am = 16.f);

  /// MIT impedance control (frame with CRC field).
  /// @param node_id Motor node ID, 1–15.
  /// @param p_des Desired position [rad], clamped to [-12.5, 12.5].
  /// @param v_des Desired velocity [rad/s], clamped to [-45, 45].
  /// @param kp Position gain, clamped to [0, 500].
  /// @param kd Derivative gain, clamped to [-5, 5].
  /// @param t_ff Feedforward torque [N·m], mapped to [-t_am, +t_am].
  /// @param t_am Torque mapping half-range [N·m]; default 8; values <= 0 use 8 internally.
  /// @return true if sent successfully.
  bool SetMitControlCrc(uint8_t node_id, float p_des, float v_des, float kp,
                        float kd, float t_ff, float t_am = 8.f);

  /// Configure collision protection.
  /// @param node_id Motor node ID, 1–15.
  /// @param current_threshold Trip current [A].
  /// @param time_threshold Trip duration [s].
  /// @return true if sent successfully.
  bool SetCollisionThreshold(uint8_t node_id, float current_threshold,
                             float time_threshold);

  /// Read a motor parameter (blocking).
  /// @param node_id Motor node ID, 1–15.
  /// @param param Parameter ID; units see MotorParam.
  /// @param timeout_s Response wait [s].
  /// @return Parameter value; nullopt on timeout or failure.
  std::optional<float> GetParam(uint8_t node_id, MotorParam param,
                                float timeout_s = 1.f);

  /// Write a motor parameter (blocking).
  /// @param node_id Motor node ID, 1–15.
  /// @param param Parameter ID; units see MotorParam.
  /// @param value Value to write; units match the parameter.
  /// @param timeout_s Response wait [s].
  /// @return true on success.
  bool SetParam(uint8_t node_id, MotorParam param, float value,
                float timeout_s = 1.f);

 private:
  ArmDriver(std::shared_ptr<ArmProtocolLibrary> protocol, CanPort* can);

  std::shared_ptr<ArmProtocolLibrary> arm_protocol_;
};

}  // namespace motor
}  // namespace agx
