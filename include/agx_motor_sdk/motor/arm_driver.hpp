#pragma once

// Arm-protocol motor driver (1 Mbps CAN). Units: rad, rad/s, A.

#include <cstdint>
#include <memory>
#include <optional>

#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/motor/protocol_loader.hpp"
#include "agx_motor_sdk/motor/types.hpp"

namespace agx {
namespace motor {

class ProtocolLibrary;

/// Motor parameter IDs for GetParam / SetParam (two ASCII bytes each).
enum class ArmMotorParam {
  kId,  ///< Node ID (integer, 1–15).
  kAc,  ///< Profile acceleration [rad/s²], max 12.56.
  kDc,  ///< Profile deceleration [rad/s²], max 12.56.
  kVv,  ///< Profile velocity [rad/s], max 20.
  kIq,  ///< Torque-loop max current limit [A], max 12.
  kOi,  ///< Collision protection current threshold [A], max 12.
  kOt,  ///< Collision protection time threshold [s], max 2; OT=30 and OI=30 disables protection.
  kTx,  ///< Fast feedback mode: 0=active push, 1=response (default 0, stored in flash).
  kTf,  ///< MIT feedforward torque limit [N·m] (t_am), default ±8.
  kSo,  ///< Joint zero offset [rad].
  kPp,  ///< Position-loop proportional gain Kp.
  kKp,  ///< Velocity-loop proportional gain Kp.
  kKi,  ///< Velocity-loop integral gain Ki.
};

/// Arm-protocol motor driver.
///
/// Node IDs: 1–15. When using AttachCanPort, RX is passive: incoming frames are
/// delivered by the CanPort receive handler and forwarded to HandleRxOnce.
class ArmDriver {
 public:
  /// Construct driver and load libagx_motor_protocol_arm.
  /// @param can Optional opened CanPort; if non-null, AttachCanPort is called.
  explicit ArmDriver(std::shared_ptr<CanPort> can = nullptr);
  ~ArmDriver();

  ArmDriver(const ArmDriver&) = delete;
  ArmDriver& operator=(const ArmDriver&) = delete;

  /// Bind CAN for TX/RX. Registers an internal TX callback and a receive handler.
  /// @param can Opened CanPort shared pointer. Must remain valid until Close().
  void AttachCanPort(std::shared_ptr<CanPort> can);

  /// Detach CAN handler and destroy the native protocol handle (motor_destroy).
  void Close();

  /// Parse one received CAN frame and update internal caches.
  /// @param frame Received frame (id, data, timestamp_ns).
  /// @return true if the frame was recognized and state was updated; false if ignored.
  bool HandleRxOnce(const CanFrame& frame);

  /// Read cached high-speed feedback.
  /// @param node_id Motor node ID, 1–15.
  /// @return position [rad], velocity [rad/s], current [A], timestamp_ns; nullopt if no data.
  std::optional<HighSpeedState> GetHighSpeedState(uint8_t node_id) const;

  /// Read cached low-speed feedback.
  /// @param node_id Motor node ID, 1–15.
  /// @return bus voltage, temperatures, status bits; nullopt if no data.
  std::optional<LowSpeedState> GetLowSpeedState(uint8_t node_id) const;

  /// Query firmware version (blocking).
  /// @param node_id Motor node ID, 1–15.
  /// @param timeout_s Response wait time [s].
  /// @return software/hardware/motor strings; nullopt on timeout or send failure.
  std::optional<VersionInfo> GetVersion(uint8_t node_id, float timeout_s = 1.f);

  /// Enable or disable driver output.
  /// @param node_id Motor node ID, 1–15.
  /// @param enable true = enable, false = disable.
  /// @param has_brake true to include brake command in the frame; false if no brake.
  /// @param brake_on Brake intent when has_brake is true.
  /// @return true if the command was sent successfully.
  bool SetEnable(uint8_t node_id, bool enable, bool has_brake = false,
                 bool brake_on = false);

  /// Velocity mode target.
  /// @param node_id Motor node ID, 1–15.
  /// @param rad_s Target angular velocity [rad/s].
  /// @param timeout_s Command timeout [s]; 0 = no timeout monitoring.
  /// @return true if sent successfully.
  bool SetTargetVelocity(uint8_t node_id, float rad_s, float timeout_s = 0.f);

  /// Position mode target.
  /// @param node_id Motor node ID, 1–15.
  /// @param rad Target joint position [rad].
  /// @param timeout_s Command timeout [s]; 0 = no timeout monitoring.
  /// @return true if sent successfully.
  bool SetTargetPosition(uint8_t node_id, float rad, float timeout_s = 0.f);

  /// Current / torque mode target.
  /// @param node_id Motor node ID, 1–15.
  /// @param current_a Target phase current [A].
  /// @param timeout_s Command timeout [s]; 0 = no timeout monitoring.
  /// @return true if sent successfully.
  bool SetTargetCurrent(uint8_t node_id, float current_a, float timeout_s = 0.f);

  /// Reset or enter/save calibration.
  /// @param node_id Motor node ID, 1–15.
  /// @param mode 0 = normal reset; 1 = enter calibration; 2 = save calibration (reset again after).
  /// @param timeout_s Blocking response wait [s]; if > 0, waits for ACK.
  /// @return true on successful send (and ACK when timeout_s > 0).
  bool SetReset(uint8_t node_id, int mode = 0, float timeout_s = 1.f);

  /// Clear all fault and alarm states.
  /// @param node_id Motor node ID, 1–15.
  /// @param timeout_s Blocking response wait [s].
  /// @return true on successful send (and ACK when timeout_s > 0).
  bool SetClearError(uint8_t node_id, float timeout_s = 1.f);

  /// Set mechanical zero offset.
  /// @param node_id Motor node ID, 1–15.
  /// @param zero_offset Offset added to zero [rad]; 0 = mark current position as zero.
  /// @param save_to_flash true to persist to non-volatile memory.
  /// @param timeout_s Blocking response wait [s].
  /// @return true on successful send (and ACK when timeout_s > 0).
  bool SetZeroOffset(uint8_t node_id, float zero_offset = 0.f,
                     bool save_to_flash = false, float timeout_s = 1.f);

  /// Set motion profile acceleration and deceleration.
  /// @param node_id Motor node ID, 1–15.
  /// @param acceleration Profile acceleration [rad/s²].
  /// @param deceleration Profile deceleration [rad/s²].
  /// @return true if sent successfully.
  bool SetProfileAccDec(uint8_t node_id, float acceleration, float deceleration);

  /// Set motion profile velocity limit.
  /// @param node_id Motor node ID, 1–15.
  /// @param rad_s Profile velocity cap [rad/s].
  /// @return true if sent successfully.
  bool SetProfileVelocity(uint8_t node_id, float rad_s);

  /// Set phase current limit.
  /// @param node_id Motor node ID, 1–15.
  /// @param current_a Current limit [A].
  /// @return true if sent successfully.
  bool SetCurrentLimit(uint8_t node_id, float current_a);

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
  /// @param param Parameter ID; units see ArmMotorParam.
  /// @param timeout_s Response wait [s].
  /// @return Parameter value; nullopt on timeout or failure.
  std::optional<float> GetParam(uint8_t node_id, ArmMotorParam param,
                                float timeout_s = 1.f);

  /// Write a motor parameter (blocking).
  /// @param node_id Motor node ID, 1–15.
  /// @param param Parameter ID; units see ArmMotorParam.
  /// @param value Value to write; units match the parameter.
  /// @param timeout_s Response wait [s].
  /// @return true on success.
  bool SetParam(uint8_t node_id, ArmMotorParam param, float value,
                float timeout_s = 1.f);

 private:
  std::shared_ptr<ProtocolLibrary> protocol_;
  std::shared_ptr<CanPort> can_;
  AgxMotorProtocolHandle* handle_{nullptr};

  static int TxCallback(void* ctx, uint32_t can_id, uint8_t* data, uint8_t dlc);
  void OnCanFrame(const CanFrame& frame);
};

}  // namespace motor
}  // namespace agx
