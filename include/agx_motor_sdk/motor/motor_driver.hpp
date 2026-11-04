#pragma once

// Protocol-independent motor driver base. Units: rad, rad/s, A.
// Protocol drivers: arm/arm_driver.hpp (ArmDriver), rb200/rb200_driver.hpp (Rb200Driver).

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/motor/protocol_loader.hpp"
#include "agx_motor_sdk/motor/types.hpp"

namespace agx {
namespace motor {

/// Common motor driver: owns one protocol handle and the commands every
/// protocol library supports.
///
/// Node IDs: 1–15. When using AttachCanPort, RX is passive: incoming frames are
/// delivered by the CanPort receive handler and forwarded to HandleRxOnce.
class MotorDriver {
 public:
  virtual ~MotorDriver();

  MotorDriver(const MotorDriver&) = delete;
  MotorDriver& operator=(const MotorDriver&) = delete;

  /// Protocol name: ``"arm"`` or ``"rb200"``.
  const std::string& protocol_name() const { return protocol_name_; }

  /// Bind CAN for TX/RX. Registers an internal TX callback and a receive handler.
  /// @param can Opened CanPort pointer. Must remain valid until :meth:`Close`.
  ///        Does not take ownership; only detaches handlers on :meth:`Close`.
  void AttachCanPort(CanPort* can);

  /// Detach CAN handlers and destroy the native protocol handle (motor_destroy).
  /// Does not close or destroy the CanPort; that remains the owner's duty.
  void Close();

  /// Parse one received CAN frame and update internal caches.
  /// @param frame Received frame (id, data, timestamp_ns).
  /// @return true if the frame was recognized and state was updated; false if ignored.
  bool HandleRxOnce(const CanFrame& frame);

  /// Read cached high-speed feedback.
  /// @param node_id Motor node ID, 1–15.
  /// @return position [rad], velocity [rad/s], current [A], timestamp_ns; nullopt if no data.
  std::optional<HighSpeedState> GetHighSpeedState(uint8_t node_id) const;

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

  /// Generic target command with explicit control mode byte.
  /// @param node_id Motor node ID, 1–15.
  /// @param value Target value in motor space; units depend on ``mode``.
  /// @param mode Control mode byte (protocol-defined; e.g. 0=vel, 1=pos, 2=current, 0xFF=unchanged).
  /// @param timeout_s Command timeout [s]; 0 = no timeout monitoring.
  /// @return true if sent successfully.
  bool SetTarget(uint8_t node_id, float value, uint8_t mode, float timeout_s = 0.f);

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

 protected:
  /// Validate ``protocol``, create the native handle and attach ``can`` if non-null.
  /// @throws std::runtime_error if the library failed to load or the handle cannot be created.
  MotorDriver(std::shared_ptr<ProtocolLibrary> protocol, std::string protocol_name,
              CanPort* can);

  AgxMotorProtocolHandle* handle() const { return handle_; }

 private:
  std::shared_ptr<ProtocolLibrary> protocol_;
  std::string protocol_name_;
  CanPort* can_{nullptr};
  AgxMotorProtocolHandle* handle_{nullptr};

  static int TxCallback(void* ctx, uint32_t can_id, uint8_t* data, uint8_t dlc);
  void OnCanFrame(const CanFrame& frame);
};

}  // namespace motor
}  // namespace agx
