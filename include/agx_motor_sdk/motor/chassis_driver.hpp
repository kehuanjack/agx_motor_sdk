#pragma once

// Chassis-protocol motor driver (500 kbps CAN). Units: INC, RPM, A.

#include <cstdint>
#include <memory>
#include <optional>

#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/motor/protocol_loader.hpp"
#include "agx_motor_sdk/motor/types.hpp"

namespace agx {
namespace motor {

/// Chassis-protocol motor driver.
///
/// Node IDs: 1–15. Uses a different CAN format than ArmDriver. Do not attach
/// both drivers to the same CanPort; dispatch RX by node_id if protocols share a bus.
///
/// Low-speed status field semantics: collision_tripped = sensor fault;
/// stall_tripped = homed; enabled = status bit6 set.
class ChassisDriver {
 public:
  /// Construct driver and load libagx_motor_protocol_chassis.
  /// @param can Optional opened CanPort; if non-null, AttachCanPort is called.
  explicit ChassisDriver(std::shared_ptr<CanPort> can = nullptr);
  ~ChassisDriver();

  ChassisDriver(const ChassisDriver&) = delete;
  ChassisDriver& operator=(const ChassisDriver&) = delete;

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
  /// @return position [INC], velocity [RPM], current [A], timestamp_ns; nullopt if no data.
  std::optional<HighSpeedState> GetHighSpeedState(uint8_t node_id) const;

  /// Read cached low-speed feedback.
  /// @param node_id Motor node ID, 1–15.
  /// @return bus voltage, temperatures, status bits; nullopt if no data.
  std::optional<LowSpeedState> GetLowSpeedState(uint8_t node_id) const;

  /// Enable or disable driver output (0x420).
  /// @param node_id Motor node ID, 1–15.
  /// @param enable true = enable, false = disable.
  /// @param has_brake true to include brake command; false if no brake hardware.
  /// @param brake_on Brake intent when has_brake is true.
  /// @return true if the command was sent successfully.
  bool SetEnable(uint8_t node_id, bool enable, bool has_brake = false,
                 bool brake_on = false);

  /// RPM mode target (0x410 mode 0).
  /// @param node_id Motor node ID, 1–15.
  /// @param rpm Target speed [RPM].
  /// @param timeout_s Command timeout [s]; 0 = no timeout monitoring.
  /// @return true if sent successfully.
  bool SetTargetRpm(uint8_t node_id, float rpm, float timeout_s = 0.f);

  /// Position mode target (0x410 mode 1).
  /// @param node_id Motor node ID, 1–15.
  /// @param inc Target encoder increment [INC].
  /// @param timeout_s Command timeout [s]; 0 = no timeout monitoring.
  /// @return true if sent successfully.
  bool SetTargetInc(uint8_t node_id, float inc, float timeout_s = 0.f);

  /// Current mode target (0x410 mode 2).
  /// @param node_id Motor node ID, 1–15.
  /// @param current_a Target phase current [A].
  /// @param timeout_s Command timeout [s]; 0 = no timeout monitoring.
  /// @return true if sent successfully.
  bool SetTargetCurrent(uint8_t node_id, float current_a, float timeout_s = 0.f);

  /// Reset or enter/save calibration (0x000).
  /// @param node_id Motor node ID, 1–15.
  /// @param mode 0 = normal reset; 1 = enter calibration; 2 = save calibration.
  /// @param timeout_s Blocking response wait [s].
  /// @return true on successful send (and ACK when timeout_s > 0).
  bool SetReset(uint8_t node_id, int mode = 0, float timeout_s = 1.f);

  /// Clear all fault and alarm states (0x010).
  /// @param node_id Motor node ID, 1–15.
  /// @param timeout_s Blocking response wait [s].
  /// @return true on successful send (and ACK when timeout_s > 0).
  bool SetClearError(uint8_t node_id, float timeout_s = 1.f);

  /// Set encoder zero offset (0x020).
  /// @param node_id Motor node ID, 1–15.
  /// @param zero_offset Offset [INC]; 0 = mark current position as zero.
  /// @param save_to_flash true to persist to non-volatile memory.
  /// @param timeout_s Blocking response wait [s].
  /// @return true on successful send (and ACK when timeout_s > 0).
  bool SetZeroOffset(uint8_t node_id, float zero_offset = 0.f,
                     bool save_to_flash = false, float timeout_s = 1.f);

  /// Set motion profile acceleration and deceleration (0x430).
  /// @param node_id Motor node ID, 1–15.
  /// @param acceleration Profile acceleration [RPM/s] (uint16, clamped in library).
  /// @param deceleration Profile deceleration [RPM/s] (uint16, clamped in library).
  /// @return true if sent successfully.
  bool SetProfileAccDec(uint8_t node_id, float acceleration, float deceleration);

  /// Set motion profile velocity limit (0x440).
  /// @param node_id Motor node ID, 1–15.
  /// @param rpm Profile velocity cap [RPM] (uint16, clamped in library).
  /// @return true if sent successfully.
  bool SetProfileVelocity(uint8_t node_id, float rpm);

  /// Set phase current limit (0x450).
  /// @param node_id Motor node ID, 1–15.
  /// @param current_a Current limit [A].
  /// @return true if sent successfully.
  bool SetCurrentLimit(uint8_t node_id, float current_a);

  /// Set chassis stiffness (0x470).
  /// @param node_id Motor node ID, 1–15.
  /// @param stiffness Stiffness wire value; library clamps to 10–2000.
  /// @return true if sent successfully.
  bool SetStiffness(uint8_t node_id, uint16_t stiffness);

 private:
  std::shared_ptr<ProtocolLibrary> protocol_;
  std::shared_ptr<CanPort> can_;
  AgxMotorProtocolHandle* handle_{nullptr};

  static int TxCallback(void* ctx, uint32_t can_id, uint8_t* data, uint8_t dlc);
  void OnCanFrame(const CanFrame& frame);
};

}  // namespace motor
}  // namespace agx
