#pragma once

// Motor feedback and status types shared by arm and chassis drivers.

#include <cstdint>
#include <optional>
#include <string>

namespace agx {
namespace motor {

/// Status bits decoded from low-speed feedback.
struct DriverStatusBits {
  bool undervoltage{false};       ///< Bus undervoltage.
  bool motor_overtemp{false};     ///< Motor winding over-temperature.
  bool driver_overcurrent{false}; ///< Driver over-current.
  bool driver_overtemp{false};    ///< Driver board over-temperature.
  bool collision_tripped{false};  ///< Arm: collision; chassis: sensor fault.
  bool driver_error{false};       ///< General driver fault.
  bool enabled{false};            ///< Output enabled (status byte bit6).
  bool stall_tripped{false};      ///< Chassis: homing / zero complete.
};

/// High-speed feedback snapshot. Unit of position/velocity depends on protocol:
/// arm = [rad], [rad/s]; chassis = [INC], [RPM].
struct HighSpeedState {
  float position{0.f};      ///< Joint position or encoder count.
  float velocity{0.f};      ///< Joint velocity or RPM.
  float current{0.f};       ///< Phase current [A].
  uint64_t timestamp_ns{0}; ///< Cache update time [ns].
};

/// Low-speed supplementary feedback (slower periodic or on-demand fields).
struct LowSpeedState {
  float bus_voltage_v{0.f};     ///< DC bus voltage [V].
  int16_t driver_temp_deg{0};   ///< Driver temperature [°C].
  int8_t motor_temp_deg{0};     ///< Motor temperature [°C].
  float bus_current{0.f};       ///< Bus current [A].
  uint8_t status_raw{0};        ///< Raw status byte from firmware.
  DriverStatusBits status{};    ///< Parsed status bits.
  uint64_t timestamp_ns{0};    ///< Cache update time [ns].
};

/// Version query result (arm protocol only).
struct VersionInfo {
  std::string software;         ///< Software / firmware version string.
  std::string hardware;         ///< Hardware version string.
  std::string motor;            ///< Motor model / variant string.
  uint64_t timestamp_ns{0};    ///< Response receive time [ns].
};

}  // namespace motor
}  // namespace agx
