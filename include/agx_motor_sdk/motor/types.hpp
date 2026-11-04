#pragma once

// Feedback types shared by every motor protocol driver.
// Protocol-specific types: arm/types.hpp, rb200/types.hpp.

#include <cstdint>
#include <string>

namespace agx {
namespace motor {

/// High-speed feedback snapshot (position [rad], velocity [rad/s], current [A]).
struct HighSpeedState {
  float position{0.f};      ///< Joint position [rad].
  float velocity{0.f};      ///< Joint velocity [rad/s].
  float current{0.f};       ///< Phase current [A].
  uint64_t timestamp_ns{0}; ///< Cache update time [ns].
};

/// Version query result.
struct VersionInfo {
  std::string software;         ///< Software / firmware version string.
  std::string hardware;         ///< Hardware version string.
  std::string motor;            ///< Motor model / variant string.
  uint64_t timestamp_ns{0};    ///< Response receive time [ns].
};

}  // namespace motor
}  // namespace agx
