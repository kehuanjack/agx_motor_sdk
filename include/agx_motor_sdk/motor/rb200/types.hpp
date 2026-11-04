#pragma once

// RB200 protocol feedback types.

#include <cstdint>

#include "agx_motor_sdk/motor/types.hpp"

namespace agx {
namespace motor {

/// Decoded CiA402 status word bits (RB200 low-speed feedback).
struct Cia402StatusBits {
  bool ready_to_switch_on{false};    ///< bit0
  bool switched_on{false};           ///< bit1
  bool operation_enabled{false};     ///< bit2
  bool fault{false};                 ///< bit3
  bool voltage_enabled{false};       ///< bit4
  bool quick_stop{false};            ///< bit5 (false = quick stop active)
  bool switch_on_disabled{false};    ///< bit6
  bool warning{false};               ///< bit7
  bool remote{false};                ///< bit9
  bool target_reached{false};        ///< bit10
  bool internal_limit_active{false}; ///< bit11
  bool homing_complete{false};       ///< bit15
};

/// RB200 low-speed feedback (CAN ID 0x530 + node).
struct Rb200LowSpeedState {
  uint16_t status_word{0};    ///< Raw CiA402 status word.
  uint16_t error_code{0};     ///< Driver error code.
  uint16_t tpdo2_count{0};    ///< TPDO2 frame counter.
  Cia402StatusBits status{};  ///< Decoded status_word bits.
  uint64_t timestamp_ns{0};   ///< Cache update time [ns].
};

}  // namespace motor
}  // namespace agx
