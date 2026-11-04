#pragma once

// RB200 motor driver (CiA402-style low-speed feedback). Units: rad, rad/s, A.

#include <cstdint>
#include <memory>
#include <optional>

#include "agx_motor_sdk/motor/motor_driver.hpp"
#include "agx_motor_sdk/motor/rb200/rb200_protocol_library.hpp"
#include "agx_motor_sdk/motor/rb200/types.hpp"

namespace agx {
namespace motor {

/// RB200 motor driver (libagx_motor_protocol_rb200.so).
///
/// Supports the common :class:`MotorDriver` commands plus RB200 low-speed
/// feedback. MIT, collision protection and motor parameters are not available.
class Rb200Driver : public MotorDriver {
 public:
  /// Construct driver and load the RB200 protocol library.
  /// @param can Optional opened CanPort; if non-null, AttachCanPort is called.
  ///        Non-owning: the caller must keep ``can`` alive until :meth:`Close`.
  /// @throws std::runtime_error if the library cannot be loaded.
  explicit Rb200Driver(CanPort* can = nullptr);

  /// Read cached RB200 low-speed feedback (CiA402 status word, error code).
  /// @param node_id Motor node ID, 1–15.
  /// @return nullopt if no data.
  std::optional<Rb200LowSpeedState> GetLowSpeedState(uint8_t node_id) const;

 private:
  Rb200Driver(std::shared_ptr<Rb200ProtocolLibrary> protocol, CanPort* can);

  std::shared_ptr<Rb200ProtocolLibrary> rb200_protocol_;
};

}  // namespace motor
}  // namespace agx
