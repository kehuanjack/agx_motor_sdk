#include "agx_motor_sdk/mechanism/rb200/rb200_motor.hpp"

namespace agx::motor::mechanism {

std::optional<agx::motor::Rb200LowSpeedState> Rb200Motor::GetLowSpeedState() const {
  const auto driver = std::dynamic_pointer_cast<agx::motor::Rb200Driver>(LockDriver());
  return driver ? driver->GetLowSpeedState(NodeId()) : std::nullopt;
}

}  // namespace agx::motor::mechanism
