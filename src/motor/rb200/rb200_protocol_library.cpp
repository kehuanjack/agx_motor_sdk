#include "agx_motor_sdk/motor/rb200/rb200_protocol_library.hpp"

namespace agx {
namespace motor {

Rb200ProtocolLibrary::Rb200ProtocolLibrary(const std::string& path)
    : ProtocolLibrary(kLibraryName, kEnvVar, path) {
  if (!is_open()) {
    return;
  }
  ls_ = LoadSymbol<AgxMotorRb200GetLowSpeedFeedbackFn>("motor_get_low_speed_feedback");
}

std::shared_ptr<Rb200ProtocolLibrary> Rb200ProtocolLibrary::Load(const std::string& path) {
  return std::shared_ptr<Rb200ProtocolLibrary>(new Rb200ProtocolLibrary(path));
}

int Rb200ProtocolLibrary::GetLowSpeedFeedback(
    AgxMotorProtocolHandle* handle, uint8_t node_id,
    AgxMotorRb200LowSpeedFeedback* out) const {
  return ls_ != nullptr ? ls_(handle, node_id, out) : 0;
}

}  // namespace motor
}  // namespace agx
