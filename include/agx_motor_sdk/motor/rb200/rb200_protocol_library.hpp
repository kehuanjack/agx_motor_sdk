#pragma once

#include <memory>
#include <string>

#include "agx_motor_sdk/motor/protocol_loader.hpp"
#include "agx_motor_sdk/motor/rb200/rb200_protocol_api.h"

namespace agx {
namespace motor {

/// libagx_motor_protocol_rb200.so (override with ``AGX_MOTOR_SDK_RB200_LIB``).
class Rb200ProtocolLibrary : public ProtocolLibrary {
 public:
  static constexpr const char* kLibraryName = "libagx_motor_protocol_rb200.so";
  static constexpr const char* kEnvVar = "AGX_MOTOR_SDK_RB200_LIB";

  /// Load the RB200 protocol library; check ``load_error()`` before use.
  static std::shared_ptr<Rb200ProtocolLibrary> Load(const std::string& path = {});

  int GetLowSpeedFeedback(AgxMotorProtocolHandle* handle, uint8_t node_id,
                          AgxMotorRb200LowSpeedFeedback* out) const;

 private:
  explicit Rb200ProtocolLibrary(const std::string& path);

  AgxMotorRb200GetLowSpeedFeedbackFn ls_{nullptr};
};

}  // namespace motor
}  // namespace agx
