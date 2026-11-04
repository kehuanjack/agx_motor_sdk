#pragma once

#include <memory>
#include <string>

#include "agx_motor_sdk/motor/arm/arm_protocol_api.h"
#include "agx_motor_sdk/motor/protocol_loader.hpp"

namespace agx {
namespace motor {

/// libagx_motor_protocol_arm.so (override with ``AGX_MOTOR_SDK_ARM_LIB``).
class ArmProtocolLibrary : public ProtocolLibrary {
 public:
  static constexpr const char* kLibraryName = "libagx_motor_protocol_arm.so";
  static constexpr const char* kEnvVar = "AGX_MOTOR_SDK_ARM_LIB";

  /// Load the arm protocol library; check ``load_error()`` before use.
  static std::shared_ptr<ArmProtocolLibrary> Load(const std::string& path = {});

  int GetLowSpeedFeedback(AgxMotorProtocolHandle* handle, uint8_t node_id,
                          AgxMotorLowSpeedFeedback* out) const;
  int SetMitControl(AgxMotorProtocolHandle* handle, uint8_t node_id, float p_des,
                    float v_des, float kp, float kd, float t_ff, float t_am);
  int SetMitControlCrc(AgxMotorProtocolHandle* handle, uint8_t node_id,
                       float p_des, float v_des, float kp, float kd, float t_ff,
                       float t_am);
  int SetCollisionThreshold(AgxMotorProtocolHandle* handle, uint8_t node_id,
                            float current_threshold, float time_threshold);
  int GetParam(AgxMotorProtocolHandle* handle, uint8_t node_id, uint8_t idx1,
               uint8_t idx2, float timeout_s, float* out);
  int SetParam(AgxMotorProtocolHandle* handle, uint8_t node_id, uint8_t idx1,
               uint8_t idx2, float value, float timeout_s);

 private:
  explicit ArmProtocolLibrary(const std::string& path);

  using MitFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float, float, float,
                        float, float, float);
  using CollisionFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float, float);
  using GetParamFn = int (*)(AgxMotorProtocolHandle*, uint8_t, uint8_t,
                             uint8_t, float, float*);
  using SetParamFn = int (*)(AgxMotorProtocolHandle*, uint8_t, uint8_t,
                             uint8_t, float, float);

  AgxMotorArmGetLowSpeedFeedbackFn ls_{nullptr};
  MitFn mit_{nullptr};
  MitFn mit_crc_{nullptr};
  CollisionFn collision_{nullptr};
  GetParamFn get_param_{nullptr};
  SetParamFn set_param_{nullptr};
};

}  // namespace motor
}  // namespace agx
