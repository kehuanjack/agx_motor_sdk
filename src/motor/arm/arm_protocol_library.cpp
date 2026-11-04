#include "agx_motor_sdk/motor/arm/arm_protocol_library.hpp"

namespace agx {
namespace motor {

ArmProtocolLibrary::ArmProtocolLibrary(const std::string& path)
    : ProtocolLibrary(kLibraryName, kEnvVar, path) {
  if (!is_open()) {
    return;
  }
  ls_ = LoadSymbol<AgxMotorArmGetLowSpeedFeedbackFn>("motor_get_low_speed_feedback");
  mit_ = LoadSymbol<MitFn>("motor_set_mit_control");
  mit_crc_ = LoadSymbol<MitFn>("motor_set_mit_control_crc");
  collision_ = LoadSymbol<CollisionFn>("motor_set_collision_threshold");
  get_param_ = LoadSymbol<GetParamFn>("motor_get_param");
  set_param_ = LoadSymbol<SetParamFn>("motor_set_param");
}

std::shared_ptr<ArmProtocolLibrary> ArmProtocolLibrary::Load(const std::string& path) {
  return std::shared_ptr<ArmProtocolLibrary>(new ArmProtocolLibrary(path));
}

int ArmProtocolLibrary::GetLowSpeedFeedback(AgxMotorProtocolHandle* handle,
                                            uint8_t node_id,
                                            AgxMotorLowSpeedFeedback* out) const {
  return ls_ != nullptr ? ls_(handle, node_id, out) : 0;
}

int ArmProtocolLibrary::SetMitControl(AgxMotorProtocolHandle* handle, uint8_t node_id,
                                      float p_des, float v_des, float kp, float kd,
                                      float t_ff, float t_am) {
  return mit_ != nullptr ? mit_(handle, node_id, p_des, v_des, kp, kd, t_ff, t_am) : 0;
}

int ArmProtocolLibrary::SetMitControlCrc(AgxMotorProtocolHandle* handle,
                                         uint8_t node_id, float p_des, float v_des,
                                         float kp, float kd, float t_ff, float t_am) {
  return mit_crc_ != nullptr
             ? mit_crc_(handle, node_id, p_des, v_des, kp, kd, t_ff, t_am)
             : 0;
}

int ArmProtocolLibrary::SetCollisionThreshold(AgxMotorProtocolHandle* handle,
                                              uint8_t node_id,
                                              float current_threshold,
                                              float time_threshold) {
  return collision_ != nullptr
             ? collision_(handle, node_id, current_threshold, time_threshold)
             : 0;
}

int ArmProtocolLibrary::GetParam(AgxMotorProtocolHandle* handle, uint8_t node_id,
                                 uint8_t idx1, uint8_t idx2, float timeout_s,
                                 float* out) {
  return get_param_ != nullptr
             ? get_param_(handle, node_id, idx1, idx2, timeout_s, out)
             : 0;
}

int ArmProtocolLibrary::SetParam(AgxMotorProtocolHandle* handle, uint8_t node_id,
                                 uint8_t idx1, uint8_t idx2, float value,
                                 float timeout_s) {
  return set_param_ != nullptr
             ? set_param_(handle, node_id, idx1, idx2, value, timeout_s)
             : 0;
}

}  // namespace motor
}  // namespace agx
