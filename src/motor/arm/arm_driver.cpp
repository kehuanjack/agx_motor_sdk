#include "agx_motor_sdk/motor/arm/arm_driver.hpp"

#include <utility>

namespace agx {
namespace motor {
namespace {

LowSpeedState ToLowSpeed(const AgxMotorLowSpeedFeedback& raw) {
  LowSpeedState out;
  out.bus_voltage_v = raw.bus_voltage_v;
  out.driver_temp_deg = raw.driver_temp_deg;
  out.motor_temp_deg = raw.motor_temp_deg;
  out.bus_current = raw.bus_current;
  out.status_raw = raw.status_raw;
  out.timestamp_ns = raw.timestamp_ns;
  out.status.undervoltage = raw.status.undervoltage != 0;
  out.status.motor_overtemp = raw.status.motor_overtemp != 0;
  out.status.driver_overcurrent = raw.status.driver_overcurrent != 0;
  out.status.driver_overtemp = raw.status.driver_overtemp != 0;
  out.status.collision_tripped = raw.status.collision_tripped != 0;
  out.status.driver_error = raw.status.driver_error != 0;
  out.status.enabled = raw.status.enabled != 0;
  out.status.stall_tripped = raw.status.stall_tripped != 0;
  return out;
}

std::pair<uint8_t, uint8_t> MotorParamIndices(ArmDriver::MotorParam param) {
  switch (param) {
    case ArmDriver::MotorParam::kId:
      return {'i', 'd'};
    case ArmDriver::MotorParam::kAc:
      return {'a', 'c'};
    case ArmDriver::MotorParam::kDc:
      return {'d', 'c'};
    case ArmDriver::MotorParam::kVv:
      return {'v', 'v'};
    case ArmDriver::MotorParam::kIq:
      return {'i', 'q'};
    case ArmDriver::MotorParam::kOi:
      return {'o', 'i'};
    case ArmDriver::MotorParam::kOt:
      return {'o', 't'};
    case ArmDriver::MotorParam::kTx:
      return {'t', 'x'};
    case ArmDriver::MotorParam::kTc:
      return {'t', 'c'};
    case ArmDriver::MotorParam::kTf:
      return {'t', 'f'};
    case ArmDriver::MotorParam::kSo:
      return {'s', 'o'};
    case ArmDriver::MotorParam::kPp:
      return {'p', 'p'};
    case ArmDriver::MotorParam::kKp:
      return {'k', 'p'};
    case ArmDriver::MotorParam::kKi:
      return {'k', 'i'};
  }
  return {0, 0};
}

}  // namespace

ArmDriver::ArmDriver(CanPort* can) : ArmDriver(ArmProtocolLibrary::Load(), can) {}

ArmDriver::ArmDriver(std::shared_ptr<ArmProtocolLibrary> protocol, CanPort* can)
    : MotorDriver(protocol, "arm", can), arm_protocol_(std::move(protocol)) {}

std::optional<LowSpeedState> ArmDriver::GetLowSpeedState(uint8_t node_id) const {
  if (handle() == nullptr) {
    return std::nullopt;
  }
  AgxMotorLowSpeedFeedback raw{};
  if (!arm_protocol_->GetLowSpeedFeedback(handle(), node_id, &raw)) {
    return std::nullopt;
  }
  return ToLowSpeed(raw);
}

bool ArmDriver::SetTargetCurrent(uint8_t node_id, float current_a,
                                float timeout_s) {
  return SetTarget(node_id, current_a, 2, timeout_s);
}

bool ArmDriver::SetMitControl(uint8_t node_id, float p_des, float v_des, float kp,
                              float kd, float t_ff, float t_am) {
  return handle() != nullptr &&
         arm_protocol_->SetMitControl(handle(), node_id, p_des, v_des, kp, kd, t_ff,
                                  t_am) != 0;
}

bool ArmDriver::SetMitControlCrc(uint8_t node_id, float p_des, float v_des,
                                 float kp, float kd, float t_ff, float t_am) {
  return handle() != nullptr &&
         arm_protocol_->SetMitControlCrc(handle(), node_id, p_des, v_des, kp, kd,
                                     t_ff, t_am) != 0;
}

bool ArmDriver::SetCollisionThreshold(uint8_t node_id, float current_threshold,
                                     float time_threshold) {
  return handle() != nullptr &&
         arm_protocol_->SetCollisionThreshold(handle(), node_id, current_threshold,
                                          time_threshold) != 0;
}

std::optional<float> ArmDriver::GetParam(uint8_t node_id, MotorParam param,
                                         float timeout_s) {
  if (handle() == nullptr) {
    return std::nullopt;
  }
  const auto [idx1, idx2] = MotorParamIndices(param);
  float value = 0.f;
  if (!arm_protocol_->GetParam(handle(), node_id, idx1, idx2, timeout_s, &value)) {
    return std::nullopt;
  }
  return value;
}

bool ArmDriver::SetParam(uint8_t node_id, MotorParam param, float value,
                         float timeout_s) {
  if (handle() == nullptr) {
    return false;
  }
  const auto [idx1, idx2] = MotorParamIndices(param);
  return arm_protocol_->SetParam(handle(), node_id, idx1, idx2, value, timeout_s) != 0;
}

}  // namespace motor
}  // namespace agx
