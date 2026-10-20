#include "agx_motor_sdk/motor/arm_driver.hpp"

#include <cstring>
#include <stdexcept>

namespace agx {
namespace motor {
namespace {

HighSpeedState ToHighSpeed(const AgxMotorHighSpeedFeedback& raw) {
  return HighSpeedState{raw.position, raw.velocity, raw.current,
                        raw.timestamp_ns};
}

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

VersionInfo ToVersion(const AgxMotorVersionInfo& raw) {
  VersionInfo out;
  out.software.assign(raw.software, strnlen(raw.software, sizeof(raw.software)));
  out.hardware.assign(raw.hardware, strnlen(raw.hardware, sizeof(raw.hardware)));
  out.motor.assign(raw.motor, strnlen(raw.motor, sizeof(raw.motor)));
  out.timestamp_ns = raw.timestamp_ns;
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

bool AwaitOk(int sent, float timeout_s, int ok) {
  if (sent == 0) {
    return false;
  }
  if (timeout_s > 0.f) {
    return ok != 0;
  }
  return true;
}

}  // namespace

ArmDriver::ArmDriver(CanPort* can) {
  protocol_ = ProtocolLibrary::Load(ProtocolVariant::kArm);
  if (!protocol_->load_error().empty()) {
    throw std::runtime_error("failed to load arm motor protocol library: " +
                             protocol_->load_error());
  }
  handle_ = protocol_->Create();
  if (handle_ == nullptr) {
    throw std::runtime_error("failed to create arm motor protocol handle");
  }
  if (can != nullptr) {
    AttachCanPort(can);
  }
}

ArmDriver::~ArmDriver() { Close(); }

void ArmDriver::AttachCanPort(CanPort* can) {
  can_ = can;
  if (handle_ == nullptr || can_ == nullptr) {
    return;
  }
  protocol_->SetTxCallback(handle_, &ArmDriver::TxCallback, this);
  can_->SetReceiveHandler([this](const CanFrame& frame) { OnCanFrame(frame); });
}

void ArmDriver::Close() {
  if (can_ != nullptr) {
    can_->SetReceiveHandler(nullptr);
    can_ = nullptr;
  }
  if (protocol_ != nullptr && handle_ != nullptr) {
    protocol_->Destroy(handle_);
    handle_ = nullptr;
  }
}

bool ArmDriver::HandleRxOnce(const CanFrame& frame) {
  if (handle_ == nullptr) {
    return false;
  }
  return protocol_->HandleRxOnce(handle_, frame.id, frame.data.data(),
                                 static_cast<uint8_t>(frame.data.size()),
                                 frame.timestamp_ns) != 0;
}

std::optional<HighSpeedState> ArmDriver::GetHighSpeedState(uint8_t node_id) const {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  AgxMotorHighSpeedFeedback raw{};
  if (!protocol_->GetHighSpeedFeedback(handle_, node_id, &raw)) {
    return std::nullopt;
  }
  return ToHighSpeed(raw);
}

std::optional<LowSpeedState> ArmDriver::GetLowSpeedState(uint8_t node_id) const {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  AgxMotorLowSpeedFeedback raw{};
  if (!protocol_->GetLowSpeedFeedback(handle_, node_id, &raw)) {
    return std::nullopt;
  }
  return ToLowSpeed(raw);
}

std::optional<VersionInfo> ArmDriver::GetVersion(uint8_t node_id, float timeout_s) {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  AgxMotorVersionInfo raw{};
  if (!protocol_->GetVersion(handle_, node_id, timeout_s, &raw)) {
    return std::nullopt;
  }
  return ToVersion(raw);
}

bool ArmDriver::SetEnable(uint8_t node_id, bool enable, bool has_brake,
                          bool brake_on) {
  return handle_ != nullptr &&
         protocol_->SetEnable(handle_, node_id, enable ? 1 : 0, has_brake ? 1 : 0,
                              brake_on ? 1 : 0) != 0;
}

bool ArmDriver::SetTarget(uint8_t node_id, float value, uint8_t mode,
                          float timeout_s) {
  return handle_ != nullptr &&
         protocol_->SetTarget(handle_, node_id, value, mode, timeout_s) != 0;
}

bool ArmDriver::SetTargetVelocity(uint8_t node_id, float rad_s, float timeout_s) {
  return SetTarget(node_id, rad_s, 0, timeout_s);
}

bool ArmDriver::SetTargetPosition(uint8_t node_id, float rad, float timeout_s) {
  return SetTarget(node_id, rad, 1, timeout_s);
}

bool ArmDriver::SetTargetCurrent(uint8_t node_id, float current_a,
                                float timeout_s) {
  return SetTarget(node_id, current_a, 2, timeout_s);
}

bool ArmDriver::SetReset(uint8_t node_id, int mode, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetReset(handle_, node_id, mode, timeout_s, &ok),
                 timeout_s, ok);
}

bool ArmDriver::SetClearError(uint8_t node_id, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetClearError(handle_, node_id, timeout_s, &ok),
                 timeout_s, ok);
}

bool ArmDriver::SetZeroOffset(uint8_t node_id, float zero_offset,
                              bool save_to_flash, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetZeroOffset(handle_, node_id, zero_offset,
                                          save_to_flash ? 1 : 0, timeout_s, &ok),
                 timeout_s, ok);
}

bool ArmDriver::SetProfileAccDec(uint8_t node_id, float acceleration,
                                 float deceleration) {
  return handle_ != nullptr &&
         protocol_->SetProfileAccDec(handle_, node_id, acceleration,
                                     deceleration) != 0;
}

bool ArmDriver::SetProfileVelocity(uint8_t node_id, float rad_s) {
  return handle_ != nullptr &&
         protocol_->SetProfileVel(handle_, node_id, rad_s) != 0;
}

bool ArmDriver::SetCurrentLimit(uint8_t node_id, float current_a) {
  return handle_ != nullptr &&
         protocol_->SetCurrentLimit(handle_, node_id, current_a) != 0;
}

bool ArmDriver::SetMitControl(uint8_t node_id, float p_des, float v_des, float kp,
                              float kd, float t_ff, float t_am) {
  return handle_ != nullptr &&
         protocol_->SetMitControl(handle_, node_id, p_des, v_des, kp, kd, t_ff,
                                  t_am) != 0;
}

bool ArmDriver::SetMitControlCrc(uint8_t node_id, float p_des, float v_des,
                                 float kp, float kd, float t_ff, float t_am) {
  return handle_ != nullptr &&
         protocol_->SetMitControlCrc(handle_, node_id, p_des, v_des, kp, kd,
                                     t_ff, t_am) != 0;
}

bool ArmDriver::SetCollisionThreshold(uint8_t node_id, float current_threshold,
                                     float time_threshold) {
  return handle_ != nullptr &&
         protocol_->SetCollisionThreshold(handle_, node_id, current_threshold,
                                          time_threshold) != 0;
}

std::optional<float> ArmDriver::GetParam(uint8_t node_id, MotorParam param,
                                         float timeout_s) {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  const auto [idx1, idx2] = MotorParamIndices(param);
  float value = 0.f;
  if (!protocol_->GetParam(handle_, node_id, idx1, idx2, timeout_s, &value)) {
    return std::nullopt;
  }
  return value;
}

bool ArmDriver::SetParam(uint8_t node_id, MotorParam param, float value,
                         float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  const auto [idx1, idx2] = MotorParamIndices(param);
  return protocol_->SetParam(handle_, node_id, idx1, idx2, value, timeout_s) != 0;
}

int ArmDriver::TxCallback(void* ctx, uint32_t can_id, uint8_t* data, uint8_t dlc) {
  auto* self = static_cast<ArmDriver*>(ctx);
  if (self == nullptr || self->can_ == nullptr) {
    return 0;
  }
  CanFrame frame;
  frame.id = can_id;
  frame.data.assign(data, data + dlc);
  self->can_->Send(frame);
  return 1;
}

void ArmDriver::OnCanFrame(const CanFrame& frame) { HandleRxOnce(frame); }

}  // namespace motor
}  // namespace agx
