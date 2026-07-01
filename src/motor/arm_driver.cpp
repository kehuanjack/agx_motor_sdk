#include "agx_motor_sdk/motor/arm_driver.hpp"

#include <cstring>
#include <utility>

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

std::pair<uint8_t, uint8_t> ArmMotorParamIndices(ArmMotorParam param) {
  switch (param) {
    case ArmMotorParam::kId:
      return {'i', 'd'};
    case ArmMotorParam::kAc:
      return {'a', 'c'};
    case ArmMotorParam::kDc:
      return {'d', 'c'};
    case ArmMotorParam::kVv:
      return {'v', 'v'};
    case ArmMotorParam::kIq:
      return {'i', 'q'};
    case ArmMotorParam::kOi:
      return {'o', 'i'};
    case ArmMotorParam::kOt:
      return {'o', 't'};
    case ArmMotorParam::kTx:
      return {'t', 'x'};
    case ArmMotorParam::kTf:
      return {'t', 'f'};
    case ArmMotorParam::kSo:
      return {'s', 'o'};
    case ArmMotorParam::kPp:
      return {'p', 'p'};
    case ArmMotorParam::kKp:
      return {'k', 'p'};
    case ArmMotorParam::kKi:
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

ArmDriver::ArmDriver(std::shared_ptr<CanPort> can) {
  protocol_ = ProtocolLibrary::Load(ProtocolVariant::kArm);
  if (protocol_->load_error().empty()) {
    handle_ = protocol_->Create();
    if (handle_ != nullptr && can != nullptr) {
      AttachCanPort(std::move(can));
    }
  }
}

ArmDriver::~ArmDriver() { Close(); }

void ArmDriver::AttachCanPort(std::shared_ptr<CanPort> can) {
  can_ = std::move(can);
  if (handle_ == nullptr || can_ == nullptr) {
    return;
  }
  protocol_->SetTxCallback(handle_, &ArmDriver::TxCallback, this);
  can_->SetReceiveHandler([this](const CanFrame& frame) { OnCanFrame(frame); });
}

void ArmDriver::Close() {
  if (can_ != nullptr) {
    can_->SetReceiveHandler(nullptr);
    can_.reset();
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

bool ArmDriver::SetTargetVelocity(uint8_t node_id, float rad_s, float timeout_s) {
  return handle_ != nullptr &&
         protocol_->SetTarget(handle_, node_id, rad_s, 0, timeout_s) != 0;
}

bool ArmDriver::SetTargetPosition(uint8_t node_id, float rad, float timeout_s) {
  return handle_ != nullptr &&
         protocol_->SetTarget(handle_, node_id, rad, 1, timeout_s) != 0;
}

bool ArmDriver::SetTargetCurrent(uint8_t node_id, float current_a,
                                float timeout_s) {
  return handle_ != nullptr &&
         protocol_->SetTarget(handle_, node_id, current_a, 2, timeout_s) != 0;
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

std::optional<float> ArmDriver::GetParam(uint8_t node_id, ArmMotorParam param,
                                         float timeout_s) {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  const auto [idx1, idx2] = ArmMotorParamIndices(param);
  float value = 0.f;
  if (!protocol_->GetParam(handle_, node_id, idx1, idx2, timeout_s, &value)) {
    return std::nullopt;
  }
  return value;
}

bool ArmDriver::SetParam(uint8_t node_id, ArmMotorParam param, float value,
                         float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  const auto [idx1, idx2] = ArmMotorParamIndices(param);
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
