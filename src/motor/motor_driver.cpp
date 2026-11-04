#include "agx_motor_sdk/motor/motor_driver.hpp"

#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace agx {
namespace motor {
namespace {

HighSpeedState ToHighSpeed(const AgxMotorHighSpeedFeedback& raw) {
  return HighSpeedState{raw.position, raw.velocity, raw.current,
                        raw.timestamp_ns};
}

VersionInfo ToVersion(const AgxMotorVersionInfo& raw) {
  VersionInfo out;
  out.software.assign(raw.software, strnlen(raw.software, sizeof(raw.software)));
  out.hardware.assign(raw.hardware, strnlen(raw.hardware, sizeof(raw.hardware)));
  out.motor.assign(raw.motor, strnlen(raw.motor, sizeof(raw.motor)));
  out.timestamp_ns = raw.timestamp_ns;
  return out;
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

MotorDriver::MotorDriver(std::shared_ptr<ProtocolLibrary> protocol,
                         std::string protocol_name, CanPort* can)
    : protocol_(std::move(protocol)), protocol_name_(std::move(protocol_name)) {
  if (!protocol_ || !protocol_->load_error().empty()) {
    throw std::runtime_error("failed to load " + protocol_name_ +
                             " motor protocol library: " +
                             (protocol_ ? protocol_->load_error() : std::string("null library")));
  }
  handle_ = protocol_->Create();
  if (handle_ == nullptr) {
    throw std::runtime_error("failed to create " + protocol_name_ +
                             " motor protocol handle");
  }
  if (can != nullptr) {
    AttachCanPort(can);
  }
}

MotorDriver::~MotorDriver() { Close(); }

void MotorDriver::AttachCanPort(CanPort* can) {
  can_ = can;
  if (handle_ == nullptr || can_ == nullptr) {
    return;
  }
  protocol_->SetTxCallback(handle_, &MotorDriver::TxCallback, this);
  can_->SetReceiveHandler([this](const CanFrame& frame) { OnCanFrame(frame); });
}

void MotorDriver::Close() {
  if (can_ != nullptr) {
    can_->SetReceiveHandler(nullptr);
    can_ = nullptr;
  }
  if (protocol_ != nullptr && handle_ != nullptr) {
    protocol_->Destroy(handle_);
    handle_ = nullptr;
  }
}

bool MotorDriver::HandleRxOnce(const CanFrame& frame) {
  if (handle_ == nullptr) {
    return false;
  }
  return protocol_->HandleRxOnce(handle_, frame.id, frame.data.data(),
                                 static_cast<uint8_t>(frame.data.size()),
                                 frame.timestamp_ns) != 0;
}

std::optional<HighSpeedState> MotorDriver::GetHighSpeedState(uint8_t node_id) const {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  AgxMotorHighSpeedFeedback raw{};
  if (!protocol_->GetHighSpeedFeedback(handle_, node_id, &raw)) {
    return std::nullopt;
  }
  return ToHighSpeed(raw);
}

std::optional<VersionInfo> MotorDriver::GetVersion(uint8_t node_id, float timeout_s) {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  AgxMotorVersionInfo raw{};
  if (!protocol_->GetVersion(handle_, node_id, timeout_s, &raw)) {
    return std::nullopt;
  }
  return ToVersion(raw);
}

bool MotorDriver::SetEnable(uint8_t node_id, bool enable, bool has_brake,
                          bool brake_on) {
  return handle_ != nullptr &&
         protocol_->SetEnable(handle_, node_id, enable ? 1 : 0, has_brake ? 1 : 0,
                              brake_on ? 1 : 0) != 0;
}

bool MotorDriver::SetTarget(uint8_t node_id, float value, uint8_t mode,
                          float timeout_s) {
  return handle_ != nullptr &&
         protocol_->SetTarget(handle_, node_id, value, mode, timeout_s) != 0;
}

bool MotorDriver::SetTargetVelocity(uint8_t node_id, float rad_s, float timeout_s) {
  return SetTarget(node_id, rad_s, 0, timeout_s);
}

bool MotorDriver::SetTargetPosition(uint8_t node_id, float rad, float timeout_s) {
  return SetTarget(node_id, rad, 1, timeout_s);
}

bool MotorDriver::SetReset(uint8_t node_id, int mode, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetReset(handle_, node_id, mode, timeout_s, &ok),
                 timeout_s, ok);
}

bool MotorDriver::SetClearError(uint8_t node_id, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetClearError(handle_, node_id, timeout_s, &ok),
                 timeout_s, ok);
}

bool MotorDriver::SetZeroOffset(uint8_t node_id, float zero_offset,
                              bool save_to_flash, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetZeroOffset(handle_, node_id, zero_offset,
                                          save_to_flash ? 1 : 0, timeout_s, &ok),
                 timeout_s, ok);
}

bool MotorDriver::SetProfileAccDec(uint8_t node_id, float acceleration,
                                 float deceleration) {
  return handle_ != nullptr &&
         protocol_->SetProfileAccDec(handle_, node_id, acceleration,
                                     deceleration) != 0;
}

bool MotorDriver::SetProfileVelocity(uint8_t node_id, float rad_s) {
  return handle_ != nullptr &&
         protocol_->SetProfileVel(handle_, node_id, rad_s) != 0;
}

bool MotorDriver::SetCurrentLimit(uint8_t node_id, float current_a) {
  return handle_ != nullptr &&
         protocol_->SetCurrentLimit(handle_, node_id, current_a) != 0;
}

int MotorDriver::TxCallback(void* ctx, uint32_t can_id, uint8_t* data, uint8_t dlc) {
  auto* self = static_cast<MotorDriver*>(ctx);
  if (self == nullptr || self->can_ == nullptr) {
    return 0;
  }
  CanFrame frame;
  frame.id = can_id;
  frame.data.assign(data, data + dlc);
  self->can_->Send(frame);
  return 1;
}

void MotorDriver::OnCanFrame(const CanFrame& frame) { HandleRxOnce(frame); }

}  // namespace motor
}  // namespace agx
