#include "agx_motor_sdk/motor/chassis_driver.hpp"

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

ChassisDriver::ChassisDriver(std::shared_ptr<CanPort> can) {
  protocol_ = ProtocolLibrary::Load(ProtocolVariant::kChassis);
  if (protocol_->load_error().empty()) {
    handle_ = protocol_->Create();
    if (handle_ != nullptr && can != nullptr) {
      AttachCanPort(std::move(can));
    }
  }
}

ChassisDriver::~ChassisDriver() { Close(); }

void ChassisDriver::AttachCanPort(std::shared_ptr<CanPort> can) {
  can_ = std::move(can);
  if (handle_ == nullptr || can_ == nullptr) {
    return;
  }
  protocol_->SetTxCallback(handle_, &ChassisDriver::TxCallback, this);
  can_->SetReceiveHandler(
      [this](const CanFrame& frame) { OnCanFrame(frame); });
}

void ChassisDriver::Close() {
  if (can_ != nullptr) {
    can_->SetReceiveHandler(nullptr);
    can_.reset();
  }
  if (protocol_ != nullptr && handle_ != nullptr) {
    protocol_->Destroy(handle_);
    handle_ = nullptr;
  }
}

bool ChassisDriver::HandleRxOnce(const CanFrame& frame) {
  if (handle_ == nullptr) {
    return false;
  }
  return protocol_->HandleRxOnce(handle_, frame.id, frame.data.data(),
                                 static_cast<uint8_t>(frame.data.size()),
                                 frame.timestamp_ns) != 0;
}

std::optional<HighSpeedState> ChassisDriver::GetHighSpeedState(
    uint8_t node_id) const {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  AgxMotorHighSpeedFeedback raw{};
  if (!protocol_->GetHighSpeedFeedback(handle_, node_id, &raw)) {
    return std::nullopt;
  }
  return ToHighSpeed(raw);
}

std::optional<LowSpeedState> ChassisDriver::GetLowSpeedState(
    uint8_t node_id) const {
  if (handle_ == nullptr) {
    return std::nullopt;
  }
  AgxMotorLowSpeedFeedback raw{};
  if (!protocol_->GetLowSpeedFeedback(handle_, node_id, &raw)) {
    return std::nullopt;
  }
  return ToLowSpeed(raw);
}

bool ChassisDriver::SetEnable(uint8_t node_id, bool enable, bool has_brake,
                              bool brake_on) {
  return handle_ != nullptr &&
         protocol_->SetEnable(handle_, node_id, enable ? 1 : 0, has_brake ? 1 : 0,
                              brake_on ? 1 : 0) != 0;
}

bool ChassisDriver::SetTargetRpm(uint8_t node_id, float rpm, float timeout_s) {
  return handle_ != nullptr &&
         protocol_->SetTarget(handle_, node_id, rpm, 0, timeout_s) != 0;
}

bool ChassisDriver::SetTargetInc(uint8_t node_id, float inc, float timeout_s) {
  return handle_ != nullptr &&
         protocol_->SetTarget(handle_, node_id, inc, 1, timeout_s) != 0;
}

bool ChassisDriver::SetTargetCurrent(uint8_t node_id, float current_a,
                                   float timeout_s) {
  return handle_ != nullptr &&
         protocol_->SetTarget(handle_, node_id, current_a, 2, timeout_s) != 0;
}

bool ChassisDriver::SetReset(uint8_t node_id, int mode, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetReset(handle_, node_id, mode, timeout_s, &ok),
                 timeout_s, ok);
}

bool ChassisDriver::SetClearError(uint8_t node_id, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetClearError(handle_, node_id, timeout_s, &ok),
                 timeout_s, ok);
}

bool ChassisDriver::SetZeroOffset(uint8_t node_id, float zero_offset,
                                  bool save_to_flash, float timeout_s) {
  if (handle_ == nullptr) {
    return false;
  }
  int ok = 0;
  return AwaitOk(protocol_->SetZeroOffset(handle_, node_id, zero_offset,
                                          save_to_flash ? 1 : 0, timeout_s, &ok),
                 timeout_s, ok);
}

bool ChassisDriver::SetProfileAccDec(uint8_t node_id, float acceleration,
                                     float deceleration) {
  return handle_ != nullptr &&
         protocol_->SetProfileAccDec(handle_, node_id, acceleration,
                                     deceleration) != 0;
}

bool ChassisDriver::SetProfileVelocity(uint8_t node_id, float rpm) {
  return handle_ != nullptr &&
         protocol_->SetProfileVel(handle_, node_id, rpm) != 0;
}

bool ChassisDriver::SetCurrentLimit(uint8_t node_id, float current_a) {
  return handle_ != nullptr &&
         protocol_->SetCurrentLimit(handle_, node_id, current_a) != 0;
}

bool ChassisDriver::SetStiffness(uint8_t node_id, uint16_t stiffness) {
  return handle_ != nullptr &&
         protocol_->SetStiffness(handle_, node_id, stiffness) != 0;
}

int ChassisDriver::TxCallback(void* ctx, uint32_t can_id, uint8_t* data,
                              uint8_t dlc) {
  auto* self = static_cast<ChassisDriver*>(ctx);
  if (self == nullptr || self->can_ == nullptr) {
    return 0;
  }
  CanFrame frame;
  frame.id = can_id;
  frame.data.assign(data, data + dlc);
  self->can_->Send(frame);
  return 1;
}

void ChassisDriver::OnCanFrame(const CanFrame& frame) { HandleRxOnce(frame); }

}  // namespace motor
}  // namespace agx
