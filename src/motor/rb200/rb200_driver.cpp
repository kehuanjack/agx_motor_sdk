#include "agx_motor_sdk/motor/rb200/rb200_driver.hpp"

#include <utility>

namespace agx {
namespace motor {
namespace {

Rb200LowSpeedState ToRb200LowSpeed(const AgxMotorRb200LowSpeedFeedback& raw) {
  Rb200LowSpeedState out;
  out.status_word = raw.status_word;
  out.error_code = raw.error_code;
  out.tpdo2_count = raw.tpdo2_count;
  out.timestamp_ns = raw.timestamp_ns;
  const auto bit = [&](int n) { return ((raw.status_word >> n) & 1u) != 0; };
  out.status.ready_to_switch_on = bit(0);
  out.status.switched_on = bit(1);
  out.status.operation_enabled = bit(2);
  out.status.fault = bit(3);
  out.status.voltage_enabled = bit(4);
  out.status.quick_stop = bit(5);
  out.status.switch_on_disabled = bit(6);
  out.status.warning = bit(7);
  out.status.remote = bit(9);
  out.status.target_reached = bit(10);
  out.status.internal_limit_active = bit(11);
  out.status.homing_complete = bit(15);
  return out;
}

}  // namespace

Rb200Driver::Rb200Driver(CanPort* can) : Rb200Driver(Rb200ProtocolLibrary::Load(), can) {}

Rb200Driver::Rb200Driver(std::shared_ptr<Rb200ProtocolLibrary> protocol, CanPort* can)
    : MotorDriver(protocol, "rb200", can), rb200_protocol_(std::move(protocol)) {}

std::optional<Rb200LowSpeedState> Rb200Driver::GetLowSpeedState(
    uint8_t node_id) const {
  if (handle() == nullptr) {
    return std::nullopt;
  }
  AgxMotorRb200LowSpeedFeedback raw{};
  if (!rb200_protocol_->GetLowSpeedFeedback(handle(), node_id, &raw)) {
    return std::nullopt;
  }
  return ToRb200LowSpeed(raw);
}

}  // namespace motor
}  // namespace agx
