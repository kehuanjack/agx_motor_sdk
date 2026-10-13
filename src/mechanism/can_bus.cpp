#include "agx_motor_sdk/mechanism/can_bus.hpp"

#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/motor/arm_driver.hpp"

#include <stdexcept>

namespace agx::motor::mechanism {

CanBus::CanBus(std::string channel, Protocol protocol,
               std::shared_ptr<agx::motor::CanPort> can, bool owns_can)
    : channel_(std::move(channel)),
      protocol_(protocol),
      can_(std::move(can)),
      owns_can_(owns_can) {
  if (!can_) {
    throw std::invalid_argument("CanBus requires a non-null CanPort");
  }
  if (protocol_ != Protocol::kArm) {
    throw std::invalid_argument("CanBus supports arm protocol only");
  }
  arm_driver_ = std::shared_ptr<agx::motor::ArmDriver>(
      new agx::motor::ArmDriver(can_.get()),
      [can = can_](agx::motor::ArmDriver* driver) { delete driver; });
}

CanBus::~CanBus() {
  if (owns_can_) {
    can_->Close();
  }
  arm_driver_.reset();
}

agx::motor::CanPort* CanBus::can_port() { return can_.get(); }

const agx::motor::CanPort* CanBus::can_port() const { return can_.get(); }

agx::motor::ArmDriver* CanBus::arm_driver() { return arm_driver_.get(); }

const agx::motor::ArmDriver* CanBus::arm_driver() const { return arm_driver_.get(); }

std::shared_ptr<agx::motor::ArmDriver> CanBus::arm_driver_shared() const {
  return arm_driver_;
}

CanBusRegistry& CanBusRegistry::Default() {
  static CanBusRegistry registry;
  return registry;
}

std::shared_ptr<CanBus> CanBusRegistry::Acquire(
    const std::string& channel, const std::shared_ptr<agx::motor::CanPort>& injected) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto existing_it = buses_.find(channel);
  if (existing_it != buses_.end()) {
    if (auto existing = existing_it->second.lock()) {
      return existing;
    }
  }

  std::shared_ptr<agx::motor::CanPort> can = injected;
  bool owns_can = false;
  if (!can) {
    can = agx::motor::CreateSocketCanPort(channel, false);
    owns_can = true;
  }
  if (!can->IsOpen() && !can->Open()) {
    throw std::runtime_error("failed to open CAN channel '" + channel + "'");
  }

  auto bus = std::shared_ptr<CanBus>(new CanBus(channel, Protocol::kArm, can, owns_can));
  buses_[channel] = bus;
  return bus;
}

void CanBusRegistry::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  buses_.clear();
}

}  // namespace agx::motor::mechanism
