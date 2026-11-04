#include "agx_motor_sdk/mechanism/motor_registry.hpp"

#include "agx_motor_sdk/mechanism/arm/arm_motor.hpp"
#include "agx_motor_sdk/mechanism/rb200/rb200_motor.hpp"

namespace agx::motor::mechanism {
namespace {

std::size_t HashCombine(std::size_t seed, std::size_t value) {
  return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

}  // namespace

bool MotorRegistry::Key::operator==(const Key& other) const {
  return node_id == other.node_id && channel == other.channel;
}

std::size_t MotorRegistry::KeyHash::operator()(const Key& key) const {
  std::size_t seed = std::hash<std::string>{}(key.channel);
  seed = HashCombine(seed, static_cast<std::size_t>(key.node_id));
  return seed;
}

MotorRegistry& MotorRegistry::Default() {
  static MotorRegistry instance;
  return instance;
}

MotorBase& MotorRegistry::Acquire(const std::string& can_channel,
                                  const std::shared_ptr<agx::motor::MotorDriver>& driver,
                                  uint8_t node_id, const JointConfig& config) {
  const Key key{can_channel, node_id};
  auto it = motors_.find(key);
  if (it != motors_.end() && it->second->protocol() == config.protocol) {
    it->second->BindDriver(driver);
    it->second->BindConfig(config);
    return *it->second;
  }
  std::unique_ptr<MotorBase> motor;
  if (config.protocol == Protocol::kRb200) {
    motor.reset(new Rb200Motor(driver, node_id, can_channel, config));
  } else {
    motor.reset(new ArmMotor(driver, node_id, can_channel, config));
  }
  auto* ptr = motor.get();
  motors_[key] = std::move(motor);
  return *ptr;
}

void MotorRegistry::Clear() {
  motors_.clear();
}

}  // namespace agx::motor::mechanism
