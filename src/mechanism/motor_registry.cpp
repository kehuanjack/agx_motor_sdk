#include "agx_motor_sdk/mechanism/motor_registry.hpp"

#include "agx_motor_sdk/mechanism/arm_motor.hpp"

namespace agx::motor::mechanism {
namespace {

std::size_t HashCombine(std::size_t seed, std::size_t value) {
  return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

}  // namespace

bool MotorRegistry::ArmKey::operator==(const ArmKey& other) const {
  return node_id == other.node_id && channel == other.channel;
}

std::size_t MotorRegistry::ArmKeyHash::operator()(const ArmKey& key) const {
  std::size_t seed = std::hash<std::string>{}(key.channel);
  seed = HashCombine(seed, static_cast<std::size_t>(key.node_id));
  return seed;
}

MotorRegistry& MotorRegistry::Default() {
  static MotorRegistry instance;
  return instance;
}

ArmMotor& MotorRegistry::AcquireArm(const std::string& can_channel,
                                    const std::shared_ptr<agx::motor::ArmDriver>& driver,
                                    uint8_t node_id,
                                    const JointConfig& config) {
  const ArmKey key{can_channel, node_id};
  auto it = arm_motors_.find(key);
  if (it == arm_motors_.end()) {
    auto motor = std::unique_ptr<ArmMotor>(
        new ArmMotor(driver, node_id, can_channel, config));
    auto* ptr = motor.get();
    arm_motors_.emplace(key, std::move(motor));
    return *ptr;
  }
  it->second->BindDriver(driver);
  it->second->BindConfig(config);
  return *it->second;
}

void MotorRegistry::Clear() {
  arm_motors_.clear();
}

}  // namespace agx::motor::mechanism
