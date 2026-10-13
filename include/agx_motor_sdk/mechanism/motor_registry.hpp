#pragma once

#include "agx_motor_sdk/mechanism/types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

namespace agx::motor {
class ArmDriver;
}  // namespace agx::motor

namespace agx::motor::mechanism {

class ArmMotor;

/// Internal singleton pool of shared ``ArmMotor`` instances.
///
/// Key: ``can_channel`` + ``node_id``. The same physical motor is reused across
/// joint groups. Not part of the public Python API.
class MotorRegistry {
 public:
  static MotorRegistry& Default();

  /// Return an existing arm motor or create and cache a new one.
  /// @param can_channel Resolved CAN channel for the joint.
  /// @param driver Shared arm driver handle for this channel (from ``CanBus``).
  /// @param node_id CAN node ID (1–15).
  /// @param config Static joint configuration stored on the motor.
  ArmMotor& AcquireArm(const std::string& can_channel,
                       const std::shared_ptr<agx::motor::ArmDriver>& driver,
                       uint8_t node_id, const JointConfig& config);

  /// Drop all cached motors (tests / process teardown).
  void Clear();

 private:
  struct ArmKey {
    std::string channel;
    uint8_t node_id{0};
    bool operator==(const ArmKey& other) const;
  };

  struct ArmKeyHash {
    std::size_t operator()(const ArmKey& key) const;
  };

  std::unordered_map<ArmKey, std::unique_ptr<ArmMotor>, ArmKeyHash> arm_motors_;
};

}  // namespace agx::motor::mechanism
