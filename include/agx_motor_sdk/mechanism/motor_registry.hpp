#pragma once

#include "agx_motor_sdk/mechanism/types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

namespace agx::motor {
class MotorDriver;
}  // namespace agx::motor

namespace agx::motor::mechanism {

class MotorBase;

/// Internal singleton pool of shared joint motors (``ArmMotor`` / ``Rb200Motor``).
///
/// Key: ``can_channel`` + ``node_id``. The same physical motor is reused across
/// joint groups. Not part of the public Python API.
class MotorRegistry {
 public:
  static MotorRegistry& Default();

  /// Return an existing motor or create and cache a new one.
  ///
  /// The motor type follows ``config.protocol`` (``ArmMotor`` or ``Rb200Motor``);
  /// a cached motor of another protocol is replaced.
  /// @param can_channel Resolved CAN channel for the joint.
  /// @param driver Shared driver for this channel and protocol (from ``CanBus``).
  /// @param node_id CAN node ID (1–15).
  /// @param config Static joint configuration stored on the motor.
  MotorBase& Acquire(const std::string& can_channel,
                     const std::shared_ptr<agx::motor::MotorDriver>& driver,
                     uint8_t node_id, const JointConfig& config);

  /// Drop all cached motors (tests / process teardown).
  void Clear();

 private:
  struct Key {
    std::string channel;
    uint8_t node_id{0};
    bool operator==(const Key& other) const;
  };

  struct KeyHash {
    std::size_t operator()(const Key& key) const;
  };

  std::unordered_map<Key, std::unique_ptr<MotorBase>, KeyHash> motors_;
};

}  // namespace agx::motor::mechanism
