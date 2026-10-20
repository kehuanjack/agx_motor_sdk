#pragma once

#include "agx_motor_sdk/mechanism/types.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace agx::motor {
class ArmDriver;
class CanPort;
}  // namespace agx::motor

namespace agx::motor::mechanism {

/// One CAN channel backend: a single ``CanPort`` and an arm protocol driver.
///
/// Owns the ``CanPort`` (``can_``); the embedded driver only borrows it via a
/// non-owning pointer. All modules on the same YAML ``can_channel`` share one
/// ``CanBus`` instance.
class CanBus {
 public:
  ~CanBus();
  CanBus(const CanBus&) = delete;
  CanBus& operator=(const CanBus&) = delete;

  /// CAN interface name (e.g. ``can0``).
  const std::string& channel() const { return channel_; }
  /// Protocol bound when the bus was created (always arm).
  Protocol protocol() const { return protocol_; }

  /// Owned or injected CAN port; null if the bus failed to open.
  agx::motor::CanPort* can_port();
  const agx::motor::CanPort* can_port() const;

  agx::motor::ArmDriver* arm_driver();
  const agx::motor::ArmDriver* arm_driver() const;
  std::shared_ptr<agx::motor::ArmDriver> arm_driver_shared() const;

 private:
  friend class CanBusRegistry;

  CanBus(std::string channel, Protocol protocol, std::shared_ptr<agx::motor::CanPort> can,
         bool owns_can);

  std::string channel_;
  Protocol protocol_;
  std::shared_ptr<agx::motor::CanPort> can_;
  /// true when this bus created/opened the port; false when injected externally.
  bool owns_can_{false};
  std::shared_ptr<agx::motor::ArmDriver> arm_driver_;
};

/// Global registry: one live ``CanBus`` per ``can_channel`` (no duplicate sockets).
class CanBusRegistry {
 public:
  static CanBusRegistry& Default();

  /// Return an existing bus for ``channel`` or create and open a new one.
  ///
  /// @param channel CAN interface name.
  /// @param injected Optional external ``CanPort``; when set, the bus does not
  ///        own the port (``owns_can == false`` on the ``CanBus``).
  std::shared_ptr<CanBus> Acquire(const std::string& channel,
                                  const std::shared_ptr<agx::motor::CanPort>& injected = {});

  /// Close and drop all cached buses (tests / process teardown).
  void Clear();

 private:
  std::mutex mutex_;
  std::unordered_map<std::string, std::weak_ptr<CanBus>> buses_;
};

}  // namespace agx::motor::mechanism
