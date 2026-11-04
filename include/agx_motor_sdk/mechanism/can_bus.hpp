#pragma once

#include "agx_motor_sdk/mechanism/types.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace agx::motor {
class CanPort;
class MotorDriver;
}  // namespace agx::motor

namespace agx::motor::mechanism {

class CanChannelMux;

/// One CAN channel backend: a single ``CanPort`` shared by one protocol driver
/// per protocol in use (arm and/or RB200).
///
/// Several protocols may share a channel. Received frames are routed by node ID
/// (low 4 bits of the CAN ID) to the driver bound to that node; frames for
/// unbound nodes go to every driver. Owns the ``CanPort`` (``can_``); drivers
/// only borrow it. All modules on the same YAML ``can_channel`` share one
/// ``CanBus`` instance.
class CanBus {
 public:
  ~CanBus();
  CanBus(const CanBus&) = delete;
  CanBus& operator=(const CanBus&) = delete;

  /// CAN interface name (e.g. ``can0``).
  const std::string& channel() const { return channel_; }
  /// Protocol of the default driver (the one requested when the bus was created).
  Protocol protocol() const { return protocol_; }

  /// Owned or injected CAN port; null if the bus failed to open.
  agx::motor::CanPort* can_port();
  const agx::motor::CanPort* can_port() const;

  /// Default-protocol driver (``ArmDriver`` or ``Rb200Driver``).
  agx::motor::MotorDriver* driver() const;

  /// Driver for ``protocol``; created on first use (throws if its library fails to load).
  std::shared_ptr<agx::motor::MotorDriver> EnsureDriver(Protocol protocol);
  /// Existing driver for ``protocol``, or null if none was created.
  std::shared_ptr<agx::motor::MotorDriver> driver_shared(Protocol protocol) const;
  agx::motor::MotorDriver* driver(Protocol protocol) const;

  /// Route frames of ``node_id`` to the ``protocol`` driver.
  /// Throws ``std::invalid_argument`` if the node is already bound to another protocol.
  void BindNode(uint8_t node_id, Protocol protocol);

 private:
  friend class CanBusRegistry;

  CanBus(std::string channel, Protocol protocol, std::shared_ptr<agx::motor::CanPort> can,
         bool owns_can);

  std::string channel_;
  Protocol protocol_;
  std::shared_ptr<agx::motor::CanPort> can_;
  /// true when this bus created/opened the port; false when injected externally.
  bool owns_can_{false};
  std::shared_ptr<CanChannelMux> mux_;
  mutable std::mutex drivers_mutex_;
  std::map<Protocol, std::shared_ptr<agx::motor::MotorDriver>> drivers_;
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
  /// @param protocol Driver protocol; on a live bus the driver for this
  ///        protocol is added if missing.
  std::shared_ptr<CanBus> Acquire(const std::string& channel,
                                  const std::shared_ptr<agx::motor::CanPort>& injected = {},
                                  Protocol protocol = Protocol::kArm);

  /// Close and drop all cached buses (tests / process teardown).
  void Clear();

 private:
  std::mutex mutex_;
  std::unordered_map<std::string, std::weak_ptr<CanBus>> buses_;
};

}  // namespace agx::motor::mechanism
