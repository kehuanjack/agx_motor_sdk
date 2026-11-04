#include "agx_motor_sdk/mechanism/can_bus.hpp"

#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/motor/arm/arm_driver.hpp"
#include "agx_motor_sdk/motor/rb200/rb200_driver.hpp"

#include <array>
#include <optional>
#include <stdexcept>
#include <vector>

namespace agx::motor::mechanism {

/// Shares one CanPort between protocol drivers and routes RX frames by node ID.
/// Lives as long as the bus or any driver using it, so drivers outliving the
/// bus keep receiving frames.
class CanChannelMux {
 public:
  explicit CanChannelMux(std::shared_ptr<agx::motor::CanPort> port) : port_(std::move(port)) {
    port_->SetReceiveHandler([this](const agx::motor::CanFrame& frame) { Dispatch(frame); });
  }
  ~CanChannelMux() { port_->SetReceiveHandler(nullptr); }
  CanChannelMux(const CanChannelMux&) = delete;
  CanChannelMux& operator=(const CanChannelMux&) = delete;

  agx::motor::CanPort& port() { return *port_; }

  void SetHandler(Protocol protocol, agx::motor::CanReceiveHandler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (handler) {
      handlers_[protocol] = std::make_shared<agx::motor::CanReceiveHandler>(std::move(handler));
    } else {
      handlers_.erase(protocol);
    }
  }

  void BindNode(uint8_t node_id, Protocol protocol, const std::string& channel) {
    if (node_id >= nodes_.size()) {
      throw std::invalid_argument("node_id out of range");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto& bound = nodes_[node_id];
    if (bound && *bound != protocol) {
      throw std::invalid_argument("CAN channel '" + channel + "' node " +
                                  std::to_string(node_id) + " already uses protocol '" +
                                  ProtocolToString(*bound) + "', requested '" +
                                  ProtocolToString(protocol) + "'");
    }
    bound = protocol;
  }

 private:
  void Dispatch(const agx::motor::CanFrame& frame) {
    std::vector<std::shared_ptr<agx::motor::CanReceiveHandler>> targets;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto& bound = nodes_[frame.id & 0x0Fu];
      if (bound) {
        const auto it = handlers_.find(*bound);
        if (it != handlers_.end()) {
          targets.push_back(it->second);
        }
      } else {
        for (const auto& entry : handlers_) {
          targets.push_back(entry.second);
        }
      }
    }
    for (const auto& handler : targets) {
      (*handler)(frame);
    }
  }

  std::shared_ptr<agx::motor::CanPort> port_;
  std::mutex mutex_;
  std::map<Protocol, std::shared_ptr<agx::motor::CanReceiveHandler>> handlers_;
  std::array<std::optional<Protocol>, 16> nodes_{};
};

namespace {

/// Per-protocol view of the shared port handed to one protocol driver.
class MuxCanPort final : public agx::motor::CanPort {
 public:
  MuxCanPort(std::shared_ptr<CanChannelMux> mux, Protocol protocol)
      : mux_(std::move(mux)), protocol_(protocol) {}

  bool Open() override { return mux_->port().IsOpen() || mux_->port().Open(); }
  void Close() override {}
  bool IsOpen() const override { return mux_->port().IsOpen(); }
  void Send(const agx::motor::CanFrame& frame) override { mux_->port().Send(frame); }
  void SetReceiveHandler(agx::motor::CanReceiveHandler handler) override {
    mux_->SetHandler(protocol_, std::move(handler));
  }

 private:
  std::shared_ptr<CanChannelMux> mux_;
  Protocol protocol_;
};

agx::motor::MotorDriver* CreateDriver(Protocol protocol, agx::motor::CanPort* port) {
  if (protocol == Protocol::kRb200) {
    return new agx::motor::Rb200Driver(port);
  }
  return new agx::motor::ArmDriver(port);
}

}  // namespace

CanBus::CanBus(std::string channel, Protocol protocol,
               std::shared_ptr<agx::motor::CanPort> can, bool owns_can)
    : channel_(std::move(channel)),
      protocol_(protocol),
      can_(std::move(can)),
      owns_can_(owns_can) {
  if (!can_) {
    throw std::invalid_argument("CanBus requires a non-null CanPort");
  }
  mux_ = std::make_shared<CanChannelMux>(can_);
  EnsureDriver(protocol_);
}

CanBus::~CanBus() {
  if (owns_can_) {
    can_->Close();
  }
  {
    std::lock_guard<std::mutex> lock(drivers_mutex_);
    drivers_.clear();
  }
  mux_.reset();
}

agx::motor::CanPort* CanBus::can_port() { return can_.get(); }

const agx::motor::CanPort* CanBus::can_port() const { return can_.get(); }

agx::motor::MotorDriver* CanBus::driver() const { return driver(protocol_); }

std::shared_ptr<agx::motor::MotorDriver> CanBus::EnsureDriver(Protocol protocol) {
  std::lock_guard<std::mutex> lock(drivers_mutex_);
  auto& slot = drivers_[protocol];
  if (!slot) {
    auto port = std::make_shared<MuxCanPort>(mux_, protocol);
    try {
      slot = std::shared_ptr<agx::motor::MotorDriver>(
          CreateDriver(protocol, port.get()),
          // Release the port (and the mux once unused) on dispose, not when the
          // control block dies with the last weak_ptr.
          [port](agx::motor::MotorDriver* driver) mutable {
            delete driver;
            port.reset();
          });
    } catch (...) {
      drivers_.erase(protocol);
      throw;
    }
  }
  return slot;
}

std::shared_ptr<agx::motor::MotorDriver> CanBus::driver_shared(Protocol protocol) const {
  std::lock_guard<std::mutex> lock(drivers_mutex_);
  const auto it = drivers_.find(protocol);
  return it == drivers_.end() ? nullptr : it->second;
}

agx::motor::MotorDriver* CanBus::driver(Protocol protocol) const {
  return driver_shared(protocol).get();
}

void CanBus::BindNode(uint8_t node_id, Protocol protocol) {
  mux_->BindNode(node_id, protocol, channel_);
}

CanBusRegistry& CanBusRegistry::Default() {
  static CanBusRegistry registry;
  return registry;
}

std::shared_ptr<CanBus> CanBusRegistry::Acquire(
    const std::string& channel, const std::shared_ptr<agx::motor::CanPort>& injected,
    Protocol protocol) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto existing_it = buses_.find(channel);
  if (existing_it != buses_.end()) {
    if (auto existing = existing_it->second.lock()) {
      existing->EnsureDriver(protocol);
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

  auto bus = std::shared_ptr<CanBus>(new CanBus(channel, protocol, can, owns_can));
  buses_[channel] = bus;
  return bus;
}

void CanBusRegistry::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  buses_.clear();
}

}  // namespace agx::motor::mechanism
