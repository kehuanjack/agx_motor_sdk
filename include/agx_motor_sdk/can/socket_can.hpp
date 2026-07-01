#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "agx_motor_sdk/can/can_port.hpp"

namespace agx {
namespace motor {

/// Linux socketcan CanPort with a dedicated background read thread.
class SocketCanPort : public CanPort,
                      public std::enable_shared_from_this<SocketCanPort> {
 public:
  /// @param channel CAN interface name (e.g. "can0"). Interface is not opened until Open().
  /// @param local_loopback If true, enable CAN_RAW loopback and receive own TX frames.
  explicit SocketCanPort(std::string channel, bool local_loopback = false);
  ~SocketCanPort() override;

  SocketCanPort(const SocketCanPort&) = delete;
  SocketCanPort& operator=(const SocketCanPort&) = delete;

  bool Open() override;
  void Close() override;
  bool IsOpen() const override;

  void Send(const CanFrame& frame) override;
  void SetReceiveHandler(CanReceiveHandler handler) override;

 private:
  std::string channel_;
  bool local_loopback_{false};
  std::atomic<bool> opened_{false};
  int can_fd_{-1};
  CanReceiveHandler handler_;

  void ReadLoop();
  std::thread read_thread_;
};

}  // namespace motor
}  // namespace agx
