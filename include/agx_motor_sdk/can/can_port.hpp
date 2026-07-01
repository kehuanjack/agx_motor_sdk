#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace agx {
namespace motor {

/// One CAN frame exchanged between CanPort and motor drivers.
struct CanFrame {
  uint32_t id{0};              ///< CAN arbitration ID (11-bit standard frame).
  std::vector<uint8_t> data;   ///< Payload; DLC equals data.size() (0–8).
  uint64_t timestamp_ns{0};    ///< Receive time [ns]; 0 if not provided by backend.
};

/// Callback invoked for each frame received on a CanPort.
using CanReceiveHandler = std::function<void(const CanFrame&)>;

/// Abstract CAN transport layer.
class CanPort {
 public:
  virtual ~CanPort() = default;

  /// Open the CAN interface.
  /// @return true on success, false on failure (interface missing, permission, etc.).
  virtual bool Open() = 0;

  /// Close the interface and stop any background receive thread.
  virtual void Close() = 0;

  /// @return true if Open() succeeded and the port is active.
  virtual bool IsOpen() const = 0;

  /// Transmit one frame. Behavior when closed is implementation-defined.
  /// @param frame Frame to send; only id and data are used for TX.
  virtual void Send(const CanFrame& frame) = 0;

  /// Register handler for received frames. Pass nullptr to clear.
  /// @param handler Called from the receive thread; must be thread-safe if shared state is used.
  virtual void SetReceiveHandler(CanReceiveHandler handler) = 0;
};

/// Factory: Linux socketcan backend for the given interface name.
/// @param channel Interface name, e.g. "can0". Call Open() before use.
/// @param local_loopback Enable CAN_RAW loopback and receive own TX frames.
/// @return Shared SocketCanPort instance.
std::shared_ptr<CanPort> CreateSocketCanPort(const std::string& channel,
                                             bool local_loopback = false);

}  // namespace motor
}  // namespace agx
