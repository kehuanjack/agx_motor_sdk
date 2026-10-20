#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/can/socket_can.hpp"

namespace agx {
namespace motor {

std::shared_ptr<CanPort> CreateSocketCanPort(const std::string& channel,
                                             bool local_loopback) {
  return std::make_shared<SocketCanPort>(channel, local_loopback);
}

}  // namespace motor
}  // namespace agx
