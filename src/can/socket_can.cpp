#include "agx_motor_sdk/can/socket_can.hpp"

#include <chrono>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace agx {
namespace motor {

SocketCanPort::SocketCanPort(std::string channel, bool local_loopback)
    : channel_(std::move(channel)), local_loopback_(local_loopback) {}

SocketCanPort::~SocketCanPort() { Close(); }

bool SocketCanPort::Open() {
  if (opened_) {
    return true;
  }
  if (read_thread_.joinable()) {
    if (read_thread_.get_id() == std::this_thread::get_id()) return false;
    Close();
  }

  can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_fd_ < 0) {
    return false;
  }

  const int loopback = local_loopback_ ? 1 : 0;
  if (setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback,
                 sizeof(loopback)) < 0) {
    Close();
    return false;
  }
  const int recv_own = local_loopback_ ? 1 : 0;
  if (setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own,
                 sizeof(recv_own)) < 0) {
    Close();
    return false;
  }

  ifreq ifr{};
  if (channel_.size() >= IFNAMSIZ) {
    Close();
    return false;
  }
  std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);

  if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
    Close();
    return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    Close();
    return false;
  }

  opened_ = true;
  read_thread_ = std::thread([this]() { ReadLoop(); });
  return true;
}

void SocketCanPort::Close() {
  opened_ = false;
  if (read_thread_.joinable()) {
    if (read_thread_.get_id() == std::this_thread::get_id()) {
      return;
    }
    read_thread_.join();
  }
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (can_fd_ >= 0) {
    ::close(can_fd_);
    can_fd_ = -1;
  }
}

bool SocketCanPort::IsOpen() const { return opened_; }

void SocketCanPort::Send(const CanFrame& frame) {
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (!opened_ || can_fd_ < 0) {
    return;
  }

  can_frame raw{};
  raw.can_id = frame.id;
  raw.can_dlc = static_cast<__u8>(std::min(frame.data.size(), size_t(8)));
  std::memcpy(raw.data, frame.data.data(), raw.can_dlc);
  const ssize_t n = ::write(can_fd_, &raw, sizeof(raw));
  if (n != static_cast<ssize_t>(sizeof(raw))) {
    return;
  }
}

void SocketCanPort::SetReceiveHandler(CanReceiveHandler handler) {
  std::lock_guard<std::recursive_mutex> lock(handler_mutex_);
  handler_ = std::move(handler);
}

void SocketCanPort::ReadLoop() {
  while (opened_ && can_fd_ >= 0) {
    pollfd pfd{};
    pfd.fd = can_fd_;
    pfd.events = POLLIN;
    const int ready = ::poll(&pfd, 1, 100);
    if (ready <= 0) {
      continue;
    }

    can_frame raw{};
    const ssize_t n = ::read(can_fd_, &raw, sizeof(raw));
    if (n != static_cast<ssize_t>(sizeof(raw))) {
      continue;
    }

    std::lock_guard<std::recursive_mutex> lock(handler_mutex_);
    if (!handler_) {
      continue;
    }

    CanFrame frame;
    frame.id = raw.can_id;
    frame.data.assign(raw.data, raw.data + raw.can_dlc);
    frame.timestamp_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    const auto handler = handler_;
    handler(frame);
  }
}

}  // namespace motor
}  // namespace agx
