#include "agx_motor_sdk/can/socket_can.hpp"

#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <linux/can.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

using agx::motor::SocketCanPort;
using namespace std::chrono_literals;

template <typename Tag, typename Tag::Type Member>
struct PrivateAccess {
  friend typename Tag::Type Access(Tag) { return Member; }
};
struct DescriptorTag {
  using Type = int SocketCanPort::*;
  friend Type Access(DescriptorTag);
};
struct OpenedTag {
  using Type = std::atomic<bool> SocketCanPort::*;
  friend Type Access(OpenedTag);
};
struct ReadLoopTag {
  using Type = void (SocketCanPort::*)();
  friend Type Access(ReadLoopTag);
};
template struct PrivateAccess<DescriptorTag, &SocketCanPort::can_fd_>;
template struct PrivateAccess<OpenedTag, &SocketCanPort::opened_>;
template struct PrivateAccess<ReadLoopTag, &SocketCanPort::ReadLoop>;

int main() {
  SocketCanPort port("unused-no-CAN");
  int descriptors[2];
  if (socketpair(AF_UNIX, SOCK_DGRAM, 0, descriptors) != 0) return 1;
  port.*Access(DescriptorTag{}) = descriptors[0];
  port.*Access(OpenedTag{}) = true;
  std::mutex mutex;
  std::condition_variable changed;
  bool entered = false;
  bool release = false;
  std::atomic<int> calls{0};
  port.SetReceiveHandler([&](const agx::motor::CanFrame&) {
    ++calls;
    std::unique_lock<std::mutex> lock(mutex);
    entered = true;
    changed.notify_all();
    changed.wait(lock, [&] { return release; });
  });
  std::thread reader([&] { (port.*Access(ReadLoopTag{}))(); });
  std::future<void> unregister;
  int result = 0;
  try {
    can_frame frame{};
    frame.can_id = 1;
    if (write(descriptors[1], &frame, sizeof(frame)) != sizeof(frame))
      throw std::runtime_error("socketpair write failed");
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (!changed.wait_for(lock, 2s, [&] { return entered; }))
        throw std::runtime_error("ReadLoop did not invoke callback");
    }
    std::promise<void> started;
    auto started_future = started.get_future();
    unregister = std::async(std::launch::async, [&] {
      started.set_value();
      port.SetReceiveHandler(nullptr);
    });
    started_future.get();
    if (unregister.wait_for(100ms) != std::future_status::timeout)
      throw std::runtime_error("unregister returned while callback was active");
    {
      std::lock_guard<std::mutex> lock(mutex);
      release = true;
    }
    changed.notify_all();
    if (unregister.wait_for(2s) != std::future_status::ready)
      throw std::runtime_error("unregister did not finish after callback");
    unregister.get();
    auto replacement_called = std::make_shared<std::promise<void>>();
    auto replacement_future = replacement_called->get_future();
    port.SetReceiveHandler([&, replacement_called](const agx::motor::CanFrame&) {
      replacement_called->set_value();
      port.SetReceiveHandler(nullptr);
    });
    if (write(descriptors[1], &frame, sizeof(frame)) != sizeof(frame))
      throw std::runtime_error("replacement write failed");
    if (replacement_future.wait_for(2s) != std::future_status::ready)
      throw std::runtime_error("replacement/self-unregister callback failed");
    port.SetReceiveHandler(nullptr);
    if (calls != 1) throw std::runtime_error("old callback invoked after unregister");
    std::cout << "ReadLoop callback unregister and self-unregister checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  changed.notify_all();
  if (unregister.valid()) unregister.wait();
  port.*Access(OpenedTag{}) = false;
  reader.join();
  port.Close();
  close(descriptors[1]);
  return result;
}