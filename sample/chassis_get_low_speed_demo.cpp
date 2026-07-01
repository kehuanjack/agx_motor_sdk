#include <chrono>
#include <iostream>
#include <thread>

#include "agx_motor_sdk/motor/chassis_driver.hpp"
#include "demo_utils.hpp"

int main(int argc, char** argv) {
  std::string channel;
  uint8_t node_id = 1;
  if (!agx::motor::demo::ParseChannelAndNode(argc, argv, channel, node_id)) {
    agx::motor::demo::PrintUsage(argv[0]);
    return 1;
  }

  auto can = agx::motor::demo::OpenCanOrExit(channel);
  agx::motor::ChassisDriver driver(can);

  while (true) {
    if (auto ls = driver.GetLowSpeedState(node_id)) {
      agx::motor::demo::PrintLowSpeed(*ls);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
