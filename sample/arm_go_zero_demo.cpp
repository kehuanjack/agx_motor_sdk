#include <iostream>

#include "agx_motor_sdk/motor/arm_driver.hpp"
#include "demo_utils.hpp"

int main(int argc, char** argv) {
  std::string channel;
  uint8_t node_id = 1;
  if (!agx::motor::demo::ParseChannelAndNode(argc, argv, channel, node_id)) {
    agx::motor::demo::PrintUsage(argv[0]);
    return 1;
  }

  auto can = agx::motor::demo::OpenCanOrExit(channel);
  agx::motor::ArmDriver driver(can);

  if (auto ver = driver.GetVersion(node_id, 0.5f)) {
    agx::motor::demo::PrintVersion(*ver);
  }

  if (!driver.SetEnable(node_id, true)) {
    std::cerr << "enable failed" << std::endl;
    driver.Close();
    can->Close();
    return 1;
  }
  if (!driver.SetProfileVelocity(node_id, 3.f)) {
    std::cerr << "set profile velocity failed" << std::endl;
  }
  if (!driver.SetTargetPosition(node_id, 0.f)) {
    std::cerr << "set target position failed" << std::endl;
  }
  std::cout << "go_zero command sent" << std::endl;

  driver.Close();
  can->Close();
  return 0;
}
