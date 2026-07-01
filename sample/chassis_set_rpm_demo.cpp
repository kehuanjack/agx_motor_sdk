#include <iostream>

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

  if (!driver.SetEnable(node_id, true)) {
    std::cerr << "enable failed" << std::endl;
    return 1;
  }
  if (!driver.SetStiffness(node_id, 20)) {
    std::cerr << "set stiffness failed" << std::endl;
  }
  if (!driver.SetProfileVelocity(node_id, 500.f)) {
    std::cerr << "set profile velocity failed" << std::endl;
  }
  if (!driver.SetTargetRpm(node_id, 100.f)) {
    std::cerr << "set target rpm failed" << std::endl;
  }
  std::cout << "set_rpm command sent" << std::endl;

  driver.Close();
  can->Close();
  return 0;
}
