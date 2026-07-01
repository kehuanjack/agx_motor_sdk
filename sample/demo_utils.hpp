#pragma once

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "agx_motor_sdk/can/can_port.hpp"
#include "agx_motor_sdk/motor/types.hpp"

namespace agx::motor::demo {

inline void PrintUsage(const char* prog, const char* extra = "") {
  std::cerr << "usage: " << prog << " [can_channel] [node_id]";
  if (extra[0] != '\0') {
    std::cerr << " " << extra;
  }
  std::cerr << std::endl;
  std::cerr << "  default: can0 node_id=1" << std::endl;
}

inline bool ParseChannelAndNode(int argc, char** argv, std::string& channel,
                                uint8_t& node_id) {
  channel = argc > 1 ? argv[1] : "can0";
  node_id = 1;
  if (argc > 2) {
    try {
      const int id = std::stoi(argv[2]);
      if (id < 1 || id > 15) {
        std::cerr << "node_id must be 1..15" << std::endl;
        return false;
      }
      node_id = static_cast<uint8_t>(id);
    } catch (...) {
      std::cerr << "invalid node_id: " << argv[2] << std::endl;
      return false;
    }
  }
  return true;
}

inline std::shared_ptr<CanPort> OpenCanOrExit(const std::string& channel) {
  auto can = CreateSocketCanPort(channel);
  if (!can->Open()) {
    std::cerr << "failed to open CAN port: " << channel << std::endl;
    std::exit(1);
  }
  // Allow async RX thread to start.
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  return can;
}

inline void PrintHighSpeed(const HighSpeedState& hs, const char* unit_pos,
                           const char* unit_vel) {
  std::cout << "position=" << hs.position << " [" << unit_pos << "]"
            << " velocity=" << hs.velocity << " [" << unit_vel << "]"
            << " current=" << hs.current << " [A]"
            << " timestamp_ns=" << hs.timestamp_ns << std::endl;
}

inline void PrintLowSpeed(const LowSpeedState& ls) {
  std::cout << "bus_voltage_v=" << ls.bus_voltage_v
            << " driver_temp_deg=" << ls.driver_temp_deg
            << " motor_temp_deg=" << static_cast<int>(ls.motor_temp_deg)
            << " bus_current=" << ls.bus_current
            << " status_raw=" << static_cast<int>(ls.status_raw)
            << " enabled=" << ls.status.enabled << std::endl;
}

inline void PrintVersion(const VersionInfo& ver) {
  std::cout << "software=" << ver.software << " hardware=" << ver.hardware
            << " motor=" << ver.motor << std::endl;
}

}  // namespace agx::motor::demo
