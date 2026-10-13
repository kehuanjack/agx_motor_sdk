#include <iostream>
#include <string>

#include "agx_motor_sdk/mechanism/joint_group.hpp"
#include "demo_utils.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " <yaml_path> [can_channel] [position_rad]"
              << std::endl;
    std::cerr << "  example: " << argv[0]
              << " ../examples/mechanism/robot_motors.yaml can0 0" << std::endl;
    return 1;
  }

  const std::string yaml_path = argv[1];
  const std::string can_channel = argc > 2 ? argv[2] : "";
  const bool send_position = argc > 3;
  float position_rad = 0.f;
  if (send_position) {
    try {
      position_rad = std::stof(argv[3]);
    } catch (...) {
      std::cerr << "invalid position_rad: " << argv[3] << std::endl;
      return 1;
    }
  }

  agx::motor::mechanism::ArmJointGroup group =
      can_channel.empty()
          ? agx::motor::mechanism::ArmJointGroup::FromYaml(yaml_path, "head")
          : agx::motor::mechanism::ArmJointGroup::FromYaml(yaml_path, "head",
                                                           can_channel);

  if (!group.Connect()) {
    std::cerr << "connect failed" << std::endl;
    return 1;
  }
  group.Configure();
  group.Enable();

  for (const auto& name : group.JointNames()) {
    const int idx = group.IndexOf(name);
    if (!group.IsValidJointIndex(idx)) {
      continue;
    }
    auto& motor = group.MotorArm(idx);
    if (!motor.IsValid()) {
      std::cout << name << ": invalid motor handle" << std::endl;
      continue;
    }

    if (send_position) {
      const bool ok = group.SetTargetPosition(static_cast<std::size_t>(idx), position_rad);
      std::cout << name << ": SetTargetPosition(" << position_rad << ") -> " << ok
                << std::endl;
    }

    if (auto state = motor.GetState()) {
      std::cout << name << " node_id=" << static_cast<int>(motor.NodeId())
                << " pos=" << state->position_rad << " rad"
                << " vel=" << state->velocity_rad_s << " rad/s"
                << " torque=" << state->torque_nm << " N·m" << std::endl;
    } else {
      std::cout << name << " node_id=" << static_cast<int>(motor.NodeId())
                << " state=None (no feedback yet)" << std::endl;
    }
  }

  group.Close();
  return 0;
}
