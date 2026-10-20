#include <agx_motor_sdk/mechanism/config.hpp>
#include <agx_motor_sdk/mechanism/arm_motor.hpp>
#include <agx_motor_sdk/mechanism/can_bus.hpp>
#include <agx_motor_sdk/mechanism/motor_registry.hpp>

int main() {
  agx::motor::mechanism::GroupConfig config;
  config.can_channel = "unused-no-CAN";
  agx::motor::mechanism::JointConfig joint;
  joint.name = "installed";
  joint.node_id = 1;
  config.joints.push_back(joint);
  agx::motor::mechanism::ValidateGroupConfig(config);
  auto& registry = agx::motor::mechanism::MotorRegistry::Default();
  auto& motor = registry.AcquireArm(config.can_channel, {}, 1, joint);
  const bool failed = motor.SetEnable() || motor.GetState().has_value();
  registry.Clear();
  return failed ? 1 : 0;
}