#include "agx_motor_sdk/mechanism/arm_motor_base.hpp"

namespace agx::motor::mechanism {
namespace {

const std::string kEmptyString;
const JointConfig kEmptyJointConfig;

}  // namespace

NullArmMotor& NullArmMotor::Instance() {
  static NullArmMotor instance;
  return instance;
}

bool NullArmMotor::IsValid() const { return false; }
Protocol NullArmMotor::protocol() const { return Protocol::kArm; }
uint8_t NullArmMotor::NodeId() const { return 0; }
const std::string& NullArmMotor::CanChannel() const { return kEmptyString; }
const JointConfig& NullArmMotor::Config() const { return kEmptyJointConfig; }
const std::string& NullArmMotor::Name() const { return kEmptyString; }

bool NullArmMotor::SetEnable(bool, bool, bool) { return false; }
std::optional<JointState> NullArmMotor::GetState() const { return std::nullopt; }
bool NullArmMotor::SetMit(float, float, float) { return false; }
bool NullArmMotor::Configure() { return false; }

std::optional<agx::motor::HighSpeedState> NullArmMotor::GetHighSpeedState() const {
  return std::nullopt;
}
std::optional<agx::motor::LowSpeedState> NullArmMotor::GetLowSpeedState() const {
  return std::nullopt;
}
std::optional<agx::motor::VersionInfo> NullArmMotor::GetVersion(float) { return std::nullopt; }
bool NullArmMotor::SetTarget(float, uint8_t, float) { return false; }
bool NullArmMotor::SetTargetPosition(float, float) { return false; }
bool NullArmMotor::SetTargetVelocity(float, float) { return false; }
bool NullArmMotor::SetTargetCurrent(float, float) { return false; }
bool NullArmMotor::SetReset(int, float) { return false; }
bool NullArmMotor::SetClearError(float) { return false; }
bool NullArmMotor::SetZeroOffset(float, bool, float) { return false; }
bool NullArmMotor::SetProfileAccDec(float, float) { return false; }
bool NullArmMotor::SetProfileVelocity(float) { return false; }
bool NullArmMotor::SetCurrentLimit(float) { return false; }
bool NullArmMotor::SetMitControl(float, float, float, float, float, float) { return false; }
bool NullArmMotor::SetMitControlCrc(float, float, float, float, float, float) { return false; }
bool NullArmMotor::SetCollisionThreshold(float, float) { return false; }
std::optional<float> NullArmMotor::GetParam(agx::motor::ArmDriver::MotorParam, float) {
  return std::nullopt;
}
bool NullArmMotor::SetParam(agx::motor::ArmDriver::MotorParam, float, float) { return false; }

}  // namespace agx::motor::mechanism
