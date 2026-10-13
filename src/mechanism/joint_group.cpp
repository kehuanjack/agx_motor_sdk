#include "agx_motor_sdk/mechanism/joint_group.hpp"

#include "agx_motor_sdk/mechanism/config.hpp"
#include "agx_motor_sdk/mechanism/motor_registry.hpp"

#include <algorithm>
#include <stdexcept>

namespace agx::motor::mechanism {
namespace {

void ValidateModuleConfig(const GroupConfig& config, const char* context) {
  ValidateGroupConfig(config);
  if (config.joints.empty()) {
    throw std::invalid_argument(std::string(context) + ": group config has no joints");
  }
}

}  // namespace

JointGroupBase::JointGroupBase(GroupConfig config, std::string can_channel)
    : config_(std::move(config)), can_channel_(std::move(can_channel)) {
  ValidateModuleConfig(config_, "JointGroupBase");
}

JointGroupBase::~JointGroupBase() { Close(); }

JointGroupBase::JointGroupBase(JointGroupBase&& other) noexcept
    : config_(std::move(other.config_)),
      can_channel_(std::move(other.can_channel_)),
      injected_cans_(std::move(other.injected_cans_)),
      buses_(std::move(other.buses_)) {}

JointGroupBase& JointGroupBase::operator=(JointGroupBase&& other) noexcept {
  if (this != &other) {
    Close();
    config_ = std::move(other.config_);
    can_channel_ = std::move(other.can_channel_);
    injected_cans_ = std::move(other.injected_cans_);
    buses_ = std::move(other.buses_);
  }
  return *this;
}

void JointGroupBase::ApplyDefaultChannelOverride(const std::string& can_channel) {
  const std::string previous_default = can_channel_;
  auto updated = config_;
  updated.can_channel = can_channel;
  for (auto& joint : updated.joints) {
    if (joint.can_channel.empty() || joint.can_channel == previous_default) {
      joint.can_channel = can_channel;
    }
  }
  ValidateGroupConfig(updated);
  config_ = std::move(updated);
  can_channel_ = can_channel;
}

void JointGroupBase::AttachCanPort(const std::shared_ptr<agx::motor::CanPort>& can) {
  injected_cans_[can_channel_] = can;
}

void JointGroupBase::AttachCanPort(const std::string& channel,
                               const std::shared_ptr<agx::motor::CanPort>& can) {
  injected_cans_[channel] = can;
}

bool JointGroupBase::Connect() {
  const auto channels = CanChannels();
  if (channels.empty()) {
    return false;
  }

  bool ok = true;
  for (const auto& channel : channels) {
    if (buses_.count(channel) != 0 && buses_[channel] && buses_[channel]->can_port() &&
        buses_[channel]->can_port()->IsOpen()) {
      continue;
    }
    std::shared_ptr<agx::motor::CanPort> injected;
    const auto injected_it = injected_cans_.find(channel);
    if (injected_it != injected_cans_.end()) {
      injected = injected_it->second;
    }
    buses_[channel] = CanBusRegistry::Default().Acquire(channel, injected);
    ok = buses_[channel] && buses_[channel]->can_port() &&
         buses_[channel]->can_port()->IsOpen() && ok;
  }
  return ok;
}

void JointGroupBase::Close() { buses_.clear(); }

bool JointGroupBase::IsConnected() const {
  const auto channels = CanChannels();
  if (channels.empty() || buses_.size() != channels.size()) {
    return false;
  }
  for (const auto& channel : channels) {
    const auto bus_it = buses_.find(channel);
    if (bus_it == buses_.end() || !bus_it->second || !bus_it->second->can_port() ||
        !bus_it->second->can_port()->IsOpen()) {
      return false;
    }
    if (!bus_it->second->arm_driver()) {
      return false;
    }
  }
  return true;
}

CanBus* JointGroupBase::Bus(const std::string& channel) {
  const auto it = buses_.find(channel);
  return it == buses_.end() ? nullptr : it->second.get();
}

const CanBus* JointGroupBase::Bus(const std::string& channel) const {
  const auto it = buses_.find(channel);
  return it == buses_.end() ? nullptr : it->second.get();
}

CanBus* JointGroupBase::BusForJoint(std::size_t index) {
  return Bus(JointCanChannel(index));
}

const CanBus* JointGroupBase::BusForJoint(std::size_t index) const {
  return Bus(JointCanChannel(index));
}

agx::motor::ArmDriver* JointGroupBase::ArmDriverPtr(const std::string& channel) {
  CanBus* bus = Bus(channel);
  return bus ? bus->arm_driver() : nullptr;
}

const agx::motor::ArmDriver* JointGroupBase::ArmDriverPtr(const std::string& channel) const {
  const CanBus* bus = Bus(channel);
  return bus ? bus->arm_driver() : nullptr;
}

agx::motor::ArmDriver* JointGroupBase::ArmDriverForJoint(std::size_t index) {
  return ArmDriverPtr(JointCanChannel(index));
}

const agx::motor::ArmDriver* JointGroupBase::ArmDriverForJoint(std::size_t index) const {
  return ArmDriverPtr(JointCanChannel(index));
}

const JointConfig& JointGroupBase::JointCfg(std::size_t index) const {
  if (index >= config_.joints.size()) {
    throw std::out_of_range("joint index out of range");
  }
  return config_.joints[index];
}

std::size_t JointGroupBase::JointCount() const { return config_.joints.size(); }

std::vector<std::string> JointGroupBase::JointNames() const {
  std::vector<std::string> names;
  names.reserve(config_.joints.size());
  for (const auto& joint : config_.joints) {
    names.push_back(joint.name);
  }
  return names;
}

int JointGroupBase::IndexOf(const std::string& joint_name) const {
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    if (config_.joints[i].name == joint_name) {
      return static_cast<int>(i);
    }
  }
  return kInvalidJointIndex;
}

bool JointGroupBase::IsValidJointIndex(int index) const {
  return IsJointIndexInRange(index, JointCount());
}

const JointConfig& JointGroupBase::GetJointConfig(std::size_t index) const {
  return JointCfg(index);
}

uint8_t JointGroupBase::NodeId(std::size_t index) const { return JointCfg(index).node_id; }

std::string JointGroupBase::JointCanChannel(std::size_t index) const {
  return JointCfg(index).can_channel;
}

std::vector<std::string> JointGroupBase::CanChannels() const {
  std::vector<std::string> channels;
  channels.reserve(config_.joints.size());
  for (const auto& joint : config_.joints) {
    if (std::find(channels.begin(), channels.end(), joint.can_channel) == channels.end()) {
      channels.push_back(joint.can_channel);
    }
  }
  return channels;
}

ArmMotor& JointGroupBase::ArmMotorAt(std::size_t index) {
  if (index >= config_.joints.size()) {
    throw std::out_of_range("joint index out of range");
  }
  if (!IsConnected() && !Connect()) {
    throw std::runtime_error("joint group not connected");
  }
  const auto* bus = BusForJoint(index);
  const auto driver = bus ? bus->arm_driver_shared() : nullptr;
  if (!driver) {
    throw std::runtime_error("arm driver not available");
  }
  return MotorRegistry::Default().AcquireArm(JointCanChannel(index), driver, NodeId(index),
                                            JointCfg(index));
}

const ArmMotor& JointGroupBase::ArmMotorAt(std::size_t index) const {
  return const_cast<JointGroupBase*>(this)->ArmMotorAt(index);
}

bool JointGroupBase::Enable() {
  if (!IsConnected() && !Connect()) {
    return false;
  }
  bool ok = true;
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    ok = ArmMotorAt(i).SetEnable(true) && ok;
  }
  return ok;
}

bool JointGroupBase::Disable() {
  if (!IsConnected()) {
    return false;
  }
  bool ok = true;
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    ok = ArmMotorAt(i).SetEnable(false) && ok;
  }
  return ok;
}

bool JointGroupBase::Configure() {
  if (!IsConnected() && !Connect()) {
    return false;
  }
  bool ok = true;
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    ok = ArmMotorAt(i).Configure() && ok;
  }
  return ok;
}

bool JointGroupBase::SetTargetPosition(std::size_t index, float position_rad, float timeout_s) {
  if (!IsValidJointIndex(static_cast<int>(index))) {
    return false;
  }
  if (!IsConnected() && !Connect()) {
    return false;
  }
  return ArmMotorAt(index).SetTargetPosition(position_rad, timeout_s);
}

std::optional<JointState> JointGroupBase::GetState(std::size_t index) const {
  if (!IsValidJointIndex(static_cast<int>(index))) {
    return std::nullopt;
  }
  return const_cast<JointGroupBase*>(this)->ArmMotorAt(index).GetState();
}

ArmJointGroup::ArmJointGroup(const GroupConfig& config)
    : JointGroupBase(config, config.can_channel) {
  ValidateModuleConfig(config_, "ArmJointGroup");
}

ArmJointGroup::ArmJointGroup(const GroupConfig& config, std::string can_channel)
    : JointGroupBase(config, config.can_channel) {
  ValidateModuleConfig(config_, "ArmJointGroup");
  ApplyDefaultChannelOverride(std::move(can_channel));
}

ArmJointGroup ArmJointGroup::FromYaml(const std::string& yaml_path,
                                      const std::string& module_key) {
  return ArmJointGroup(LoadModuleConfig(yaml_path, module_key));
}

ArmJointGroup ArmJointGroup::FromYaml(const std::string& yaml_path,
                                      const std::string& module_key,
                                      const std::string& can_channel) {
  return ArmJointGroup(LoadModuleConfig(yaml_path, module_key), can_channel);
}

ArmMotorBase& ArmJointGroup::MotorArm(int index) {
  if (!IsValidJointIndex(index)) {
    return NullArmMotor::Instance();
  }
  return ArmMotorAt(static_cast<std::size_t>(index));
}

const ArmMotorBase& ArmJointGroup::MotorArm(int index) const {
  return const_cast<ArmJointGroup*>(this)->MotorArm(index);
}

agx::motor::ArmDriver* ArmJointGroup::GetArmDriver(const std::string& channel) {
  const std::string resolved = channel.empty() ? can_channel() : channel;
  return ArmDriverPtr(resolved);
}

const agx::motor::ArmDriver* ArmJointGroup::GetArmDriver(const std::string& channel) const {
  const std::string resolved = channel.empty() ? can_channel() : channel;
  return ArmDriverPtr(resolved);
}

agx::motor::ArmDriver* ArmJointGroup::GetArmDriverForJoint(std::size_t index) {
  return ArmDriverForJoint(index);
}

const agx::motor::ArmDriver* ArmJointGroup::GetArmDriverForJoint(std::size_t index) const {
  return ArmDriverForJoint(index);
}

bool ArmJointGroup::SetMit(std::size_t index, float position_rad, float velocity_rad_s,
                           float torque_nm) {
  if (index >= config_.joints.size()) {
    return false;
  }
  if (!IsConnected() && !Connect()) {
    return false;
  }
  return ArmMotorAt(index).SetMit(position_rad, velocity_rad_s, torque_nm);
}

std::unique_ptr<JointGroupBase> CreateJointGroup(const GroupConfig& config) {
  return std::make_unique<ArmJointGroup>(config);
}

std::unique_ptr<JointGroupBase> CreateJointGroup(const GroupConfig& config,
                                             std::string can_channel) {
  return std::make_unique<ArmJointGroup>(config, std::move(can_channel));
}

std::unique_ptr<JointGroupBase> CreateJointGroupFromYaml(const std::string& yaml_path,
                                                     const std::string& module_key) {
  return CreateJointGroup(LoadModuleConfig(yaml_path, module_key));
}

std::unique_ptr<JointGroupBase> CreateJointGroupFromYaml(const std::string& yaml_path,
                                                     const std::string& module_key,
                                                     const std::string& can_channel) {
  return CreateJointGroup(LoadModuleConfig(yaml_path, module_key), can_channel);
}

std::unique_ptr<ArmJointGroup> CreateArmJointGroup(const GroupConfig& config) {
  ValidateModuleConfig(config, "CreateArmJointGroup");
  return std::make_unique<ArmJointGroup>(config);
}

std::unique_ptr<ArmJointGroup> CreateArmJointGroup(const GroupConfig& config,
                                                   std::string can_channel) {
  ValidateModuleConfig(config, "CreateArmJointGroup");
  return std::make_unique<ArmJointGroup>(config, std::move(can_channel));
}

}  // namespace agx::motor::mechanism
