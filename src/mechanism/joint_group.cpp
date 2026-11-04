#include "agx_motor_sdk/mechanism/joint_group.hpp"

#include "agx_motor_sdk/mechanism/config.hpp"
#include "agx_motor_sdk/mechanism/arm/arm_motor.hpp"
#include "agx_motor_sdk/mechanism/motor_registry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace agx::motor::mechanism {
namespace {

void ValidateModuleConfig(const GroupConfig& config, const char* context) {
  ValidateGroupConfig(config);
  if (config.joints.empty()) {
    throw std::invalid_argument(std::string(context) + ": group config has no joints");
  }
}

void ValidateInterval(double interval_s) {
  if (!std::isfinite(interval_s) || interval_s < 0.0) {
    throw std::invalid_argument("interval_s must be a finite value >= 0");
  }
}

void SleepInterval(double interval_s) {
  if (interval_s > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(interval_s));
  }
}

}  // namespace

JointGroup::JointGroup(const GroupConfig& config)
    : config_(config), can_channel_(config.can_channel) {
  ValidateModuleConfig(config_, "JointGroup");
}

JointGroup::JointGroup(const GroupConfig& config, std::string can_channel)
    : JointGroup(config) {
  ApplyDefaultChannelOverride(can_channel);
}

JointGroup JointGroup::FromYaml(const std::string& yaml_path, const std::string& module_key) {
  return JointGroup(LoadModuleConfig(yaml_path, module_key));
}

JointGroup JointGroup::FromYaml(const std::string& yaml_path, const std::string& module_key,
                                const std::string& can_channel) {
  return JointGroup(LoadModuleConfig(yaml_path, module_key), can_channel);
}

JointGroup::~JointGroup() { Close(); }

JointGroup::JointGroup(JointGroup&& other) noexcept
    : config_(std::move(other.config_)),
      can_channel_(std::move(other.can_channel_)),
      injected_cans_(std::move(other.injected_cans_)),
      buses_(std::move(other.buses_)) {}

JointGroup& JointGroup::operator=(JointGroup&& other) noexcept {
  if (this != &other) {
    Close();
    config_ = std::move(other.config_);
    can_channel_ = std::move(other.can_channel_);
    injected_cans_ = std::move(other.injected_cans_);
    buses_ = std::move(other.buses_);
  }
  return *this;
}

void JointGroup::ApplyDefaultChannelOverride(const std::string& can_channel) {
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

void JointGroup::AttachCanPort(const std::shared_ptr<agx::motor::CanPort>& can) {
  injected_cans_[can_channel_] = can;
}

void JointGroup::AttachCanPort(const std::string& channel,
                               const std::shared_ptr<agx::motor::CanPort>& can) {
  injected_cans_[channel] = can;
}

bool JointGroup::Connect() {
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
    std::shared_ptr<CanBus> bus;
    for (std::size_t i = 0; i < config_.joints.size(); ++i) {
      if (JointCanChannel(i) != channel) {
        continue;
      }
      const auto& joint = config_.joints[i];
      if (!bus) {
        bus = CanBusRegistry::Default().Acquire(channel, injected, joint.protocol);
      } else {
        bus->EnsureDriver(joint.protocol);
      }
      bus->BindNode(joint.node_id, joint.protocol);
    }
    buses_[channel] = bus;
    ok = bus && bus->can_port() && bus->can_port()->IsOpen() && ok;
  }
  return ok;
}

void JointGroup::Close() { buses_.clear(); }

bool JointGroup::IsConnected() const {
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
  }
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    if (!BusForJoint(i)->driver(config_.joints[i].protocol)) {
      return false;
    }
  }
  return true;
}

CanBus* JointGroup::Bus(const std::string& channel) {
  const auto it = buses_.find(channel);
  return it == buses_.end() ? nullptr : it->second.get();
}

const CanBus* JointGroup::Bus(const std::string& channel) const {
  const auto it = buses_.find(channel);
  return it == buses_.end() ? nullptr : it->second.get();
}

CanBus* JointGroup::BusForJoint(std::size_t index) {
  return Bus(JointCanChannel(index));
}

const CanBus* JointGroup::BusForJoint(std::size_t index) const {
  return Bus(JointCanChannel(index));
}

agx::motor::MotorDriver* JointGroup::GetDriver(const std::string& channel) const {
  const CanBus* bus = Bus(channel.empty() ? can_channel_ : channel);
  return bus ? bus->driver() : nullptr;
}

agx::motor::MotorDriver* JointGroup::GetDriverForJoint(std::size_t index) const {
  const CanBus* bus = BusForJoint(index);
  return bus ? bus->driver(JointCfg(index).protocol) : nullptr;
}

const JointConfig& JointGroup::JointCfg(std::size_t index) const {
  if (index >= config_.joints.size()) {
    throw std::out_of_range("joint index out of range");
  }
  return config_.joints[index];
}

std::size_t JointGroup::JointCount() const { return config_.joints.size(); }

std::vector<std::string> JointGroup::JointNames() const {
  std::vector<std::string> names;
  names.reserve(config_.joints.size());
  for (const auto& joint : config_.joints) {
    names.push_back(joint.name);
  }
  return names;
}

int JointGroup::IndexOf(const std::string& joint_name) const {
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    if (config_.joints[i].name == joint_name) {
      return static_cast<int>(i);
    }
  }
  return kInvalidJointIndex;
}

bool JointGroup::IsValidJointIndex(int index) const {
  return IsJointIndexInRange(index, JointCount());
}

const JointConfig& JointGroup::GetJointConfig(std::size_t index) const {
  return JointCfg(index);
}

uint8_t JointGroup::NodeId(std::size_t index) const { return JointCfg(index).node_id; }

std::string JointGroup::JointCanChannel(std::size_t index) const {
  return JointCfg(index).can_channel;
}

std::vector<std::string> JointGroup::CanChannels() const {
  std::vector<std::string> channels;
  channels.reserve(config_.joints.size());
  for (const auto& joint : config_.joints) {
    if (std::find(channels.begin(), channels.end(), joint.can_channel) == channels.end()) {
      channels.push_back(joint.can_channel);
    }
  }
  return channels;
}

MotorBase& JointGroup::MotorAt(std::size_t index) {
  if (index >= config_.joints.size()) {
    throw std::out_of_range("joint index out of range");
  }
  if (!IsConnected() && !Connect()) {
    throw std::runtime_error("joint group not connected");
  }
  const auto* bus = BusForJoint(index);
  const auto driver = bus ? bus->driver_shared(JointCfg(index).protocol) : nullptr;
  if (!driver) {
    throw std::runtime_error("motor driver not available");
  }
  return MotorRegistry::Default().Acquire(JointCanChannel(index), driver, NodeId(index),
                                          JointCfg(index));
}

MotorBase& JointGroup::Motor(int index) {
  if (!IsValidJointIndex(index)) {
    return MotorBase::Null();
  }
  return MotorAt(static_cast<std::size_t>(index));
}

const MotorBase& JointGroup::Motor(int index) const {
  return const_cast<JointGroup*>(this)->Motor(index);
}

bool JointGroup::Enable(double interval_s) {
  ValidateInterval(interval_s);
  if (!IsConnected() && !Connect()) {
    return false;
  }
  bool ok = true;
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    if (i > 0) {
      SleepInterval(interval_s);
    }
    ok = MotorAt(i).SetEnable(true) && ok;
  }
  return ok;
}

bool JointGroup::Disable(double interval_s) {
  ValidateInterval(interval_s);
  if (!IsConnected()) {
    return false;
  }
  bool ok = true;
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    if (i > 0) {
      SleepInterval(interval_s);
    }
    ok = MotorAt(i).SetEnable(false) && ok;
  }
  return ok;
}

bool JointGroup::Configure() {
  if (!IsConnected() && !Connect()) {
    return false;
  }
  bool ok = true;
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    ok = MotorAt(i).Configure() && ok;
  }
  return ok;
}

bool JointGroup::SetTargetPosition(std::size_t index, float position_rad, float timeout_s) {
  if (!IsValidJointIndex(static_cast<int>(index))) {
    return false;
  }
  if (!IsConnected() && !Connect()) {
    return false;
  }
  return MotorAt(index).SetTargetPosition(position_rad, timeout_s);
}

std::optional<JointState> JointGroup::GetState(std::size_t index) const {
  if (!IsValidJointIndex(static_cast<int>(index))) {
    return std::nullopt;
  }
  return const_cast<JointGroup*>(this)->MotorAt(index).GetState();
}

bool JointGroup::SetMit(std::size_t index, float position_rad, float velocity_rad_s,
                           float torque_nm) {
  if (index >= config_.joints.size()) {
    return false;
  }
  if (!IsConnected() && !Connect()) {
    return false;
  }
  auto* arm = dynamic_cast<ArmMotor*>(&MotorAt(index));
  return arm != nullptr && arm->SetMit(position_rad, velocity_rad_s, torque_nm);
}

std::unique_ptr<JointGroup> CreateJointGroup(const GroupConfig& config) {
  return std::make_unique<JointGroup>(config);
}

std::unique_ptr<JointGroup> CreateJointGroup(const GroupConfig& config,
                                             std::string can_channel) {
  return std::make_unique<JointGroup>(config, std::move(can_channel));
}

std::unique_ptr<JointGroup> CreateJointGroupFromYaml(const std::string& yaml_path,
                                                     const std::string& module_key) {
  return CreateJointGroup(LoadModuleConfig(yaml_path, module_key));
}

std::unique_ptr<JointGroup> CreateJointGroupFromYaml(const std::string& yaml_path,
                                                     const std::string& module_key,
                                                     const std::string& can_channel) {
  return CreateJointGroup(LoadModuleConfig(yaml_path, module_key), can_channel);
}

}  // namespace agx::motor::mechanism
