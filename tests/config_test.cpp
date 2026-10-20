#include "agx_motor_sdk/mechanism/config.hpp"
#include "agx_motor_sdk/mechanism/joint_group.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

using agx::motor::mechanism::GroupConfig;
using agx::motor::mechanism::JointConfig;
using agx::motor::mechanism::LoadModuleConfig;
using agx::motor::mechanism::ValidateGroupConfig;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Operation>
void Reject(Operation operation, const std::string& message) {
  try {
    operation();
  } catch (const std::invalid_argument&) {
    return;
  }
  throw std::runtime_error("accepted invalid config: " + message);
}

GroupConfig ValidConfig() {
  GroupConfig config;
  config.can_channel = "fake0";
  JointConfig joint;
  joint.name = "joint";
  joint.node_id = 1;
  config.joints.push_back(joint);
  return config;
}

struct TemporaryYaml {
  std::string path;
  TemporaryYaml() {
    char pattern[] = "/tmp/agx-sdk-config-XXXXXX";
    const int descriptor = mkstemp(pattern);
    Require(descriptor >= 0, "mkstemp failed");
    close(descriptor);
    path = pattern;
  }
  ~TemporaryYaml() { std::remove(path.c_str()); }
  void Write(int node) {
    std::ofstream stream(path);
    stream << "head:\n  can_channel: fake0\n  profile_vel: 2.5\n"
              "  joints:\n    - joint_name: yaw\n      node_id: "
           << node << "\n    - joint_name: pitch\n      node_id: 1\n"
                       "      can_channel: fake1\n      polarity: -1\n";
    stream.close();
    Require(!stream.fail(), "temporary YAML write failed");
  }
};

int main() {
  try {
    auto config = ValidConfig();
    ValidateGroupConfig(config);
    config.joints.front().node_id = 15;
    ValidateGroupConfig(config);
    for (const int node : {0, 16, 255}) {
      config = ValidConfig();
      config.joints.front().node_id = static_cast<uint8_t>(node);
      Reject([&] { ValidateGroupConfig(config); }, "node boundary");
    }
    config = ValidConfig();
    config.can_channel.clear();
    Reject([&] { ValidateGroupConfig(config); }, "empty channel");
    config = ValidConfig();
    config.joints.clear();
    Reject([&] { ValidateGroupConfig(config); }, "empty joints");
    config = ValidConfig();
    config.joints.front().name.clear();
    Reject([&] { ValidateGroupConfig(config); }, "empty name");
    config = ValidConfig();
    config.joints.push_back(config.joints.front());
    config.joints.back().node_id = 2;
    Reject([&] { ValidateGroupConfig(config); }, "duplicate name");
    config.joints.back().name = "other";
    config.joints.back().node_id = 1;
    config.joints.back().can_channel = "fake0";
    Reject([&] { ValidateGroupConfig(config); }, "duplicate resolved node");
    config.joints.back().can_channel = "fake1";
    ValidateGroupConfig(config);
        Reject([&] { agx::motor::mechanism::ArmJointGroup group(config, "fake1"); },
          "channel override merges duplicate nodes");
        Reject([&] { agx::motor::mechanism::ArmJointGroup group(config, ""); },
          "empty channel override");

    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (auto member : {&GroupConfig::default_profile_vel_rad_s,
                        &GroupConfig::profile_accel_rad_s2, &GroupConfig::profile_decel_rad_s2,
                        &GroupConfig::mit_tf, &GroupConfig::mit_kp, &GroupConfig::mit_kd,
                        &GroupConfig::torque_k, &GroupConfig::torque_b}) {
      config = ValidConfig();
      config.*member = nan;
      Reject([&] { ValidateGroupConfig(config); }, "module NaN");
    }
    for (auto member : {&JointConfig::profile_vel_rad_s, &JointConfig::profile_accel_rad_s2,
                        &JointConfig::profile_decel_rad_s2, &JointConfig::mit_tf,
                        &JointConfig::mit_kp, &JointConfig::mit_kd}) {
      config = ValidConfig();
      config.joints.front().*member = nan;
      Reject([&] { ValidateGroupConfig(config); }, "joint optional NaN");
    }
    for (auto member : {&JointConfig::polarity, &JointConfig::torque_k, &JointConfig::torque_b,
                        &JointConfig::lower_limit_rad, &JointConfig::upper_limit_rad}) {
      config = ValidConfig();
      config.joints.front().has_lower_limit = true;
      config.joints.front().has_upper_limit = true;
      config.joints.front().*member = nan;
      Reject([&] { ValidateGroupConfig(config); }, "joint scalar NaN");
    }
    config = ValidConfig();
    auto& joint = config.joints.front();
    joint.has_lower_limit = joint.has_upper_limit = true;
    joint.lower_limit_rad = 1.f;
    joint.upper_limit_rad = -1.f;
    Reject([&] { ValidateGroupConfig(config); }, "inverted limits");
    joint.upper_limit_rad = 1.f;
    ValidateGroupConfig(config);
    joint.polarity = 0.f;
    ValidateGroupConfig(config);
    joint.polarity = std::numeric_limits<float>::infinity();
    Reject([&] { ValidateGroupConfig(config); }, "infinity");

    TemporaryYaml yaml;
    for (const int node : {-1, 0, 16, 257}) {
      yaml.Write(node);
      Reject([&] { LoadModuleConfig(yaml.path, "head"); }, "YAML node " + std::to_string(node));
    }
    for (const int node : {1, 15}) {
      yaml.Write(node);
      const auto loaded = LoadModuleConfig(yaml.path, "head");
      Require(loaded.joints.size() == 2, "YAML joint count");
      Require(loaded.joints.front().node_id == node, "YAML node narrowed incorrectly");
      Require(loaded.joints.front().can_channel == "fake0", "module channel inheritance");
      Require(loaded.joints.back().can_channel == "fake1", "joint channel override");
      Require(loaded.joints.front().profile_vel_rad_s == 2.5f, "profile inheritance");
      Require(loaded.joints.back().polarity == -1.f, "polarity override");
    }
    ValidateGroupConfig(LoadModuleConfig(AGX_SDK_EXAMPLE_YAML, "head"));
    std::cout << "configuration software checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}