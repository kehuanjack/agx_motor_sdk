#include "agx_motor_sdk/mechanism/config.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <cmath>
#include <set>
#include <stdexcept>

namespace agx::motor::mechanism {
namespace {

void RequireKey(const YAML::Node& node, const char* key, const std::string& context) {
  if (!node || !node[key]) {
    throw std::runtime_error("missing key '" + std::string(key) + "' in " + context);
  }
}

std::string RequireString(const YAML::Node& node, const char* key, const std::string& context) {
  RequireKey(node, key, context);
  return node[key].as<std::string>();
}

std::optional<float> OptionalFloat(const YAML::Node& node, const char* key) {
  if (node && node[key]) {
    return node[key].as<float>();
  }
  return std::nullopt;
}

std::optional<float> ResolveJointValue(const YAML::Node& node, const char* key,
                                       const std::optional<float>& module_default) {
  if (node && node[key]) {
    return node[key].as<float>();
  }
  return module_default;
}

float ResolveJointScalar(const YAML::Node& node, const char* key,
                         const std::optional<float>& module_default, float fallback) {
  const auto resolved = ResolveJointValue(node, key, module_default);
  return resolved.has_value() ? *resolved : fallback;
}

struct ModuleJointDefaults {
  std::optional<float> profile_vel;
  std::optional<float> profile_accel;
  std::optional<float> profile_decel;
  std::optional<float> mit_tf;
  std::optional<float> mit_kp;
  std::optional<float> mit_kd;
  std::optional<float> torque_k;
  std::optional<float> torque_b;
};

JointConfig ParseJointConfig(const YAML::Node& node, const std::string& context,
                             const ModuleJointDefaults& defaults,
                             const std::string& module_can_channel) {
  if (!node) {
    throw std::runtime_error("missing joint block in " + context);
  }
  JointConfig cfg;
  cfg.name = RequireString(node, "joint_name", context);
  RequireKey(node, "node_id", context);
  const int node_id = node["node_id"].as<int>();
  if (node_id < 1 || node_id > 15) {
    throw std::invalid_argument(context + ": node_id must be in 1..15");
  }
  cfg.node_id = static_cast<uint8_t>(node_id);
  if (node["can_channel"]) {
    cfg.can_channel = node["can_channel"].as<std::string>();
  } else {
    cfg.can_channel = module_can_channel;
  }
  cfg.polarity = node["polarity"] ? node["polarity"].as<float>() : 1.f;
  cfg.profile_vel_rad_s = ResolveJointValue(node, "profile_vel", defaults.profile_vel);
  cfg.profile_accel_rad_s2 = ResolveJointValue(node, "profile_accel", defaults.profile_accel);
  cfg.profile_decel_rad_s2 = ResolveJointValue(node, "profile_decel", defaults.profile_decel);
  cfg.mit_tf = ResolveJointValue(node, "mit_tf", defaults.mit_tf);
  cfg.mit_kp = ResolveJointValue(node, "mit_kp", defaults.mit_kp);
  cfg.mit_kd = ResolveJointValue(node, "mit_kd", defaults.mit_kd);
  cfg.torque_k = ResolveJointScalar(node, "torque_k", defaults.torque_k, 1.f);
  cfg.torque_b = ResolveJointScalar(node, "torque_b", defaults.torque_b, 1.f);
  if (node["lower_limit"]) {
    cfg.lower_limit_rad = node["lower_limit"].as<float>();
    cfg.has_lower_limit = true;
  }
  if (node["upper_limit"]) {
    cfg.upper_limit_rad = node["upper_limit"].as<float>();
    cfg.has_upper_limit = true;
  }
  return cfg;
}

}  // namespace

void ValidateGroupConfig(const GroupConfig& config) {
  if (config.can_channel.empty() || config.joints.empty()) {
    throw std::invalid_argument("group requires a CAN channel and non-empty joints");
  }
  const auto finite = [](const std::optional<float>& value) {
    if (value && !std::isfinite(*value)) {
      throw std::invalid_argument("configuration values must be finite");
    }
  };
  for (const auto& value : {config.default_profile_vel_rad_s, config.profile_accel_rad_s2,
                           config.profile_decel_rad_s2, config.mit_tf, config.mit_kp,
                           config.mit_kd, config.torque_k, config.torque_b}) {
    finite(value);
  }
  std::set<std::string> names;
  std::set<std::pair<std::string, uint8_t>> nodes;
  for (const auto& joint : config.joints) {
    const auto channel = joint.can_channel.empty() ? config.can_channel : joint.can_channel;
    if (joint.name.empty() || joint.node_id < 1 || joint.node_id > 15) {
      throw std::invalid_argument("joint requires a name and node_id in 1..15");
    }
    if (!names.insert(joint.name).second || !nodes.emplace(channel, joint.node_id).second) {
      throw std::invalid_argument("duplicate joint name or CAN channel/node_id");
    }
    for (const auto& value : {joint.profile_vel_rad_s, joint.profile_accel_rad_s2,
                             joint.profile_decel_rad_s2, joint.mit_tf, joint.mit_kp,
                             joint.mit_kd, std::optional<float>(joint.polarity),
                             std::optional<float>(joint.torque_k),
                             std::optional<float>(joint.torque_b)}) {
      finite(value);
    }
    if (joint.has_lower_limit) finite(joint.lower_limit_rad);
    if (joint.has_upper_limit) finite(joint.upper_limit_rad);
    if (joint.has_lower_limit && joint.has_upper_limit &&
        joint.lower_limit_rad > joint.upper_limit_rad) {
      throw std::invalid_argument("joint lower_limit must not exceed upper_limit");
    }
  }
}

GroupConfig LoadModuleConfig(const std::string& yaml_path, const std::string& module_key) {
  std::ifstream file(yaml_path);
  if (!file.is_open()) {
    throw std::runtime_error("cannot open yaml: " + yaml_path);
  }

  YAML::Node root;
  try {
    root = YAML::Load(file);
  } catch (const YAML::Exception& ex) {
    throw std::runtime_error(std::string("yaml parse error: ") + ex.what());
  }

  const YAML::Node module = root[module_key];
  if (!module) {
    throw std::runtime_error("missing key '" + module_key + "' in " + yaml_path);
  }

  GroupConfig config;
  config.can_channel = RequireString(module, "can_channel", yaml_path);
  config.default_profile_vel_rad_s = OptionalFloat(module, "profile_vel");
  config.profile_accel_rad_s2 = OptionalFloat(module, "profile_accel");
  config.profile_decel_rad_s2 = OptionalFloat(module, "profile_decel");
  config.mit_tf = OptionalFloat(module, "mit_tf");
  config.mit_kp = OptionalFloat(module, "mit_kp");
  config.mit_kd = OptionalFloat(module, "mit_kd");
  config.torque_k = OptionalFloat(module, "torque_k");
  config.torque_b = OptionalFloat(module, "torque_b");

  const ModuleJointDefaults defaults{
      config.default_profile_vel_rad_s,
      config.profile_accel_rad_s2,
      config.profile_decel_rad_s2,
      config.mit_tf,
      config.mit_kp,
      config.mit_kd,
      config.torque_k,
      config.torque_b,
  };

  const YAML::Node joints = module["joints"];
  if (!joints || !joints.IsSequence() || joints.size() == 0) {
    throw std::runtime_error("missing or empty 'joints' list in " + yaml_path +
                             " (module '" + module_key + "')");
  }
  config.joints.reserve(joints.size());
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const std::string context =
        yaml_path + " (" + module_key + "[" + std::to_string(i) + "])";
    config.joints.push_back(ParseJointConfig(joints[i], context, defaults, config.can_channel));
  }
  ValidateGroupConfig(config);
  return config;
}

}  // namespace agx::motor::mechanism
