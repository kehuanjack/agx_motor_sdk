#pragma once

#include "agx_motor_sdk/mechanism/types.hpp"

#include <string>

namespace agx::motor::mechanism {

/// Load one module section from a robot motors YAML file.
/// @param yaml_path Path to the YAML file.
/// @param module_key Top-level module key (e.g. ``head``, ``fold``).
/// @return Parsed ``GroupConfig`` with resolved per-joint ``can_channel`` values.
/// @throws std::runtime_error if the file cannot be read, the key is missing, or
///         ``joints`` is empty.
GroupConfig LoadModuleConfig(const std::string& yaml_path, const std::string& module_key);

void ValidateGroupConfig(const GroupConfig& config);

}  // namespace agx::motor::mechanism
