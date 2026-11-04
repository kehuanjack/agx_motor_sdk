#include "agx_motor_sdk/motor/protocol_loader.hpp"

#include <dlfcn.h>

#include <cstdlib>
#include <filesystem>
#include <vector>

namespace agx {
namespace motor {
namespace {

std::string DefaultLibraryPath(const char* suffix, const char* env_var) {
  const char* env = std::getenv(env_var);
  if (env != nullptr && env[0] != '\0') {
    return env;
  }

  std::vector<std::filesystem::path> candidates;

  Dl_info sdk_info{};
  if (dladdr(reinterpret_cast<const void*>(&DefaultLibraryPath), &sdk_info) != 0 &&
      sdk_info.dli_fname != nullptr) {
    const auto sdk_dir =
        std::filesystem::absolute(sdk_info.dli_fname).parent_path();
    candidates.push_back(sdk_dir / "protocol" / suffix);
    candidates.push_back(sdk_dir / "agx_motor_sdk" / "protocol" / suffix);
  }

  const std::vector<std::filesystem::path> fallback_candidates = {
      std::filesystem::path("/usr/local/lib/agx_motor_sdk/protocol") / suffix,
      std::filesystem::path("/usr/lib/agx_motor_sdk/protocol") / suffix,
      std::filesystem::path("/usr/lib") / suffix,
      std::filesystem::path("/usr/local/lib") / suffix,
  };
  candidates.insert(candidates.end(), fallback_candidates.begin(),
                    fallback_candidates.end());

  auto append_prefix_paths = [&](const char* env_name) {
    const char* prefixes = std::getenv(env_name);
    if (prefixes == nullptr || prefixes[0] == '\0') {
      return;
    }
    std::string prefix;
    for (const char* p = prefixes; *p != '\0'; ++p) {
      if (*p == ':') {
        if (!prefix.empty()) {
          candidates.push_back(std::filesystem::path(prefix) / "lib" /
                               "agx_motor_sdk" / "protocol" / suffix);
          prefix.clear();
        }
      } else {
        prefix.push_back(*p);
      }
    }
    if (!prefix.empty()) {
      candidates.push_back(std::filesystem::path(prefix) / "lib" /
                           "agx_motor_sdk" / "protocol" / suffix);
    }
  };
  append_prefix_paths("AMENT_PREFIX_PATH");
  append_prefix_paths("COLCON_PREFIX_PATH");

  for (const auto& p : candidates) {
    if (std::filesystem::is_regular_file(p)) {
      return p.string();
    }
  }
  return suffix;
}

}  // namespace

ProtocolLibrary::ProtocolLibrary(const char* lib_filename, const char* env_var,
                                 const std::string& path) {
  const std::string lib_path =
      path.empty() ? DefaultLibraryPath(lib_filename, env_var) : path;
  dlerror();
  dl_handle_ = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (dl_handle_ == nullptr) {
    const char* dl_err = dlerror();
    load_error_ = dl_err != nullptr ? dl_err : "dlopen failed";
    return;
  }
  create_ = LoadSymbol<CreateFn>("motor_create");
  destroy_ = LoadSymbol<DestroyFn>("motor_destroy");
  set_tx_ = LoadSymbol<SetTxFn>("motor_set_tx_callback");
  has_tx_ = LoadSymbol<HasTxFn>("motor_has_tx_callback");
  rx_ = LoadSymbol<RxFn>("motor_handle_rx_once");
  hs_ = LoadSymbol<HsFn>("motor_get_high_speed_feedback");
  target_ = LoadSymbol<TargetFn>("motor_set_target");
  enable_ = LoadSymbol<EnableFn>("motor_set_enable");
  reset_ = LoadSymbol<ResetFn>("motor_set_reset");
  clear_ = LoadSymbol<ClearFn>("motor_set_clear_error");
  zero_offset_ = LoadSymbol<ZeroOffsetFn>("motor_set_zero_offset");
  profile_acc_dec_ = LoadSymbol<ProfileAccDecFn>("motor_set_profile_acc_dec");
  profile_vel_ = LoadSymbol<ProfileVelFn>("motor_set_profile_vel");
  current_limit_ = LoadSymbol<CurrentLimitFn>("motor_set_current_limit");
  version_ = LoadSymbol<VersionFn>("motor_get_version");
  if (create_ == nullptr || destroy_ == nullptr || rx_ == nullptr) {
    load_error_ = "protocol library missing required symbols";
  }
}

ProtocolLibrary::~ProtocolLibrary() {
  if (dl_handle_ != nullptr) {
    dlclose(dl_handle_);
  }
}

void* ProtocolLibrary::Symbol(const char* name) const {
  if (dl_handle_ == nullptr) {
    return nullptr;
  }
  dlerror();
  return dlsym(dl_handle_, name);
}

AgxMotorProtocolHandle* ProtocolLibrary::Create() {
  return create_ != nullptr ? create_() : nullptr;
}

void ProtocolLibrary::Destroy(AgxMotorProtocolHandle* handle) {
  if (destroy_ != nullptr && handle != nullptr) {
    destroy_(handle);
  }
}

void ProtocolLibrary::SetTxCallback(AgxMotorProtocolHandle* handle,
                                    AgxMotorTxCallback cb, void* ctx) {
  if (set_tx_ != nullptr) {
    set_tx_(handle, cb, ctx);
  }
}

int ProtocolLibrary::HasTxCallback(AgxMotorProtocolHandle* handle) const {
  return has_tx_ != nullptr ? has_tx_(handle) : 0;
}

int ProtocolLibrary::HandleRxOnce(AgxMotorProtocolHandle* handle, uint32_t can_id,
                                  const uint8_t* data, uint8_t dlc,
                                  uint64_t timestamp_ns) {
  return rx_ != nullptr ? rx_(handle, can_id, data, dlc, timestamp_ns) : 0;
}

int ProtocolLibrary::GetHighSpeedFeedback(AgxMotorProtocolHandle* handle,
                                          uint8_t node_id,
                                          AgxMotorHighSpeedFeedback* out) const {
  return hs_ != nullptr ? hs_(handle, node_id, out) : 0;
}

int ProtocolLibrary::SetTarget(AgxMotorProtocolHandle* handle, uint8_t node_id,
                               float value, uint8_t mode, float timeout_s) {
  return target_ != nullptr ? target_(handle, node_id, value, mode, timeout_s)
                            : 0;
}

int ProtocolLibrary::SetEnable(AgxMotorProtocolHandle* handle, uint8_t node_id,
                               int enable, int has_brake, int brake_on) {
  return enable_ != nullptr ? enable_(handle, node_id, enable, has_brake, brake_on)
                            : 0;
}

int ProtocolLibrary::SetReset(AgxMotorProtocolHandle* handle, uint8_t node_id,
                              int mode, float timeout_s, int* out_ok) {
  return reset_ != nullptr ? reset_(handle, node_id, mode, timeout_s, out_ok) : 0;
}

int ProtocolLibrary::SetClearError(AgxMotorProtocolHandle* handle,
                                  uint8_t node_id, float timeout_s,
                                  int* out_ok) {
  return clear_ != nullptr ? clear_(handle, node_id, 0, timeout_s, out_ok) : 0;
}

int ProtocolLibrary::SetZeroOffset(AgxMotorProtocolHandle* handle,
                                  uint8_t node_id, float offset,
                                  int save_to_flash, float timeout_s,
                                  int* out_ok) {
  return zero_offset_ != nullptr
             ? zero_offset_(handle, node_id, offset, save_to_flash, timeout_s,
                            out_ok)
             : 0;
}

int ProtocolLibrary::SetProfileAccDec(AgxMotorProtocolHandle* handle,
                                     uint8_t node_id, float accel,
                                     float decel) {
  return profile_acc_dec_ != nullptr
             ? profile_acc_dec_(handle, node_id, accel, decel)
             : 0;
}

int ProtocolLibrary::SetProfileVel(AgxMotorProtocolHandle* handle,
                                 uint8_t node_id, float velocity) {
  return profile_vel_ != nullptr ? profile_vel_(handle, node_id, velocity) : 0;
}

int ProtocolLibrary::SetCurrentLimit(AgxMotorProtocolHandle* handle,
                                    uint8_t node_id, float current) {
  return current_limit_ != nullptr ? current_limit_(handle, node_id, current) : 0;
}

int ProtocolLibrary::GetVersion(AgxMotorProtocolHandle* handle, uint8_t node_id,
                                float timeout_s, AgxMotorVersionInfo* out) {
  return version_ != nullptr ? version_(handle, node_id, timeout_s, out) : 0;
}

}  // namespace motor
}  // namespace agx
