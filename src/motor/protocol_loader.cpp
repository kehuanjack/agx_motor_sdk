#include "agx_motor_sdk/motor/protocol_loader.hpp"

#include <dlfcn.h>

#include <cstdlib>
#include <filesystem>
#include <vector>

namespace agx {
namespace motor {
namespace {

template <typename T>
T LoadSymbol(void* handle, const char* name) {
  dlerror();
  auto* sym = dlsym(handle, name);
  if (!sym) {
    return nullptr;
  }
  return reinterpret_cast<T>(sym);
}

std::string DefaultLibraryPath(ProtocolVariant variant) {
  const char* env = variant == ProtocolVariant::kArm
                        ? std::getenv("AGX_MOTOR_SDK_ARM_LIB")
                        : std::getenv("AGX_MOTOR_SDK_CHASSIS_LIB");
  if (env != nullptr && env[0] != '\0') {
    return env;
  }

  const char* suffix = "libagx_motor_protocol_arm.so";
  if (variant == ProtocolVariant::kChassis) {
    suffix = "libagx_motor_protocol_chassis.so";
  }

  std::vector<std::filesystem::path> candidates = {
      std::filesystem::path("protocol") / suffix,
      std::filesystem::path("agx_motor_sdk/protocol") / suffix,
      std::filesystem::path("/usr/local/lib/agx_motor_sdk/protocol") / suffix,
      std::filesystem::path("/usr/lib/agx_motor_sdk/protocol") / suffix,
      std::filesystem::path("/usr/lib") / suffix,
      std::filesystem::path("/usr/local/lib") / suffix,
  };

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

ProtocolLibrary::ProtocolLibrary(void* dl_handle, ProtocolVariant variant)
    : dl_handle_(dl_handle), variant_(variant) {
  if (dl_handle_ == nullptr) {
    return;
  }
  create_ = LoadSymbol<CreateFn>(dl_handle_, "motor_create");
  destroy_ = LoadSymbol<DestroyFn>(dl_handle_, "motor_destroy");
  set_tx_ = LoadSymbol<SetTxFn>(dl_handle_, "motor_set_tx_callback");
  has_tx_ = LoadSymbol<HasTxFn>(dl_handle_, "motor_has_tx_callback");
  rx_ = LoadSymbol<RxFn>(dl_handle_, "motor_handle_rx_once");
  hs_ = LoadSymbol<HsFn>(dl_handle_, "motor_get_high_speed_feedback");
  ls_ = LoadSymbol<LsFn>(dl_handle_, "motor_get_low_speed_feedback");
  target_ = LoadSymbol<TargetFn>(dl_handle_, "motor_set_target");
  enable_ = LoadSymbol<EnableFn>(dl_handle_, "motor_set_enable");
  reset_ = LoadSymbol<ResetFn>(dl_handle_, "motor_set_reset");
  clear_ = LoadSymbol<ClearFn>(dl_handle_, "motor_set_clear_error");
  zero_offset_ = LoadSymbol<ZeroOffsetFn>(dl_handle_, "motor_set_zero_offset");
  profile_acc_dec_ =
      LoadSymbol<ProfileAccDecFn>(dl_handle_, "motor_set_profile_acc_dec");
  profile_vel_ = LoadSymbol<ProfileVelFn>(dl_handle_, "motor_set_profile_vel");
  current_limit_ =
      LoadSymbol<CurrentLimitFn>(dl_handle_, "motor_set_current_limit");

  if (variant_ == ProtocolVariant::kArm) {
    mit_ = LoadSymbol<MitFn>(dl_handle_, "motor_set_mit_control");
    mit_crc_ = LoadSymbol<MitCrcFn>(dl_handle_, "motor_set_mit_control_crc");
    collision_ =
        LoadSymbol<CollisionFn>(dl_handle_, "motor_set_collision_threshold");
    get_param_ = LoadSymbol<GetParamFn>(dl_handle_, "motor_get_param");
    set_param_ = LoadSymbol<SetParamFn>(dl_handle_, "motor_set_param");
    version_ = LoadSymbol<VersionFn>(dl_handle_, "motor_get_version");
  } else {
    stiffness_ = LoadSymbol<StiffnessFn>(dl_handle_, "motor_set_stiffness");
  }
}

ProtocolLibrary::~ProtocolLibrary() {
  if (dl_handle_ != nullptr) {
    dlclose(dl_handle_);
  }
}

std::shared_ptr<ProtocolLibrary> ProtocolLibrary::Load(ProtocolVariant variant,
                                                       const std::string& path) {
  const std::string lib_path = path.empty() ? DefaultLibraryPath(variant) : path;
  dlerror();
  void* handle = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (handle == nullptr) {
    auto lib = std::shared_ptr<ProtocolLibrary>(
        new ProtocolLibrary(nullptr, variant));
    const char* dl_err = dlerror();
    lib->load_error_ = dl_err != nullptr ? dl_err : "dlopen failed";
    return lib;
  }

  auto lib = std::shared_ptr<ProtocolLibrary>(new ProtocolLibrary(handle, variant));
  if (lib->create_ == nullptr || lib->destroy_ == nullptr || lib->rx_ == nullptr) {
    lib->load_error_ = "protocol library missing required symbols";
    return lib;
  }
  return lib;
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

int ProtocolLibrary::GetLowSpeedFeedback(AgxMotorProtocolHandle* handle,
                                         uint8_t node_id,
                                         AgxMotorLowSpeedFeedback* out) const {
  return ls_ != nullptr ? ls_(handle, node_id, out) : 0;
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

int ProtocolLibrary::SetMitControl(AgxMotorProtocolHandle* handle,
                                  uint8_t node_id, float p_des, float v_des,
                                  float kp, float kd, float t_ff, float t_am) {
  return mit_ != nullptr
             ? mit_(handle, node_id, p_des, v_des, kp, kd, t_ff, t_am)
             : 0;
}

int ProtocolLibrary::SetMitControlCrc(AgxMotorProtocolHandle* handle,
                                     uint8_t node_id, float p_des, float v_des,
                                     float kp, float kd, float t_ff,
                                     float t_am) {
  return mit_crc_ != nullptr
             ? mit_crc_(handle, node_id, p_des, v_des, kp, kd, t_ff, t_am)
             : 0;
}

int ProtocolLibrary::SetCollisionThreshold(AgxMotorProtocolHandle* handle,
                                          uint8_t node_id,
                                          float current_threshold,
                                          float time_threshold) {
  return collision_ != nullptr
             ? collision_(handle, node_id, current_threshold, time_threshold)
             : 0;
}

int ProtocolLibrary::GetParam(AgxMotorProtocolHandle* handle, uint8_t node_id,
                              uint8_t idx1, uint8_t idx2, float timeout_s,
                              float* out) {
  return get_param_ != nullptr
             ? get_param_(handle, node_id, idx1, idx2, timeout_s, out)
             : 0;
}

int ProtocolLibrary::SetParam(AgxMotorProtocolHandle* handle, uint8_t node_id,
                              uint8_t idx1, uint8_t idx2, float value,
                              float timeout_s) {
  return set_param_ != nullptr
             ? set_param_(handle, node_id, idx1, idx2, value, timeout_s)
             : 0;
}

int ProtocolLibrary::GetVersion(AgxMotorProtocolHandle* handle, uint8_t node_id,
                                float timeout_s, AgxMotorVersionInfo* out) {
  return version_ != nullptr ? version_(handle, node_id, timeout_s, out) : 0;
}

int ProtocolLibrary::SetStiffness(AgxMotorProtocolHandle* handle,
                                  uint8_t node_id, uint16_t stiffness) {
  return stiffness_ != nullptr ? stiffness_(handle, node_id, stiffness) : 0;
}

}  // namespace motor
}  // namespace agx
