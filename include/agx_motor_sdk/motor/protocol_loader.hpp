#pragma once

#include <memory>
#include <string>

#include "agx_motor_sdk/motor/protocol_api.h"

namespace agx {
namespace motor {

/// dlopen wrapper for a motor protocol library: resolves the common C ABI
/// (protocol_api.h). Protocol-specific symbols are resolved by subclasses
/// (ArmProtocolLibrary, Rb200ProtocolLibrary).
class ProtocolLibrary {
 public:
  virtual ~ProtocolLibrary();

  ProtocolLibrary(const ProtocolLibrary&) = delete;
  ProtocolLibrary& operator=(const ProtocolLibrary&) = delete;

  AgxMotorProtocolHandle* Create();
  void Destroy(AgxMotorProtocolHandle* handle);

  void SetTxCallback(AgxMotorProtocolHandle* handle, AgxMotorTxCallback cb,
                     void* ctx);
  int HasTxCallback(AgxMotorProtocolHandle* handle) const;

  int HandleRxOnce(AgxMotorProtocolHandle* handle, uint32_t can_id,
                   const uint8_t* data, uint8_t dlc, uint64_t timestamp_ns);

  int GetHighSpeedFeedback(AgxMotorProtocolHandle* handle, uint8_t node_id,
                           AgxMotorHighSpeedFeedback* out) const;

  int SetTarget(AgxMotorProtocolHandle* handle, uint8_t node_id, float value,
                uint8_t mode, float timeout_s);
  int SetEnable(AgxMotorProtocolHandle* handle, uint8_t node_id, int enable,
                int has_brake, int brake_on);
  int SetReset(AgxMotorProtocolHandle* handle, uint8_t node_id, int mode,
               float timeout_s, int* out_ok);
  int SetClearError(AgxMotorProtocolHandle* handle, uint8_t node_id,
                    float timeout_s, int* out_ok);
  int SetZeroOffset(AgxMotorProtocolHandle* handle, uint8_t node_id,
                    float offset, int save_to_flash, float timeout_s,
                    int* out_ok);
  int SetProfileAccDec(AgxMotorProtocolHandle* handle, uint8_t node_id,
                       float accel, float decel);
  int SetProfileVel(AgxMotorProtocolHandle* handle, uint8_t node_id,
                    float velocity);
  int SetCurrentLimit(AgxMotorProtocolHandle* handle, uint8_t node_id,
                      float current);
  int GetVersion(AgxMotorProtocolHandle* handle, uint8_t node_id,
                 float timeout_s, AgxMotorVersionInfo* out);

  /// Empty on success; dlopen / symbol error otherwise.
  const std::string& load_error() const { return load_error_; }

 protected:
  /// Open the library. ``path`` empty: use ``env_var`` if set, otherwise search
  /// the SDK ``protocol/`` dir, system dirs and AMENT/COLCON prefixes for
  /// ``lib_filename``.
  ProtocolLibrary(const char* lib_filename, const char* env_var,
                  const std::string& path);

  bool is_open() const { return dl_handle_ != nullptr && load_error_.empty(); }
  void* Symbol(const char* name) const;
  template <typename T>
  T LoadSymbol(const char* name) const {
    return reinterpret_cast<T>(Symbol(name));
  }

 private:
  void* dl_handle_{nullptr};
  std::string load_error_;

  using CreateFn = AgxMotorProtocolHandle* (*)();
  using DestroyFn = void (*)(AgxMotorProtocolHandle*);
  using SetTxFn = void (*)(AgxMotorProtocolHandle*, AgxMotorTxCallback, void*);
  using HasTxFn = int (*)(AgxMotorProtocolHandle*);
  using RxFn = int (*)(AgxMotorProtocolHandle*, uint32_t, const uint8_t*,
                       uint8_t, uint64_t);
  using HsFn = int (*)(AgxMotorProtocolHandle*, uint8_t,
                       AgxMotorHighSpeedFeedback*);
  using TargetFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float, uint8_t,
                           float);
  using EnableFn = int (*)(AgxMotorProtocolHandle*, uint8_t, int, int, int);
  using ResetFn = int (*)(AgxMotorProtocolHandle*, uint8_t, int, float, int*);
  using ClearFn = int (*)(AgxMotorProtocolHandle*, uint8_t, uint8_t, float,
                          int*);
  using ZeroOffsetFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float, int,
                               float, int*);
  using ProfileAccDecFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float,
                                  float);
  using ProfileVelFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float);
  using CurrentLimitFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float);
  using VersionFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float,
                            AgxMotorVersionInfo*);

  CreateFn create_{nullptr};
  DestroyFn destroy_{nullptr};
  SetTxFn set_tx_{nullptr};
  HasTxFn has_tx_{nullptr};
  RxFn rx_{nullptr};
  HsFn hs_{nullptr};
  TargetFn target_{nullptr};
  EnableFn enable_{nullptr};
  ResetFn reset_{nullptr};
  ClearFn clear_{nullptr};
  ZeroOffsetFn zero_offset_{nullptr};
  ProfileAccDecFn profile_acc_dec_{nullptr};
  ProfileVelFn profile_vel_{nullptr};
  CurrentLimitFn current_limit_{nullptr};
  VersionFn version_{nullptr};
};

}  // namespace motor
}  // namespace agx
