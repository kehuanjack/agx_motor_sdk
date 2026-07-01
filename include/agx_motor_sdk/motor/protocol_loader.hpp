#pragma once

#include <memory>
#include <string>

#include "agx_motor_sdk/motor/protocol_api.h"

namespace agx {
namespace motor {

enum class ProtocolVariant { kArm, kChassis };

class ProtocolLibrary {
 public:
  static std::shared_ptr<ProtocolLibrary> Load(ProtocolVariant variant,
                                               const std::string& path = {});

  ~ProtocolLibrary();

  AgxMotorProtocolHandle* Create();
  void Destroy(AgxMotorProtocolHandle* handle);

  void SetTxCallback(AgxMotorProtocolHandle* handle, AgxMotorTxCallback cb,
                     void* ctx);
  int HasTxCallback(AgxMotorProtocolHandle* handle) const;

  int HandleRxOnce(AgxMotorProtocolHandle* handle, uint32_t can_id,
                   const uint8_t* data, uint8_t dlc, uint64_t timestamp_ns);

  int GetHighSpeedFeedback(AgxMotorProtocolHandle* handle, uint8_t node_id,
                           AgxMotorHighSpeedFeedback* out) const;
  int GetLowSpeedFeedback(AgxMotorProtocolHandle* handle, uint8_t node_id,
                          AgxMotorLowSpeedFeedback* out) const;

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

  int SetMitControl(AgxMotorProtocolHandle* handle, uint8_t node_id, float p_des,
                    float v_des, float kp, float kd, float t_ff, float t_am);
  int SetMitControlCrc(AgxMotorProtocolHandle* handle, uint8_t node_id,
                       float p_des, float v_des, float kp, float kd, float t_ff,
                       float t_am);
  int SetCollisionThreshold(AgxMotorProtocolHandle* handle, uint8_t node_id,
                            float current_threshold, float time_threshold);
  int GetParam(AgxMotorProtocolHandle* handle, uint8_t node_id, uint8_t idx1,
               uint8_t idx2, float timeout_s, float* out);
  int SetParam(AgxMotorProtocolHandle* handle, uint8_t node_id, uint8_t idx1,
               uint8_t idx2, float value, float timeout_s);
  int GetVersion(AgxMotorProtocolHandle* handle, uint8_t node_id,
                 float timeout_s, AgxMotorVersionInfo* out);

  int SetStiffness(AgxMotorProtocolHandle* handle, uint8_t node_id,
                   uint16_t stiffness);

  const std::string& load_error() const { return load_error_; }

 private:
  explicit ProtocolLibrary(void* dl_handle, ProtocolVariant variant);

  void* dl_handle_{nullptr};
  ProtocolVariant variant_;
  std::string load_error_;

  using CreateFn = AgxMotorProtocolHandle* (*)();
  using DestroyFn = void (*)(AgxMotorProtocolHandle*);
  using SetTxFn = void (*)(AgxMotorProtocolHandle*, AgxMotorTxCallback, void*);
  using HasTxFn = int (*)(AgxMotorProtocolHandle*);
  using RxFn = int (*)(AgxMotorProtocolHandle*, uint32_t, const uint8_t*,
                       uint8_t, uint64_t);
  using HsFn = int (*)(AgxMotorProtocolHandle*, uint8_t,
                       AgxMotorHighSpeedFeedback*);
  using LsFn = int (*)(AgxMotorProtocolHandle*, uint8_t,
                       AgxMotorLowSpeedFeedback*);
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
  using MitFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float, float, float,
                        float, float, float);
  using MitCrcFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float, float,
                           float, float, float, float);
  using CollisionFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float, float);
  using GetParamFn = int (*)(AgxMotorProtocolHandle*, uint8_t, uint8_t,
                             uint8_t, float, float*);
  using SetParamFn = int (*)(AgxMotorProtocolHandle*, uint8_t, uint8_t,
                             uint8_t, float, float);
  using VersionFn = int (*)(AgxMotorProtocolHandle*, uint8_t, float,
                            AgxMotorVersionInfo*);
  using StiffnessFn = int (*)(AgxMotorProtocolHandle*, uint8_t, uint16_t);

  CreateFn create_{nullptr};
  DestroyFn destroy_{nullptr};
  SetTxFn set_tx_{nullptr};
  HasTxFn has_tx_{nullptr};
  RxFn rx_{nullptr};
  HsFn hs_{nullptr};
  LsFn ls_{nullptr};
  TargetFn target_{nullptr};
  EnableFn enable_{nullptr};
  ResetFn reset_{nullptr};
  ClearFn clear_{nullptr};
  ZeroOffsetFn zero_offset_{nullptr};
  ProfileAccDecFn profile_acc_dec_{nullptr};
  ProfileVelFn profile_vel_{nullptr};
  CurrentLimitFn current_limit_{nullptr};
  MitFn mit_{nullptr};
  MitCrcFn mit_crc_{nullptr};
  CollisionFn collision_{nullptr};
  GetParamFn get_param_{nullptr};
  SetParamFn set_param_{nullptr};
  VersionFn version_{nullptr};
  StiffnessFn stiffness_{nullptr};
};

}  // namespace motor
}  // namespace agx
