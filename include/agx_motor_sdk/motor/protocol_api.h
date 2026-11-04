#pragma once

/* Common C ABI shared by every motor protocol library (documentation of the
 * symbols resolved by ProtocolLibrary). Protocol-specific structs and symbols
 * live in arm/arm_protocol_api.h and rb200/rb200_protocol_api.h. */

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AgxMotorProtocolHandle AgxMotorProtocolHandle;

typedef int (*AgxMotorTxCallback)(void* ctx, uint32_t can_id, uint8_t* data,
                                  uint8_t dlc);

AgxMotorProtocolHandle* agx_motor_protocol_create();
void agx_motor_protocol_destroy(AgxMotorProtocolHandle* handle);

void agx_motor_protocol_set_tx_callback(AgxMotorProtocolHandle* handle,
                                        AgxMotorTxCallback cb, void* ctx);
int agx_motor_protocol_has_tx_callback(AgxMotorProtocolHandle* handle);

int agx_motor_protocol_handle_rx_once(AgxMotorProtocolHandle* handle,
                                    uint32_t can_id, const uint8_t* data,
                                    uint8_t dlc, uint64_t timestamp_ns);

typedef struct {
  float position;
  float velocity;
  float current;
  uint64_t timestamp_ns;
} AgxMotorHighSpeedFeedback;

typedef struct {
  char software[64];
  char hardware[64];
  char motor[64];
  uint64_t timestamp_ns;
} AgxMotorVersionInfo;

int agx_motor_protocol_get_high_speed_feedback(
    AgxMotorProtocolHandle* handle, uint8_t node_id,
    AgxMotorHighSpeedFeedback* out);

int agx_motor_protocol_set_target(AgxMotorProtocolHandle* handle,
                                  uint8_t node_id, float value, uint8_t mode,
                                  float timeout_s);
int agx_motor_protocol_set_enable(AgxMotorProtocolHandle* handle,
                                  uint8_t node_id, int enable, int has_brake,
                                  int brake_on);
int agx_motor_protocol_set_reset(AgxMotorProtocolHandle* handle,
                                 uint8_t node_id, int mode, float timeout_s,
                                 int* out_ok);
int agx_motor_protocol_set_clear_error(AgxMotorProtocolHandle* handle,
                                       uint8_t node_id, uint8_t reserved,
                                       float timeout_s, int* out_ok);
int agx_motor_protocol_set_zero_offset(AgxMotorProtocolHandle* handle,
                                       uint8_t node_id, float offset,
                                       int save_to_flash, float timeout_s,
                                       int* out_ok);
int agx_motor_protocol_set_profile_acc_dec(AgxMotorProtocolHandle* handle,
                                           uint8_t node_id, float accel,
                                           float decel);
int agx_motor_protocol_set_profile_vel(AgxMotorProtocolHandle* handle,
                                       uint8_t node_id, float velocity);
int agx_motor_protocol_set_current_limit(AgxMotorProtocolHandle* handle,
                                         uint8_t node_id, float current);

int agx_motor_protocol_get_version(AgxMotorProtocolHandle* handle,
                                   uint8_t node_id, float timeout_s,
                                   AgxMotorVersionInfo* out);

#ifdef __cplusplus
}
#endif
