#pragma once

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
  uint8_t undervoltage;
  uint8_t motor_overtemp;
  uint8_t driver_overcurrent;
  uint8_t driver_overtemp;
  uint8_t collision_tripped;
  uint8_t driver_error;
  uint8_t enabled;
  uint8_t stall_tripped;
} AgxMotorDriverStatusBits;

typedef struct {
  float bus_voltage_v;
  int16_t driver_temp_deg;
  int8_t motor_temp_deg;
  uint8_t pad0;
  float bus_current;
  uint8_t status_raw;
  AgxMotorDriverStatusBits status;
  uint8_t pad_tail[3];
  uint64_t timestamp_ns;
} AgxMotorLowSpeedFeedback;

typedef struct {
  char software[64];
  char hardware[64];
  char motor[64];
  uint64_t timestamp_ns;
} AgxMotorVersionInfo;

int agx_motor_protocol_get_high_speed_feedback(
    AgxMotorProtocolHandle* handle, uint8_t node_id,
    AgxMotorHighSpeedFeedback* out);
int agx_motor_protocol_get_low_speed_feedback(
    AgxMotorProtocolHandle* handle, uint8_t node_id,
    AgxMotorLowSpeedFeedback* out);

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

/* arm protocol extensions */
int agx_motor_protocol_set_mit_control(AgxMotorProtocolHandle* handle,
                                       uint8_t node_id, float p_des,
                                       float v_des, float kp, float kd,
                                       float t_ff, float t_am);
int agx_motor_protocol_set_mit_control_crc(AgxMotorProtocolHandle* handle,
                                           uint8_t node_id, float p_des,
                                           float v_des, float kp, float kd,
                                           float t_ff, float t_am);
int agx_motor_protocol_set_collision_threshold(AgxMotorProtocolHandle* handle,
                                               uint8_t node_id,
                                               float current_threshold,
                                               float time_threshold);
int agx_motor_protocol_get_param(AgxMotorProtocolHandle* handle,
                                 uint8_t node_id, uint8_t idx1, uint8_t idx2,
                                 float timeout_s, float* out);
int agx_motor_protocol_set_param(AgxMotorProtocolHandle* handle,
                                 uint8_t node_id, uint8_t idx1, uint8_t idx2,
                                 float value, float timeout_s);
int agx_motor_protocol_get_version(AgxMotorProtocolHandle* handle,
                                   uint8_t node_id, float timeout_s,
                                   AgxMotorVersionInfo* out);

/* chassis protocol extensions */
int agx_motor_protocol_set_stiffness(AgxMotorProtocolHandle* handle,
                                     uint8_t node_id, uint16_t stiffness);

#ifdef __cplusplus
}
#endif
