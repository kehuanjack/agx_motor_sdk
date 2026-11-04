#pragma once

/* Arm protocol C ABI extensions (libagx_motor_protocol_arm.so). */

#include "agx_motor_sdk/motor/protocol_api.h"

#ifdef __cplusplus
extern "C" {
#endif

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

/* Signature of the library's motor_get_low_speed_feedback symbol. */
typedef int (*AgxMotorArmGetLowSpeedFeedbackFn)(AgxMotorProtocolHandle* handle,
                                               uint8_t node_id, AgxMotorLowSpeedFeedback* out);

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

#ifdef __cplusplus
}
#endif
