#pragma once

/* RB200 protocol C ABI extensions (libagx_motor_protocol_rb200.so). */

#include "agx_motor_sdk/motor/protocol_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RB200 low-speed feedback (0x530 + node): CiA402 status word, error code,
 * TPDO2 counter. Returned by motor_get_low_speed_feedback in the RB200 library. */
typedef struct {
  uint16_t status_word;
  uint16_t error_code;
  uint16_t tpdo2_count;
  uint64_t timestamp_ns;
} AgxMotorRb200LowSpeedFeedback;

/* Signature of the library's motor_get_low_speed_feedback symbol. */
typedef int (*AgxMotorRb200GetLowSpeedFeedbackFn)(AgxMotorProtocolHandle* handle,
                                                 uint8_t node_id, AgxMotorRb200LowSpeedFeedback* out);

#ifdef __cplusplus
}
#endif
