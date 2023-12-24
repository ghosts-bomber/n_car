#ifndef SERVO_H
#define SERVO_H
#include "driver/mcpwm_types.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t pin;
  int max_degree;
  int min_degree;
  uint32_t min_pulsewidth_us;
  uint32_t max_pulsewidth_us;
  uint32_t control_hz;
  mcpwm_cmpr_handle_t comparator;
  mcpwm_gen_handle_t generator;
}servo_dev_t;
int servo_init(servo_dev_t* dev);
int set_servo_degree(servo_dev_t* dev,int degree);

#ifdef __cplusplus
}
#endif

#endif //SERVO_H
