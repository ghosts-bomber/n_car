#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int16_t motor0;
  int16_t motor1;
  int16_t motor2;
  int16_t motor3;
} Motor_control;

extern Motor_control motor_contorl;
extern SemaphoreHandle_t motor_control_mutex;

void create_motor_task(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_TASK_H
