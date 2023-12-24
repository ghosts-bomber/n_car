#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "servo.h"
#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_PIN (23)
void servo_task(void* param);

void create_servo_task();

#ifdef __cplusplus
}
#endif




#endif // SERVO_TASK_H
