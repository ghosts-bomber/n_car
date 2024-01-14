#ifndef PROCESS_CONTROL_H
#define PROCESS_CONTROL_H

#include <stdint.h>
#include "XboxControllerNotificationParser.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
class XboxControllerNotificationParser;
typedef struct{
  int16_t motor0;
  int16_t motor1;
  int16_t motor2;
  int16_t motor3;
}Motor_control;
extern Motor_control motor_contorl;
extern SemaphoreHandle_t motor_control_mutex;
// extern QueueHandle_t motor_control_queue;
void process_init();
void process_xbox_control(const XboxControllerNotificationParser& control);

#endif //PROCESS_CONTROL_H
