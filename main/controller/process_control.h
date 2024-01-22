#ifndef PROCESS_CONTROL_H
#define PROCESS_CONTROL_H

#include <stdint.h>
#include "XboxControllerNotificationParser.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#define JOY_MAX (65535)
typedef void(*Init_Fun)();
typedef void(*Process)(const XboxControllerNotificationParser&);
typedef struct{
  Init_Fun init;
  Process process;
}Controller;

// extern QueueHandle_t motor_control_queue;
void process_init();
void process_xbox_control(const XboxControllerNotificationParser& control);

#endif //PROCESS_CONTROL_H
