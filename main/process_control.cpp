#include "process_control.h"
#include "XboxControllerNotificationParser.h"
#include <memory.h>
// QueueHandle_t motor_control_queue = NULL;
SemaphoreHandle_t motor_control_mutex;
Motor_control motor_contorl;
#define JOY_MAX (65535)
void process_init() {
  motor_control_mutex = xSemaphoreCreateMutex();
  memset((void *)&motor_contorl, 0, sizeof(Motor_control));
}

void process_xbox_control(const XboxControllerNotificationParser &control) {
  if (xSemaphoreTake(motor_control_mutex, portMAX_DELAY) == pdTRUE) {
    int16_t val = JOY_MAX - control.joyLVert - JOY_MAX / 2;
    if (val >= -1000 && val <= 1000) {
      val = 0;
    }
    motor_contorl.motor0 = val;
    motor_contorl.motor1 = val;
    motor_contorl.motor2 = val;
    motor_contorl.motor3 = val;
    xSemaphoreGive(motor_control_mutex);
  }
}
