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
    int16_t v_val = JOY_MAX - control.joyLVert - JOY_MAX / 2;
    if (v_val >= -1000 && v_val <= 1000) {
      v_val = 0;
    }
    int16_t h_val = JOY_MAX - control.joyLHori - JOY_MAX / 2;
    if (h_val >= -1000 && h_val <= 1000) {
      h_val = 0;
    }
    h_val = v_val>0?h_val:-h_val;
    motor_contorl.motor0 = v_val-h_val;
    motor_contorl.motor1 = v_val+h_val;
    motor_contorl.motor2 = v_val-h_val;
    motor_contorl.motor3 = v_val+h_val;
    xSemaphoreGive(motor_control_mutex);
  }
}

