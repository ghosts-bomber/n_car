#include "motor_controller.h"
#include "process_control.h"
#include "motor_task.h"
static void init()
{

}

static void process_xbox(const XboxControllerNotificationParser& control)
{
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

Controller* create_motor_controller()
{
  static Controller controller;
  controller.init = init;
  controller.process = process_xbox;
  return &controller;
}
