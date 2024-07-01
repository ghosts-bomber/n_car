#include "motor_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/timers.h"
#include "process_control.h"
#include "motor_task.h"

static TimerHandle_t out_controller_timer = NULL;
static void process_out_controller(TimerHandle_t timer_handle){
  // if (xSemaphoreTake(motor_control_mutex, portMAX_DELAY) == pdTRUE) {
    motor_contorl.motor0 = 0;
    motor_contorl.motor1 = 0;
    motor_contorl.motor2 = 0;
    motor_contorl.motor3 = 0;
  //   xSemaphoreGive(motor_control_mutex);
  // }
}
static void init()
{
  out_controller_timer = xTimerCreate("out_controller", pdMS_TO_TICKS(500), pdFALSE, (void *)21, process_out_controller);
  xTimerStart(out_controller_timer, 0);
}

static void process_xbox(const XboxControllerNotificationParser& control)
{
  if (xSemaphoreTake(motor_control_mutex, portMAX_DELAY) == pdTRUE) {
    int16_t v_val = control.JOY_MID- control.joyLVert;
    if (v_val >= -ERR_CTL_VAL && v_val <= ERR_CTL_VAL) {
      v_val = 0;
    }
    int16_t h_val = control.JOY_MID - control.joyLHori;
    if (h_val >= -ERR_CTL_VAL && h_val <= ERR_CTL_VAL) {
      h_val = 0;
    }
    h_val = v_val>0?h_val:-h_val;
    motor_contorl.motor0 = v_val-h_val;
    motor_contorl.motor1 = v_val+h_val;
    motor_contorl.motor2 = v_val-h_val;
    motor_contorl.motor3 = v_val+h_val;
    xSemaphoreGive(motor_control_mutex);
    xTimerReset(out_controller_timer, 0);
  }
}

Controller* create_motor_controller()
{
  static Controller controller;
  controller.init = init;
  controller.process = process_xbox;
  return &controller;
}

