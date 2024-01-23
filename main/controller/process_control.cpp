#include "process_control.h"
#include "XboxControllerNotificationParser.h"
#include "car_light_controller.h"
#include "motor_controller.h"
#include <cstdint>
#include <memory.h>
// QueueHandle_t motor_control_queue = NULL;
static Controller *motor_controller_handle = NULL;
static Controller *car_light_controller_handle = NULL;

void process_init() {
  motor_controller_handle = create_motor_controller();
  car_light_controller_handle = create_car_light_controller();
  motor_controller_handle->init();
  car_light_controller_handle->init();
}

void process_xbox_control(const XboxControllerNotificationParser &control) {
  motor_controller_handle->process(control);
  car_light_controller_handle->process(control);
}

