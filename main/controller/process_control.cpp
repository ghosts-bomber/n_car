#include "process_control.h"
#include "XboxControllerNotificationParser.h"
#include <cstdint>
#include <memory.h>
#include "motor_controller.h"
// QueueHandle_t motor_control_queue = NULL;
static Controller* motor_controller_handle = NULL;

void process_init() {
  motor_controller_handle = create_motor_controller();  
  motor_controller_handle->init();
}

void process_xbox_control(const XboxControllerNotificationParser &control) {
  motor_controller_handle->process(control);
  
}

