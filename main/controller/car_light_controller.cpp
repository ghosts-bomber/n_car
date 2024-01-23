#include "car_light_controller.h"
#include "freertos/projdefs.h"
#include "led_task.h"
#include "lwip/err.h"
#include "portmacro.h"
#include <stdint.h>
#define LIGHT_MAX (255)
#define LIGHT_MIN (0)
static TimerHandle_t blink_timer = NULL;

static void set_light_color(Car_light *light, uint8_t r, uint8_t g, uint8_t b) {
  light->red = r;
  light->green = g;
  light->blue = b;
}

static void blink_light(TimerHandle_t timer_handle) {}

static void init_car_light() {
  set_light_color(f_left_handle, 0, 0, 0);
  set_light_color(f_right_handle, 0, 0, 0);
  set_light_color(b_left_handle, 0, 0, 0);
  set_light_color(b_right_handle, 0, 0, 0);
  set_light_color(top_handle, 0, 0, 255);
}

static void init() {
  init_car_light();
  blink_timer = xTimerCreate("blink", pdMS_TO_TICKS(500), pdTRUE, (void *)10,
                             blink_light);
}

static void process_xbox(const XboxControllerNotificationParser &parse) {
  if (xSemaphoreTake(led_control_mutex, portMAX_DELAY) == pdTRUE) {
    init_car_light();
    int32_t sub = parse.JOY_MID - parse.joyLVert;
    if (sub > ERR_CTL_VAL) {
      uint32_t light = sub * LIGHT_MAX / parse.JOY_MID;
      set_light_color(f_left_handle, light, light, light);
      set_light_color(f_right_handle, light, light, light);
    } else if (sub < -ERR_CTL_VAL) {
      uint32_t light = -sub * LIGHT_MAX / parse.JOY_MID;
      set_light_color(b_left_handle, light, 0, 0);
      set_light_color(b_right_handle, light, 0, 0);
    }
    xSemaphoreGive(led_control_mutex);
  }
}

Controller *create_car_light_controller() {
  static Controller controller;
  controller.init = init;
  controller.process = process_xbox;
  return &controller;
}
