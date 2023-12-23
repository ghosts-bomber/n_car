/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include "servo.h"

#define SERVO_PIN (10)
void app_main(void) {
  servo_dev_t servo_dev = {
    .pin = SERVO_PIN,
    .max_degree = 90,
    .min_degree = -90,
    .min_pulsewidth_us = 500,
    .max_pulsewidth_us = 2500,
  };
  servo_init(&servo_dev);
  uint16_t angle = -90;
  while(1){
    if(angle>90) angle = -90;
    set_servo_degree(&servo_dev,angle);
    angle+=5;
    vTaskDelay(500);
  }
}
