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
#include "nvs_flash.h"
#include "oled_task.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "servo_task.h"
#include "wifi.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
static void init_nvs() {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

extern "C" void app_main(void) {
  init_nvs();
  wifi_init_sta();
  create_servo_task();
  create_oled_task();

  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
