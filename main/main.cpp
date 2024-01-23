/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt.h"
#include "nvs_flash.h"
#include "oled_task.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "wifi.h"
#include "led_task.h"
#include "hid_host.h"
#include "motor_task.h"
#include "process_control.h"
static const char *TAG = "main";
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
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  init_nvs();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  // create_servo_task();
  create_oled_task();
  create_led_task();
  create_motor_task();
  wifi_init_sta();
  process_init();
  mqtt_app_start();
  create_hid_host_task();
  
  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
