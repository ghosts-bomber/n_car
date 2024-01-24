#include "hid_host.h"
#include "esp_bt.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#else
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#endif

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#define ESP_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_BD_ADDR_HEX(addr)                                                  \
  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#else
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#endif

#include "esp_hid_gap.h"
#include "esp_hidh.h"

#include "XboxControllerNotificationParser.h"
#include "process_control.h"

static const char *TAG = "ESP_HIDH_DEMO";
static TaskHandle_t ble_task_handle = NULL;
XboxControllerNotificationParser xInputParser;
static uint8_t xInputRawData[17];
SemaphoreHandle_t xbox_mutex;
void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id,
                   void *event_data) {
  esp_hidh_event_t event = (esp_hidh_event_t)id;
  esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

  switch (event) {
  case ESP_HIDH_OPEN_EVENT: {
    if (param->open.status == ESP_OK) {
      const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
      ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda),
               esp_hidh_dev_name_get(param->open.dev));
      esp_hidh_dev_dump(param->open.dev, stdout);
    } else {
      ESP_LOGE(TAG, " OPEN failed!");
    }
    break;
  }
  case ESP_HIDH_BATTERY_EVENT: {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda),
             param->battery.level);
    break;
  }
  case ESP_HIDH_INPUT_EVENT: {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
    // ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d,
    // Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage),
    // param->input.map_index, param->input.report_id, param->input.length);
    // ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
    if ((NULL != bda)) {
      if (16 == param->input.length &&
          xSemaphoreTake(xbox_mutex, portMAX_DELAY) == pdTRUE) {
        // 解析手柄蓝牙数据
        xInputRawData[0] = param->input.length;
        // TODO cancel memcpy
        memcpy(&xInputRawData[1], param->input.data, param->input.length);
        if (0 == xInputParser.update(&xInputRawData[1], xInputRawData[0])) {
          xInputParser.outOfDate = true;
          process_xbox_control(xInputParser);
          // ESP_LOGI(TAG, "xbox: %s\n", xInputParser.toString().c_str());
        } else {
          // esp_hidh_dev_output_set(param->input.dev, );
          ESP_LOGW(TAG, "invalid pack");
        }
        xSemaphoreGive(xbox_mutex);
      }
    }
    break;
  }
  case ESP_HIDH_FEATURE_EVENT: {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d",
             ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->feature.usage),
             param->feature.map_index, param->feature.report_id,
             param->feature.length);
    ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
    break;
  }
  case ESP_HIDH_CLOSE_EVENT: {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda),
             esp_hidh_dev_name_get(param->close.dev));
    vTaskResume(ble_task_handle);
    break;
  }
  default:
    ESP_LOGI(TAG, "EVENT: %d", event);
    break;
  }
}

#define SCAN_DURATION_SECONDS 5

void hid_demo_task(void *pvParameters) {
  while (1) {
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    // start scan for HID devices
    while (1) {
      esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
      ESP_LOGI(TAG, "SCAN: %u results", results_len);
      if (results_len != 0) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
    if (results_len) {
      esp_hid_scan_result_t *r = results;
      esp_hid_scan_result_t *cr = NULL;
      while (r) {
        printf("  %s: " ESP_BD_ADDR_STR ", ",
               (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ",
               ESP_BD_ADDR_HEX(r->bda));
        printf("RSSI: %d, ", r->rssi);
        printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
        if (r->transport == ESP_HID_TRANSPORT_BLE) {
          cr = r;
          printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
          printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
        }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
        if (r->transport == ESP_HID_TRANSPORT_BLE) {
          cr = r;
          printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
          printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
        }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
        if (r->transport == ESP_HID_TRANSPORT_BT) {
          cr = r;
          printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
          esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
          printf("] srv 0x%03x, ", r->bt.cod.service);
          print_uuid(&r->bt.uuid);
          printf(", ");
        }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
        printf("NAME: %s ", r->name ? r->name : "");
        printf("\n");
        r = r->next;
      }
      if (cr) {
        // open the last result
        esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
      }
      // free the results
      esp_hid_scan_results_free(results);
    }
    vTaskSuspend(ble_task_handle);
  }
  vTaskDelete(NULL);
}

#if CONFIG_BT_NIMBLE_ENABLED
void ble_hid_host_task(void *param) {
  ESP_LOGI(TAG, "BLE Host Task Started");
  /* This function will return only when nimble_port_stop() is executed */
  nimble_port_run();

  nimble_port_freertos_deinit();
}
void ble_store_config_init(void);
#endif
void create_hid_host_task(void) {
  esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
  ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
  return;
#endif
  /*  ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    */
  ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
  ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));
#if CONFIG_BT_BLE_ENABLED
  ESP_ERROR_CHECK(
      esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
#endif /* CONFIG_BT_BLE_ENABLED */
  esp_hidh_config_t config = {
      .callback = hidh_callback,
      .event_stack_size = 4096,
      .callback_arg = NULL,
  };
  ESP_ERROR_CHECK(esp_hidh_init(&config));

#if CONFIG_BT_NIMBLE_ENABLED
  /* XXX Need to have template for store */
  ble_store_config_init();

  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  /* Starting nimble task after gatts is initialized*/
  ret = esp_nimble_enable(ble_hid_host_task);
  if (ret) {
    ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
  }
#endif
  xbox_mutex = xSemaphoreCreateMutex();
  xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 1, &ble_task_handle);
}
