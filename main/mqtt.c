#include "mqtt.h"

#include "cJSON.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "mqtt_client.h"

#define MQTT_IP "8.141.156.132"
#define MQTT_PORT (1883)
#define MQTT_USER "ma"
#define MQTT_PWD "12345678"

#define TOPIC_SERVO ("/a/servo")
#define TOPIC_LED ("/a/led")
#define TOPIC_EMO ("/a/emo")

#define COMPIERE_TOPIC(t)                                                      \
  ((strncmp(topic, t, topic_len)) && (strlen(t) == topic_len))

static const char *TAG = "mqtt_";

esp_mqtt_client_handle_t client = NULL;
QueueHandle_t servo_queue = NULL;

static void log_error_if_nonzero(const char *message, int error_code) {
  if (error_code != 0) {
    ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
  }
}

static void process_msg(const char *topic, int topic_len, const char *data,
                        int data_len) {
  if (COMPIERE_TOPIC(TOPIC_SERVO)) {
    cJSON *pJsonRoot = cJSON_ParseWithLength(data, data_len);
    if (pJsonRoot != NULL) {
      int16_t angle = cJSON_GetObjectItem(pJsonRoot, "angle")->valueint;
      cJSON_Delete(pJsonRoot);
      xQueueSend(servo_queue, &angle, 0);
    } else {
      ESP_LOGE(TAG, "json parse fail, %.*s", data_len, data);
    }
  } else if (COMPIERE_TOPIC(TOPIC_LED)) {

  } else if (COMPIERE_TOPIC(TOPIC_EMO)) {
  }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG,
           "Event dispatched from event loop base=%s, event_id=%" PRIi32 "",
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    // msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1,
    // 0); ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

    msg_id = esp_mqtt_client_subscribe(client, TOPIC_SERVO, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    msg_id = esp_mqtt_client_subscribe(client, TOPIC_LED, 1);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    msg_id = esp_mqtt_client_subscribe(client, TOPIC_EMO, 1);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

    // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
    // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    process_msg(event->topic, event->topic_len, event->data, event->data_len);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      log_error_if_nonzero("reported from esp-tls",
                           event->error_handle->esp_tls_last_esp_err);
      log_error_if_nonzero("reported from tls stack",
                           event->error_handle->esp_tls_stack_err);
      log_error_if_nonzero("captured as transport's socket errno",
                           event->error_handle->esp_transport_sock_errno);
      ESP_LOGI(TAG, "Last errno string (%s)",
               strerror(event->error_handle->esp_transport_sock_errno));
    }
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

void mqtt_app_start(void) {
  servo_queue = xQueueCreate(10, sizeof(int16_t));
  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.hostname = MQTT_IP,
      .broker.address.port = MQTT_PORT,
      .credentials.username = MQTT_USER,
      .credentials.authentication.password = MQTT_PWD,
  };
#if CONFIG_BROKER_URL_FROM_STDIN
  char line[128];

  if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
    int count = 0;
    printf("Please enter url of mqtt broker\n");
    while (count < 128) {
      int c = fgetc(stdin);
      if (c == '\n') {
        line[count] = '\0';
        break;
      } else if (c > 0 && c < 127) {
        line[count] = c;
        ++count;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    mqtt_cfg.broker.address.uri = line;
    printf("Broker url: %s\n", line);
  } else {
    ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
    abort();
  }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

  client = esp_mqtt_client_init(&mqtt_cfg);
  /* The last argument may be used to pass data to the event handler, in this
   * example mqtt_event_handler */
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 NULL);
  esp_mqtt_client_start(client);
}
