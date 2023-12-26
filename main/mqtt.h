#ifndef MQTT_H
#define MQTT_H
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#ifdef __cplusplus
extern "C" {
#endif
 
extern QueueHandle_t servo_queue;

void mqtt_app_start(void);

#ifdef __cplusplus
}
#endif

#endif //MQTT_H
