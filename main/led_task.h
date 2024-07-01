
#ifndef LED_TASK_H
#define LED_TASK_H
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
 uint8_t green;
 uint8_t red;
 uint8_t blue;
} Car_light;

extern Car_light *f_left_handle;
extern Car_light *f_right_handle;
extern Car_light *b_left_handle;
extern Car_light *b_right_handle;
extern Car_light *top_handle;
extern SemaphoreHandle_t led_control_mutex;

void create_led_task(void);

#ifdef __cplusplus
}
#endif




#endif // LED_TASK_H
