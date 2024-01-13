
#ifndef HID_HOST_H
#define HID_HOST_H
// #ifdef __cplusplus
// extern "C" {
// #endif
#include <stdint.h>
#include "freertos/FreeRTOS.h"
extern SemaphoreHandle_t xbox_mutex;
void create_hid_host_task(void);
// #ifdef __cplusplus
// }
// #endif

#endif //HID_HOST_H
