#ifndef ADC_H
#define ADC_H
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
int voltage_chan0;
int voltage_chan1;
int voltage_chan2;
int voltage_chan3;
} VRVoltage;
extern VRVoltage vr_voltage;
extern SemaphoreHandle_t vr_voltage_mutex;
void create_adc_task(void);

#ifdef __cplusplus
}
#endif

#endif //ADC_H
