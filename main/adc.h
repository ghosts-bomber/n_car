#ifndef ADC_H
#define ADC_H
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#ifdef __cplusplus
extern "C" {
#endif
 

void create_dac_task(void);

#ifdef __cplusplus
}
#endif

#endif //ADC_H
