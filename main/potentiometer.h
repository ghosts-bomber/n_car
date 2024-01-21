#ifndef POTENTIOMENTER_H
#define POTENTIOMENTER_H
#include "esp_err.h"
#ifdef __cplusplus
extern "C"{
#endif
typedef struct{
 uint32_t min_r;
 uint32_t max_r;
} potentiomenter_config_t;

typedef struct potentiomenter_t potentiomenter_t;

struct potentiomenter_t{
  int (*get_value)(potentiomenter_t* pm); 
  int (*get_voltage)(potentiomenter_t* pm);
  potentiomenter_config_t potentiomenter_config;
};

typedef struct potentiomenter_t* potentiomenter_handle_t;

typedef struct{
  uint8_t group_id;
  uint8_t ch_id;
  uint8_t bitwidth;
  uint8_t atten;
}potentiomenter_adc_config_t;


esp_err_t potentiomenter_new_adc_device(const potentiomenter_config_t* ptentiomenter_config,const potentiomenter_adc_config_t* adc_config,potentiomenter_handle_t* ret_handle);


#ifdef __cplusplus
}
#endif



#endif // POTENTIOMENTER_H

