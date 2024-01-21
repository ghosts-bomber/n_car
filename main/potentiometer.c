#include "potentiometer.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "portmacro.h"

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle = NULL;
static uint8_t adc1_use_count = 0;
static uint8_t adc2_use_count = 0;
static const char *TAG = "potentiometer";
typedef struct {
  potentiomenter_t base;
  adc_cali_handle_t adc_cali_handle;
  adc_oneshot_unit_handle_t adc_handle;
  adc_channel_t ch;
} potentiomenter_adc_obj;

static esp_err_t init_unit(adc_unit_t unit_id,
                           adc_oneshot_unit_handle_t *handle) {
  if (unit_id == ADC_UNIT_1) {
    if (adc1_handle == NULL) {
      adc_oneshot_unit_init_cfg_t init_config = {
          .unit_id = ADC_UNIT_1,
      };
      ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &adc1_handle), TAG,
                          "adc unit 1 init failed");
    }
    *handle = adc1_handle;
    ++adc1_use_count;
    return ESP_OK;
  } else if (unit_id == ADC_UNIT_2) {
    if (adc2_handle == NULL) {
      adc_oneshot_unit_init_cfg_t init_config = {
          .unit_id = ADC_UNIT_2,
      };
      ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &adc2_handle), TAG,
                          "adc unit 2 init failed");
    }
    *handle = adc2_handle;
    ++adc2_use_count;
    return ESP_OK;
  }
  return ESP_ERR_INVALID_ARG;
}

static void deinit_unit(adc_unit_t unit_id) {
  if (unit_id == ADC_UNIT_1 && adc1_handle) {
    if (adc1_use_count == 1) {

      ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    }
    --adc1_use_count;
  } else if (unit_id == ADC_UNIT_2 && adc2_handle) {
    if (adc2_use_count == 1) {

      ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
    }
    --adc2_use_count;
  }
}

/*---------------------------------------------------------------
        ADC Calibration
   ---------------------------------------------------------------*/
static bool adc_calibration_init(const potentiomenter_adc_config_t *adc_config,
                                 adc_cali_handle_t *out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = adc_config->group_id,
        .atten = adc_config->atten,
        .bitwidth = adc_config->bitwidth,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

  *out_handle = handle;
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Calibration Success");
  } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else {
    ESP_LOGE(TAG, "Invalid arg or no memory");
  }

  return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static int get_value(const potentiomenter_handle_t handle) {
  potentiomenter_adc_obj *adc_obj =
      __containerof(handle, potentiomenter_adc_obj, base);
  static int adc_raw;
  adc_raw = 0;
  ESP_ERROR_CHECK(adc_oneshot_read(adc_obj->adc_handle, adc_obj->ch, &adc_raw));
  return adc_raw;
}

static int get_voltage(const potentiomenter_handle_t handle) {
  potentiomenter_adc_obj *adc_obj =
      __containerof(handle, potentiomenter_adc_obj, base);
  static int voltage, raw;
  voltage = 0;
  raw = handle->get_value(handle);
  ESP_ERROR_CHECK(
      adc_cali_raw_to_voltage(adc_obj->adc_cali_handle, raw, &voltage));
  return voltage;
}

esp_err_t potentiomenter_new_adc_device(
    const potentiomenter_config_t *ptentiomenter_config,
    const potentiomenter_adc_config_t *adc_config,
    potentiomenter_handle_t *ret_handle) {
  potentiomenter_adc_obj *potentiometer_adc = NULL;
  esp_err_t ret = ESP_OK;
  ESP_GOTO_ON_FALSE(ptentiomenter_config && ptentiomenter_config && ret_handle,
                    ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
  adc_oneshot_unit_handle_t adc_handle = NULL;
  ESP_GOTO_ON_ERROR(init_unit(adc_config->group_id, &adc_handle), err, TAG, "");
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = adc_config->bitwidth,
      .atten = adc_config->atten,
  };
  ESP_GOTO_ON_ERROR(
      adc_oneshot_config_channel(adc_handle, adc_config->ch_id, &config),
      err_cal, TAG, "channel init failed");
  adc_cali_handle_t adc_cali_handle = NULL;
  ESP_GOTO_ON_FALSE(adc_calibration_init(adc_config, &adc_cali_handle),
                    ESP_ERR_INVALID_STATE, err, TAG, "");
  potentiometer_adc = calloc(1, sizeof(potentiomenter_adc_obj));
  ESP_GOTO_ON_FALSE(potentiometer_adc, ESP_ERR_NO_MEM, err_all, TAG,
                    "no mem for potentiomenter adc");
  potentiometer_adc->adc_cali_handle = adc_cali_handle;
  potentiometer_adc->adc_handle = adc_handle;
  potentiometer_adc->ch = adc_config->ch_id;
  potentiometer_adc->base.potentiomenter_config = *ptentiomenter_config;
  potentiometer_adc->base.get_value = get_value;
  potentiometer_adc->base.get_voltage = get_voltage;
  *ret_handle = &potentiometer_adc->base;
  goto err;
err_all:
  adc_calibration_deinit(adc_cali_handle);
err_cal:
  deinit_unit(adc_config->group_id);
err:
  return ret;
}
