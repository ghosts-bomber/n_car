#include "adc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

const static char *TAG = "DAC";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC1_CHAN0 ADC_CHANNEL_4 // 32
#define ADC1_CHAN1 ADC_CHANNEL_5 // 33
#define ADC1_CHAN2 ADC_CHANNEL_6 // 34
#define ADC1_CHAN3 ADC_CHANNEL_7 // 35
#else
#define ADC1_CHAN0 ADC_CHANNEL_2
#define ADC1_CHAN1 ADC_CHANNEL_3
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) &&                                               \
    !CONFIG_IDF_TARGET_ESP32C3 /**                                             \
                                * On ESP32C3, ADC2 is no longer supported, due \
                                * to its HW limitation. Search for errata on   \
                                * espressif website for more details.          \
                                */
// #define USE_ADC2           0
#endif

#define ADC_ATTEN ADC_ATTEN_DB_12

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static adc_cali_handle_t adc1_cali_chan1_handle = NULL;
static adc_cali_handle_t adc1_cali_chan2_handle = NULL;
static adc_cali_handle_t adc1_cali_chan3_handle = NULL;

VRVoltage vr_voltage = {};
SemaphoreHandle_t vr_voltage_mutex = NULL;

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                 adc_atten_t atten,
                                 adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
static void deinit();

static int init_adc(void) {
  //-------------ADC1 Init---------------//
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  //-------------ADC1 Config---------------//
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN2, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN3, &config));

  //-------------ADC1 Calibration Init---------------//
  bool do_calibration1_chan0 = adc_calibration_init(
      ADC_UNIT_1, ADC1_CHAN0, ADC_ATTEN, &adc1_cali_chan0_handle);
  bool do_calibration1_chan1 = adc_calibration_init(
      ADC_UNIT_1, ADC1_CHAN1, ADC_ATTEN, &adc1_cali_chan1_handle);
  bool do_calibration1_chan2 = adc_calibration_init(
      ADC_UNIT_1, ADC1_CHAN2, ADC_ATTEN, &adc1_cali_chan2_handle);
  bool do_calibration1_chan3 = adc_calibration_init(
      ADC_UNIT_1, ADC1_CHAN3, ADC_ATTEN, &adc1_cali_chan3_handle);
  if (do_calibration1_chan1 && do_calibration1_chan0 && do_calibration1_chan2 &&
      do_calibration1_chan3) {
    deinit();
    return -1;
  }
  return 0;
}

/*---------------------------------------------------------------
        ADC Calibration
   ---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                 adc_atten_t atten,
                                 adc_cali_handle_t *out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
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

static void deinit() {
  // Tear Down
  if (adc1_handle) {
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
  }
  if (adc1_cali_chan0_handle) {
    adc_calibration_deinit(adc1_cali_chan0_handle);
  }
  if (adc1_cali_chan1_handle) {
    adc_calibration_deinit(adc1_cali_chan1_handle);
  }
  if (adc1_cali_chan2_handle) {
    adc_calibration_deinit(adc1_cali_chan2_handle);
  }
  if (adc1_cali_chan3_handle) {
    adc_calibration_deinit(adc1_cali_chan3_handle);
  }
}

void adc_task(void *param) {
  int adc_raw0 = 0;
  int adc_raw1 = 0;
  int adc_raw2 = 0;
  int adc_raw3 = 0;
  while (1) {
    xSemaphoreTake(vr_voltage_mutex, portMAX_DELAY);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw0));
    ESP_LOGD(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN0,
             adc_raw0);
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw0,
                                            &vr_voltage.voltage_chan0));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1,
             ADC1_CHAN0, vr_voltage.voltage_chan0);

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw1));
    ESP_LOGD(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1,
             adc_raw1);
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw1,
                                            &vr_voltage.voltage_chan1));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1,
             ADC1_CHAN1, vr_voltage.voltage_chan1);

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN2, &adc_raw2));
    ESP_LOGD(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN2,
             adc_raw2);
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan2_handle, adc_raw2,
                                            &vr_voltage.voltage_chan2));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1,
             ADC1_CHAN2, vr_voltage.voltage_chan2);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN3, &adc_raw3));
    ESP_LOGD(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN3,
             adc_raw3);
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan3_handle, adc_raw3,
                                            &vr_voltage.voltage_chan3));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1,
             ADC1_CHAN1, vr_voltage.voltage_chan3);

    xSemaphoreGive(vr_voltage_mutex);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
  deinit();
}

void create_adc_task(void) {
  if (init_adc() != 0)
    return;
  vr_voltage_mutex = xSemaphoreCreateMutex();
  TaskHandle_t adc_handle = NULL;
  xTaskCreate(adc_task, "adc_task", 1024, NULL, 1, &adc_handle);
}
