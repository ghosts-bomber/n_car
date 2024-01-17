#include "motor_task.h"
#include "bdc_motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "hal/adc_types.h"
#include "potentiometer.h"
#include "process_control.h"
#define BDC_MCPWM_FREQ_HZ 25000                // 25KHz PWM
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_DUTY_TICK_MAX                                                \
  (BDC_MCPWM_TIMER_RESOLUTION_HZ /                                             \
   BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define CONTROL_INTERVAL (20)

#define MOTOR0_A (18)
#define MOTOR0_B (19)
#define MOTOR1_A (25)
#define MOTOR1_B (26)
#define MOTOR2_A (12)
#define MOTOR2_B (13)
#define MOTOR3_A (2)
#define MOTOR3_B (4)

#define ADC1_CHAN0 ADC_CHANNEL_4 // 32
#define ADC1_CHAN1 ADC_CHANNEL_5 // 33
#define ADC1_CHAN2 ADC_CHANNEL_6 // 34
#define ADC1_CHAN3 ADC_CHANNEL_7 // 35
//
static const char *TAG = "motor_task";
extern SemaphoreHandle_t motor_control_mutex;
extern Motor_control motor_contorl;
typedef struct {
  bdc_motor_handle_t motor_handle;
  potentiomenter_handle_t pm_handle;
} Motor;

static Motor *motor0;
static Motor *motor1;
static Motor *motor2;
static Motor *motor3;

static Motor *init_motor(const bdc_motor_config_t *motor_config,
                         const bdc_motor_mcpwm_config_t *mcpwm_config,
                         const potentiomenter_config_t *pm_config,
                         const potentiomenter_adc_config_t *adc_config) {
  Motor *motor = NULL;
  motor = (Motor *)calloc(1, sizeof(Motor));
  if (motor == NULL) {
    ESP_LOGE(TAG, "no mem");
    return NULL;
  }
  bdc_motor_handle_t bdc_motor_handle = NULL;
  esp_err_t err =
      bdc_motor_new_mcpwm_device(motor_config, mcpwm_config, &bdc_motor_handle);
  bdc_motor_enable(bdc_motor_handle);
  motor->motor_handle = bdc_motor_handle;
  potentiomenter_handle_t pm_handle;
  err = potentiomenter_new_adc_device(pm_config, adc_config, &pm_handle);
  motor->pm_handle = pm_handle;
  return motor;
}

static void process_motor_control(bdc_motor_handle_t motor, int16_t val) {
  if (val == 0) {
    bdc_motor_coast(motor);
  } else {
    static uint32_t cal_val;
    cal_val =
        (uint32_t)(val > 0 ? val : -val) * BDC_MCPWM_DUTY_TICK_MAX / 65535 * 2;
    if (val > 0) {
      bdc_motor_forward(motor);
      bdc_motor_set_speed(motor, cal_val);
    } else if (val < 0) {
      bdc_motor_reverse(motor);
      bdc_motor_set_speed(motor, cal_val);
    }
  }
}

static void motor_control(void *arg) {
  while (1) {
    if (xSemaphoreTake(motor_control_mutex, portMAX_DELAY) == pdTRUE) {
      process_motor_control(motor0->motor_handle, motor_contorl.motor0);
      process_motor_control(motor1->motor_handle, motor_contorl.motor1);
      process_motor_control(motor2->motor_handle, motor_contorl.motor2);
      process_motor_control(motor3->motor_handle, motor_contorl.motor3);
      xSemaphoreGive(motor_control_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL));
  }
}
void create_motor_task(void) {
  bdc_motor_mcpwm_config_t mcpwm_config = {};
  bdc_motor_config_t motor_config = {};
  potentiomenter_config_t pm_config = {};
  potentiomenter_adc_config_t adc_config = {};
  mcpwm_config.group_id = 0;
  mcpwm_config.resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ;

  pm_config.max_r = 50000;
  pm_config.min_r = 100;

  adc_config.group_id = ADC_UNIT_1;
  adc_config.bitwidth = ADC_BITWIDTH_DEFAULT;
  adc_config.atten = ADC_ATTEN_DB_12;
  adc_config.ch_id = ADC1_CHAN0;

  motor_config.pwm_freq_hz = BDC_MCPWM_FREQ_HZ;
  motor_config.pwma_gpio_num = MOTOR0_A;
  motor_config.pwmb_gpio_num = MOTOR0_B;
  motor0 = init_motor(&motor_config, &mcpwm_config, &pm_config, &adc_config);

  mcpwm_config.group_id = 0;
  motor_config.pwma_gpio_num = MOTOR1_A;
  motor_config.pwmb_gpio_num = MOTOR1_B;
  adc_config.ch_id = ADC1_CHAN1;
  motor1 = init_motor(&motor_config, &mcpwm_config, &pm_config, &adc_config);

  mcpwm_config.group_id = 0;
  motor_config.pwma_gpio_num = MOTOR2_A;
  motor_config.pwmb_gpio_num = MOTOR2_B;
  adc_config.ch_id = ADC1_CHAN2;
  motor2 = init_motor(&motor_config, &mcpwm_config, &pm_config, &adc_config);

  mcpwm_config.group_id = 1;
  motor_config.pwma_gpio_num = MOTOR3_A;
  motor_config.pwmb_gpio_num = MOTOR3_B;
  adc_config.ch_id = ADC1_CHAN3;
  motor3 = init_motor(&motor_config, &mcpwm_config, &pm_config, &adc_config);

  TaskHandle_t motor_task_handle = NULL;
  xTaskCreate(motor_control, "motor_task", 2048, NULL, 1, &motor_task_handle);
}
