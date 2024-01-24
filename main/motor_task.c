#include "motor_task.h"
#include "bdc_motor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/adc_types.h"
#include "pid_ctrl.h"
#include "potentiometer.h"
#include "stdint.h"
#include <string.h>
#include <stdint.h>
#define BDC_MCPWM_FREQ_HZ 25000                // 25KHz PWM
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_DUTY_TICK_MAX                                                \
  (BDC_MCPWM_TIMER_RESOLUTION_HZ /                                             \
   BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define CONTROL_INTERVAL (40)

#define MOTOR0_A (18)
#define MOTOR0_B (19)
#define MOTOR1_A (12)
#define MOTOR1_B (13)
#define MOTOR2_A (25)
#define MOTOR2_B (26)
#define MOTOR3_A (2)
#define MOTOR3_B (4)

#define ADC1_CHAN0 ADC_CHANNEL_7 // 35
#define ADC1_CHAN1 ADC_CHANNEL_6 // 34
#define ADC1_CHAN2 ADC_CHANNEL_5 // 33
#define ADC1_CHAN3 ADC_CHANNEL_4 // 32
//
#define ADC_JUMP_VAL (500)
#define PI (3.14)
static const char *TAG = "motor_task";

typedef struct {
  bdc_motor_handle_t motor_handle;
  potentiomenter_handle_t pm_handle;
  pid_ctrl_block_handle_t pid_ctrl;
  int adc_max;
  int adc_min;
  int adc_val;
  int64_t time;
} Motor;

static Motor *motor0;
static Motor *motor1;
static Motor *motor2;
static Motor *motor3;

Motor_control motor_contorl;
SemaphoreHandle_t motor_control_mutex;

static Motor *init_motor(const bdc_motor_config_t *motor_config,
                         const bdc_motor_mcpwm_config_t *mcpwm_config,
                         const potentiomenter_config_t *pm_config,
                         const potentiomenter_adc_config_t *adc_config,
                         const pid_ctrl_config_t *pid_config) {
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
  potentiomenter_handle_t pm_handle = NULL;
  err = potentiomenter_new_adc_device(pm_config, adc_config, &pm_handle);
  pid_ctrl_block_handle_t pid_ctrl = NULL;
  pid_new_control_block(pid_config, &pid_ctrl);
  motor->motor_handle = bdc_motor_handle;
  motor->pm_handle = pm_handle;
  motor->pid_ctrl = pid_ctrl;
  motor->adc_max = pm_handle->potentiomenter_config.max;
  motor->adc_min = pm_handle->potentiomenter_config.min;
  motor->adc_val = 0;
  motor->time = 0;
  return motor;
}

static void process_motor_control_legancy(bdc_motor_handle_t motor, int16_t val) {
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

static void process_motor_control(bdc_motor_handle_t motor, float val) {
  if (val > -0.1 && val < 0.1) {
    bdc_motor_coast(motor);
  } else {
    if (val > 0) {
      bdc_motor_forward(motor);
      bdc_motor_set_speed(motor, (uint32_t)val);
    } else if (val < 0) {
      bdc_motor_reverse(motor);
      bdc_motor_set_speed(motor, (uint32_t)(-val));
    }
  }
}

static float get_motor_rad(Motor *motor) {
  static int64_t cur_time;
  static float rad = 0.0f;
  static float du_time = 0.0f; // s
  static float rad_v = 0.0f;
  cur_time = esp_timer_get_time();
  int v = motor->pm_handle->get_value(motor->pm_handle);
  // first
  if (motor->adc_val == 0) {
    motor->adc_val = v;
    motor->time = cur_time;
    return 0;
  }
  if (v > motor->adc_max)
    motor->adc_max = v;
  if (v < motor->adc_min)
    motor->adc_min = v;
  int sub = v - motor->adc_val;
  if (sub > ADC_JUMP_VAL) {
    sub = (motor->adc_max - motor->adc_val + v - motor->adc_min);
  } else if (sub < -ADC_JUMP_VAL) {
    sub = -(motor->adc_val - motor->adc_min + motor->adc_max - motor->adc_val);
  }
  rad = (float)sub * 2.0 * PI / (motor->adc_max - motor->adc_min);
  du_time = (cur_time - motor->time) / 1000.0f / 1000.0f; // ns -> s
  rad_v = rad / du_time;
  motor->adc_val = v;
  motor->time = cur_time;
  return rad_v;
}
static void motor_process(Motor *motor, int ctl) {

  float m_rad = get_motor_rad(motor);
  float dst_rad = 2*2*PI*ctl/32768.0;
  float error = m_rad - dst_rad;
  float new_speed = 0.0f;
  pid_compute(motor->pid_ctrl, error, &new_speed);
  // ESP_LOGI(TAG,"m_rad:%f ctl:%d ,dst_rad= %f , val= %f \n",m_rad,ctl,dst_rad,new_speed);
  process_motor_control(motor->motor_handle, new_speed);
}
static void motor_control_task(void *arg) {
  int motor0_ctl = 0;
  int motor1_ctl = 0;
  int motor2_ctl = 0;
  int motor3_ctl = 0;

  while (1) {
    if (xSemaphoreTake(motor_control_mutex, portMAX_DELAY) == pdTRUE) {
      motor0_ctl = motor_contorl.motor0;
      motor1_ctl = motor_contorl.motor1;
      motor2_ctl = motor_contorl.motor2;
      motor3_ctl = motor_contorl.motor3;
      xSemaphoreGive(motor_control_mutex);
      motor_process(motor0, motor0_ctl);
      motor_process(motor1, motor1_ctl);
      motor_process(motor2, motor2_ctl);
      motor_process(motor3, motor3_ctl);
      // ESP_LOGI(TAG,"t:%lld v0= %d , v1= %d \n",t,v0,v1);
    }
    vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL));
  }
}
void create_motor_task(void) {
  bdc_motor_mcpwm_config_t mcpwm_config = {};
  bdc_motor_config_t motor_config = {};
  potentiomenter_config_t pm_config = {};
  potentiomenter_adc_config_t adc_config = {};
  pid_ctrl_parameter_t pid_runtime_param = {};
  pid_runtime_param.kp = 0.6;
  pid_runtime_param.ki = 0.4;
  pid_runtime_param.kd = 0.2;
  pid_runtime_param.cal_type = PID_CAL_TYPE_INCREMENTAL;
  pid_runtime_param.max_output = BDC_MCPWM_DUTY_TICK_MAX - 1;
  pid_runtime_param.min_output = -(BDC_MCPWM_DUTY_TICK_MAX-1);
  pid_runtime_param.max_integral = 1000;
  pid_runtime_param.min_integral = -1000;
  pid_ctrl_config_t pid_config = {};
  pid_config.init_param = pid_runtime_param;
  mcpwm_config.group_id = 0;
  mcpwm_config.resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ;

  pm_config.max = 1600;
  pm_config.min = 900;

  adc_config.group_id = ADC_UNIT_1;
  adc_config.bitwidth = ADC_BITWIDTH_DEFAULT;
  adc_config.atten = ADC_ATTEN_DB_12;
  adc_config.ch_id = ADC1_CHAN0;

  motor_config.pwm_freq_hz = BDC_MCPWM_FREQ_HZ;
  motor_config.pwma_gpio_num = MOTOR0_A;
  motor_config.pwmb_gpio_num = MOTOR0_B;
  motor0 = init_motor(&motor_config, &mcpwm_config, &pm_config, &adc_config,
                      &pid_config);

  mcpwm_config.group_id = 0;
  motor_config.pwma_gpio_num = MOTOR1_A;
  motor_config.pwmb_gpio_num = MOTOR1_B;
  adc_config.ch_id = ADC1_CHAN1;
  motor1 = init_motor(&motor_config, &mcpwm_config, &pm_config, &adc_config,
                      &pid_config);

  mcpwm_config.group_id = 0;
  motor_config.pwma_gpio_num = MOTOR2_A;
  motor_config.pwmb_gpio_num = MOTOR2_B;
  adc_config.ch_id = ADC1_CHAN2;
  motor2 = init_motor(&motor_config, &mcpwm_config, &pm_config, &adc_config,
                      &pid_config);

  mcpwm_config.group_id = 1;
  motor_config.pwma_gpio_num = MOTOR3_A;
  motor_config.pwmb_gpio_num = MOTOR3_B;
  adc_config.ch_id = ADC1_CHAN3;
  motor3 = init_motor(&motor_config, &mcpwm_config, &pm_config, &adc_config,
                      &pid_config);

  motor_control_mutex = xSemaphoreCreateMutex();
  memset((void *)&motor_contorl, 0, sizeof(Motor_control));
  TaskHandle_t motor_task_handle = NULL;
  xTaskCreate(motor_control_task, "motor_task", 2048, NULL, 3,
              &motor_task_handle);
}
