#include "motor_task.h"
#include "bdc_motor.h"
#include "freertos/FreeRTOS.h"
#define BDC_MCPWM_FREQ_HZ 25000                // 25KHz PWM
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us

#define MOTOR0_A (1)
#define MOTOR0_B (1)
#define MOTOR1_A (1)
#define MOTOR1_B (1)
#define MOTOR2_A (1)
#define MOTOR2_B (1)
#define MOTOR3_A (1)
#define MOTOR3_B (1)
#define MOTOR0_A (1)
static const char *TAG = "motor_task";
static bdc_motor_handle_t motor0 = NULL;
static bdc_motor_handle_t motor1 = NULL;
static bdc_motor_handle_t motor2 = NULL;
static bdc_motor_handle_t motor3 = NULL;

static bdc_motor_handle_t
init_motor(const bdc_motor_config_t *motor_config,
           const bdc_motor_mcpwm_config_t *mcpwm_config) {
  bdc_motor_handle_t motor_handle = NULL;
  esp_err_t err =
      bdc_motor_new_mcpwm_device(motor_config, mcpwm_config, &motor_handle);
  return motor_handle;
}

static void motor_control(void *arg) {}

void create_motor_task(void) {
  bdc_motor_mcpwm_config_t mcpwm_config = {
      .group_id = 0,
      .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
  };
  bdc_motor_config_t motor_config = {
      .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
      .pwma_gpio_num = MOTOR0_A,
      .pwmb_gpio_num = MOTOR0_B,
  };
  motor0 = init_motor(&motor_config, &mcpwm_config);

  mcpwm_config.group_id = 1;
  motor_config.pwma_gpio_num = MOTOR1_A;
  motor_config.pwmb_gpio_num = MOTOR1_B;
  motor1 = init_motor(&motor_config, &mcpwm_config);

  mcpwm_config.group_id = 2;
  motor_config.pwma_gpio_num = MOTOR2_A;
  motor_config.pwmb_gpio_num = MOTOR2_B;
  motor2 = init_motor(&motor_config, &mcpwm_config);

  mcpwm_config.group_id = 3;
  motor_config.pwma_gpio_num = MOTOR3_A;
  motor_config.pwmb_gpio_num = MOTOR3_B;
  motor3 = init_motor(&motor_config, &mcpwm_config);

  bdc_motor_enable(motor0);
  bdc_motor_enable(motor1);
  bdc_motor_enable(motor2);
  bdc_motor_enable(motor3);

  TaskHandle_t motor_task_handle = NULL;
  xTaskCreate(motor_control,"motor_task", 2048, NULL, 1, &motor_task_handle);
}
