#include "servo.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_timer.h"
#include "esp_err.h"
#include "hal/mcpwm_types.h"
#include "soc/clk_tree_defs.h"
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick

static inline uint32_t angle_to_compare(const servo_dev_t *dev, int angle) {
  return (angle - dev->min_degree) *
             (dev->max_pulsewidth_us - dev->min_pulsewidth_us) /
             (dev->max_degree - dev->min_degree) +
         dev->min_pulsewidth_us;
  ;
}

int servo_init(servo_dev_t *dev) {
  if (dev == NULL)
    return -1;
  if (dev->max_degree == 0 || dev->control_hz == 0)
    return -1;
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
      .period_ticks = SERVO_TIMEBASE_RESOLUTION_HZ / dev->control_hz,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t operator_config = {
      .group_id = 0,
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

  mcpwm_comparator_config_t comparator_config = {
      .flags.update_cmp_on_tez = true,
  };

  ESP_ERROR_CHECK(
      mcpwm_new_comparator(oper, &comparator_config, &dev->comparator));

  mcpwm_generator_config_t generator_config = {
      .gen_gpio_num = dev->pin,
  };
  ESP_ERROR_CHECK(
      mcpwm_new_generator(oper, &generator_config, &dev->generator));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(dev->comparator,
                                                     angle_to_compare(dev, 0)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
      dev->generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                   MCPWM_TIMER_EVENT_EMPTY,
                                                   MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
      dev->generator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, dev->comparator,
                                     MCPWM_GEN_ACTION_LOW)));
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
  return 0;
}

int set_servo_degree(servo_dev_t *dev, int degree) {
  if (dev == NULL)
    return -1;
  return mcpwm_comparator_set_compare_value(dev->comparator,
                                            angle_to_compare(dev, degree));
}
