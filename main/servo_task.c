#include "servo_task.h"
#include "freertos/FreeRTOS.h"
#include "mqtt.h"
#include "portmacro.h"
#include <stdint.h>
extern QueueHandle_t servo_queue;
static void servo_task(void* param)
{
  int16_t angle = 0;
  servo_dev_t* servo_dev = (servo_dev_t*)(param);
  while(1){
    // if(angle>90) angle = -90;
    // set_servo_degree(servo_dev,angle);
    // angle+=5;
    // vTaskDelay(500/portTICK_PERIOD_MS);
    xQueueReceive(servo_queue, &angle, portMAX_DELAY); 
    set_servo_degree(servo_dev,angle);
  }
  vTaskDelete(NULL);
}
void create_servo_task()
{
  static servo_dev_t servo_dev = {
    .pin = SERVO_PIN,
    .max_degree = 90,
    .min_degree = -90,
    .min_pulsewidth_us = 500,
    .max_pulsewidth_us = 2500,
    .control_hz = 50,
  };
  servo_init(&servo_dev);
  TaskHandle_t servo_handle = NULL;
  xTaskCreate(servo_task,"servo_task",1024,&servo_dev,1,&servo_handle);

}
