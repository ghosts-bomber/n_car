#include "mpu_task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mpu6050.h"
#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 task";
static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

  esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
  ESP_ERROR_CHECK(ret);

  ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  ESP_ERROR_CHECK(ret);
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void) {
  esp_err_t ret;

  i2c_bus_init();
  mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
  ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
  ESP_ERROR_CHECK(ret);

  ret = mpu6050_wake_up(mpu6050);
  ESP_ERROR_CHECK(ret);
}

static void mpu_task(void *param) {
  esp_err_t ret;
  uint8_t mpu6050_deviceid;
  mpu6050_acce_value_t acce;
  mpu6050_gyro_value_t gyro;
  mpu6050_temp_value_t temp;

  ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
  ESP_ERROR_CHECK(ret);
  while (true) {
    ret = mpu6050_get_acce(mpu6050, &acce);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x,
             acce.acce_y, acce.acce_z);

    ret = mpu6050_get_gyro(mpu6050, &gyro);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x,
             gyro.gyro_y, gyro.gyro_z);

    ret = mpu6050_get_temp(mpu6050, &temp);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "t:%.2f \n", temp.temp);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  mpu6050_delete(mpu6050);
  ret = i2c_driver_delete(I2C_MASTER_NUM);
  ESP_ERROR_CHECK(ret);
}

void create_mpu_task() {
  i2c_sensor_mpu6050_init();

  TaskHandle_t mpu_task_handle = NULL;
  xTaskCreate(mpu_task, "mpu_task", 1024, NULL, 3, &mpu_task_handle);
}
