#include "oled_task.h"
#include "FaceEmotions.hpp"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "u8g2.h"
#include "u8x8.h"
#include "eye/Face.h"
#include <cstdint>
u8g2_t u8g2;
Face *face;
void showmain() {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);                        // 清屏
  u8g2_SetFont(&u8g2, u8g2_font_spleen32x64_me); // 设置英文字体
  u8g2_DrawStr(&u8g2, 64, 64, "1000");
  u8g2_DrawStr(&u8g2, 192, 64, "KG");
  u8g2_SendBuffer(&u8g2);
}

void showmain2() {
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);                         // 清屏
  u8g2_SetFont(&u8g2, u8g2_font_wqy16_t_gb2312b); //
  u8g2_DrawUTF8(&u8g2, 32, 32, "总重量");
  u8g2_SendBuffer(&u8g2);
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Check if x is outside the input range
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;

  // Calculate the proportion of x relative to the input range
  float proportion = (x - in_min) / (in_max - in_min);

  // Map the proportion to the output range and return the result
  return (proportion * (out_max - out_min)) + out_min;
}
static void oled_task(void *param) {
  showmain();
  showmain2();
  int i=0;
  while (1) {
  if(i>=EMOTIONS_COUNT) i=0; 
  face->Behavior.Clear();
  face->Behavior.SetEmotion((eEmotions)i++, 1.0);
  face->Update();
  vTaskDelay(pdMS_TO_TICKS(2000));
  ++i;
  }
}

void create_oled_task() {
  u8g2Init(&u8g2);
  u8g2_ClearBuffer(&u8g2);
  face = new Face(/* screenWidth = */ 128, /* screenHeight = */ 64, /* eyeSize = */ 40);
  // Assign the current expression
  face->Expression.GoTo_Normal();

  // Assign a weight to each emotion
  //face->Behavior.SetEmotion(eEmotions::Normal, 1.0);
  //face->Behavior.SetEmotion(eEmotions::Angry, 1.0);
  //face->Behavior.SetEmotion(eEmotions::Sad, 1.0);
  // Automatically switch between behaviours (selecting new behaviour randomly based on the weight assigned to each emotion)
  // face->RandomBehavior = true;

  // Automatically blink
  // face->RandomBlink = true;
  // Set blink rate
  face->Blink.Timer.SetIntervalMillis(4000);

  // Automatically choose a new random direction to look
  face->RandomLook = false;

  TaskHandle_t handle = NULL;
  xTaskCreate(oled_task, "oled_task", 4096, &u8g2, 1, &handle);
}
