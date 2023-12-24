#ifndef OLED_TASK_H
#define OLED_TASK_H
#include "u8g2.h"
#include "u8x8.h"

// #ifdef __cplusplus
// extern "C" {
// #endif
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8


extern u8g2_t u8g2;
void oled_task(void* param);

void create_oled_task();

// #ifdef __cplusplus
// }
// #endif




#endif // OLED_TASK_H
