#ifndef COMMON_h
#define COMMON_h

#include <stdint.h>
//#include <U8g2lib.h>
#include "oled_task.h"
extern u8g2_t u8g2;
// extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

int random_int(int min, int max);

#endif
