#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// Labplus mPython V2.0.3

// Buttons
#define BOOT_BUTTON_GPIO        GPIO_NUM_0   // A键 / P5
#define BUTTON_B_GPIO           GPIO_NUM_2   // B键 / P11

// RGB LED
#define RGB_LED_GPIO            GPIO_NUM_17  // P7

// Buzzer
#define BUZZER_GPIO             GPIO_NUM_16  // P6

// I2C / OLED
#define DISPLAY_SCL_PIN         GPIO_NUM_22  // P19
#define DISPLAY_SDA_PIN         GPIO_NUM_23  // P20

#define DISPLAY_WIDTH           128
#define DISPLAY_HEIGHT          64
#define DISPLAY_MIRROR_X        false
#define DISPLAY_MIRROR_Y        false

// Board-specific analog audio pins
#define LABPLUS_MIC_GPIO        GPIO_NUM_38
#define LABPLUS_AUDIO_LEFT_GPIO GPIO_NUM_26  // P8
#define LABPLUS_AUDIO_RIGHT_GPIO GPIO_NUM_25 // P9

#endif // _BOARD_CONFIG_H_
