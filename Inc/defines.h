#ifndef TM_DEFINES_H
#define TM_DEFINES_H

/* Put your global defines for all libraries here used in your project */
#define VERSION							3

#define ADCn							3
#define ADC_MAX							4095

#define DELAY_MAX_CUSTOM_TIMERS   		10

#define PERIOD_WARNING					10000L
#define PERIOD_ALARM					250
#define PERIOD_CHECK_VOLTAGE			499
#define PERIOD_SCAN_KEY					250
#define TIME_CHANGE_SCREEN				2
#define TIME_NO_PRESSED_BUTTON			120
#define TIME_SEND_VOLTAGE_VALUE_UART	60
#define TIME_SEND_VOLTAGE_VALUE_SD_CARD	600
#define REPEAT							1
#define ONCE							0
#define IMMEDIATELY						1
#define SOMETIME						0

#define LCD_BACKLIGHT_ON()				HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_Port, LCD_BACKLIGHT_Pin, GPIO_PIN_SET)
#define LCD_BACKLIGHT_OFF()				HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_Port, LCD_BACKLIGHT_Pin, GPIO_PIN_RESET)

#define LED_RED_ON()					HAL_GPIO_WritePin(LD_RED_GPIO_Port, LD_RED_Pin, GPIO_PIN_SET)
#define LED_RED_OFF()					HAL_GPIO_WritePin(LD_RED_GPIO_Port, LD_RED_Pin, GPIO_PIN_RESET)
#define LED_RED_TOGGLE()				HAL_GPIO_TogglePin(LD_RED_GPIO_Port, LD_RED_Pin);

#define SETTING_FILE_NAME				"setting.ini"

enum setting_op {
	setting_save,
	setting_load,
	setting_delete
};

#define SD_SALT	27010904

#endif
