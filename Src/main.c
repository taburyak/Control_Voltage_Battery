/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "defines.h"
#include "hd44780.h"
#include "tm_stm32_delay.h"
#include "m_rtc_2001.h"
#include "stdbool.h"
#include "MicroMenu.h"
#include "uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum
{
	BUTTON_NOTHING 	= 0,
	BUTTON_LEFT 	= 1,
	BUTTON_UP 		= 2,
	BUTTON_DOWN 	= 3,
	BUTTON_RIGHT 	= 4,
	BUTTON_SELECT 	= 5,
} Button_TypeDef;

volatile Button_TypeDef key 							= BUTTON_NOTHING;
volatile bool 			triggerScanKey 					= false;
volatile bool 			secondFlag 						= false;
volatile uint16_t 		dataADC[ADCn]					= {0, 0, 0};
volatile uint16_t 		counterChangeScreen				= 0;
volatile uint16_t 		counterNoPressedButton			= 0;
volatile uint16_t 		counterSendVoltageValueUART		= 0;
volatile uint16_t 		counterSendVoltageValueSDCard	= 0;
volatile bool 			triggerAlarmVoltage[ADCn]		= {false, false, false};
volatile bool 			stateAlarmVoltage[ADCn]			= {false, false, false};
volatile bool 			stateUserResponded[ADCn]		= {true, true, true};
		 bool			triggerNoPressedButton			= false;
		 bool 			triggerSDCardEnable 			= false;
		 uint8_t 		currentNumberPage 				= 0;

enum {
	normal,
	max,
	min
}errorVoltageEnum;

typedef struct
{
	uint8_t				id;
	char* 				name;
	float 				min;
	float 				max;
	float				maxMeasured;
	float 				voltage;
	uint8_t 			errorVoltage;
	TM_DELAY_Timer_t*	pTimerWarning;
	TM_DELAY_Timer_t*	pTimerNormal;
}VoltageStruct_TypeDef;

//-----змінні налаштувань які будуть збрігатись в EEPROM чи SD-CARD-----
typedef struct {
	int 					salt;
	bool					triggerControlVoltage; 	// true - йде перевірка виходу за межі напруги, false - такої перевірки немає (міняється в налаштуваннях)
	bool					triggerLCDBackLight;	// true - підсвітка екрану увімкенено, false - підсвітка екрану вимкнено
	uint16_t				periodWarning;			// час в мілісеках на аналіз виходу за межі
	uint16_t 				timeChangeScreen;
	uint16_t 				timeNoPressedButton;
	uint16_t 				timeSendVoltageValueUART;
	uint16_t 				timeSendVoltageValueSDCard;
	VoltageStruct_TypeDef 	voltageParam[ADCn];
}settingDeviceTypeDef;

settingDeviceTypeDef settingDevice;
//-----------------------------------------------------------

TM_DELAY_Timer_t* pTimerWarning;
TM_DELAY_Timer_t* pTimerNormal;
TM_DELAY_Timer_t* pTimerAlarmBeeper;
TM_DELAY_Timer_t* pTimerAlarmLed;
TM_DELAY_Timer_t* pTimerCheckVoltage;
TM_DELAY_Timer_t* pTimerScanKey;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void 	beep						(uint16_t tone, uint16_t time);
static void 	printSettingMenu			(const char* text);
static void 	buttonHandler				(void);
static void 	voltageAlarmHandler			(void);
static void 	showMeasuredAllVoltage		(void);
static void 	headingSingleVoltage		(const char* info);
static void 	showSolitaryMeasuredVoltage	(const uint8_t i);
static void 	settingDefaultValues		(void *settings);
static FRESULT 	settingLoadSave				(enum setting_op mode, const char string[], void *settings, uint32_t size);
static FRESULT 	logerSD						(const char string[]);
static void 	sendUARTErrorSD				(FRESULT fresult);
static void 	logerUART					(const char string[]);
static void 	sendVoltageValueUART		(void);
static void 	sendVoltageValueSDCard		(void);
static void		changeScreenDisplay			(void);
static void 	warningVoltage				(struct _TM_DELAY_Timer_t* my_timer, void* parameters);
static void 	normalVoltage				(struct _TM_DELAY_Timer_t* my_timer, void* parameters);
static void 	alarmBeeper					(struct _TM_DELAY_Timer_t* my_timer, void* parameters);
static void 	alarmLed					(struct _TM_DELAY_Timer_t* my_timer, void* parameters);
static void 	checkVoltage				(struct _TM_DELAY_Timer_t* my_timer, void* parameters);
static void 	scanKey						(struct _TM_DELAY_Timer_t* my_timer, void* parameters);

static void 	showCurrentTime				(void);
static void		showVoltagePlus60			(void);
static void		showVoltageMinus60			(void);
static void		showVoltageMinus24			(void);

static void 	settingPeriodSecond			(uint16_t* value,
											 uint8_t dig,
											 uint16_t max,
											 uint16_t min,
											 const char* str,
											 Menu_Item_t* const menu);

static void		settingVoltageMinMax		(float* value,
											 float max,
											 float min,
											 const char* strInfo,
											 const char* strMsg,
											 Menu_Item_t* const menu);

static void		settingMaxVoltageMeasured	(float* value,
											 float* currentValue,
											 const char* strInfo,
											 const char* strMsg,
											 Menu_Item_t* const menu);
// Menus:
static void Level1Item2_Enter(void);

static void Level2Item1_1_Enter(void);
static void Level2Item1_2_Enter(void);
static void Level2Item1_3_Enter(void);
static void Level2Item1_4_Enter(void);

static void Level3Item3_1_1_Enter(void);
static void Level3Item3_1_2_Enter(void);
static void Level3Item3_1_3_Enter(void);
static void Level3Item3_1_4_Enter(void);
static void Level3Item3_1_5_Enter(void);
static void Level3Item3_1_6_Enter(void);
static void Level3Item3_4_5_Enter(void);
static void Level3Item3_4_6_Enter(void);
static void Level3Item3_4_7_Enter(void);

static void Level3Item3_2_1_Select(void);
static void Level3Item3_2_2_Select(void);

static void Level3Item3_3_1_Enter(void);
static void Level3Item3_3_2_Enter(void);

static void Level4Item3_4_1_1_Enter(void);
static void Level4Item3_4_1_2_Enter(void);
static void Level4Item3_4_1_3_Enter(void);
static void Level4Item3_4_2_1_Enter(void);
static void Level4Item3_4_2_2_Enter(void);
static void Level4Item3_4_2_3_Enter(void);
static void Level4Item3_4_3_1_Enter(void);
static void Level4Item3_4_3_2_Enter(void);
static void Level4Item3_4_3_3_Enter(void);

static void Level4Item3_4_4_1_Select(void);
static void Level4Item3_4_4_2_Select(void);

static void Generic_Write(const char* Text);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct showValueDisplay 		//структура для виклику функцій показу данних на дисплей
{
	bool status;						//статус чи показувати той чи інший параметр
	void(*showValue)(void);				//посилання на функцію
}showValueDisplayTypeDef;

showValueDisplayTypeDef currentTime		= {true, showCurrentTime};
showValueDisplayTypeDef voltagePlus60	= {true, showVoltagePlus60};
showValueDisplayTypeDef voltageMinus60	= {true, showVoltageMinus60};
showValueDisplayTypeDef voltageMinus24	= {true, showVoltageMinus24};

showValueDisplayTypeDef* showDisplay[] = {
		&currentTime,
		&voltagePlus60,
		&voltageMinus60,
		&voltageMinus24
};

static const uint8_t numberPageOfScreen = sizeof(showDisplay) / sizeof(showDisplay[0]);

// Menus  Name | Next | Prev | Parent | Child | SelectFunction | EnterFunction | Text
MENU_ITEM(Menu_1, Menu_2, Menu_3, Menu_1, Menu_1_1, NULL, NULL, "\t  Menu\n\tVoltage");
MENU_ITEM(Menu_2, Menu_3, Menu_1, Menu_2, Menu_2,	NULL, Level1Item2_Enter, "\tColodar\n  Data and time");
MENU_ITEM(Menu_3, Menu_1, Menu_2, Menu_3, Menu_3_1, NULL, NULL, "\t  Menu\n\tsettings");

MENU_ITEM(Menu_1_1, Menu_1_2, Menu_1_4, Menu_1, Menu_1_1, NULL, Level2Item1_1_Enter, " Press ENTER to\nShow All Voltage");
MENU_ITEM(Menu_1_2, Menu_1_3, Menu_1_1, Menu_1, Menu_1_2, NULL, Level2Item1_2_Enter, " Press ENTER to\nShow Level: +60V");
MENU_ITEM(Menu_1_3, Menu_1_4, Menu_1_2, Menu_1, Menu_1_3, NULL, Level2Item1_3_Enter, " Press ENTER to\nShow Level: -60V");
MENU_ITEM(Menu_1_4, Menu_1_1, Menu_1_3, Menu_1, Menu_1_4, NULL, Level2Item1_4_Enter, " Press ENTER to\nShow Level: -24V");

MENU_ITEM(Menu_3_1, Menu_3_2, Menu_3_4, Menu_3, Menu_3_1_1, NULL, NULL, "\tSetting\n\tCalendar");
MENU_ITEM(Menu_3_2, Menu_3_3, Menu_3_1, Menu_3, Menu_3_2_1, NULL, NULL, "\tSetting\nLCD backlighting");
MENU_ITEM(Menu_3_3, Menu_3_4, Menu_3_2, Menu_3, Menu_3_3_1, NULL, NULL, "\tSetting\n  demo screen");
MENU_ITEM(Menu_3_4, Menu_3_1, Menu_3_3, Menu_3, Menu_3_4_1, NULL, NULL, "Setting battery\ncontrol parameter");

MENU_ITEM(Menu_3_1_1, Menu_3_1_2, Menu_3_1_6, Menu_3_1, Menu_3_1_1, NULL, Level3Item3_1_1_Enter, " Press ENTER to\n Setting Hours");
MENU_ITEM(Menu_3_1_2, Menu_3_1_3, Menu_3_1_1, Menu_3_1, Menu_3_1_2, NULL, Level3Item3_1_2_Enter, " Press ENTER to\n Setting Minutes");
MENU_ITEM(Menu_3_1_3, Menu_3_1_4, Menu_3_1_2, Menu_3_1, Menu_3_1_3, NULL, Level3Item3_1_3_Enter, " Press ENTER to\n Setting Seconds");
MENU_ITEM(Menu_3_1_4, Menu_3_1_5, Menu_3_1_3, Menu_3_1, Menu_3_1_4, NULL, Level3Item3_1_4_Enter, " Press ENTER to\n Setting Date");
MENU_ITEM(Menu_3_1_5, Menu_3_1_6, Menu_3_1_4, Menu_3_1, Menu_3_1_5, NULL, Level3Item3_1_5_Enter, " Press ENTER to\n Setting Month");
MENU_ITEM(Menu_3_1_6, Menu_3_1_1, Menu_3_1_5, Menu_3_1, Menu_3_1_6, NULL, Level3Item3_1_6_Enter, " Press ENTER to\n Setting Year");

MENU_ITEM(Menu_3_2_1, Menu_3_2_2, Menu_3_2_2, Menu_3_2, Menu_3_2_1, Level3Item3_2_1_Select, NULL, " LCD backlight\n\t is on");
MENU_ITEM(Menu_3_2_2, Menu_3_2_1, Menu_3_2_1, Menu_3_2, Menu_3_2_2, Level3Item3_2_2_Select, NULL, " LCD backlight\n\t is off");

MENU_ITEM(Menu_3_3_1, Menu_3_3_2, Menu_3_3_2, Menu_3_3, Menu_3_3_1, NULL, Level3Item3_3_1_Enter, " Press ENTER to\nno press key sec");
MENU_ITEM(Menu_3_3_2, Menu_3_3_1, Menu_3_3_1, Menu_3_3, Menu_3_3_2, NULL, Level3Item3_3_2_Enter, " Press ENTER to\n  change screen");

MENU_ITEM(Menu_3_4_1, Menu_3_4_2, Menu_3_4_7, Menu_3_4, Menu_3_4_1_1, NULL, NULL, "Set paramVoltage\n+60 battery");
MENU_ITEM(Menu_3_4_2, Menu_3_4_3, Menu_3_4_1, Menu_3_4, Menu_3_4_2_1, NULL, NULL, "Set paramVoltage\n-60 battery");
MENU_ITEM(Menu_3_4_3, Menu_3_4_4, Menu_3_4_2, Menu_3_4, Menu_3_4_3_1, NULL, NULL, "Set paramVoltage\n-24 battery");
MENU_ITEM(Menu_3_4_4, Menu_3_4_5, Menu_3_4_3, Menu_3_4, Menu_3_4_4_1, NULL, NULL, "Voltage Control\n\tON/OFF");
MENU_ITEM(Menu_3_4_5, Menu_3_4_6, Menu_3_4_4, Menu_3_4, Menu_3_4_5, NULL, Level3Item3_4_5_Enter, " Press ENTER to\nperiod warning");
MENU_ITEM(Menu_3_4_6, Menu_3_4_7, Menu_3_4_5, Menu_3_4, Menu_3_4_6, NULL, Level3Item3_4_6_Enter, " Press ENTER to\nperiod send UART");
MENU_ITEM(Menu_3_4_7, Menu_3_4_1, Menu_3_4_6, Menu_3_4, Menu_3_4_7, NULL, Level3Item3_4_7_Enter, " Press ENTER to\nperiod send SD");

MENU_ITEM(Menu_3_4_1_1, Menu_3_4_1_3, Menu_3_4_1_2, Menu_3_4_1, Menu_3_4_1_1, NULL, Level4Item3_4_1_1_Enter, " Press ENTER to\n Setting Max +60");
MENU_ITEM(Menu_3_4_1_2, Menu_3_4_1_1, Menu_3_4_1_3, Menu_3_4_1, Menu_3_4_1_2, NULL, Level4Item3_4_1_2_Enter, " Press ENTER to\n Setting Min +60");
MENU_ITEM(Menu_3_4_1_3, Menu_3_4_1_2, Menu_3_4_1_1, Menu_3_4_1, Menu_3_4_1_3, NULL, Level4Item3_4_1_3_Enter, " Press ENTER to\n Max measure +60");

MENU_ITEM(Menu_3_4_2_1, Menu_3_4_2_3, Menu_3_4_2_2, Menu_3_4_2, Menu_3_4_2_1, NULL, Level4Item3_4_2_1_Enter, " Press ENTER to\n Setting Max -60");
MENU_ITEM(Menu_3_4_2_2, Menu_3_4_2_1, Menu_3_4_2_3, Menu_3_4_2, Menu_3_4_2_2, NULL, Level4Item3_4_2_2_Enter, " Press ENTER to\n Setting Min -60");
MENU_ITEM(Menu_3_4_2_3, Menu_3_4_2_2, Menu_3_4_2_1, Menu_3_4_2, Menu_3_4_2_3, NULL, Level4Item3_4_2_3_Enter, " Press ENTER to\n Max measure -60");

MENU_ITEM(Menu_3_4_3_1, Menu_3_4_3_3, Menu_3_4_3_2, Menu_3_4_3, Menu_3_4_3_1, NULL, Level4Item3_4_3_1_Enter, " Press ENTER to\n Setting Max -24");
MENU_ITEM(Menu_3_4_3_2, Menu_3_4_3_1, Menu_3_4_3_3, Menu_3_4_3, Menu_3_4_3_2, NULL, Level4Item3_4_3_2_Enter, " Press ENTER to\n Setting Min -24");
MENU_ITEM(Menu_3_4_3_3, Menu_3_4_3_2, Menu_3_4_3_1, Menu_3_4_3, Menu_3_4_3_3, NULL, Level4Item3_4_3_3_Enter, " Press ENTER to\n Max measure -24");

MENU_ITEM(Menu_3_4_4_1, Menu_3_4_4_2, Menu_3_4_4_2, Menu_3_4_4, Menu_3_4_4_1, Level4Item3_4_4_1_Select, NULL, "Control Voltage\n\tis OFF");
MENU_ITEM(Menu_3_4_4_2, Menu_3_4_4_1, Menu_3_4_4_1, Menu_3_4_4, Menu_3_4_4_2, Level4Item3_4_4_2_Select, NULL, "Control Voltage\n\tis ON");
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  FRESULT fresult;
  mRTC_Begin(&hrtc);
  TM_DELAY_Init();
  Delayms(1000);
  beep(700,50);

  const char* str = "Device is start";

  logerUART(str);

  fresult = logerSD(str);

  if(fresult != FR_OK)
  {
	  sendUARTErrorSD(fresult);
	  triggerSDCardEnable = false;
  }
  else
  {
	  logerUART("SD-CARD: OK");
	  triggerSDCardEnable = true;
  }

  char strVersion[32];
  sprintf(strVersion, "Firmware version %d", VERSION);
  logerUART(strVersion);
  if(triggerSDCardEnable)
  {
	  logerSD(str);
  }

  //----------Setting SD-CARD----------------
  if(triggerSDCardEnable)
  {
	fresult = settingLoadSave(setting_load, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));

	if(fresult == FR_OK)
	{
		if(settingDevice.salt == SD_SALT)
		{
			logerUART("SD-CARD: load setting device is OK");
			logerSD("SD-CARD: load setting device is OK");
		}
		else
		{
			logerUART("SD-CARD: setting device is corrupt");
			logerSD("SD-CARD: setting device is corrupt");

			settingDefaultValues(&settingDevice);

			logerUART("Setting device is default");
			logerSD("Setting device is default");

			fresult = settingLoadSave(setting_delete, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));

			if(fresult == FR_OK)
			{
				logerUART("SD-CARD: old file setting device is DELETED");
				logerSD("SD-CARD: old file setting device is DELETED");
			}

			fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));

			if(fresult == FR_OK)
			{
				logerUART("SD-CARD: new file setting device is CREATED");
				logerSD("SD-CARD: new file setting device is CREATED");
			}
		}
	}
	else if(fresult == FR_NO_FILE)
	{
	  logerUART("SD-CARD: no file setting");
	  logerSD("SD-CARD: no file setting");

	  settingDefaultValues(&settingDevice);

	  logerUART("Setting device is default");
	  logerSD("Setting device is default");

	  fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));

	  if(fresult == FR_OK)
	  {
		  logerUART("SD-CARD: new file setting device is CREATED");
		  logerSD("SD-CARD: new file setting device is CREATED");
	  }
	  else
	  {
		  sendUARTErrorSD(fresult);
	  }
	}
	else
	{
	  sendUARTErrorSD(fresult);

	  settingDefaultValues(&settingDevice);

	  logerUART("Reset setting device is default");
	}
  }
  else
  {
	  settingDefaultValues(&settingDevice);

	  logerUART("Reset setting device is default");
  }
  //-----------------------------------------

  counterSendVoltageValueUART 	= settingDevice.timeSendVoltageValueUART;
  counterSendVoltageValueSDCard = settingDevice.timeSendVoltageValueSDCard;
  counterNoPressedButton 		= settingDevice.timeNoPressedButton;
  counterChangeScreen 			= settingDevice.timeChangeScreen;

  lcdInit();
  lcdClrScr();

  if(settingDevice.triggerLCDBackLight)
  {
	  LCD_BACKLIGHT_ON();
	  logerUART("LCD backlight is ON");
	  if(triggerSDCardEnable)
	  {
		  logerSD("LCD backlight is ON");
	  }
  }
  else
  {
	  LCD_BACKLIGHT_OFF();
	  logerUART("LCD backlight is OFF");
	  if(triggerSDCardEnable)
	  {
		  logerSD("LCD backlight is OFF");
	  }
  }

  lcdPuts("Control voltage\n\t  V");
  lcdItos(VERSION);
  Delayms(1000);

  if(HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK)
  {
	  logerUART("ADC calibration is OK");
	  if(triggerSDCardEnable)
	  {
		  logerSD("ADC calibration is OK");
	  }
  }
  else
  {
	  logerUART("ADC calibration is BAD");
	  if(triggerSDCardEnable)
	  {
		  logerSD("ADC calibration is BAD");
	  }
  }

  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*) dataADC, ADCn) == HAL_OK)
  {
	  logerUART("Start ADC measuring");
	  if(triggerSDCardEnable)
	  {
		  logerSD("Start ADC measuring");
	  }
  }
  else
  {
	  logerUART("Not Start ADC measuring");
	  if(triggerSDCardEnable)
	  {
		  logerSD("Not Start ADC measuring");
	  }
  }

  if (settingDevice.triggerControlVoltage)
  {
	  logerUART("Control voltage is ON");
	  if(triggerSDCardEnable)
	  {
		  logerSD("Control voltage is ON");
	  }
  }
  else
  {
	  logerUART("Control voltage is OFF");
	  if(triggerSDCardEnable)
	  {
		  logerSD("Control voltage is OFF");
	  }
  }

  char buff[64];
  sprintf(buff, "Warning period is %d second", (settingDevice.periodWarning / 1000));
  logerUART(buff);
  if(triggerSDCardEnable)
  {
	  logerSD(buff);
  }

  // Створюємо таймери на функцію зворотнього виклику
  for (int i = 0; i < ADCn; ++i)
  {
	  settingDevice.voltageParam[i].pTimerWarning = TM_DELAY_TimerCreate(settingDevice.periodWarning, ONCE, SOMETIME, warningVoltage, &settingDevice.voltageParam[i]);
	  settingDevice.voltageParam[i].pTimerNormal 	= TM_DELAY_TimerCreate(settingDevice.periodWarning, ONCE, SOMETIME, normalVoltage, &settingDevice.voltageParam[i]);
  }

  pTimerCheckVoltage		= TM_DELAY_TimerCreate(PERIOD_CHECK_VOLTAGE, REPEAT, IMMEDIATELY, checkVoltage, NULL);
  pTimerScanKey				= TM_DELAY_TimerCreate(PERIOD_SCAN_KEY, REPEAT, IMMEDIATELY, scanKey, NULL);
  pTimerAlarmBeeper			= TM_DELAY_TimerCreate(PERIOD_ALARM, REPEAT, SOMETIME, alarmBeeper, NULL);
  pTimerAlarmLed			= TM_DELAY_TimerCreate(PERIOD_ALARM, REPEAT, SOMETIME, alarmLed, NULL);

  HAL_RTCEx_SetSecond_IT(&hrtc); // Запускаємо переривання від RTC раз на секунду

  lcdClrScr();

  /* Set up the default menu text write callback, and navigate to an absolute menu item entry. */
  Menu_SetGenericWriteCallback(Generic_Write);
  Menu_Navigate(&Menu_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(key != BUTTON_NOTHING)
	  {
		  buttonHandler();
	  }

	  if(settingDevice.triggerControlVoltage)
	  {
		  voltageAlarmHandler();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|LD_RED_Pin|CS_SD_CARD_Pin|LCD_BACKLIGHT_Pin 
                          |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin 
                          |LCD_RS_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON_RIGHT_Pin BUTTON_LEFT_Pin BUTTON_UP_Pin BUTTON_DOWN_Pin 
                           BUTTON_SELECT_Pin */
  GPIO_InitStruct.Pin = BUTTON_RIGHT_Pin|BUTTON_LEFT_Pin|BUTTON_UP_Pin|BUTTON_DOWN_Pin 
                          |BUTTON_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin LD_RED_Pin CS_SD_CARD_Pin LCD_BACKLIGHT_Pin 
                           LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin 
                           LCD_RS_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LD_RED_Pin|CS_SD_CARD_Pin|LCD_BACKLIGHT_Pin 
                          |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin 
                          |LCD_RS_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void	changeScreenDisplay(void)
{
	while(!showDisplay[currentNumberPage]->status)
	{
		if(currentNumberPage < numberPageOfScreen - 1)
		{
			currentNumberPage++;
		}
		else
		{
			currentNumberPage = 0;
		}
	}

	showDisplay[currentNumberPage]->showValue();

	if(currentNumberPage < numberPageOfScreen - 1)
	{
		currentNumberPage++;
	}
	else
	{
		currentNumberPage = 0;
	}
}

static void showCurrentTime(void)
{
	uint8_t aShowTime[16] = {0};
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	mRTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	mRTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	lcdClrScr();

	sprintf((char*)aShowTime,"\t  %02d:%02d",sTime.Hours, sTime.Minutes);
	lcdGoto(LCD_2nd_LINE, 0);
	lcdPuts((char*)aShowTime);

	sprintf((char*)aShowTime,"\t%02d/%02d/%02d", sDate.Date, sDate.Month, sDate.Year);
	lcdGoto(LCD_1st_LINE, 0);
	lcdPuts((char*)aShowTime);
}

static void	showVoltagePlus60(void)
{
	lcdClrScr();
	lcdClrBar();
	headingSingleVoltage("Voltage: +60V");
	showSolitaryMeasuredVoltage(0);
}

static void	showVoltageMinus60(void)
{
	lcdClrScr();
	lcdClrBar();
	headingSingleVoltage("Voltage: -60V");
	showSolitaryMeasuredVoltage(1);
}

static void	showVoltageMinus24(void)
{
	lcdClrScr();
	lcdClrBar();
	headingSingleVoltage("Voltage: -24V");
	showSolitaryMeasuredVoltage(2);
}

//Функція біп
static void beep(uint16_t tone, uint16_t time) //Функція приймає значення тону звука і тривалість звуку
{
	for (uint16_t i = 0; i < time; ++i)
	{
		BUZZER_GPIO_Port->BSRR = BUZZER_Pin;
		Delay(tone);
		BUZZER_GPIO_Port->BRR = BUZZER_Pin;
		Delay(tone);
	}

	BUZZER_GPIO_Port->BRR = BUZZER_Pin;
}

void HAL_RTCEx_RTCEventCallback (RTC_HandleTypeDef* hrtc)
{
	if(RTC_IT_SEC)
	{
		secondFlag = true;

		if(counterSendVoltageValueUART > 0)
		{
			counterSendVoltageValueUART--;
		}
		else
		{
			sendVoltageValueUART();
			counterSendVoltageValueUART = settingDevice.timeSendVoltageValueUART;
		}

		if(triggerSDCardEnable)
		{
			if(counterSendVoltageValueSDCard > 0)
			{
				counterSendVoltageValueSDCard--;
			}
			else
			{
				sendVoltageValueSDCard();
				counterSendVoltageValueSDCard = settingDevice.timeSendVoltageValueSDCard;
			}
		}

		if((!stateAlarmVoltage[0] && stateUserResponded[0])
		&& (!stateAlarmVoltage[1] && stateUserResponded[1])
		&& (!stateAlarmVoltage[2] && stateUserResponded[2]))
		{
			if(counterNoPressedButton > 0)
			{
				counterNoPressedButton--;
			}
			else
			{
				triggerNoPressedButton = true;
			}

			if(triggerNoPressedButton)
			{
				if(counterChangeScreen > 0)
				{
					counterChangeScreen--;
				}
				else
				{
					changeScreenDisplay();
					counterChangeScreen = settingDevice.timeChangeScreen;
				}
			}
			else
			{
				counterChangeScreen = settingDevice.timeChangeScreen;
			}
		}
		else
		{
			counterChangeScreen = settingDevice.timeChangeScreen;
		}
	}
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	if(triggerScanKey)
	{
		counterNoPressedButton	= settingDevice.timeNoPressedButton;
		triggerNoPressedButton 	= false;

		if(GPIO_Pin == BUTTON_SELECT_Pin)
		{
			key = BUTTON_SELECT;
		}
		else if (GPIO_Pin == BUTTON_LEFT_Pin)
		{
			key = BUTTON_LEFT;
		}
		else if (GPIO_Pin == BUTTON_UP_Pin)
		{
			key = BUTTON_UP;
		}
		else if (GPIO_Pin == BUTTON_DOWN_Pin)
		{
			key = BUTTON_DOWN;
		}
		else if (GPIO_Pin == BUTTON_RIGHT_Pin)
		{
			key = BUTTON_RIGHT;
		}

		triggerScanKey = false;
	}
}

static void scanKey(struct _TM_DELAY_Timer_t* my_timer, void *parameters)
{
	triggerScanKey = true;
}

static void alarmBeeper(struct _TM_DELAY_Timer_t* my_timer, void *parameters)
{
	beep(500, 50);
}

static void alarmLed(struct _TM_DELAY_Timer_t* my_timer, void *parameters)
{
	LED_RED_TOGGLE();
}

static void buttonHandler(void)
{
	beep(800,10);

	switch(key)
	{
		case BUTTON_LEFT:
		{
			Menu_Navigate(MENU_PARENT);
		}
			break;
		case BUTTON_UP:
		{
			Menu_Navigate(MENU_PREVIOUS);
		}
			break;
		case BUTTON_DOWN:
		{
			Menu_Navigate(MENU_NEXT);
		}
			break;
		case BUTTON_RIGHT:
		{
			Menu_Navigate(MENU_CHILD);
		}
			break;
		case BUTTON_SELECT:
		{
			Menu_EnterCurrentItem();
		}
			break;
		default:
			break;
	}

	key = BUTTON_NOTHING;
}

static void checkVoltage(struct _TM_DELAY_Timer_t* my_timer, void *parameters)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) dataADC, ADCn);

	if(settingDevice.triggerControlVoltage)
	{
		for (int i = 0; i < ADCn; i++)
		{
			if(settingDevice.voltageParam[i].voltage > settingDevice.voltageParam[i].max
			|| settingDevice.voltageParam[i].voltage < settingDevice.voltageParam[i].min)
			{
				if(!triggerAlarmVoltage[i])
				{
					if(settingDevice.voltageParam[i].voltage > settingDevice.voltageParam[i].max)
					{
						settingDevice.voltageParam[i].errorVoltage = max;
					}
					else if(settingDevice.voltageParam[i].voltage < settingDevice.voltageParam[i].min)
					{
						settingDevice.voltageParam[i].errorVoltage = min;
					}

					triggerAlarmVoltage[i] = true;
					settingDevice.voltageParam[i].pTimerWarning = TM_DELAY_TimerStart(settingDevice.voltageParam[i].pTimerWarning);
					settingDevice.voltageParam[i].pTimerNormal = TM_DELAY_TimerStop(settingDevice.voltageParam[i].pTimerNormal);
					settingDevice.voltageParam[i].pTimerNormal = TM_DELAY_TimerReset(settingDevice.voltageParam[i].pTimerNormal);
				}
			}
			else if(settingDevice.voltageParam[i].voltage <= settingDevice.voltageParam[i].max
				 || settingDevice.voltageParam[i].voltage >= settingDevice.voltageParam[i].min)
			{
				if(triggerAlarmVoltage[i])
				{
					settingDevice.voltageParam[i].errorVoltage = normal;

					triggerAlarmVoltage[i] = false;
					settingDevice.voltageParam[i].pTimerWarning = TM_DELAY_TimerStop(settingDevice.voltageParam[i].pTimerWarning);
					settingDevice.voltageParam[i].pTimerWarning = TM_DELAY_TimerReset(settingDevice.voltageParam[i].pTimerWarning);
					settingDevice.voltageParam[i].pTimerNormal = TM_DELAY_TimerStart(settingDevice.voltageParam[i].pTimerNormal);
				}
			}
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		settingDevice.voltageParam[0].voltage = (float) dataADC[0] * settingDevice.voltageParam[0].maxMeasured / (float) ADC_MAX;
		settingDevice.voltageParam[1].voltage = settingDevice.voltageParam[1].maxMeasured - ((float) dataADC[1] * settingDevice.voltageParam[1].maxMeasured / (float) ADC_MAX);
		settingDevice.voltageParam[2].voltage = settingDevice.voltageParam[2].maxMeasured - ((float) dataADC[2] * settingDevice.voltageParam[2].maxMeasured / (float) ADC_MAX);
	}
}

static void showMeasuredAllVoltage(void)
{
	lcdGoto(LCD_1st_LINE, 0);
	lcdPuts("U1=+");
	lcdFtos(settingDevice.voltageParam[0].voltage, 1);
	lcdGoto(LCD_1st_LINE, 8);
	lcdPuts("U2=-");
	lcdFtos(settingDevice.voltageParam[1].voltage, 1);
	lcdGoto(LCD_2nd_LINE, 0);
	lcdPuts("U3=-");
	lcdFtos(settingDevice.voltageParam[2].voltage, 1);
}

static void showSolitaryMeasuredVoltage(const uint8_t i)
{
	// встановлення позиції курсора
	lcdGoto(PROGRESS_BAR_LINE, PROGRESS_BAR_WIDTH + 1);
	// друкування показів в чітко заданій позиції
	lcdFtos(settingDevice.voltageParam[i].voltage, 1);
	// показуємо поточний стан індикатора виконання
	//lcdDrawBar(settingDevice.voltageParam[i].voltage * PROGRESS_BAR_MAX_LOAD / 100);
	if(settingDevice.voltageParam[i].voltage <= settingDevice.voltageParam[i].min)
	{
		lcdDrawBar(0);
	}
	else if(settingDevice.voltageParam[i].voltage >= settingDevice.voltageParam[i].max)
	{
		lcdDrawBar(PROGRESS_BAR_MAX_LOAD);
	}
	else
	{
		lcdDrawBar(((settingDevice.voltageParam[i].voltage - settingDevice.voltageParam[i].min) * PROGRESS_BAR_MAX_LOAD) / (settingDevice.voltageParam[i].max - settingDevice.voltageParam[i].min));
	}
}

static void headingSingleVoltage(const char* info)
{
	lcdClrScr();
	lcdPuts(info);
	// встановлюємо позицію курсору
	lcdGoto(PROGRESS_BAR_LINE, PROGRESS_BAR_WIDTH);
	// друкуємо текст
	lcdPuts("[    ]");
	// встановлення позиції курсора
	lcdGoto(PROGRESS_BAR_LINE, PROGRESS_BAR_WIDTH + 1);
}

static void warningVoltage(struct _TM_DELAY_Timer_t* my_timer, void *parameters)
{
	VoltageStruct_TypeDef *voltageStruct = (VoltageStruct_TypeDef*) parameters;

	if(!stateAlarmVoltage[voltageStruct->id])
	{
		stateAlarmVoltage[voltageStruct->id] = true;
		stateUserResponded[voltageStruct->id] = false;

		pTimerAlarmBeeper = TM_DELAY_TimerStart(pTimerAlarmBeeper);
		pTimerAlarmLed = TM_DELAY_TimerStart(pTimerAlarmLed);

		char bufStr[64];
		sprintf(bufStr, "WARNING! The voltage %s = %.1fV", voltageStruct->name, voltageStruct->voltage);

		if(voltageStruct->errorVoltage == max)
		{
			strcat(bufStr, " This more maximum!");
		}
		else if(voltageStruct->errorVoltage == min)
		{
			strcat(bufStr, " This less minimum");
		}

		logerUART(bufStr);
		if(triggerSDCardEnable)
		{
			logerSD(bufStr);
		}

		key = BUTTON_LEFT;
	}
}

static void normalVoltage(struct _TM_DELAY_Timer_t* my_timer, void *parameters)
{
	VoltageStruct_TypeDef *voltageStruct = (VoltageStruct_TypeDef*) parameters;

	if(stateAlarmVoltage[voltageStruct->id])
	{
		stateAlarmVoltage[voltageStruct->id] = false;

		char bufStr[64];
		sprintf(bufStr, "Voltage %s = %.1fV This is normal", voltageStruct->name, voltageStruct->voltage);

		logerUART(bufStr);
		if(triggerSDCardEnable)
		{
			logerSD(bufStr);
		}

		if(!stateAlarmVoltage[0] && !stateAlarmVoltage[1] && !stateAlarmVoltage[2])
		{
			pTimerAlarmLed = TM_DELAY_TimerStop(pTimerAlarmLed);
			LED_RED_OFF();
		}
	}
}

static void voltageAlarmHandler(void)
{
	for (int i = 0; i < ADCn; ++i)
	{
		if(stateAlarmVoltage[i] && !stateUserResponded[i])
		{
			lcdClrScr();
			lcdGoto(LCD_1st_LINE, 0);
			lcdPuts("WARNING voltage!");
			lcdGoto(LCD_2nd_LINE, 0);
			lcdPuts(settingDevice.voltageParam[i].name);

			if(settingDevice.voltageParam[i].errorVoltage == max)
			{
				lcdPuts(" > maximum");
			}
			else if(settingDevice.voltageParam[i].errorVoltage == min)
			{
				lcdPuts(" < minimum");
			}

			while(key != BUTTON_SELECT)
			{
				//TODO: очікуємо натискання кнопки
			}

			stateUserResponded[i] = true;

			pTimerAlarmBeeper = TM_DELAY_TimerStop(pTimerAlarmBeeper);

			char str[64];
			sprintf(str, "The user reacted to the event %s", settingDevice.voltageParam[i].name);
			logerUART(str);
			if(triggerSDCardEnable)
			{
				logerSD(str);
			}

			key = BUTTON_NOTHING;
			beep(800, 10);
			lcdClrScr();

			switch (i)
			{
				case 0:
					Level2Item1_2_Enter();
					break;
				case 1:
					Level2Item1_3_Enter();
					break;
				case 2:
					Level2Item1_4_Enter();
					break;
				default:
					break;
			}
		}
	}
}

static void settingDefaultValues(void *settings)
{
	settingDeviceTypeDef* setting = (settingDeviceTypeDef*) settings;

	setting->triggerControlVoltage 				= false;
	setting->triggerLCDBackLight 				= false;
	setting->periodWarning 						= PERIOD_WARNING;
	setting->timeChangeScreen 					= TIME_CHANGE_SCREEN;
	setting->timeNoPressedButton 				= TIME_NO_PRESSED_BUTTON;
	setting->timeSendVoltageValueUART 			= TIME_SEND_VOLTAGE_VALUE_UART;
	setting->timeSendVoltageValueSDCard 		= TIME_SEND_VOLTAGE_VALUE_SD_CARD;
	setting->salt 								= SD_SALT;

	//+60V
	setting->voltageParam[0].id					= 0;
	setting->voltageParam[0].name 				= "+60";
	setting->voltageParam[0].max 				= 66.6;
	setting->voltageParam[0].min 				= 62.5;
	setting->voltageParam[0].maxMeasured		= 76.2;
	//-60V
	setting->voltageParam[1].id					= 1;
	setting->voltageParam[1].name 				= "-60";
	setting->voltageParam[1].max 				= 66.6;
	setting->voltageParam[1].min 				= 62.5;
	setting->voltageParam[1].maxMeasured		= 73.0;
	//-24V
	setting->voltageParam[2].id					= 2;
	setting->voltageParam[2].name 				= "-24";
	setting->voltageParam[2].max 				= 28;
	setting->voltageParam[2].min 				= 25.1;
	setting->voltageParam[2].maxMeasured		= 39.6;
}

static FRESULT settingLoadSave(enum setting_op mode, const char filename[], void* settings, uint32_t size)
{
	settingDeviceTypeDef* setting = (settingDeviceTypeDef*) settings;

	static FATFS g_sFatFs;
	FRESULT fresult;
	FIL file;
	UINT bytes;

	fresult = f_mount(&g_sFatFs, "0:", 0);	//mount SD card

	if(fresult != FR_OK)
	{
		return fresult;
	}

	switch(mode)
	{
	case setting_save:
		fresult = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);	//open file on SD card
		break;
	case setting_load:
		fresult = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);	//open file on SD card
		break;
	case setting_delete:
		fresult = f_unlink(filename);
		break;
	default:
		break;
	}

	if(fresult != FR_OK)
	{
		return fresult;
	}

	switch(mode)
	{
	case setting_save:
		fresult = f_write(&file, setting, size, &bytes);	//write data to the file
		break;
	case setting_load:
		fresult = f_read(&file, setting, size, &bytes);		//read data from a file
		break;
	default:
		break;
	}

	if(fresult != FR_OK)
	{
		f_close(&file);
		return fresult;
	}

	if(mode != setting_delete)
	{
		fresult = f_close(&file);
	}

	return fresult;
}

static FRESULT logerSD(const char string[])
{
	char bufferStr[64];
	char filename[14];
	int len;
	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;
	mRTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	mRTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf(filename,"%.2d%.2d%.2d.log", (2000 + sDate.Year), sDate.Month, sDate.Date);
	len = sprintf(bufferStr, "%.2d:%.2d:%.2d - %s\r\n", sTime.Hours, sTime.Minutes, sTime.Seconds, string);	//generate some string

	static FATFS g_sFatFs;
	FRESULT fresult;
	FIL file;
	UINT bytes_written;

	fresult = f_mount(&g_sFatFs, "0:", 0);							//mount SD card

	if(fresult != FR_OK)
	{
		return fresult;
	}

	fresult = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);	//open file on SD card

	if(fresult != FR_OK)
	{
		return fresult;
	}

	fresult = f_lseek(&file, file.fsize);							//go to the end of the file

	if(fresult != FR_OK)
	{
		return fresult;
	}

	fresult = f_write(&file, bufferStr, len, &bytes_written);		//write data to the file

	if(fresult != FR_OK)
	{
		f_close(&file);
		return fresult;
	}

	fresult = f_close(&file);

	return fresult;
}

static void sendUARTErrorSD(FRESULT fresult)
{
	switch (fresult)
	{
		case FR_INVALID_DRIVE:
			logerUART("SD-CARD: invalid drive");
			break;
		case FR_DISK_ERR:
			logerUART("SD-CARD: disk error");
			break;
		case FR_NOT_READY:
			logerUART("SD-CARD: not ready");
			break;
		case FR_NO_FILE:
			logerUART("SD-CARD: no file");
			break;
		case FR_INVALID_NAME:
			logerUART("SD-CARD: invalid name");
			break;
		case FR_WRITE_PROTECTED:
			logerUART("SD-CARD: write protected");
			break;
		case FR_NO_FILESYSTEM:
			logerUART("SD-CARD: no file system");
			break;
		case FR_INVALID_OBJECT:
			logerUART("SD-CARD: invalid object");
			break;
		default:
		logerUART("SD-CARD: any other error");
		break;
	}
}

static void logerUART(const char string[])
{
	char bufferStr[64];

	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;
	mRTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	mRTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf(bufferStr, "%.2d/%.2d/%.2d %.2d:%.2d:%.2d - %s\r\n",
			sDate.Date, sDate.Month, sDate.Year,
			sTime.Hours, sTime.Minutes, sTime.Seconds,
			string);
	UART_SendStr(bufferStr);
}

static void sendVoltageValueUART(void)
{
	char strBuff[64];
	for (int i = 0; i < ADCn; ++i)
	{
		sprintf(strBuff, "Battery %s = %.1f volt", settingDevice.voltageParam[i].name, settingDevice.voltageParam[i].voltage);
		logerUART(strBuff);
	}
}

static void sendVoltageValueSDCard(void)
{
	char strBuff[64];
	for (int i = 0; i < ADCn; ++i)
	{
		sprintf(strBuff, "Battery %s = %.1f volt", settingDevice.voltageParam[i].name, settingDevice.voltageParam[i].voltage);
		logerSD(strBuff);
	}
}

static void printSettingMenu(const char* text)
{
	lcdGoto(LCD_1st_LINE, 0);
	lcdPuts(text);
	lcdGoto(LCD_2nd_LINE, 4);
	lcdPutc(0xD9);
	lcdGoto(LCD_2nd_LINE, 11);
	lcdPutc(0xDA);
}

static void settingPeriodSecond(uint16_t* value, uint8_t dig, uint16_t max, uint16_t min, const char* str, Menu_Item_t* const menu)
{
	lcdClrScr();

	printSettingMenu("Period seconds:");

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdNtos(*value, dig);

		switch(key)
		{
			case BUTTON_UP:
				if(*value < max)
				{
					*value += 1;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				if(*value > min)
				{
					*value -= 1 ;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	char buff[64];
	sprintf(buff, "%s %d second", str, *value);
	logerUART(buff);

	if(triggerSDCardEnable)
	{
		logerSD(buff);

		FRESULT fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));
		if(fresult != FR_OK)
		{
			sendUARTErrorSD(fresult);
		}
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(menu);
}

static void	settingVoltageMinMax(float* value, float max, float min, const char* strInfo, const char* strMsg, Menu_Item_t* const menu)
{
	lcdClrScr();

	printSettingMenu(strInfo);

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdFtos(*value, 1);

		switch(key)
		{
			case BUTTON_UP:
				if(*value < max)
				{
					*value += 0.1;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				if(*value > min)
				{
					*value -= 0.1 ;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	char buff[64];
	sprintf(buff, "%s %.1f volt", strMsg, *value);
	logerUART(buff);

	if(triggerSDCardEnable)
	{
		logerSD(buff);

		FRESULT fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));
		if(fresult != FR_OK)
		{
			sendUARTErrorSD(fresult);
		}
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(menu);
}

static void	settingMaxVoltageMeasured(float* value, float* currentValue, const char* strInfo, const char* strMsg, Menu_Item_t* const menu)
{
	lcdClrScr();

	printSettingMenu(strInfo);

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_1st_LINE, 12);
		lcdFtos(*currentValue, 1);
		lcdGoto(LCD_2nd_LINE, 6);
		lcdFtos(*value, 1);

		switch(key)
		{
			case BUTTON_UP:
				{
					*value += 0.1;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				{
					*value -= 0.1 ;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	char buff[64];
	sprintf(buff, "%s %.1f volt", strMsg, *value);
	logerUART(buff);

	if(triggerSDCardEnable)
	{
		logerSD(buff);

		FRESULT fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));
		if(fresult != FR_OK)
		{
			sendUARTErrorSD(fresult);
		}
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(menu);
}
//-------------------------Menu Functions Start----------------------------------------------------
static void Generic_Write(const char* Text)
{
	if (Text)
	{
		lcdClrScr();
		lcdPuts(Text);
	}
}

static void Level1Item2_Enter(void)
{
	lcdClrScr();

	key = BUTTON_NOTHING;

	while(key == BUTTON_NOTHING && !triggerNoPressedButton)
	{
		if(secondFlag)
		{
			uint8_t aShowTime[16] = {0};
			RTC_TimeTypeDef sTime;
			RTC_DateTypeDef sDate;

			mRTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			mRTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			sprintf((char*)aShowTime,"\t%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);
			lcdGoto(LCD_2nd_LINE, 0);
			lcdPuts((char*)aShowTime);

			sprintf((char*)aShowTime,"\t%02d/%02d/%02d", sDate.Date, sDate.Month, sDate.Year);
			lcdGoto(LCD_1st_LINE, 0);
			lcdPuts((char*)aShowTime);

			secondFlag = false;
		}
	}

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	key = BUTTON_NOTHING;
	lcdClrScr();
	Menu_Navigate(&Menu_2);
}

static void Level2Item1_1_Enter(void)
{
	lcdClrScr();

	key = BUTTON_NOTHING;
	while(key == BUTTON_NOTHING && !triggerNoPressedButton)
	{
		showMeasuredAllVoltage();
	}

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	key = BUTTON_NOTHING;
	lcdClrScr();
	Menu_Navigate(&Menu_1_1);
}

static void Level2Item1_2_Enter(void)
{
	lcdClrScr();
	headingSingleVoltage("Voltage: +60V");

	key = BUTTON_NOTHING;
	while(key == BUTTON_NOTHING && !triggerNoPressedButton)
	{
		showSolitaryMeasuredVoltage(0);
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrBar();
	lcdClrScr();
	Menu_Navigate(&Menu_1_2);
}

static void Level2Item1_3_Enter(void)
{
	lcdClrScr();
	headingSingleVoltage("Voltage: -60V");

	key = BUTTON_NOTHING;
	while(key == BUTTON_NOTHING && !triggerNoPressedButton)
	{
		showSolitaryMeasuredVoltage(1);
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	lcdClrBar();
	Menu_Navigate(&Menu_1_3);
}

static void Level2Item1_4_Enter(void)
{
	lcdClrScr();
	headingSingleVoltage("Voltage: -24V");

	key = BUTTON_NOTHING;
	while(key == BUTTON_NOTHING && !triggerNoPressedButton)
	{
		showSolitaryMeasuredVoltage(2);
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	lcdClrBar();
	Menu_Navigate(&Menu_1_4);
}

static void Level3Item3_1_1_Enter(void)
{
	lcdClrScr();

	printSettingMenu("\tHours");

	RTC_TimeTypeDef sTime;
	mRTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdNtos(sTime.Hours, 2);

		switch(key)
		{
			case BUTTON_UP:
				if(sTime.Hours < 23)
				{
					sTime.Hours++;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				if(sTime.Hours > 0)
				{
					sTime.Hours--;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	if (mRTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(&Menu_3_1_1);
}

static void Level3Item3_1_2_Enter(void)
{
	lcdClrScr();

	printSettingMenu("\tMinutes");

	RTC_TimeTypeDef sTime;
	mRTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdNtos(sTime.Minutes, 2);

		switch(key)
		{
			case BUTTON_UP:
				if(sTime.Minutes < 59)
				{
					sTime.Minutes++;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				if(sTime.Minutes > 0)
				{
					sTime.Minutes--;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	if (mRTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(&Menu_3_1_2);
}

static void Level3Item3_1_3_Enter(void)
{
	lcdClrScr();

	printSettingMenu("\tSeconds");

	RTC_TimeTypeDef sTime;
	mRTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdNtos(SECOND(&hrtc, RTC_FORMAT_BIN), 2);

		switch(key)
		{
			case BUTTON_UP:
				{
					sTime.Seconds = 0;
					sTime.Minutes++;
					key = BUTTON_NOTHING;
					beep(800, 10);

					if (mRTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}
				}
				break;
			case BUTTON_DOWN:
				{
					sTime.Seconds = 0;
					key = BUTTON_NOTHING;
					beep(800, 10);

					if (mRTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
					{
						Error_Handler();
					}
				}
				break;
			default:
				break;
		}
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(&Menu_3_1_3);
}

static void Level3Item3_1_4_Enter(void)
{
	lcdClrScr();

	printSettingMenu("\tDate");

	RTC_DateTypeDef sDate;

	mRTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdNtos(sDate.Date, 2);

		switch(key)
		{
			case BUTTON_UP:
				if(sDate.Date < 31)
				{
					sDate.Date++;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				if(sDate.Date > 1)
				{
					sDate.Date--;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	if (mRTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(&Menu_3_1_4);
}

static void Level3Item3_1_5_Enter(void)
{
	lcdClrScr();

	printSettingMenu("\tMonth");

	RTC_DateTypeDef sDate;
	mRTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdNtos(sDate.Month, 2);

		switch(key)
		{
			case BUTTON_UP:
				if(sDate.Month < 12)
				{
					sDate.Month++;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				if(sDate.Month > 1)
				{
					sDate.Month--;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	if (mRTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(&Menu_3_1_5);
}

static void Level3Item3_1_6_Enter(void)
{
	lcdClrScr();

	printSettingMenu("\tYear");

	RTC_DateTypeDef sDate;
	mRTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdNtos(sDate.Year, 2);

		switch(key)
		{
			case BUTTON_UP:
				if(sDate.Year < 99)
				{
					sDate.Year++;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				if(sDate.Year > 0)
				{
					sDate.Year--;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	if (mRTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(&Menu_3_1_6);
}

static void Level3Item3_2_1_Select(void)
{
	if(!settingDevice.triggerLCDBackLight)
	{
		LCD_BACKLIGHT_ON();
		settingDevice.triggerLCDBackLight = true;

		logerUART("LCD backlight is ON");

		if(triggerSDCardEnable)
		{
			logerSD("LCD backlight is ON");

			FRESULT fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));

			if(fresult != FR_OK)
			{
				sendUARTErrorSD(fresult);
			}
		}
	}
}

static void Level3Item3_2_2_Select(void)
{
	if(settingDevice.triggerLCDBackLight)
	{
		LCD_BACKLIGHT_OFF();
		settingDevice.triggerLCDBackLight = false;

		logerUART("LCD backlight is OFF");

		if(triggerSDCardEnable)
		{
			logerSD("LCD backlight is OFF");

			FRESULT fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));

			if(fresult != FR_OK)
			{
				sendUARTErrorSD(fresult);
			}
		}
	}
}

static void Level3Item3_3_1_Enter(void)
{
	settingPeriodSecond(&settingDevice.timeNoPressedButton, 3,255, 1, "The period no pressed button", &Menu_3_3_1);
}

static void Level3Item3_3_2_Enter(void)
{
	settingPeriodSecond(&settingDevice.timeChangeScreen, 2, 10, 1, "The period change screen", &Menu_3_3_2);
}

static void Level3Item3_4_5_Enter(void)
{
	lcdClrScr();

	printSettingMenu("Period seconds:");

	while(key != BUTTON_LEFT && !triggerNoPressedButton)
	{
		lcdGoto(LCD_2nd_LINE, 6);
		lcdNtos(settingDevice.periodWarning / 1000, 3);

		switch(key)
		{
			case BUTTON_UP:
				if(settingDevice.periodWarning / 1000 < 60)
				{
					settingDevice.periodWarning += 1000;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			case BUTTON_DOWN:
				if(settingDevice.periodWarning / 1000 > 1)
				{
					settingDevice.periodWarning -= 1000 ;
					key = BUTTON_NOTHING;
					beep(800, 10);
				}
				break;
			default:
				break;
		}
	}

	for (int i = 0; i < ADCn; ++i)
	{
		settingDevice.voltageParam[i].pTimerWarning = TM_DELAY_TimerAutoReloadValue(settingDevice.voltageParam[i].pTimerWarning, settingDevice.periodWarning);
		settingDevice.voltageParam[i].pTimerNormal = TM_DELAY_TimerAutoReloadValue(settingDevice.voltageParam[i].pTimerNormal, settingDevice.periodWarning);
	}

	char buff[64];
	sprintf(buff, "Warning period is %d second", (settingDevice.periodWarning / 1000));
	logerUART(buff);

	if(triggerSDCardEnable)
	{
		logerSD(buff);

		FRESULT fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));
		if(fresult != FR_OK)
		{
			sendUARTErrorSD(fresult);
		}
	}

	key = BUTTON_NOTHING;

	if(!triggerNoPressedButton)
	{
		beep(800, 10);
	}

	lcdClrScr();
	Menu_Navigate(&Menu_3_4_5);
}

static void Level3Item3_4_6_Enter(void)
{
	settingPeriodSecond(&settingDevice.timeSendVoltageValueUART, 3, 255, 1, "The period for sending value UART", &Menu_3_4_6);
}

static void Level3Item3_4_7_Enter(void)
{
	settingPeriodSecond(&settingDevice.timeSendVoltageValueSDCard, 4, 7200, 1, "The period for sending value SD-Card", &Menu_3_4_7);
}

static void Level4Item3_4_1_1_Enter(void)
{
	settingVoltageMinMax(&settingDevice.voltageParam[0].max, 70.0, 60.0, "  Set Max +60:", "Maximum limit +60 =", &Menu_3_4_1_1);
}

static void Level4Item3_4_1_2_Enter(void)
{
	settingVoltageMinMax(&settingDevice.voltageParam[0].min, 70.0, 60.0, "  Set Min +60:", "Minimum limit +60 =", &Menu_3_4_1_2);
}

static void Level4Item3_4_1_3_Enter(void)
{
	settingMaxVoltageMeasured(&settingDevice.voltageParam[0].maxMeasured,
							  &settingDevice.voltageParam[0].voltage,
							  "Current +60:",
							  "Maximum measured voltage +60 =",
							  &Menu_3_4_1_3);
}

static void Level4Item3_4_2_1_Enter(void)
{
	settingVoltageMinMax(&settingDevice.voltageParam[1].max,
						 70.0,
						 60.0,
						 "  Set Max -60:",
						 "Maximum limit -60 =",
						 &Menu_3_4_2_1);
}

static void Level4Item3_4_2_2_Enter(void)
{
	settingVoltageMinMax(&settingDevice.voltageParam[1].min,
						 70.0,
						 60.0,
						 "  Set Min -60:",
						 "Minimum limit -60 =",
						 &Menu_3_4_2_2);
}

static void Level4Item3_4_2_3_Enter(void)
{
	settingMaxVoltageMeasured(&settingDevice.voltageParam[1].maxMeasured,
							  &settingDevice.voltageParam[1].voltage,
							  "Current -60:",
							  "Maximum measured voltage -60 =",
							  &Menu_3_4_2_3);
}

static void Level4Item3_4_3_1_Enter(void)
{
	settingVoltageMinMax(&settingDevice.voltageParam[2].max,
						 30.0,
						 20.0,
						 "  Set Max -24:",
						 "Maximum limit -24 =",
						 &Menu_3_4_3_1);
}

static void Level4Item3_4_3_2_Enter(void)
{
	settingVoltageMinMax(&settingDevice.voltageParam[2].min,
						 30.0,
						 20.0,
						 "  Set Min -24:",
						 "Minimum limit -24 =",
						 &Menu_3_4_3_2);
}

static void Level4Item3_4_3_3_Enter(void)
{
	settingMaxVoltageMeasured(&settingDevice.voltageParam[2].maxMeasured,
							  &settingDevice.voltageParam[2].voltage,
							  "Current -24:",
							  "Maximum measured voltage -24 =",
							  &Menu_3_4_3_3);
}

static void Level4Item3_4_4_1_Select(void)
{
	if(settingDevice.triggerControlVoltage)
	{
		settingDevice.triggerControlVoltage = false;

		for (int i = 0; i < ADCn; ++i)
		{
			triggerAlarmVoltage[i] 	= false;
			stateAlarmVoltage[i] 	= false;
		}

		pTimerAlarmLed = TM_DELAY_TimerStop(pTimerAlarmLed);
		LED_RED_OFF();

		logerUART("Control voltage is OFF");

		if(triggerSDCardEnable)
		{
			logerSD("Control voltage is OFF");

			FRESULT fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));

			if(fresult != FR_OK)
			{
				sendUARTErrorSD(fresult);
			}
		}
	}
}

static void Level4Item3_4_4_2_Select(void)
{
	if(!settingDevice.triggerControlVoltage)
	{
		settingDevice.triggerControlVoltage = true;

		logerUART("Control voltage is ON");

		if(triggerSDCardEnable)
		{
			logerSD("Control voltage is ON");

			FRESULT fresult = settingLoadSave(setting_save, SETTING_FILE_NAME, &settingDevice, sizeof(settingDevice));

			if(fresult != FR_OK)
			{
				sendUARTErrorSD(fresult);
			}
		}
	}
}
//-------------------------Menu Functions End----------------------------------------------------
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	//як помилка, блимаємо червоним світлодіодом
	LED_RED_TOGGLE();
	HAL_Delay(50);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
