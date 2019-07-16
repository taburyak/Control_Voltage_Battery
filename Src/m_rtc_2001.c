/*
 * m_rtc_2001.c
 *
 *  Created on: 30 квіт. 2018 р.
 *      Author: Andriy
 */

#include <m_rtc_2001.h>

#define SECOND_A_DAY 86400
#define JD0 2451911 // зміщення днів з 24 жовтня 4714 р. до н.е. по 01 січня 2001 ПН
#define RTC_ALARM_RESETVALUE_REGISTER    (uint16_t)0xFFFF
#define RTC_ALARM_RESETVALUE             0xFFFFFFFFU

static uint32_t				lastDay = 0;

static uint32_t 			mRTC_ReadCounter(RTC_HandleTypeDef* hrtc);
static HAL_StatusTypeDef 	mRTC_WriteCounter(RTC_HandleTypeDef* hrtc, uint32_t TimeCounter);
static uint8_t 				mRTC_WeekDayNum(uint32_t nYear, uint8_t nMonth, uint8_t nDay);
static HAL_StatusTypeDef 	mRTC_EnterInitMode(RTC_HandleTypeDef* hrtc);
static HAL_StatusTypeDef 	mRTC_ExitInitMode(RTC_HandleTypeDef* hrtc);
static uint32_t 			mRTC_GetSecondsInDate(RTC_HandleTypeDef* hrtc);
static uint8_t 				mRTC_ByteToBcd2(uint8_t Value);
static uint8_t 				mRTC_Bcd2ToByte(uint8_t Value);
static HAL_StatusTypeDef  	mRTC_WriteAlarmCounter(RTC_HandleTypeDef* hrtc, uint32_t AlarmCounter);
static uint32_t 			mRTC_ReadAlarmCounter(RTC_HandleTypeDef* hrtc);

void mRTC_Begin(RTC_HandleTypeDef* hrtc)
{
	RTC_TimeTypeDef stime = {0U};
	mRTC_GetTime(hrtc, &stime, RTC_FORMAT_BIN);
}

uint8_t mRTC_GetSecond(RTC_HandleTypeDef* hrtc, uint32_t Format)
{
	/* Check input parameters */
	if(hrtc == NULL)
	{
		Error_Handler();
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));

	/* Check the input parameters format */
	if(Format != RTC_FORMAT_BIN)
	{
		/* Convert the time structure parameters to BCD format */
		return (uint8_t)mRTC_ByteToBcd2(mRTC_ReadCounter(hrtc) % 60);
	}
	else
	{
		return mRTC_ReadCounter(hrtc) % 60;
	}
}

uint8_t mRTC_GetMinute(RTC_HandleTypeDef* hrtc, uint32_t Format)
{
	/* Check input parameters */
	if(hrtc == NULL)
	{
		Error_Handler();
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));

	/* Check the input parameters format */
	if(Format != RTC_FORMAT_BIN)
	{
		/* Convert the time structure parameters to BCD format */
		return (uint8_t)mRTC_ByteToBcd2((mRTC_ReadCounter(hrtc) / 60) % 60);
	}
	else
	{
		return (mRTC_ReadCounter(hrtc) / 60) % 60;
	}
}

uint8_t mRTC_GetHour(RTC_HandleTypeDef* hrtc, uint32_t Format)
{
	/* Check input parameters */
	if(hrtc == NULL)
	{
		Error_Handler();
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));

	/* Check the input parameters format */
	if(Format != RTC_FORMAT_BIN)
	{
		/* Convert the time structure parameters to BCD format */
		return (uint8_t)mRTC_ByteToBcd2((mRTC_ReadCounter(hrtc) / 3600) % 24);
	}
	else
	{
		return (mRTC_ReadCounter(hrtc) / 3600) % 24;
	}
}

uint8_t	mRTC_GetWeekDay(RTC_HandleTypeDef* hrtc)
{
	/* Check input parameters */
	if(hrtc == NULL)
	{
		Error_Handler();
	}

	return hrtc->DateToUpdate.WeekDay;
}

uint8_t mRTC_GetDay(RTC_HandleTypeDef* hrtc, uint32_t Format)
{
	/* Check input parameters */
	if(hrtc == NULL)
	{
		Error_Handler();
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));

	/* Check the input parameters format */
	if(Format != RTC_FORMAT_BIN)
	{
		/* Convert the date structure parameters to BCD format */
		return (uint8_t)mRTC_ByteToBcd2(hrtc->DateToUpdate.Date);
	}
	else
	{
		return hrtc->DateToUpdate.Date;
	}
}

uint8_t mRTC_GetMonth(RTC_HandleTypeDef* hrtc, uint32_t Format)
{
	/* Check input parameters */
	if(hrtc == NULL)
	{
		Error_Handler();
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));

	/* Check the input parameters format */
	if(Format != RTC_FORMAT_BIN)
	{
		/* Convert the date structure parameters to BCD format */
		return (uint8_t)mRTC_ByteToBcd2(hrtc->DateToUpdate.Month);
	}
	else
	{
		return hrtc->DateToUpdate.Month;
	}
}

uint16_t mRTC_GetYear(RTC_HandleTypeDef* hrtc, uint32_t Format)
{
	/* Check input parameters */
	if(hrtc == NULL)
	{
		Error_Handler();
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));

	/* Check the input parameters format */
	if(Format != RTC_FORMAT_BIN)
	{
		/* Convert the date structure parameters to BCD format */
		return (uint8_t)mRTC_ByteToBcd2(hrtc->DateToUpdate.Year);
	}
	else
	{
		return hrtc->DateToUpdate.Year;
	}
}

HAL_StatusTypeDef mRTC_GetTime(RTC_HandleTypeDef* hrtc, RTC_TimeTypeDef* sTime, uint32_t Format)
{
	//uint32_t counter_time = 0U, currentDay = 0U;

	/* Check input parameters */
	if((hrtc == NULL) || (sTime == NULL))
	{
	   return HAL_ERROR;
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));

	/* Check if counter overflow occurred */
	if (__HAL_RTC_OVERFLOW_GET_FLAG(hrtc, RTC_FLAG_OW))
	{
	    return HAL_ERROR;
	}

	uint32_t ace, counter_time, time;
	uint8_t b, d, m;

	counter_time = mRTC_ReadCounter(hrtc);
	lastDay = counter_time / SECOND_A_DAY;
	time = counter_time % SECOND_A_DAY;

	ace = (lastDay) + 32044 + JD0;
	b = (4 * ace + 3) / 146097;
	ace = ace - ((146097 * b) / 4);
	d = (4 * ace + 3) / 1461;
	ace = ace - ((1461 * d) / 4);
	m = (5 * ace + 2) / 153;

	hrtc->DateToUpdate.Date = ace - ((153 * m + 2) / 5) + 1;
	hrtc->DateToUpdate.Month = m + 3 - (12 * (m / 10));
	hrtc->DateToUpdate.Year = (100 * b + d - 4800 + (m / 10)) - 2000;
	hrtc->DateToUpdate.WeekDay = mRTC_WeekDayNum(hrtc->DateToUpdate.Year, hrtc->DateToUpdate.Month, hrtc->DateToUpdate.Date);

	sTime->Hours = time / 3600;
	sTime->Minutes = (time % 3600) / 60;
	sTime->Seconds = (time % 3600) % 60;

	/* Check the input parameters format */
	if(Format != RTC_FORMAT_BIN)
	{
		/* Convert the time structure parameters to BCD format */
		sTime->Hours    = (uint8_t)mRTC_ByteToBcd2(sTime->Hours);
		sTime->Minutes  = (uint8_t)mRTC_ByteToBcd2(sTime->Minutes);
		sTime->Seconds  = (uint8_t)mRTC_ByteToBcd2(sTime->Seconds);
	}

	return HAL_OK;
}

HAL_StatusTypeDef mRTC_GetDate(RTC_HandleTypeDef* hrtc, RTC_DateTypeDef* sDate, uint32_t Format)
{
	/* Check input parameters */
	if((hrtc == NULL) || (sDate == NULL))
	{
	   return HAL_ERROR;
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));

	if(((mRTC_ReadCounter(hrtc) / SECOND_A_DAY) - lastDay) > 0)
	{
		RTC_TimeTypeDef stime = {0U};
		/* Call HAL_RTC_GetTime function to update date if counter higher than 24 hours */
		if (mRTC_GetTime(hrtc, &stime, RTC_FORMAT_BIN) != HAL_OK)
		{
		  return HAL_ERROR;
		}
	}

	sDate->Date = hrtc->DateToUpdate.Date;
	sDate->Month = hrtc->DateToUpdate.Month;
	sDate->Year = hrtc->DateToUpdate.Year;
	sDate->WeekDay = hrtc->DateToUpdate.WeekDay;

	/* Check the input parameters format */
	if(Format != RTC_FORMAT_BIN)
	{
		/* Convert the date structure parameters to BCD format */
		sDate->Year   = (uint8_t)mRTC_ByteToBcd2(sDate->Year);
		sDate->Month  = (uint8_t)mRTC_ByteToBcd2(sDate->Month);
		sDate->Date   = (uint8_t)mRTC_ByteToBcd2(sDate->Date);
	}

	return HAL_OK;
}

HAL_StatusTypeDef mRTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format)
{
	uint32_t counter_alarm = 0U, counter_time;

	/* Check input parameters */
	if((hrtc == NULL) || (sAlarm == NULL))
	{
		return HAL_ERROR;
	}

	/* Check the parameters */
	assert_param(IS_RTC_FORMAT(Format));
	assert_param(IS_RTC_ALARM(sAlarm->Alarm));

	/* Process Locked */
	__HAL_LOCK(hrtc);

	hrtc->State = HAL_RTC_STATE_BUSY;

	if(Format == RTC_FORMAT_BIN)
	{
		assert_param(IS_RTC_HOUR24(sAlarm->AlarmTime.Hours));
		assert_param(IS_RTC_MINUTES(sAlarm->AlarmTime.Minutes));
		assert_param(IS_RTC_SECONDS(sAlarm->AlarmTime.Seconds));

		counter_alarm = (uint32_t)(((uint32_t)sAlarm->AlarmTime.Hours * 3600U) + \
									((uint32_t)sAlarm->AlarmTime.Minutes * 60U) + \
									((uint32_t)sAlarm->AlarmTime.Seconds));
	}
	else
	{
		assert_param(IS_RTC_HOUR24(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Hours)));
		assert_param(IS_RTC_MINUTES(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Minutes)));
		assert_param(IS_RTC_SECONDS(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Seconds)));

		counter_alarm = (((uint32_t)(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Hours)) * 3600U) + \
						((uint32_t)(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Minutes)) * 60U) + \
						((uint32_t)mRTC_Bcd2ToByte(sAlarm->AlarmTime.Seconds)));
	}

	counter_time = mRTC_ReadCounter(hrtc);

	counter_alarm = counter_alarm + (counter_time - (counter_time % 86400));

	/* Check that requested alarm should expire in the same day (otherwise add 1 day) */
	if (counter_alarm < counter_time)
	{
		/* Add 1 day to alarm counter*/
		counter_alarm = counter_alarm + (uint32_t) (24U * 3600U);
	}

	/* Write Alarm counter in RTC registers */
	if (mRTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
	{
		/* Set RTC state */
		hrtc->State = HAL_RTC_STATE_ERROR;

		/* Process Unlocked */
		__HAL_UNLOCK(hrtc);

		return HAL_ERROR;
	}
	else
	{
		hrtc->State = HAL_RTC_STATE_READY;

		__HAL_UNLOCK(hrtc);

		return HAL_OK;
	}
}

HAL_StatusTypeDef mRTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format)
{
	  uint32_t counter_alarm = 0U, counter_time;

	  /* Check input parameters */
	  if((hrtc == NULL) || (sAlarm == NULL))
	  {
	     return HAL_ERROR;
	  }

	  /* Check the parameters */
	  assert_param(IS_RTC_FORMAT(Format));
	  assert_param(IS_RTC_ALARM(sAlarm->Alarm));

	  /* Process Locked */
	  __HAL_LOCK(hrtc);

	  hrtc->State = HAL_RTC_STATE_BUSY;

	  if(Format == RTC_FORMAT_BIN)
	  {
	    assert_param(IS_RTC_HOUR24(sAlarm->AlarmTime.Hours));
	    assert_param(IS_RTC_MINUTES(sAlarm->AlarmTime.Minutes));
	    assert_param(IS_RTC_SECONDS(sAlarm->AlarmTime.Seconds));

	    counter_alarm = (uint32_t)(((uint32_t)sAlarm->AlarmTime.Hours * 3600U) + \
	    							((uint32_t)sAlarm->AlarmTime.Minutes * 60U) + \
									((uint32_t)sAlarm->AlarmTime.Seconds));
	  }
	  else
	  {
	    assert_param(IS_RTC_HOUR24(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Hours)));
	    assert_param(IS_RTC_MINUTES(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Minutes)));
	    assert_param(IS_RTC_SECONDS(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Seconds)));

	    counter_alarm = (((uint32_t)(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Hours)) * 3600U) + \
	                     ((uint32_t)(mRTC_Bcd2ToByte(sAlarm->AlarmTime.Minutes)) * 60U) + \
	                     ((uint32_t)mRTC_Bcd2ToByte(sAlarm->AlarmTime.Seconds)));
	  }

	  counter_time = mRTC_ReadCounter(hrtc);

	  counter_alarm = counter_alarm + (counter_time - (counter_time % 86400));

	  /* Check that requested alarm should expire in the same day (otherwise add 1 day) */
	  if (counter_alarm < counter_time)
	  {
	    /* Add 1 day to alarm counter*/
	    counter_alarm = counter_alarm + (uint32_t) (24U * 3600U);
	  }

	  /* Write alarm counter in RTC registers */
	  if (mRTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
	  {
	    /* Set RTC state */
	    hrtc->State = HAL_RTC_STATE_ERROR;

	    /* Process Unlocked */
	    __HAL_UNLOCK(hrtc);

	    return HAL_ERROR;
	  }
	  else
	  {
	    /* Clear flag alarm A */
	    __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);

	    /* Configure the Alarm interrupt */
	    __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);

	    /* RTC Alarm Interrupt Configuration: EXTI configuration */
	    __HAL_RTC_ALARM_EXTI_ENABLE_IT();

	    __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();

	    hrtc->State = HAL_RTC_STATE_READY;

	   __HAL_UNLOCK(hrtc);

	   return HAL_OK;
	  }
}

HAL_StatusTypeDef mRTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
  uint32_t counter_day_of_seconds = 0U, counter_time = 0U, counter = 0U, counter_alarm = 0U;

  /* Check input parameters */
  if((hrtc == NULL) || (sTime == NULL))
  {
     return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));

  /* Process Locked */
  __HAL_LOCK(hrtc);

  hrtc->State = HAL_RTC_STATE_BUSY;

  if(Format == RTC_FORMAT_BIN)
  {
	  assert_param(IS_RTC_HOUR24(sTime->Hours));
	  assert_param(IS_RTC_MINUTES(sTime->Minutes));
	  assert_param(IS_RTC_SECONDS(sTime->Seconds));

	  counter_time = (uint32_t)(((uint32_t)sTime->Hours * 3600U) + \
								((uint32_t)sTime->Minutes * 60U) + \
								((uint32_t)sTime->Seconds));
  }
  else
  {
	  assert_param(IS_RTC_HOUR24(mRTC_Bcd2ToByte(sTime->Hours)));
	  assert_param(IS_RTC_MINUTES(mRTC_Bcd2ToByte(sTime->Minutes)));
	  assert_param(IS_RTC_SECONDS(mRTC_Bcd2ToByte(sTime->Seconds)));

	  counter_time = (((uint32_t)(mRTC_Bcd2ToByte(sTime->Hours)) * 3600U) + \
			  	  	  ((uint32_t)(mRTC_Bcd2ToByte(sTime->Minutes)) * 60U) + \
					  ((uint32_t)(mRTC_Bcd2ToByte(sTime->Seconds))));
  }

  counter = mRTC_ReadCounter(hrtc);
  counter_day_of_seconds = counter - (counter % 86400);

  /* Write time counter in RTC registers */
  if (mRTC_WriteCounter(hrtc, counter_time + counter_day_of_seconds) != HAL_OK)
  {
    /* Set RTC state */
    hrtc->State = HAL_RTC_STATE_ERROR;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_ERROR;
  }
  else
  {
    /* Clear Second and overflow flags */
    CLEAR_BIT(hrtc->Instance->CRL, (RTC_FLAG_SEC | RTC_FLAG_OW));

    /* Read current Alarm counter in RTC registers */
    counter_alarm = mRTC_ReadAlarmCounter(hrtc) % 86400;

    /* Set again alarm to match with new time if enabled */
    if (counter_alarm != RTC_ALARM_RESETVALUE)
    {
      if(counter_alarm < counter_time)
      {
    	  /* Add 1 day to alarm counter*/
    	  counter_alarm += (uint32_t)(24U * 3600U);
      }
      /* Write new Alarm counter in RTC registers */
	  if (mRTC_WriteAlarmCounter(hrtc, counter_alarm + counter_day_of_seconds) != HAL_OK)
	  {
		/* Set RTC state */
		hrtc->State = HAL_RTC_STATE_ERROR;

		/* Process Unlocked */
		__HAL_UNLOCK(hrtc);

		return HAL_ERROR;
	  }
    }

    hrtc->State = HAL_RTC_STATE_READY;

   __HAL_UNLOCK(hrtc);

   return HAL_OK;
  }
}

HAL_StatusTypeDef mRTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
  uint32_t counter_time = 0U, counter_alarm = 0U;

  /* Check input parameters */
  if((hrtc == NULL) || (sDate == NULL))
  {
     return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));

 /* Process Locked */
 __HAL_LOCK(hrtc);

  hrtc->State = HAL_RTC_STATE_BUSY;

  if(Format == RTC_FORMAT_BIN)
  {
	  assert_param(IS_RTC_YEAR(sDate->Year));
	  assert_param(IS_RTC_MONTH(sDate->Month));
	  assert_param(IS_RTC_DATE(sDate->Date));

	  /* Change the current date */
	  hrtc->DateToUpdate.Year  = sDate->Year;
	  hrtc->DateToUpdate.Month = sDate->Month;
	  hrtc->DateToUpdate.Date  = sDate->Date;
  }
  else
  {
	  assert_param(IS_RTC_YEAR(mRTC_Bcd2ToByte(sDate->Year)));
	  assert_param(IS_RTC_MONTH(mRTC_Bcd2ToByte(sDate->Month)));
	  assert_param(IS_RTC_DATE(mRTC_Bcd2ToByte(sDate->Date)));

	  /* Change the current date */
	  hrtc->DateToUpdate.Year  = mRTC_Bcd2ToByte(sDate->Year);
	  hrtc->DateToUpdate.Month = mRTC_Bcd2ToByte(sDate->Month);
	  hrtc->DateToUpdate.Date  = mRTC_Bcd2ToByte(sDate->Date);
  }

  /* WeekDay set by user can be ignored because automatically calculated */
  hrtc->DateToUpdate.WeekDay = mRTC_WeekDayNum(hrtc->DateToUpdate.Year, hrtc->DateToUpdate.Month, hrtc->DateToUpdate.Date);
  sDate->WeekDay = hrtc->DateToUpdate.WeekDay;

  /* Reset time to be aligned on the same day */
  /* Read the time counter*/
  counter_time = (mRTC_ReadCounter(hrtc) % 86400) + mRTC_GetSecondsInDate(hrtc);

  /* Write time counter in RTC registers */
  if (mRTC_WriteCounter(hrtc, counter_time) != HAL_OK)
  {
	/* Set RTC state */
	hrtc->State = HAL_RTC_STATE_ERROR;

	/* Process Unlocked */
	__HAL_UNLOCK(hrtc);

	return HAL_ERROR;
   }

    /* Read current Alarm counter in RTC registers */
    counter_alarm = mRTC_ReadAlarmCounter(hrtc) % 86400;
    counter_time = counter_time % 86400;

    /* Set again alarm to match with new time if enabled */
    if (counter_alarm != RTC_ALARM_RESETVALUE)
    {
	  if(counter_alarm < counter_time)
	  {
		  /* Add 1 day to alarm counter*/
		  counter_alarm += (uint32_t)(24U * 3600U);
	  }
		/* Write new Alarm counter in RTC registers */
		if (mRTC_WriteAlarmCounter(hrtc, counter_alarm + mRTC_GetSecondsInDate(hrtc)) != HAL_OK)
		{
		  /* Set RTC state */
		  hrtc->State = HAL_RTC_STATE_ERROR;

		  /* Process Unlocked */
		  __HAL_UNLOCK(hrtc);

		  return HAL_ERROR;
		}
     }

  hrtc->State = HAL_RTC_STATE_READY ;

  /* Process Unlocked */
  __HAL_UNLOCK(hrtc);

  return HAL_OK;
}

static uint32_t mRTC_GetSecondsInDate(RTC_HandleTypeDef* hrtc)
{
	uint8_t a = (14 - hrtc->DateToUpdate.Month) / 12;
	uint16_t y = hrtc->DateToUpdate.Year + 2000 + 4800 - a;
	uint8_t m = hrtc->DateToUpdate.Month + (12 * a) - 3;
	uint32_t counter;

	counter = hrtc->DateToUpdate.Date;
	counter += (153 * m + 2) / 5;
	counter += 365 * y;
	counter += y / 4;
	counter += -y / 100;
	counter += y / 400;
	counter += -32045;
	counter += -JD0;
	counter *= 86400;

	return counter;
}

static uint32_t mRTC_ReadCounter(RTC_HandleTypeDef* hrtc)
{
  uint16_t high1 = 0U, high2 = 0U, low = 0U;
  uint32_t timecounter = 0U;

  high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);

  if (high1 != high2)
  { /* In this case the counter roll over during reading of CNTL and CNTH registers,
       read again CNTL register then return the counter value */
    timecounter = (((uint32_t) high2 << 16U) | READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  { /* No counter roll over during reading of CNTL and CNTH registers, counter
       value is equal to first value of CNTL and CNTH */
    timecounter = (((uint32_t) high1 << 16U) | low);
  }

  return timecounter;
}

static HAL_StatusTypeDef mRTC_WriteCounter(RTC_HandleTypeDef* hrtc, uint32_t TimeCounter)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Set Initialization mode */
  if(mRTC_EnterInitMode(hrtc) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  else
  {
    /* Set RTC COUNTER MSB word */
    WRITE_REG(hrtc->Instance->CNTH, (TimeCounter >> 16U));
    /* Set RTC COUNTER LSB word */
    WRITE_REG(hrtc->Instance->CNTL, (TimeCounter & RTC_CNTL_RTC_CNT));

    /* Wait for synchro */
    if(mRTC_ExitInitMode(hrtc) != HAL_OK)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}

static HAL_StatusTypeDef mRTC_EnterInitMode(RTC_HandleTypeDef* hrtc)
{
  uint32_t tickstart = 0U;

  tickstart = HAL_GetTick();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);


  return HAL_OK;
}

static HAL_StatusTypeDef mRTC_ExitInitMode(RTC_HandleTypeDef* hrtc)
{
  uint32_t tickstart = 0U;

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  tickstart = HAL_GetTick();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

static uint8_t mRTC_WeekDayNum(uint32_t nYear, uint8_t nMonth, uint8_t nDay)
{
  uint32_t year = 0U, weekday = 0U;

  year = 2000U + nYear;

  if(nMonth < 3U)
  {
    /*D = { [(23 x month)/9] + day + 4 + year + [(year-1)/4] - [(year-1)/100] + [(year-1)/400] } mod 7*/
    weekday = (((23U * nMonth)/9U) + nDay + 4U + year + ((year-1U)/4U) - ((year-1U)/100U) + ((year-1U)/400U)) % 7U;
  }
  else
  {
    /*D = { [(23 x month)/9] + day + 4 + year + [year/4] - [year/100] + [year/400] - 2 } mod 7*/
    weekday = (((23U * nMonth)/9U) + nDay + 4U + year + (year/4U) - (year/100U) + (year/400U) - 2U ) % 7U;
  }

  return (uint8_t)weekday;
}

static uint8_t mRTC_ByteToBcd2(uint8_t Value)
{
  uint32_t bcdhigh = 0U;

  while(Value >= 10U)
  {
    bcdhigh++;
    Value -= 10U;
  }

  return  ((uint8_t)(bcdhigh << 4U) | Value);
}

static uint8_t mRTC_Bcd2ToByte(uint8_t Value)
{
  uint32_t tmp = 0U;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10U;
  return (tmp + (Value & (uint8_t)0x0F));
}

static HAL_StatusTypeDef mRTC_WriteAlarmCounter(RTC_HandleTypeDef* hrtc, uint32_t AlarmCounter)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Set Initialization mode */
  if(mRTC_EnterInitMode(hrtc) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  else
  {
    /* Set RTC COUNTER MSB word */
    WRITE_REG(hrtc->Instance->ALRH, (AlarmCounter >> 16U));
    /* Set RTC COUNTER LSB word */
    WRITE_REG(hrtc->Instance->ALRL, (AlarmCounter & RTC_ALRL_RTC_ALR));

    /* Wait for synchro */
    if(mRTC_ExitInitMode(hrtc) != HAL_OK)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}

static uint32_t mRTC_ReadAlarmCounter(RTC_HandleTypeDef* hrtc)
{
  uint16_t high1 = 0U, low = 0U;

  high1 = READ_REG(hrtc->Instance->ALRH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->ALRL & RTC_CNTL_RTC_CNT);

  return (((uint32_t) high1 << 16U) | low);
}
