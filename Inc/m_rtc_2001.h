/*
 * m_rtc_2001.h
 *
 *  Created on: 30 квіт. 2018 р.
 *      Author: Andriy
 */

#ifndef M_RTC_2001_H_
#define M_RTC_2001_H_

#include "main.h"

#define				TIME_COUNTER(hrtc)		mRTC_ReadCounter(hrtc)
#define				ALARM_COUNTER(hrtc)		mRTC_ReadAlarmCounter(hrtc)
#define 			SECOND(hrtc, format)	mRTC_GetSecond(hrtc, format)
#define 			MINUTE(hrtc, format)	mRTC_GetMinute(hrtc, format)
#define 			HOUR(hrtc, format)		mRTC_GetHour(hrtc, format)
#define				WEEKDAY(hrtc)			mRTC_GetWeekDay(hrtc)
#define 			DAY(hrtc, format) 		mRTC_GetDay(hrtc, format)
#define 			MONTH(hrtc, format) 	mRTC_GetMonth(hrtc, format)
#define 			YEAR(hrtc, format) 		mRTC_GetYear(hrtc, format)

void				mRTC_Begin(RTC_HandleTypeDef* hrtc);
uint8_t				mRTC_GetSecond(RTC_HandleTypeDef* hrtc, uint32_t Format);
uint8_t				mRTC_GetMinute(RTC_HandleTypeDef* hrtc, uint32_t Format);
uint8_t				mRTC_GetHour(RTC_HandleTypeDef* hrtc, uint32_t Format);
uint8_t				mRTC_GetWeekDay(RTC_HandleTypeDef* hrtc);
uint8_t				mRTC_GetDay(RTC_HandleTypeDef* hrtc, uint32_t Format);
uint8_t				mRTC_GetMonth(RTC_HandleTypeDef* hrtc, uint32_t Format);
uint16_t			mRTC_GetYear(RTC_HandleTypeDef* hrtc, uint32_t Format);

HAL_StatusTypeDef	mRTC_GetTime(RTC_HandleTypeDef* hrtc, RTC_TimeTypeDef* sTime, uint32_t Format);
HAL_StatusTypeDef 	mRTC_GetDate(RTC_HandleTypeDef* hrtc, RTC_DateTypeDef* sDate, uint32_t Format);
HAL_StatusTypeDef 	mRTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef 	mRTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);

HAL_StatusTypeDef	mRTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef	mRTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);

#endif /* M_RTC_2001_H_ */
