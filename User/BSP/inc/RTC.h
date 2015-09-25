/**
  ******************************************************************************
  * @file    RTC.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   RealTime Clock
  ******************************************************************************
  */

#ifndef __RTC_H__
#define __RTC_H__

#include "stm32f10x_conf.h"

s8 RTC_Init(void);
void Time_Adjust(u8 HH, u8 MM, u8 SS);
void Time_Display(void);
void RTC_AlarmSet(u32 val);
s8 RTC_AlarmGet(void);

#endif /* !__RTC_H__ */