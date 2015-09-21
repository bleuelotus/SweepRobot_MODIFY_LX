/**
  ******************************************************************************
  * @file    Schedule.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Schedule delayed work
  ******************************************************************************
  */

#include "stm32f10x_conf.h"

#define USER_SCHEDULE_TIM_PERIPH_ID         RCC_APB1Periph_TIM7
#define USER_SCHEDULE_TIM                   TIM7
#define USER_SCHEDULE_TIM_IRQn              TIM7_IRQn
#define USER_SCHEDULE_TIM_IRQ_PP            2
#define USER_SCHEDULE_TIM_IRQ_SP            2
#define USER_SCHEDULE_TIM_INT_IDX           STM32F10x_INT_TIM7

void Schedule_Init(void);
void Schedule_DelayWorkInit(void *pFunc);