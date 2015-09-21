/**
  ******************************************************************************
  * @file    Schedule.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Schedule delayed work
  ******************************************************************************
  */

#include "Schedule.h"


void Schedule_Init(void)
{
    NVIC_InitTypeDef            NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USER_SCHEDULE_TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USER_SCHEDULE_TIM_IRQ_PP;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = USER_SCHEDULE_TIM_IRQ_SP;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(USER_SCHEDULE_TIM_PERIPH_ID , ENABLE);
    TIM_DeInit(USER_SCHEDULE_TIM);
    TIM_TimeBaseStructure.TIM_Period = 25-1;                                 // 0.25ms
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(USER_SCHEDULE_TIM, &TIM_TimeBaseStructure);
    TIM_ClearFlag(USER_SCHEDULE_TIM, TIM_FLAG_Update);
    TIM_ITConfig(USER_SCHEDULE_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(USER_SCHEDULE_TIM, ENABLE);

    plat_int_reg_cb(USER_SCHEDULE_TIM_INT_IDX, (void*)NULL);
}

void Schedule_DelayWorkInit(void *pFunc)
{

}