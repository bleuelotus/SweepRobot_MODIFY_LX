/**
  ******************************************************************************
  * @file    Key.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   GPIO key driver
  ******************************************************************************
  */

#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x_conf.h"

#define KEY_SCAN_TIM_PERIOD_MSx10           20          //2ms
#define KEY_SCAN_INTERVAL_MS                100

#define KEY_POLLING_TIM_PERIPH_ID           RCC_APB1Periph_TIM3
#define KEY_POLLING_TIM                     TIM3
#define KEY_POLLING_TIM_IRQn                TIM3_IRQn
#define KEY_POLLING_TIM_IRQ_PP              0
#define KEY_POLLING_TIM_IRQ_SP              1
#define KEY_POLLING_TIM_INT_IDX             STM32F10x_INT_TIM3

typedef struct{
    void (*Down)(void);
    void (*Up)(void);
} KeyEvtCB_t;

typedef struct{

    u32             RCCResID;
    GPIO_TypeDef*   GPIOx;
    u16             GPIO_Pin;
    KeyEvtCB_t      EvtCB;
} Key_t;

void Key_CoreInit(void);
void Key_StartListen(void);
void Key_StopListen(void);
s8 Key_Register(Key_t *key);
s8 Key_Deregister(Key_t *key);

#endif  //end __KEY_H