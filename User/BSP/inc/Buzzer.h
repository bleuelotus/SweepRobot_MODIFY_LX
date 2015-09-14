/**
  ******************************************************************************
  * @file    Buzzer.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Buzzer driver
  ******************************************************************************
  */

#include "stm32f10x_conf.h"

#define BUZZER_SW_GPIO_PERIPH_ID            RCC_APB2Periph_GPIOD
#define BUZZER_SW_GPIO                      GPIOD
#define BUZZER_SW_PIN                       GPIO_Pin_11


#define BUZZER_PWM_SRC_TIM_PERIPH_ID        RCC_APB1Periph_TIM3
#define BUZZER_PWM_SRC_TIM                  TIM3
#define BUZZER_PWM_SRC_TIM_IRQn             TIM3_IRQn
#define BUZZER_PWM_SRC_TIM_IRQ_PP           3
#define BUZZER_PWM_SRC_TIM_IRQ_SP           3
#define BUZZER_PWM_SRC_TIM_INT_IDX          STM32F10x_INT_TIM3


enum BuzzerSndType {

    BUZZER_ONE_PULS = 1,
    BUZZER_TWO_PULS,
    BUZZER_TRI_PULS,
    BUZZER_CONSECUTIVE_PULS,
};

void Buzzer_Init(void);
void Buzzer_Play(enum BuzzerSndType snd);
void Buzzer_Stop(void);
