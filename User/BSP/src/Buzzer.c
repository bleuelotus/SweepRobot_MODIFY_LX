/**
  ******************************************************************************
  * @file    Buzzer.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Buzzer driver
  ******************************************************************************
  */

#include "Buzzer.h"

#define BUZZER_ONE_PULS_LEN             300                                     // 750ms

static u8 BuzzerPulsCnt = 0;
static u16 BuzzerInput = 0;

void BuzzerClock(void)
{
    BuzzerInput++;
    if(!(BuzzerPulsCnt%2)){
        if(!(BuzzerInput%BUZZER_ONE_PULS_LEN)){
            BuzzerPulsCnt--;
        }
        GPIO_WriteBit(BUZZER_SW_GPIO, BUZZER_SW_PIN, (BitAction)(BuzzerInput%2));
    }
    else {
        if(1==BuzzerPulsCnt){
            TIM_Cmd(BUZZER_PWM_SRC_TIM, DISABLE);
            BuzzerInput = 0;
        }
        else{
            if(!(BuzzerInput%BUZZER_ONE_PULS_LEN)){
                BuzzerPulsCnt--;
            }
        }
    }
}

void Buzzer_Init(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(BUZZER_SW_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BUZZER_SW_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUZZER_SW_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(BUZZER_SW_GPIO, BUZZER_SW_PIN);

    NVIC_InitStructure.NVIC_IRQChannel = BUZZER_PWM_SRC_TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BUZZER_PWM_SRC_TIM_IRQ_PP;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = BUZZER_PWM_SRC_TIM_IRQ_SP;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(BUZZER_PWM_SRC_TIM_PERIPH_ID , ENABLE);
    TIM_DeInit(BUZZER_PWM_SRC_TIM);
    TIM_TimeBaseStructure.TIM_Period = 25-1;                                 // 0.25ms
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(BUZZER_PWM_SRC_TIM, &TIM_TimeBaseStructure);
    TIM_ClearFlag(BUZZER_PWM_SRC_TIM, TIM_FLAG_Update);
    TIM_ITConfig(BUZZER_PWM_SRC_TIM, TIM_IT_Update, ENABLE);

    plat_int_reg_cb(BUZZER_PWM_SRC_TIM_INT_IDX, (void*)BuzzerClock);
}

void Buzzer_Play(enum BuzzerSndType snd)
{
    Buzzer_Stop();

    switch(snd){

        case BUZZER_ONE_PULS:
            BuzzerPulsCnt = BUZZER_ONE_PULS * 2;
            break;
        case BUZZER_TWO_PULS:
            BuzzerPulsCnt = BUZZER_TWO_PULS * 2;
            break;
        case BUZZER_TRI_PULS:
            BuzzerPulsCnt = BUZZER_TRI_PULS * 2;
            break;
        case BUZZER_CONSECUTIVE_PULS:
            BuzzerPulsCnt = 0xFF;
            break;
        default:return;
    }

    TIM_Cmd(BUZZER_PWM_SRC_TIM, ENABLE);
}

void Buzzer_Stop(void)
{
    TIM_Cmd(BUZZER_PWM_SRC_TIM, DISABLE);
    BuzzerPulsCnt = 0;
    BuzzerInput = 0;
}