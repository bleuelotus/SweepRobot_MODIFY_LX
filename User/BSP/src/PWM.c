/**
  ******************************************************************************
  * @file    PWM.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   PWM resource manage drvier
  ******************************************************************************
  */

#include "PWM.h"

typedef struct _PWMChannel_s {

    u8                      ChanIdx;
    u8                      ControllerIdx;
    u32                     RCCResID;
    GPIO_TypeDef*           GPIO;
    u16                     Pin;
} PWMChannel_t;

typedef struct _PWMController_s {

    u32                     RCCResID;
    TIM_TypeDef*            TIMx;
    u32                     FRQ;
    u16                     Period;

} PWMController_t;
                                                /* Chan     Ctr     RCCResID                PWMGPIO     PWMPin      */
static const PWMChannel_t       PWMChanRes[] = { { 3,       0,      RCC_APB2Periph_GPIOA,   GPIOA,      GPIO_Pin_10 },
                                                 { 4,       0,      RCC_APB2Periph_GPIOA,   GPIOA,      GPIO_Pin_11 },
                                                 { 1,       1,      RCC_APB2Periph_GPIOC,   GPIOC,      GPIO_Pin_6  },
                                                 { 1,       2,      RCC_APB2Periph_GPIOB,   GPIOB,      GPIO_Pin_6  },
                                                 { 2,       2,      RCC_APB2Periph_GPIOB,   GPIOB,      GPIO_Pin_7  },
                                                 { 3,       2,      RCC_APB2Periph_GPIOB,   GPIOB,      GPIO_Pin_8  },
                                                 { 4,       2,      RCC_APB2Periph_GPIOB,   GPIOB,      GPIO_Pin_9  },
                                               };

static const PWMController_t    PWMController[] = { { RCC_APB2Periph_TIM1,    TIM1,     10000,      100 },
                                                    { RCC_APB2Periph_TIM8,    TIM8,     200000,     360 },
                                                    { RCC_APB1Periph_TIM4,    TIM4,     10000,      100 },
                                                  };


s8 PWM_ControllerInit(void)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    GPIO_InitTypeDef            GPIO_InitStructure;
    u8                          i = 0;

    /* PWM Controller init */
    for(i = 0; i < sizeof(PWMController)/sizeof(PWMController[0]); i++ ){

        TIM_DeInit(PWMController[i].TIMx);

        if(PWMController[i].RCCResID > 0x100){
            RCC_APB2PeriphClockCmd(PWMController[i].RCCResID , ENABLE);
        }
        else{
            RCC_APB1PeriphClockCmd(PWMController[i].RCCResID , ENABLE);
        }

        TIM_TimeBaseStructure.TIM_Period = PWMController[i].Period-1;
        TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/PWMController[i].Period/PWMController[i].FRQ-1;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(PWMController[i].TIMx, &TIM_TimeBaseStructure);
        TIM_ARRPreloadConfig(PWMController[i].TIMx, ENABLE);
    }

    for(i = 0; i < sizeof(PWMChanRes)/sizeof(PWMChanRes[0]); i++ ){

        RCC_APB2PeriphClockCmd(PWMChanRes[i].RCCResID | RCC_APB2Periph_AFIO, ENABLE);

        /* PWM Output Pin init */
        GPIO_InitStructure.GPIO_Pin = PWMChanRes[i].Pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(PWMChanRes[i].GPIO, &GPIO_InitStructure);
    }
    return 0;
}

s8 PWM_ChanInit(enum PWMChannelRes ChanResIdx, u8 Polarity, u16 DutyCycle)
{
    TIM_OCInitTypeDef           TIM_OCInitStructure;

    if(ChanResIdx >= PWM_CHAN_NULL)
        return -1;

    if( DutyCycle >= PWMController[PWMChanRes[ChanResIdx].ControllerIdx].Period ){
        return -1;
    }

    /* Channel init */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = Polarity ? TIM_OCPolarity_High : TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

    TIM_OCInitStructure.TIM_Pulse = DutyCycle;

    switch(PWMChanRes[ChanResIdx].ChanIdx)
    {
        case 1:
//            TIM_OC1PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Enable);
            TIM_OC1Init(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, &TIM_OCInitStructure);
            break;
        case 2:
//            TIM_OC2PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Enable);
            TIM_OC2Init(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, &TIM_OCInitStructure);
            break;
        case 3:
//            TIM_OC3PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Enable);
            TIM_OC3Init(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, &TIM_OCInitStructure);
            break;
        case 4:
//            TIM_OC4PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Enable);
            TIM_OC4Init(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, &TIM_OCInitStructure);
            break;
        default:;
    }
    return 0;
}

s8 PWM_DutyCycleSet(enum PWMChannelRes ChanResIdx, u16 DutyCycle)
{
    switch(PWMChanRes[ChanResIdx].ChanIdx)
    {
        case 1:
//            TIM_OC1PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Disable);
            TIM_SetCompare1(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, DutyCycle);
//            TIM_OC1PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Enable);
            break;
        case 2:
//            TIM_OC2PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Disable);
            TIM_SetCompare2(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, DutyCycle);
//            TIM_OC2PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Enable);
            break;
        case 3:
//            TIM_OC3PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Disable);
            TIM_SetCompare3(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, DutyCycle);
//            TIM_OC3PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Enable);
            break;
        case 4:
//            TIM_OC4PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Disable);
            TIM_SetCompare4(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, DutyCycle);
//            TIM_OC4PreloadConfig(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx, TIM_OCPreload_Enable);
            break;
        default:
            return -1;
    }
    return 0;
}

s8 PWM_DutyCycleGet(enum PWMChannelRes ChanResIdx, u16 *DutyCycle)
{
    u16     CCRVal = 0;

    if(ChanResIdx >= PWM_CHAN_NULL)
        return -1;

    switch(PWMChanRes[ChanResIdx].ChanIdx)
    {
        case 1:
            CCRVal = TIM_GetCapture1(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx);
            break;
        case 2:
            CCRVal = TIM_GetCapture2(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx);
            break;
        case 3:
            CCRVal = TIM_GetCapture3(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx);
            break;
        case 4:
            CCRVal = TIM_GetCapture4(PWMController[PWMChanRes[ChanResIdx].ControllerIdx].TIMx);
            break;
    }

    *DutyCycle = CCRVal;

    return 0;
}

void PWM_ControllerStart(void)
{
    u8      i = 0;

    /* Contorller init */
    for(i = 0; i < sizeof(PWMController)/sizeof(PWMController[0]); i++ ){

        TIM_Cmd(PWMController[i].TIMx, ENABLE);
        if(TIM1 == PWMController[i].TIMx || TIM8 == PWMController[i].TIMx){
            TIM_CtrlPWMOutputs(PWMController[i].TIMx, ENABLE);
        }
    }
}

void PWM_ControllerStop(void)
{
    u8      i = 0;

    /* Contorller init */
    for(i = 0; i < sizeof(PWMController)/sizeof(PWMController[0]); i++ ){

        TIM_Cmd(PWMController[i].TIMx, DISABLE);
        if(TIM1 == PWMController[i].TIMx || TIM8 == PWMController[i].TIMx){
            TIM_CtrlPWMOutputs(PWMController[i].TIMx, DISABLE);
        }
    }
}
