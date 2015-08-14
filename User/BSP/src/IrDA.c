/**
  ******************************************************************************
  * @file    IrDA.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   IrDA 1838 receive demodulation
  ******************************************************************************
  */
#include <stdlib.h>
#include "IrDA.h"
#include "delay.h"
#include "CtrlPanel.h"
#include "MsgQueue.h"


#define TIME_VAL                                    (TIM6->CNT)

#ifdef IRDA_MODE_EJE
#define IRDA_FRAME_LEN                              (1+8)
static u8  IrDA_FrameData[IRDA_LIGHT_NUM] = {0};
#elif IRDA_MODE_NEC
#define IRDA_FRAME_LEN                              (1+33)
static u32 IrDA_FrameData[IRDA_LIGHT_NUM] = {0};
#endif
static u8  IrDA_FrameFlag[IRDA_LIGHT_NUM] = {0};
static u8  IrDA_FrameBuf = 0, IrDA_FrameBufFlag = 0;

static u8  PulsCnt[IRDA_LIGHT_NUM] = {0};
static u8  PulsStart[IRDA_LIGHT_NUM] = {0};
static u8  HeadPuls[IRDA_LIGHT_NUM] = {0};
static u16 PulsTimeS[IRDA_LIGHT_NUM] = {0}, PulsTimeE[IRDA_LIGHT_NUM] = {0};
static u16 PulsLen[IRDA_LIGHT_NUM] = {0};


void IrDA_BackLightIsr(void);
void IrDA_LeftLightIsr(void);
void IrDA_FrontLLightIsr(void);
void IrDA_FrontRLightIsr(void);
void IrDA_RightLightIsr(void);
void PwrStationHomingSigProc(u8 idx, u8 code);

void IrDA_TimeCounterInit(void)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
    TIM_DeInit(TIM6);
    TIM_TimeBaseStructure.TIM_Period = 0xFFFE;                                  // 655.35ms
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1;                                // 0.01msPerCount
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM6, ENABLE);
}

/* NEC CODE: 9ms L + 4.5ms H + (0.56ms L + 0.565ms H) X 32 + [2.2ms H] */
static inline void IrDA_ParseNEC(enum IRAD_Light idx)
{
    u8  data = 0;

    /* Second falling edge of cycle end */
    if(PulsStart[idx]){
        PulsTimeE[idx] = TIME_VAL;
        PulsLen[idx] = (PulsTimeE[idx] > PulsTimeS[idx]) ? (PulsTimeE[idx] - PulsTimeS[idx]) : ((u16)0xFFFF - PulsTimeS[idx] + PulsTimeE[idx]);
        PulsStart[idx] = 0;
        if(HeadPuls[idx]){
            if(PulsLen[idx] > 100 && PulsLen[idx] < 150){                       // 0.56ms+0.565ms=1.125ms: 1.0ms~ 1.5ms
                data = 0;
            }
            else if(PulsLen[idx] > 200 && PulsLen[idx] < 250){                  // 0.56ms+1.68ms=2.2ms: 2.0ms~ 2.5ms
                data = 1;
            }
            else{
                PulsCnt[idx] = 0;
                HeadPuls[idx] = 0;
                return;
            }
            IrDA_FrameData[idx] <<= 1;
            IrDA_FrameData[idx] |= data;
        }
        else{
            /* Head puls */
            if(PulsLen[idx] > 1300 && PulsLen[idx] < 1500){               // 9ms+4.5ms=13.5ms: 13.0ms~ 15.0ms
                HeadPuls[idx] = 1;
                IrDA_FrameFlag[idx] = 0;
            }
            else{
                /* Repeat code */
                if(PulsLen[idx] > 200 && PulsLen[idx] < 250){             // 2.2ms: 2ms~ 2.5ms
                    /* Repeat process */
                }
                return;
            }
        }
        PulsCnt[idx]++;
        if(IRDA_FRAME_LEN==PulsCnt[idx]){
            PulsCnt[idx] = 0;
            IrDA_FrameFlag[idx] = 1;
            HeadPuls[idx] = 0;
        }
    }
    /* First falling edge of cycle start */
    else{
        PulsTimeS[idx] = TIME_VAL;
        PulsStart[idx] = 1;
    }
}

static inline void IrDA_ParseEJE(enum IRAD_Light idx)
{
    u8  data = 0;

    /* Rising */
    if(PulsStart[idx]){
        PulsTimeE[idx] = TIME_VAL;
        PulsLen[idx] = (PulsTimeE[idx] > PulsTimeS[idx]) ? (PulsTimeE[idx] - PulsTimeS[idx]) : ((u16)0xFFFF - PulsTimeS[idx] + PulsTimeE[idx]);
        PulsStart[idx] = 0;
        if(HeadPuls[idx]){
            if(PulsLen[idx] > 130 && PulsLen[idx] < 180){                       // 1.6ms: 1.3ms~ 1.8ms
                data = 0;
            }
            else if(PulsLen[idx] > 60 && PulsLen[idx] < 100){                   // 0.8ms: 0.6ms~ 1.0ms
                data = 1;
            }
            else{
                PulsCnt[idx] = 0;
                HeadPuls[idx] = 0;
                return;
            }
            IrDA_FrameData[idx] <<= 1;
            IrDA_FrameData[idx] |= data;
        }
        else{
            if(PulsLen[idx] > 280 && PulsLen[idx] < 320){                       // 3ms: 2.8ms~ 3.2ms
                HeadPuls[idx] = 1;
//                IrDA_FrameFlag[idx] = 0;
            }
            else{
                PulsCnt[idx] = 0;
                return;
            }
        }
        PulsCnt[idx]++;
        if(IRDA_FRAME_LEN==PulsCnt[idx]){
//            IrDA_FrameFlag[idx] = 1;
            HeadPuls[idx] = 0;
            PulsCnt[idx] = 0;
            if(IS_IRDA_HOMING_CODE(IrDA_FrameData[idx])){
                PwrStationHomingSigProc(idx, IrDA_FrameData[idx]);
            }
            else{
                IrDA_FrameBuf = IrDA_FrameData[idx];
                IrDA_FrameBufFlag = 1;
            }
        }
    }
    /* Falling */
    else{
        PulsTimeS[idx] = TIME_VAL;
        PulsStart[idx] = 1;
    }
}

void IrDA_BackLightIsr(void)
{
#if defined IRDA_MODE_EJE
    IrDA_ParseEJE(IRDA_BACK_LIGHT);
#elif defined IRDA_MODE_NEC
    IrDA_ParseNEC(IRDA_BACK_LIGHT);
#endif
}

void IrDA_LeftLightIsr(void)
{
#if defined IRDA_MODE_EJE
    IrDA_ParseEJE(IRDA_LEFT_LIGHT);
#elif defined IRDA_MODE_NEC
    IrDA_ParseNEC(IRDA_LEFT_LIGHT);
#endif
}

void IrDA_FrontLLightIsr(void)
{
#if defined IRDA_MODE_EJE
    IrDA_ParseEJE(IRDA_FRONT_L_LIGHT);
#elif defined IRDA_MODE_NEC
    IrDA_ParseNEC(IRDA_FRONT_L_LIGHT);
#endif
}

void IrDA_FrontRLightIsr(void)
{
#if defined IRDA_MODE_EJE
    IrDA_ParseEJE(IRDA_FRONT_R_LIGHT);
#elif defined IRDA_MODE_NEC
    IrDA_ParseNEC(IRDA_FRONT_R_LIGHT);
#endif
}

void IrDA_RightLightIsr(void)
{
#if defined IRDA_MODE_EJE
    IrDA_ParseEJE(IRDA_RIGHT_LIGHT);
#elif defined IRDA_MODE_NEC
    IrDA_ParseNEC(IRDA_RIGHT_LIGHT);
#endif
}

void IrDA_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    IrDA_TimeCounterInit();

    RCC_APB2PeriphClockCmd(IRDA_LIGHT_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = IRDA_BACK_LIGHT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(IRDA_BACK_LIGHT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = IRDA_LEFT_LIGHT_PIN;
    GPIO_Init(IRDA_LEFT_LIGHT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = IRDA_FRONT_L_LIGHT_PIN;
    GPIO_Init(IRDA_FRONT_L_LIGHT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = IRDA_FRONT_R_LIGHT_PIN;
    GPIO_Init(IRDA_FRONT_R_LIGHT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = IRDA_RIGHT_LIGHT_PIN;
    GPIO_Init(IRDA_RIGHT_LIGHT_GPIO, &GPIO_InitStructure);

    GPIO_EXTILineConfig(IRDA_BACK_LIGHT_EXTI_GPIO,       IRDA_BACK_LIGHT_EXTI_PIN);
    GPIO_EXTILineConfig(IRDA_LEFT_LIGHT_EXTI_GPIO,       IRDA_LEFT_LIGHT_EXTI_PIN);
    GPIO_EXTILineConfig(IRDA_FRONT_L_LIGHT_EXTI_GPIO,    IRDA_FRONT_L_LIGHT_EXTI_PIN);
    GPIO_EXTILineConfig(IRDA_FRONT_R_LIGHT_EXTI_GPIO,    IRDA_FRONT_R_LIGHT_EXTI_PIN);
    GPIO_EXTILineConfig(IRDA_RIGHT_LIGHT_EXTI_GPIO,      IRDA_RIGHT_LIGHT_EXTI_PIN);

    NVIC_InitStructure.NVIC_IRQChannel = IRDA_BACK_LIGHT_EXTI_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = IRDA_LEFT_LIGHT_EXTI_IRQ;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = IRDA_FRONT_L_LIGHT_EXTI_IRQ;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = IRDA_FRONT_R_LIGHT_EXTI_IRQ;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = IRDA_RIGHT_LIGHT_EXTI_IRQ;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_InitStructure.EXTI_Line = IRDA_BACK_LIGHT_EXTI_LINE
                                  |IRDA_LEFT_LIGHT_EXTI_LINE
                                  |IRDA_FRONT_L_LIGHT_EXTI_LINE
                                  |IRDA_FRONT_R_LIGHT_EXTI_LINE
                                  |IRDA_RIGHT_LIGHT_EXTI_LINE
                                  ;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#if defined IRDA_MODE_EJE
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
#elif defined IRDA_MODE_NEC
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
#endif
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    plat_int_reg_cb(IRDA_BACK_LIGHT_INT_INDEX,       (void*)IrDA_BackLightIsr);
    plat_int_reg_cb(IRDA_LEFT_LIGHT_INT_INDEX,       (void*)IrDA_LeftLightIsr);
    plat_int_reg_cb(IRDA_FRONT_L_LIGHT_INT_INDEX,    (void*)IrDA_FrontLLightIsr);
    plat_int_reg_cb(IRDA_FRONT_R_LIGHT_INT_INDEX,    (void*)IrDA_FrontRLightIsr);
    plat_int_reg_cb(IRDA_RIGHT_LIGHT_INT_INDEX,      (void*)IrDA_RightLightIsr);

    mDelay(100);
}


u8 IrDA_ProcessNEC(u8 idx)
{
    u8 first_byte, sec_byte, tir_byte, fou_byte;

    first_byte = IrDA_FrameData[idx] >> 24;
    sec_byte = (IrDA_FrameData[idx]>>16) & 0xff;
    tir_byte = IrDA_FrameData[idx] >> 8;
    fou_byte = IrDA_FrameData[idx];

    //  IrDA_FrameFlag = 0;

    if( (first_byte==(u8)~sec_byte) && (first_byte==IRDA_ID) )
    {
    if( tir_byte == (u8)~fou_byte )
      return tir_byte;
    }

    return 0;
}

void IrDA_ProcessEJE(void (*RemoteCB)(u8 code))
{
    if(IrDA_FrameBufFlag){
        RemoteCB(IrDA_FrameBuf);
        IrDA_FrameBufFlag = 0;
    }
}

void PwrStationHomingSigProc(u8 idx, u8 code)
{
    Msg_t   Msg;

    Msg.expire = 0;
    Msg.prio = MSG_PRIO_HIGH;
    Msg.type = MSG_TYPE_PWR_STATION;
    Msg.MsgCB = NULL;
    Msg.Data.PSSigDat.src = (enum IrDARecvPos)idx;
    Msg.Data.PSSigDat.sig = (enum PwrStationSignal)code;
    SweepRobot_SendMsg(&Msg);
}

/*********************************************END OF FILE**********************/
