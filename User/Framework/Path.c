/**
  ******************************************************************************
  * @file    Path.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Robot Path Planning
  ******************************************************************************
  */

#include "Path.h"
#include "PWM.h"
#include "MotorCtrl.h"
#include "MotionCtrl.h"
#include <stdlib.h>

#define INFRARED_PWM_DUTY               50
#define INFRARED_RX_CHAN_NUM            5

#define INFRARED_SENSE                  150     // LSB of ADC1 ~= 0.15V
#define INFRARED_RX_DEBOUNCE_CNT        3

#define ADC1_DR_Address                 ((u32)0x4001244C)
#define ADC_SAMPLE_CNT                  1

#define IFRD_CHAN_IDX_L                 1
#define IFRD_CHAN_IDX_CL                2
#define IFRD_CHAN_IDX_C                 3
#define IFRD_CHAN_IDX_CR                4
#define IFRD_CHAN_IDX_R                 5

#define INFRARED_RECV_LEFT_GPIO         GPIOA
#define INFRARED_RECV_CENTER_L_GPIO     GPIOA
#define INFRARED_RECV_CENTER_GPIO       GPIOA
#define INFRARED_RECV_CENTER_R_GPIO     GPIOA
#define INFRARED_RECV_RIGHT_GPIO        GPIOA
#define INFRARED_RECV_LEFT_PIN          GPIO_Pin_6
#define INFRARED_RECV_CENTER_L_PIN      GPIO_Pin_3
#define INFRARED_RECV_CENTER_PIN        GPIO_Pin_4
#define INFRARED_RECV_CENTER_R_PIN      GPIO_Pin_7
#define INFRARED_RECV_RIGHT_PIN         GPIO_Pin_5

#define COLLISION_DETECT_LEFT_GPIO      GPIOB
#define COLLISION_DETECT_RIGHT_GPIO     GPIOB
#define COLLISION_DETECT_LEFT_PIN       GPIO_Pin_14
#define COLLISION_DETECT_RIGHT_PIN      GPIO_Pin_15

#define EDGE_DETECT_LEFT_GPIO           GPIOE
#define EDGE_DETECT_CENTER_L_GPIO       GPIOE
#define EDGE_DETECT_CENTER_R_GPIO       GPIOE
#define EDGE_DETECT_RIGHT_GPIO          GPIOE
#define EDGE_DETECT_LEFT_PIN            GPIO_Pin_12
#define EDGE_DETECT_CENTER_L_PIN        GPIO_Pin_13
#define EDGE_DETECT_CENTER_R_PIN        GPIO_Pin_15
#define EDGE_DETECT_RIGHT_PIN           GPIO_Pin_14

#define COLLISION_DETECT_LEFT()         GPIO_ReadInputDataBit(COLLISION_DETECT_LEFT_GPIO, COLLISION_DETECT_LEFT_PIN)
#define COLLISION_DETECT_RIGHT()        GPIO_ReadInputDataBit(COLLISION_DETECT_RIGHT_GPIO, COLLISION_DETECT_RIGHT_PIN)

#define EDGE_DETECT_LEFT()              GPIO_ReadInputDataBit(EDGE_DETECT_LEFT_GPIO, EDGE_DETECT_LEFT_PIN)
#define EDGE_DETECT_CENTER_L()          GPIO_ReadInputDataBit(EDGE_DETECT_CENTER_L_GPIO, EDGE_DETECT_CENTER_L_PIN)
#define EDGE_DETECT_CENTER_R()          GPIO_ReadInputDataBit(EDGE_DETECT_CENTER_R_GPIO, EDGE_DETECT_CENTER_R_PIN)
#define EDGE_DETECT_RIGHT()             GPIO_ReadInputDataBit(EDGE_DETECT_RIGHT_GPIO, EDGE_DETECT_RIGHT_PIN)

enum PathCondtion   PathCond = PATH_COND_OK;

static u8 IFrDRxCnt[5] = {0};
static u16 LastIFrDRxLvl[5] = {0};
static u8 WheelSpeedChkPeriod = 0;
static u32 LastWheelCnt[2] = {0};
static u8 DeltaCnt[2] = {0};
static s16 CurSpeed[2];
u8 IsProcPath = 0, IsProcPath__ = 0;

static __IO u16 IFRDADCConvertedLSB[INFRARED_RX_CHAN_NUM] = {0};

void Path_Monitor(void)
{
    u8  i = 0;
    u16 IFrDRxLvl = 0;

    for(i = 0; i < INFRARED_RX_CHAN_NUM; i++){

        IFrDRxLvl = IFRDADCConvertedLSB[i];
        if(INFRARED_SENSE < abs(LastIFrDRxLvl[i] - IFrDRxLvl)){
            if(IFrDRxCnt[i] < INFRARED_RX_DEBOUNCE_CNT*2)
                IFrDRxCnt[i]++;
        }
        else{
            if(IFrDRxCnt[i] > 0)
                IFrDRxCnt[i]--;
        }
        LastIFrDRxLvl[i] = IFrDRxLvl;
    }

    if(!IsProcPath){
        if( ((IFrDRxCnt[IFRD_CHAN_IDX_R-1] >= INFRARED_RX_DEBOUNCE_CNT) && (IFrDRxCnt[IFRD_CHAN_IDX_L-1] >= INFRARED_RX_DEBOUNCE_CNT))
          ){
            PathCond = PATH_FAULT_180;
            LastWheelCnt[0] = 0;
            LastWheelCnt[1] = 0;
            MotionCtrl_AvoidLeft();
        }
        else if(
    //            (0 == EDGE_DETECT_LEFT()) ||
                (IFrDRxCnt[IFRD_CHAN_IDX_L-1] >= INFRARED_RX_DEBOUNCE_CNT)
                || (IFrDRxCnt[IFRD_CHAN_IDX_CL-1] >= INFRARED_RX_DEBOUNCE_CNT)
                || (0 == COLLISION_DETECT_LEFT())
               ){
            PathCond = PATH_FAULT_L;
            MotionCtrl_AvoidLeft();
            LastWheelCnt[0] = 0;
            LastWheelCnt[1] = 0;
        }
        else if(
    //            (0 == EDGE_DETECT_RIGHT()) ||
                (IFrDRxCnt[IFRD_CHAN_IDX_R-1] >= INFRARED_RX_DEBOUNCE_CNT)
                || (IFrDRxCnt[IFRD_CHAN_IDX_CR-1] >= INFRARED_RX_DEBOUNCE_CNT)
                || (0 == COLLISION_DETECT_RIGHT())
               ){
            PathCond = PATH_FAULT_R;
            MotionCtrl_AvoidRight();
            LastWheelCnt[0] = 0;
            LastWheelCnt[1] = 0;
        }
        else if(
    //            (0 == EDGE_DETECT_CENTER_L())
    //            || (0 == EDGE_DETECT_CENTER_R()) ||
                (IFrDRxCnt[IFRD_CHAN_IDX_C-1] >= INFRARED_RX_DEBOUNCE_CNT)
                || (0 == COLLISION_DETECT_LEFT())
                || (0 == COLLISION_DETECT_RIGHT())
               ){
            PathCond = PATH_FAULT_HEAD;
            MotionCtrl_AvoidLeft();
            LastWheelCnt[0] = 0;
            LastWheelCnt[1] = 0;
        }
        else{
            PathCond = PATH_COND_OK;
        }
    }

    if(!IsProcPath__){
        CurSpeed[0] = MotorCtrl_ChanSpeedLevelGet(MOTOR_CTRL_CHAN_LWHEEL);
        CurSpeed[1] = MotorCtrl_ChanSpeedLevelGet(MOTOR_CTRL_CHAN_RWHEEL);

        if(CurSpeed[0]==0){
            MotorCtrl_ChanUp(MOTOR_CTRL_CHAN_LWHEEL);
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, MOTOR_SPEED_LVL15);
            CurSpeed[0] = MOTOR_SPEED_LVL15;
        }
        if(CurSpeed[1]==0){
            MotorCtrl_ChanUp(MOTOR_CTRL_CHAN_RWHEEL);
            MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, MOTOR_SPEED_LVL15);
            CurSpeed[1] = MOTOR_SPEED_LVL15;
        }

        WheelSpeedChkPeriod++;
        if(0 == WheelSpeedChkPeriod%3){
            if(LastWheelCnt[0]||LastWheelCnt[1]){
                DeltaCnt[0] = (u8)(LWHEEL_COUNTER() - LastWheelCnt[0]);
                DeltaCnt[1] = (u8)(RWHEEL_COUNTER() - LastWheelCnt[1]);

                if( (DeltaCnt[0] - WHEEL_CRIUZE_SPEED) > 1 ){
                    MotorCtrl_ChanSpeedDec(MOTOR_CTRL_CHAN_LWHEEL);
                }
                else if( (WHEEL_CRIUZE_SPEED - DeltaCnt[0]) > 1 ){
                    MotorCtrl_ChanSpeedInc(MOTOR_CTRL_CHAN_LWHEEL);
                }

                if( (DeltaCnt[1] - WHEEL_CRIUZE_SPEED) > 1 ){
                    MotorCtrl_ChanSpeedDec(MOTOR_CTRL_CHAN_RWHEEL);
                }
                else if( (WHEEL_CRIUZE_SPEED - DeltaCnt[1]) > 1 ){
                    MotorCtrl_ChanSpeedInc(MOTOR_CTRL_CHAN_RWHEEL);
                }
            }
            LastWheelCnt[0] = LWHEEL_COUNTER();
            LastWheelCnt[1] = RWHEEL_COUNTER();
        }
    }
}

void Path_Init(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;
    ADC_InitTypeDef             ADC_InitStructure;
    DMA_InitTypeDef             DMA_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = INFRARED_RECV_LEFT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(INFRARED_RECV_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = INFRARED_RECV_CENTER_L_PIN;
    GPIO_Init(INFRARED_RECV_CENTER_L_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = INFRARED_RECV_CENTER_PIN;
    GPIO_Init(INFRARED_RECV_CENTER_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = INFRARED_RECV_CENTER_R_PIN;
    GPIO_Init(INFRARED_RECV_CENTER_R_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = INFRARED_RECV_RIGHT_PIN;
    GPIO_Init(INFRARED_RECV_RIGHT_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = COLLISION_DETECT_LEFT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COLLISION_DETECT_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = COLLISION_DETECT_RIGHT_PIN;
    GPIO_Init(COLLISION_DETECT_RIGHT_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = EDGE_DETECT_LEFT_PIN;
    GPIO_Init(EDGE_DETECT_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = EDGE_DETECT_CENTER_L_PIN;
    GPIO_Init(EDGE_DETECT_CENTER_L_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = EDGE_DETECT_CENTER_R_PIN;
    GPIO_Init(EDGE_DETECT_CENTER_R_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = EDGE_DETECT_RIGHT_PIN;
    GPIO_Init(EDGE_DETECT_RIGHT_GPIO, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&IFRDADCConvertedLSB;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = INFRARED_RX_CHAN_NUM * ADC_SAMPLE_CNT;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* Enable ADC1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* 12Mhz */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    /* ADC1 Config */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = INFRARED_RX_CHAN_NUM;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, IFRD_CHAN_IDX_L,   ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, IFRD_CHAN_IDX_CL,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, IFRD_CHAN_IDX_C,   ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, IFRD_CHAN_IDX_CR,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, IFRD_CHAN_IDX_R,   ADC_SampleTime_239Cycles5);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period = 900-1;                                     /* 9ms */
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 430;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM2, & TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_UpdateDisableConfig(TIM2, DISABLE);

    plat_int_reg_cb(STM32F10x_INT_TIM2, (void *)Path_Monitor);
}

void Path_PlanningStart(void)
{
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1,ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    LastWheelCnt[0] = 0;
        LastWheelCnt[1] = 0;
}

void Path_PlanningStop(void)
{
    DMA_Cmd(DMA1_Channel1, DISABLE);

    ADC_DMACmd(ADC1, DISABLE);
	ADC_Cmd(ADC1,DISABLE);

	ADC_ExternalTrigConvCmd(ADC1, DISABLE);

    TIM_Cmd(TIM2, DISABLE);
}