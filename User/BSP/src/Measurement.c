/**
  ******************************************************************************
  * @file    Measurement.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   ADC measurement collection
  ******************************************************************************
  */

#include "Measurement.h"

#define ADC1_DR_Address                 ((u32)0x4001244C)


__IO u16 ADCConvertedLSB[MEAS_CHAN_NUM] = {0};
__IO float ADC2ValueConvertedLSB[MEAS_CHAN_NUM] = {0};

void Meas_Init(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    ADC_InitTypeDef             ADC_InitStructure;
    DMA_InitTypeDef             DMA_InitStructure;

    RCC_APB2PeriphClockCmd(AD_CHAN_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = AD_IFRD_FRONT_RX_LEFT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(AD_IFRD_FRONT_RX_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_SIDE_RX_LEFT_PIN;
    GPIO_Init(AD_IFRD_SIDE_RX_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_FRONT_RX_RIGHT_PIN;
    GPIO_Init(AD_IFRD_FRONT_RX_RIGHT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_SIDE_RX_RIGHT_PIN;
    GPIO_Init(AD_IFRD_SIDE_RX_RIGHT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_BOTTOM_RX_LEFT_PIN;
    GPIO_Init(AD_IFRD_BOTTOM_RX_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_BOTTOM_RX_RIGHT_PIN;
    GPIO_Init(AD_IFRD_BOTTOM_RX_RIGHT_GPIO, &GPIO_InitStructure);
#ifdef REVISION_1_0
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_BOTTOM_RX_BL_PIN;
    GPIO_Init(AD_IFRD_BOTTOM_RX_BL_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_BOTTOM_RX_BR_PIN;
    GPIO_Init(AD_IFRD_BOTTOM_RX_BR_GPIO, &GPIO_InitStructure);
#elif defined REVISION_1_1
    GPIO_InitStructure.GPIO_Pin = AD_UNIVERSAL_WHEEL_SIG_PIN;
    GPIO_Init(AD_UNIVERSAL_WHEEL_SIG_GPIO, &GPIO_InitStructure);
#elif defined REVISION_1_2
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_BOTTOM_RX_BACK_PIN;
    GPIO_Init(AD_IFRD_BOTTOM_RX_BACK_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_UNIVERSAL_WHEEL_SIG_PIN;
    GPIO_Init(AD_UNIVERSAL_WHEEL_SIG_GPIO, &GPIO_InitStructure);
#endif
    GPIO_InitStructure.GPIO_Pin = AD_BRUSH_CUR_LEFT_PIN;
    GPIO_Init(AD_BRUSH_CUR_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_BRUSH_CUR_RIGHT_PIN;
    GPIO_Init(AD_BRUSH_CUR_RIGHT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_BRUSH_CUR_MIDDLE_PIN;
    GPIO_Init(AD_BRUSH_CUR_MIDDLE_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_FAN_CUR_PIN;
    GPIO_Init(AD_FAN_CUR_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_BAT_CHARGE_CUR_PIN;
    GPIO_Init(AD_BAT_CHARGE_CUR_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_BAT_VOL_PIN;
    GPIO_Init(AD_BAT_VOL_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = AD_ASH_TRAY_LVL_PIN;
    GPIO_Init(AD_ASH_TRAY_LVL_GPIO, &GPIO_InitStructure);

#ifdef REVISION_1_2
    RCC_APB2PeriphClockCmd(AD_IFRD_BOTTOM_RX_SWITCH_GPIO_PERIPH_ID, ENABLE);
    GPIO_InitStructure.GPIO_Pin = AD_IFRD_BOTTOM_RX_SWITCH_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(AD_IFRD_BOTTOM_RX_SWITCH_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(AD_IFRD_BOTTOM_RX_SWITCH_GPIO, AD_IFRD_BOTTOM_RX_SWITCH_PIN);
#endif

    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADCConvertedLSB;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = MEAS_CHAN_NUM * MEAS_SAMPLE_CNT;
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
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = MEAS_CHAN_NUM;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2,   MEAS_CHAN_IFRD_FRONT_RX_L,      ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1,   MEAS_CHAN_IFRD_FRONT_RX_R,      ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,   MEAS_CHAN_IFRD_SIDE_RX_L,       ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10,  MEAS_CHAN_IFRD_SIDE_RX_R,       ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3,   MEAS_CHAN_IFRD_BOTTOM_RX_L,     ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11,  MEAS_CHAN_IFRD_BOTTOM_RX_R,     ADC_SampleTime_55Cycles5);
#ifdef REVISION_1_0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15,  MEAS_CHAN_IFRD_BOTTOM_RX_BL,    ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8,   MEAS_CHAN_IFRD_BOTTOM_RX_BR,    ADC_SampleTime_55Cycles5);
#elif defined REVISION_1_1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15,  MEAS_CHAN_UNIVERSAL_WHEEL_SIG,  ADC_SampleTime_55Cycles5);
#elif defined REVISION_1_2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8,   MEAS_CHAN_IFRD_BOTTOM_RX_B,     ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15,  MEAS_CHAN_UNIVERSAL_WHEEL_SIG,  ADC_SampleTime_55Cycles5);
#endif
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5,   MEAS_CHAN_BRUSH_CUR_LEFT,       ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9,   MEAS_CHAN_BRUSH_CUR_RIGHT,      ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7,   MEAS_CHAN_BRUSH_CUR_MIDDLE,     ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6,   MEAS_CHAN_FAN_CUR,              ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12,  MEAS_CHAN_BAT_CHARGE_CUR,       ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13,  MEAS_CHAN_BAT_VOL,              ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14,  MEAS_CHAN_ASH_TRAY_LVL,         ADC_SampleTime_55Cycles5);
#ifdef REVISION_1_2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_17,  MEAS_CHAN_INTERNAL_REFVOL,      ADC_SampleTime_55Cycles5);
#endif
}

void Meas_Start(void)
{
    ADC_TempSensorVrefintCmd(ENABLE);

    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void Meas_Stop(void)
{
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);

    ADC_TempSensorVrefintCmd(DISABLE);

    DMA_Cmd(DMA1_Channel1, DISABLE);

    ADC_DMACmd(ADC1, DISABLE);
    ADC_Cmd(ADC1,DISABLE);
}

void Meas_IFRD_BOTTOM_RX_SWITCH(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, u8 GPIO_Switch_Lvl)
{
    GPIO_WriteBit(GPIOx, GPIO_Pin, GPIO_Switch_Lvl);
}
