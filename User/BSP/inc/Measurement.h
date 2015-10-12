/**
  ******************************************************************************
  * @file    Measurement.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   ADC measurement collection
  ******************************************************************************
  */

#ifndef __MEASUREMENT_H__
#define __MEASUREMENT_H__

#include "stm32f10x_conf.h"

#define MEAS_SAMPLE_CNT                 1
#define MEAS_CHAN_NUM                   (MEAS_CHAN_BOUND-1)

enum {
    MEAS_CHAN_IFRD_FRONT_RX_L = 1,
    MEAS_CHAN_IFRD_FRONT_RX_R,
    MEAS_CHAN_IFRD_SIDE_RX_L,
    MEAS_CHAN_IFRD_SIDE_RX_R,
    MEAS_CHAN_IFRD_BOTTOM_RX_L,
    MEAS_CHAN_IFRD_BOTTOM_RX_R,
#ifdef REVISION_1_0
    MEAS_CHAN_IFRD_BOTTOM_RX_BL,
    MEAS_CHAN_IFRD_BOTTOM_RX_BR,
#elif defined REVISION_1_1
    MEAS_CHAN_UNIVERSAL_WHEEL_SIG,
#endif
    MEAS_CHAN_BRUSH_CUR_LEFT,
    MEAS_CHAN_BRUSH_CUR_RIGHT,
    MEAS_CHAN_BRUSH_CUR_MIDDLE,
    MEAS_CHAN_FAN_CUR,
    MEAS_CHAN_BAT_CHARGE_CUR,
    MEAS_CHAN_BAT_VOL,
    MEAS_CHAN_ASH_TRAY_LVL,
    MEAS_CHAN_BOUND
};

#define AD_CHAN_GPIO_PERIPH_ID          (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC)
#define AD_IFRD_FRONT_RX_LEFT_GPIO      GPIOA
#define AD_IFRD_FRONT_RX_LEFT_PIN       GPIO_Pin_2
#define AD_IFRD_SIDE_RX_LEFT_GPIO       GPIOA
#define AD_IFRD_SIDE_RX_LEFT_PIN        GPIO_Pin_4
#define AD_IFRD_FRONT_RX_RIGHT_GPIO     GPIOA
#define AD_IFRD_FRONT_RX_RIGHT_PIN      GPIO_Pin_1
#define AD_IFRD_SIDE_RX_RIGHT_GPIO      GPIOC
#define AD_IFRD_SIDE_RX_RIGHT_PIN       GPIO_Pin_0
#define AD_IFRD_BOTTOM_RX_LEFT_GPIO     GPIOA
#define AD_IFRD_BOTTOM_RX_LEFT_PIN      GPIO_Pin_3
#define AD_IFRD_BOTTOM_RX_RIGHT_GPIO    GPIOC
#define AD_IFRD_BOTTOM_RX_RIGHT_PIN     GPIO_Pin_1
#ifdef REVISION_1_0
#define AD_IFRD_BOTTOM_RX_BL_GPIO       GPIOC
#define AD_IFRD_BOTTOM_RX_BL_PIN        GPIO_Pin_5
#define AD_IFRD_BOTTOM_RX_BR_GPIO       GPIOB
#define AD_IFRD_BOTTOM_RX_BR_PIN        GPIO_Pin_0
#elif defined REVISION_1_1
#define AD_UNIVERSAL_WHEEL_SIG_GPIO     GPIOC
#define AD_UNIVERSAL_WHEEL_SIG_PIN      GPIO_Pin_5
#endif
#define AD_BRUSH_CUR_LEFT_GPIO          GPIOA
#define AD_BRUSH_CUR_LEFT_PIN           GPIO_Pin_5
#define AD_BRUSH_CUR_RIGHT_GPIO         GPIOB
#define AD_BRUSH_CUR_RIGHT_PIN          GPIO_Pin_1
#define AD_BRUSH_CUR_MIDDLE_GPIO        GPIOA
#define AD_BRUSH_CUR_MIDDLE_PIN         GPIO_Pin_7
#define AD_FAN_CUR_GPIO                 GPIOA
#define AD_FAN_CUR_PIN                  GPIO_Pin_6
#define AD_BAT_CHARGE_CUR_GPIO          GPIOC
#define AD_BAT_CHARGE_CUR_PIN           GPIO_Pin_2
#define AD_BAT_VOL_GPIO                 GPIOC
#define AD_BAT_VOL_PIN                  GPIO_Pin_3
#define AD_ASH_TRAY_LVL_GPIO            GPIOC
#define AD_ASH_TRAY_LVL_PIN             GPIO_Pin_4

extern __IO u16 ADCConvertedLSB[MEAS_CHAN_NUM];

void Meas_Init(void);
void Meas_Start(void);
void Meas_Stop(void);

#endif /* __MEASUREMENT_H__ */
