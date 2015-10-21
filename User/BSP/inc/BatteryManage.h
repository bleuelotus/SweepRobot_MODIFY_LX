/**
  ******************************************************************************
  * @file    Battery.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Battery management driver
  ******************************************************************************
  */

#ifndef __BATTERY_MANAGE_H__
#define __BATTERY_MANAGE_H__

#include "stm32f10x_conf.h"

#ifdef REVISION_1_0
#define BM_CHARGE_SW_STATUS_GPIO_PERIPH_ID      RCC_APB2Periph_GPIOD
#define BM_CHARGE_SW_STATUS_GPIO                GPIOD
#define BM_CHARGE_SW_STATUS_PIN                 GPIO_Pin_7
#elif defined REVISION_1_1
#define BM_CHARGE_SW_STATUS_GPIO_PERIPH_ID      RCC_APB2Periph_GPIOB
#define BM_CHARGE_SW_STATUS_GPIO                GPIOB
#define BM_CHARGE_SW_STATUS_PIN                 GPIO_Pin_5
#elif defined REVISION_1_2
#define BM_CHARGE_SW_STATUS_GPIO_PERIPH_ID      RCC_APB2Periph_GPIOB
#define BM_CHARGE_SW_STATUS_GPIO                GPIOB
#define BM_CHARGE_SW_STATUS_PIN                 GPIO_Pin_5
#endif

#define BAT_MONITOR_TIM_PERIPH_ID               RCC_APB1Periph_TIM5
#define BAT_MONITOR_TIM                         TIM5
#define BAT_MONITOR_TIM_IRQn                    TIM5_IRQn
#define BAT_MONITOR_TIM_IRQ_PP                  3
#define BAT_MONITOR_TIM_IRQ_SP                  3
#define BAT_MONITOR_TIM_INT_IDX                 STM32F10x_INT_TIM5

enum BatteryState {

    BAT_STATE_DISCHARGING,
    BAT_STATE_WAIT_FOR_CHARGE,
    BAT_STATE_CHARGING,
    BAT_STATE_CHARGE_COMPLETE,
    BAT_STATE_UNKNOWN,
};

enum BatteryEvt {
    
    BM_EVT_POWER_LOSS,
    BM_EVT_POWER_LINK,
    BM_EVT_LOW_LEVEL,
    BM_EVT_WARNING_LOW_LEVEL,
    BM_EVT_CHARGE_COMPLETE,
};

typedef struct _BatteryCond_s {

    u8                      level;
    enum BatteryState       state;
    enum BatteryState       LastState;
//    u8                      TmpDeg;
} BatteryCond_t;

extern BatteryCond_t gBM_Cond;

#define BM_BAT_WARNING_LVL                      5            /* when battery level lower than this value, then stop robot and buzzer to warning */
#define BM_BAT_CRITCAL_LVL                      15          /* 15 Percent about 10 minutes */
#define BM_BAT_FULL_LVL                         100         /* 100 Percent */

void BM_Init(void);

#endif /* !__BATTERY_MANAGE_H__ */
