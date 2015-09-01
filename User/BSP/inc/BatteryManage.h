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

#define BM_CHARGE_SW_STATUS_GPIO_PERIPH_ID      RCC_APB2Periph_GPIOD
#define BM_CHARGE_SW_STATUS_GPIO                GPIOD
#define BM_CHARGE_SW_STATUS_PIN                 GPIO_Pin_7

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
    BM_EVT_CHARGE_COMPLETE,
};

typedef struct _BatteryCond_s {

    u8                      level;
    enum BatteryState       state;
    enum BatteryState       LastState;
//    u8                      TmpDeg;
} BatteryCond_t;

extern BatteryCond_t gBM_Cond;

#define BM_BAT_CRITCAL_LVL                      10       /* 10 Percent */
#define BM_BAT_FULL_LVL                         100      /* 100 Percent */

void BM_Init(void);

#endif /* !__BATTERY_MANAGE_H__ */
