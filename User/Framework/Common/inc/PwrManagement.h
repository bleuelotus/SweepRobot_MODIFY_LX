/******************** (C) COPYRIGHT 2007 EJE ********************
* File Name          : PwrManagement.h
* Author             : Reason Chen
* Version            : V1.0
* Date               : 05/5/2015
* Description        : Power Management Policy
*******************************************************************************/

#ifndef __PWRMANAGEMENT_H__
#define __PWRMANAGEMENT_H__

#include "stm32f10x.h"

enum PM_Mode {

    PM_MODE_NONE,
    PM_MODE_RUN,
    PM_MODE_SLEEP,
    PM_MODE_STOP,
    PM_MODE_STANDBY
};

enum PM_Switch {

    PM_OFF,
    PM_ON
};

typedef struct {

    enum PM_Mode    LastPwrState;
    enum PM_Mode    CurPwrState;
    s8              (*Resume)(enum PM_Mode LastState);
    s8              (*Suspend)(enum PM_Mode State);
} PM_Device_t;

extern u8 gSystemIdleCnt;

#define PM_SYS_SB_SEC                       15

#define POWER_SUPPLY_CTRL_GPIO_PERIPH_ID    RCC_APB2Periph_GPIOA
#define POWER_SUPPLY_CTRL_GPIO              GPIOA
#define POWER_SUPPLY_CTRL_PIN               GPIO_Pin_9

#define PM_POWER_SUPPLY_CTRL(s)             GPIO_WriteBit(POWER_SUPPLY_CTRL_GPIO, POWER_SUPPLY_CTRL_PIN, (BitAction)s)

#define PM_ResetSysIdleState()              do{gSystemIdleCnt=PM_SYS_SB_SEC;}while(0);

void PM_Init(void);
void PM_DeInit(void);
void PM_SysTryToStandby(void);
s8 PM_EnterPwrMode(enum PM_Mode mode);

#endif /* !__PWRMANAGEMENT_H__ */
