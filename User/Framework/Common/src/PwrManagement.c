/******************** (C) COPYRIGHT 2007 EJ ********************
* File Name          : PwrManagement.c
* Author             : Reason Chen
* Version            : V1.0
* Date               : 06/12/2014
* Description        : Power Management Policy
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "boardcfg.h"
#include "PwrManagement.h"
#include "delay.h"
#include "IrDA.h"
#include "SweepRobot.h"

u8 gSystemIdleCnt = PM_SYS_SB_SEC;

void PM_SysTryToStandby(void)
{
    Msg_t   Msg;

    gSystemIdleCnt--;
    if(gSystemIdleCnt==0){
        Msg.expire = 0;
        Msg.prio = MSG_PRIO_HIGHEST;
        Msg.type = MSG_TYPE_PM;
        Msg.MsgCB = NULL;
        Msg.Data.PMEvt = PM_MODE_STANDBY;
        SweepRobot_SendMsg(&Msg);
    }
}

void PM_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(POWER_SUPPLY_CTRL_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = POWER_SUPPLY_CTRL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(POWER_SUPPLY_CTRL_GPIO, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    /* Enable Wake up pin */
    PWR_WakeUpPinCmd(ENABLE);

    /* Powered on by default */
    PM_POWER_SUPPLY_CTRL(PM_ON);

    mDelay(500);
}

void PM_DeInit(void)
{

}

s8 PM_EnterPwrMode(enum PM_Mode mode)
{
    switch(mode)
    {
        case PM_MODE_RUN:
            break;
        case PM_MODE_SLEEP:
        case PM_MODE_STOP:
        case PM_MODE_STANDBY:
            /* Prepare for enterring to low power state */
            IrDA_DeInit();
            PM_POWER_SUPPLY_CTRL(PM_OFF);
            /* Enter to low power state */
            PWR_EnterSTANDBYMode();
            break;
    }
    return 0;
}