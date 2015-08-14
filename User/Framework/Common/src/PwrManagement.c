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

void PM_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(POWER_SUPPLY_CTRL_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = POWER_SUPPLY_CTRL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(POWER_SUPPLY_CTRL_GPIO, &GPIO_InitStructure);

    /* Powered on by default */
    PM_POWER_SUPPLY_CTRL(PM_ON);

    mDelay(500);
}

void PM_DeInit(void)
{

}

s8 PM_EnterPwrMode(enum PM_Mode mode)
{
    return 0;
}