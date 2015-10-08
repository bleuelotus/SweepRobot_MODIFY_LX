/**
  ******************************************************************************
  * @file    RTC.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   RealTime Clock
  ******************************************************************************
  */

#include "RTC.h"
#include "PwrManagement.h"

#define RTC_CFG_FLAG                    0xA5A5
#define RTC_CFG_FLAG_BKP_REG            BKP_DR1
#define RTC_ALR_EN_BKP_REG             BKP_DR2
#define RTC_ALR_LSB_BKP_REG             BKP_DR3
#define RTC_ALR_MSB_BKP_REG             BKP_DR4

void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready TODO: timeout proc */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

void Time_Adjust(u8 HH, u8 MM, u8 SS)
{
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Change the current time */
    RTC_SetCounter((HH*3600 + MM*60 + SS));

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

void Time_Display(void)
{
    u32 THH = 0, TMM = 0, TSS = 0;
    u32 TimeVar = RTC_GetCounter();

    /* Reset RTC Counter when Time is 23:59:59 */
    if (TimeVar == 0x0001517F)
    {
        RTC_SetCounter(0x0);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }

    /* Compute  hours */
    THH = TimeVar / 3600;
    /* Compute minutes */
    TMM = (TimeVar % 3600) / 60;
    /* Compute seconds */
    TSS = (TimeVar % 3600) % 60;
#ifdef DEBUG_LOG
    printf("Time: %0.2d:%0.2d:%0.2d\r", THH, TMM, TSS);
#endif
}

void RTC_AlarmWork(void)
{
    u32 RTCAlarmCnt = 0;

    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    RTCAlarmCnt = RTC_GetCounter() + 0x0001517F;

    RTC_SetAlarm(RTCAlarmCnt);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* save the current cnt */
    BKP_WriteBackupRegister(RTC_ALR_LSB_BKP_REG, (RTCAlarmCnt&0xFFFF));
    BKP_WriteBackupRegister(RTC_ALR_MSB_BKP_REG, ((RTCAlarmCnt>>16)&0xFFFF));

    PM_SysTryToResume();
}

void RTC_AlarmSet(u32 val)
{
    u32 RTCAlarmCnt = 0;

    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    if(val > 0){

        RTCAlarmCnt = RTC_GetCounter()+val;

        /* save the current cnt */
        BKP_WriteBackupRegister(RTC_ALR_LSB_BKP_REG, (RTCAlarmCnt&0xFFFF));
        BKP_WriteBackupRegister(RTC_ALR_MSB_BKP_REG, ((RTCAlarmCnt>>16)&0xFFFF));
    }
    else{
        RTCAlarmCnt = 0;
    }

    RTC_SetAlarm(RTCAlarmCnt);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    /* save user setting */
    BKP_WriteBackupRegister(RTC_ALR_EN_BKP_REG, val ? 1 : 0);
}

s8 RTC_AlarmGet(void)
{
    return (BKP_ReadBackupRegister(RTC_ALR_EN_BKP_REG) ? 1 : 0);
}

s8 RTC_LoadAlarmCfg(void)
{
    u16 AlarmEN = 0;
    u32 AlarmCnt = 0;

    AlarmEN = BKP_ReadBackupRegister(RTC_ALR_EN_BKP_REG);

    if(AlarmEN != 0){

        AlarmCnt = BKP_ReadBackupRegister(RTC_ALR_MSB_BKP_REG);
        AlarmCnt = AlarmCnt << 16;
        AlarmCnt |= BKP_ReadBackupRegister(RTC_ALR_LSB_BKP_REG);

        /* Wakeup by alarm */
        if(AlarmCnt < RTC_GetCounter()){
            /* Enable PWR and BKP clocks */
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

            /* Allow access to BKP Domain */
            PWR_BackupAccessCmd(ENABLE);

            AlarmCnt = RTC_GetCounter() + 0x0001517F;

            RTC_SetAlarm(AlarmCnt);
            /* Wait until last write operation on RTC registers has finished */
            RTC_WaitForLastTask();

            /* save the current cnt */
            BKP_WriteBackupRegister(RTC_ALR_LSB_BKP_REG, (AlarmCnt&0xFFFF));
            BKP_WriteBackupRegister(RTC_ALR_MSB_BKP_REG, ((AlarmCnt>>16)&0xFFFF));

            return 1;
        }
    }
    return 0;
}

s8 RTC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    s8 flag = 0;

    if (BKP_ReadBackupRegister(RTC_CFG_FLAG_BKP_REG) != RTC_CFG_FLAG)
    {
        /* Backup data register value is not correct or not yet programmed (when
           the first time the program is executed) */
        RTC_Configuration();

        BKP_WriteBackupRegister(RTC_CFG_FLAG_BKP_REG, RTC_CFG_FLAG);

        BKP_WriteBackupRegister(RTC_ALR_EN_BKP_REG, 0);
    }

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    flag = RTC_LoadAlarmCfg();

    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the RTC Second/ALARM */
    RTC_ITConfig(RTC_IT_SEC|RTC_IT_ALR, ENABLE);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Clear reset flags */
    RCC_ClearFlag();

    plat_int_reg_cb(STM32F10x_INT_RTC_SEC, (void*)PM_SysTryToStandby);
    plat_int_reg_cb(STM32F10x_INT_RTC_ALR, (void*)RTC_AlarmWork);

    return flag;
}

