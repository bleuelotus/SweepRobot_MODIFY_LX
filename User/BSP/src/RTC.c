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

    /* Wait till LSE is ready */
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

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

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

void RTC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
    {
        /* Backup data register value is not correct or not yet programmed (when
           the first time the program is executed) */
        RTC_Configuration();

        BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    }
    else
    {
//        /* Check if the Power On Reset flag is set */
//        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
//        {
//          printf("\r\n\n Power On Reset occurred....");
//        }
//        /* Check if the Pin Reset flag is set */
//        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
//        {
//          printf("\r\n\n External Reset occurred....");
//        }

        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();

        /* Enable the RTC Second */
        RTC_ITConfig(RTC_IT_SEC, ENABLE);

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }

    /* Clear reset flags */
    RCC_ClearFlag();

    plat_int_reg_cb(STM32F10x_INT_RTC_SEC, (void*)Time_Display);
}

void RTC_AlarmWork(void)
{

}

