/******************** (C) COPYRIGHT 2011 MiraMEMS ********************
* File Name          : stm32f10x_UserMEM.c
* Author             : lschen@miramems.com
* Version            : V1.0
* Date               : 07/07/2014
* Description        : Read/Write operations to user memory
*******************************************************************************/

#include <stdio.h>
#include "stm32f10x_UserMEM.h"

void UserMEM_Init(void)
{
    /* Unlock flash memory */
    FLASH_Unlock();
    /* Clear All pending flags */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
}

void UserMEM_DeInit(void)
{
    /* Unlock flash memory */
    FLASH_Lock();
}

void UserMEM_ByteWrite(u32 addr, u8 dat)
{
    volatile FLASH_Status   FLASHStatus = FLASH_COMPLETE;
    u16                     HalfWord = (dat<<8)|dat;

    FLASHStatus = FLASH_ProgramHalfWord(addr+STM32F10X_USER_MEM_BASE, HalfWord);
    /* Wait until End of programming */
    if(FLASH_COMPLETE != FLASHStatus){
        printf("Failed to program addr %d.\r\n", addr+STM32F10X_USER_MEM_BASE);
    }
}

u8 UserMEM_ByteRead(u32 addr)
{
   return (u8)(0xFF|(*((u16*)(addr+STM32F10X_USER_MEM_BASE))));
}

void UserMEM_EraseByte(u32 addr)
{
    //UserMEM_ByteWrite(addr, 'N');
    FLASH_ErasePage(STM32F10X_USER_MEM_BASE);
}
