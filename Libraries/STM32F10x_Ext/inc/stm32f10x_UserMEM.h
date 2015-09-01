/******************** (C) COPYRIGHT 2011 MiraMEMS ********************
* File Name          : stm32f10x_UserMEM.h
* Author             : lschen@miramems.com
* Version            : V1.0
* Date               : 01/10/2014
* Description        : Read/Write operations to EEPROM
*******************************************************************************/

#ifndef __STM32F10X_USERMEM_H__
#define __STM32F10X_USERMEM_H__

#include "stm32f10x_flash.h"

#define STM32F10X_FLASH_BASE        0x08000000
#define STM32F10X_FLASH_SIZE        0x00040000          // 256KB
#define STM32F10X_USER_MEM_SIZE     0x800               // 2Kb per-page for HD, 1Kb for LD/MD
#define STM32F10X_USER_MEM_BASE     (STM32F10X_FLASH_BASE+STM32F10X_FLASH_SIZE-STM32F10X_USER_MEM_SIZE)

#define USER_MEM_START_ADDR         0
#define USER_MEM_END_ADDR           (STM32F10X_USER_MEM_SIZE-1)
#define USER_MEM_UNIT               2                   // byte

void UserMEM_Init(void);
void UserMEM_DeInit(void);
void UserMEM_ByteWrite(u32 addr, u8 dat);
u8 UserMEM_ByteRead(u32 addr);
void UserMEM_EraseByte(u32 addr);

#endif /* !__STM32F10X_USERMEM_H__ */
