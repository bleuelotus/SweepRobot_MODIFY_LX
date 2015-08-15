/******************** (C) COPYRIGHT 2011 MiraMEMS ********************
* File Name          : nvram.h
* Author             : lschen@miramems.com
* Version            : V1.0
* Date               : 07/07/2014
* Description        : Memory to save user data
*******************************************************************************/

#ifndef __NVRAM_H__
#define __NVRAM_H__

#include "stm32f10x.h"

#define NVRAM_START                 USER_MEM_START_ADDR
#define NVRAM_END                   USER_MEM_END_ADDR
#define NVRAM_PAGE_SIZE             USER_MEM_PAGE_SIZE
#define NVRAM_UNIT                  USER_MEM_UNIT

#define NVRAM_ADDR_IS_VALID(A)      (((A)>=NVRAM_START)&&((A)<=NVRAM_END))

typedef struct NVRAM_Entry_s {
    u8      valid;
    u8      *data;
    u32     offset;
    u32     len;
} NVRAM_Entry_t;

/*
 * Temporary solution
 */
void NVRAM_ProtectOff(void);
void NVRAM_ProtectOn(void);
s8 NVRAM_WriteEntry(NVRAM_Entry_t *Entry);
s8 NVRAM_ReadEntry(NVRAM_Entry_t *Entry);
s8 NVRAM_EraseEntry(u32 EntryOffset);

#endif /* __NVRAM_H__ */