/******************** (C) COPYRIGHT 2011 MiraMEMS ********************
* File Name          : nvram.c
* Author             : lschen@miramems.com
* Version            : V1.0
* Date               : 07/07/2014
* Description        : Memory to save user data
*******************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_UserMEM.h"
#include "nvram.h"
#include <stdio.h>

void NVRAM_ProtectOff(void)
{
    UserMEM_Init();
}

void NVRAM_ProtectOn(void)
{
    UserMEM_DeInit();
}

s8 NVRAM_WriteEntry(NVRAM_Entry_t *Entry)
{
    u32     i;

    if( Entry->valid ){
        for(i=1;i<Entry->len;i++){
            if( !NVRAM_ADDR_IS_VALID(USER_MEM_UNIT*(Entry->offset+i)) )
                return -1;
            UserMEM_ByteWrite(USER_MEM_UNIT*(Entry->offset+i), *(Entry->data++));
        }
    }
    /* Program valid bit */
    UserMEM_ByteWrite(USER_MEM_UNIT*Entry->offset, Entry->valid ? 'Y':'N');

    return 0;
}

s8 NVRAM_ReadEntry(NVRAM_Entry_t *Entry)
{
    u32     i;
    u8      tmp = 0;

    /* READ valid bit */
    tmp = UserMEM_ByteRead(USER_MEM_UNIT*Entry->offset);
//    printf("--%c--%d--\r\n",tmp, tmp);
    Entry->valid = (tmp == 'Y') ? 1 : 0;

    if( Entry->valid ){
        for(i=1;i<Entry->len;i++){
            if( !NVRAM_ADDR_IS_VALID(USER_MEM_UNIT*(Entry->offset+i)) )
                return -1;
            *(Entry->data++) = UserMEM_ByteRead(USER_MEM_UNIT*(Entry->offset+i));
        }
    }

    return 0;
}

s8 NVRAM_EraseEntry(u32 EntryOffset)
{
    /* Program valid bit */
    UserMEM_EraseByte(EntryOffset);

    return 0;
}
