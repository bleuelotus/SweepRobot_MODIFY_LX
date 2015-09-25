/**
  ******************************************************************************
  * @file    MsgQueue.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Message Queue driver
  ******************************************************************************
  */

#include "MsgQueue.h"
#include <stdlib.h>
#include <string.h>


MsgQueue_t* MsgQueue_Create(u8 QueueSize)
{
    MsgQueue_t* pMsgQ = NULL;
    MsgQueue_t* pMsgQHead = NULL;
    MsgQueue_t* pLastMsgQueue = NULL;
    u8          i = 0;

    pMsgQHead = (MsgQueue_t*)malloc(sizeof(MsgQueue_t));
    memset(pMsgQHead, 0, sizeof(MsgQueue_t));
    /* expired by default */
    pMsgQHead->Msg.expire = 1;
    pLastMsgQueue = pMsgQHead;

    for(i = 0; i < QueueSize-1; i++){

        pMsgQ = malloc(sizeof(MsgQueue_t));
        memset(pMsgQ, 0, sizeof(MsgQueue_t));
        /* expired by default */
        pMsgQ->Msg.expire = 1;

        pLastMsgQueue->Next = pMsgQ;
        pMsgQ->Prev = pLastMsgQueue;
        pLastMsgQueue = pMsgQ;
    }

    pMsgQ->Next = pMsgQHead;
    pMsgQHead->Prev = pMsgQ;

    return pMsgQHead;
}

s8 MsgQueue_InQueue(MsgQueue_t *MsgQ, u8 Qsize, Msg_t *Msg)
{
    u8  i = 0;
    MsgQueue_t *pMsgQ = MsgQ;

    if(MsgQ == NULL || Msg == NULL){
        return -1;
    }

    /* FIXME: critcal area protection */
//    __disable_irq();

    for(i = 0; i < Qsize; i++){
        if(!pMsgQ->Msg.expire){
            pMsgQ = pMsgQ->Next;
            continue;
        }
        else{
            memcpy(&pMsgQ->Msg, Msg, sizeof(Msg_t));
            break;
        }
    }

//    __enable_irq();

    return ((i==Qsize) ? -1 : 0);
}

s8 MsgQueue_DeQueue(MsgQueue_t *MsgQ)
{
    if(MsgQ == NULL)
        return -1;

    MsgQ->Msg.expire = 1;

    return 0;
}
