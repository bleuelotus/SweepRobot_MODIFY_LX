/**
  ******************************************************************************
  * @file    Key.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   GPIO key driver
  ******************************************************************************
  */

#include "key.h"
#include <stdlib.h>
#include <string.h>

#ifdef KEY_PROC_WITH_REMOTE
#include "IrDA.h"
#include "CtrlPanel.h"
#endif
#ifdef KEY_PROC_WITH_LED
#include "CtrlPanel.h"
#define KEY_SCAN_RATIO                      (KEY_SCAN_INTERVAL_MS*10/KEY_SCAN_TIM_PERIOD_MSx10)
static u16 KeyScanCnt = 0;
#endif

typedef enum {

    KEY_STATE_NONE = 0,
    KEY_STATE_DOWN,
    KEY_STATE_WAITUP
} KeyState_t;

typedef struct KeyList_s{

    struct KeyList_s    *Prev;
    struct KeyList_s    *Next;
    Key_t               Key;
    KeyState_t          State;
} KeyList_t;

KeyList_t    *KeyListHead = NULL;
void Key_EvtProccess(void);

void Key_CoreInit(void)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = KEY_POLLING_TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = KEY_POLLING_TIM_IRQ_PP;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = KEY_POLLING_TIM_IRQ_SP;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(KEY_POLLING_TIM_PERIPH_ID , ENABLE);
    TIM_DeInit(KEY_POLLING_TIM);
    TIM_TimeBaseStructure.TIM_Period = KEY_SCAN_TIM_PERIOD_MSx10*10-1;          //ms
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(KEY_POLLING_TIM, &TIM_TimeBaseStructure);
    TIM_ClearFlag(KEY_POLLING_TIM, TIM_FLAG_Update);
    TIM_ITConfig(KEY_POLLING_TIM, TIM_IT_Update, ENABLE);

    plat_int_reg_cb(KEY_POLLING_TIM_INT_IDX, (void*)Key_EvtProccess);
}
/******************************************************************************/

void Key_StartListen(void)
{
    TIM_Cmd(KEY_POLLING_TIM, ENABLE);
}

void Key_StopListen(void)
{
    TIM_Cmd(KEY_POLLING_TIM, DISABLE);
}

s8 Key_Register(Key_t *key)
{
    KeyList_t           *pKeyList = KeyListHead;
    GPIO_InitTypeDef    GPIO_InitStructure;
    KeyList_t           *KeyNode = NULL;

    if( NULL==key )
        return -1;

    RCC_APB2PeriphClockCmd(key->RCCResID, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = key->GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(key->GPIOx, &GPIO_InitStructure);

    KeyNode = malloc(sizeof(KeyList_t));
    if( NULL==KeyNode )
        return -1;
    memset(KeyNode, 0, sizeof(KeyList_t));
    memcpy(&KeyNode->Key, key, sizeof(Key_t));

    if( NULL == KeyListHead ){
        KeyListHead = KeyNode;
    }
    else{
        while( pKeyList->Next ){
            pKeyList = pKeyList->Next;
        }
        pKeyList->Next = KeyNode;
        KeyNode->Prev = pKeyList;
    }

    return 0;
}

s8 Key_Deregister(Key_t *key)
{
    KeyList_t   *pKeyList = KeyListHead;

    if( NULL==key )
        return -1;

    while( NULL != pKeyList ){
        if( pKeyList->Key.GPIOx==key->GPIOx && pKeyList->Key.GPIO_Pin==key->GPIO_Pin ){
            if( pKeyList->Prev ){
                pKeyList->Prev->Next = pKeyList->Next;
            }
            if( pKeyList->Next ){
                pKeyList->Next->Prev = pKeyList->Prev;
            }
            free(pKeyList);
            pKeyList = NULL;
            return 0;
        }
        pKeyList = pKeyList->Next;
    }
    return -1;
}

/******************************************************************************/
void Key_EvtProccess(void)
{
    KeyList_t   *pKeyList = KeyListHead;

    if((KeyScanCnt++)%KEY_SCAN_RATIO){
#ifdef KEY_PROC_WITH_LED
        CtrlPanel_LEDCtrlProc();
#endif
        return;
    }

    while( pKeyList ){
        if(GPIO_ReadInputDataBit(pKeyList->Key.GPIOx, pKeyList->Key.GPIO_Pin)){
            switch(pKeyList->State){
                case KEY_STATE_NONE:
                    pKeyList->State = KEY_STATE_DOWN;
                    break;
                case KEY_STATE_DOWN:
                    pKeyList->State = KEY_STATE_WAITUP;
                    if(pKeyList->Key.EvtCB.Down){
                        pKeyList->Key.EvtCB.Down();
                    }
                    break;
                default:
                    break;
            }
        }
        else{
            if(pKeyList->State == KEY_STATE_WAITUP){
                if(pKeyList->Key.EvtCB.Up){
                    pKeyList->Key.EvtCB.Up();
                }
            }
            pKeyList->State = KEY_STATE_NONE;
        }
        pKeyList = pKeyList->Next;
    }
#ifdef KEY_PROC_WITH_REMOTE
    IrDA_ProcessEJE(CtrlPanel_RemoteCB);
#endif
    return;
} 

/***************************** end key file************************************/
