/**
  ******************************************************************************
  * @file    MsgQueue.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Message Queue driver
  ******************************************************************************
  */

#ifndef __MSG_QUEUE_H__
#define __MSG_QUEUE_H__

#include "stm32f10x_conf.h"
#include "BatteryManage.h"
#include "MotionCtrl.h"
#include "IrDA.h"
#include "PwrManagement.h"

enum {

    MSG_PRIO_HIGHEST = 0,
    MSG_PRIO_HIGH,
    MSG_PRIO_NORMAL,
    MSG_PRIO_LOW,
    MSG_PRIO_LOWEST
};

typedef enum {

    MSG_TYPE_PM = 0,
    MSG_TYPE_BM,
    MSG_TYPE_CTRL,
    MSG_TYPE_PWR_STATION,
    MSG_TYPE_MOTION,
} MsgType_t;

typedef union _DataType_s{

    enum PM_Mode        PMEvt;
    u8                  ByteVal;
    enum BatteryEvt     BatEvt;
    enum MotionEvt      MEvt;
    PwrStationSigData_t PSSigDat;
} MsgData_t;

typedef struct _Msg_s {

    u8              expire;
    u8              prio;
    MsgType_t       type;
    MsgData_t       Data;
    void            (*MsgCB)(void);
} Msg_t;

typedef struct _MsgQueue_s{

    struct _MsgQueue_s  *Prev;
    struct _MsgQueue_s  *Next;
    Msg_t               Msg;
} MsgQueue_t;


MsgQueue_t* MsgQueue_Create(u8 QueueSize);
s8 MsgQueue_InQueue(MsgQueue_t *MsgQ, u8 Qsize, Msg_t *Msg);
s8 MsgQueue_DeQueue(MsgQueue_t *MsgQ);

#endif /* __MSG_QUEUE_H__ */
