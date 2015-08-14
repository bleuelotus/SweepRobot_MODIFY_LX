/**
  ******************************************************************************
  * @file    MotionCtrl.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Robot motion control driver
  ******************************************************************************
  */

#include <stdlib.h>
#include "MotionCtrl.h"
#include "Measurement.h"
#include "delay.h"
#include "MsgQueue.h"
#include "SweepRobot.h"
#include "CtrlPanel.h"

#define IFRD_CHAN_FRONT_L           0
#define IFRD_CHAN_FRONT_R           1
#define IFRD_CHAN_SIDE_L            2
#define IFRD_CHAN_SIDE_R            3
#define IFRD_CHAN_BOTTOM_L          4
#define IFRD_CHAN_BOTTOM_R          5
#define IFRD_CHAN_BOTTOM_BL         6
#define IFRD_CHAN_BOTTOM_BR         7

/* Infrared based proximity detection sensitivity */
const u16 gProximityDetectionThreshold[IFRD_TxRx_CHAN_NUM] = { 800, 800, 500, 200, 1500, 1500 };

static u16 gWheelCnt[WHEEL_NUM] = {0};
static u16 gLWheelExpCnt = 0xFFFF, gRWheelExpCnt = 0xFFFF;

typedef void (*WheelExpFunc_t)(void);

static WheelExpFunc_t  gLWheelExpCB = NULL;
static WheelExpFunc_t  gRWheelExpCB = NULL;

static u16 gLastWheelCnt[WHEEL_NUM] = {0};
static u16 gDeltaWheelCnt[WHEEL_NUM] = {0};
static u8 gCurLWheelSpeed = WHEEL_CRUISE_SPEED, gCurRWheelSpeed = WHEEL_CRUISE_SPEED;

MCtrl_Act_t gActSequence[MCTRL_ACT_MAX_DEPTH] = {0};
u8 gActSeqDepth = 0;
u8 gActSeqDepLIndicator = 0, gActSeqDepRIndicator = 0;

#define WHEEL_IDX_L                                 0
#define WHEEL_IDX_R                                 1
#define WHEEL_IDX_F                                 2
#define FWHEEL_CNT                                  gWheelCnt[WHEEL_IDX_F]
#define LWHEEL_CNT                                  gWheelCnt[WHEEL_IDX_L]
#define RWHEEL_CNT                                  gWheelCnt[WHEEL_IDX_R]
#define FWHEEL_CNT_CLR()                            do{FWHEEL_CNT=0;}while(0)
#define LWHEEL_CNT_CLR()                            do{LWHEEL_CNT=0;}while(0)
#define RWHEEL_CNT_CLR()                            do{RWHEEL_CNT=0;}while(0)
#define LWHEEL_EXP_CNT_SET(n)                       do{gLWheelExpCnt=n;}while(0)
#define RWHEEL_EXP_CNT_SET(n)                       do{gRWheelExpCnt=n;}while(0)
#define LWHEEL_EXP_CB_REG(f)                        do{gLWheelExpCB=f;}while(0)
#define RWHEEL_EXP_CB_REG(f)                        do{gRWheelExpCB=f;}while(0)
#define LWHEEL_EXP_SPEED_SET(s)                     do{gCurLWheelSpeed=s;}while(0)
#define RWHEEL_EXP_SPEED_SET(s)                     do{gCurRWheelSpeed=s;}while(0)

static u8 gtmpCnt = 0;
static u16 gIFRDTxOffRxVal[IFRD_TxRx_CHAN_NUM] = {0};

void FWheelCounterISR(void)
{
    FWHEEL_CNT++;
}

void LWheelCounterISR(void)
{
    LWHEEL_CNT++;
    if(LWHEEL_CNT >= gLWheelExpCnt){
        if(NULL != gLWheelExpCB){
            gLWheelExpCB();
        }
    }
}

void RWheelCounterISR(void)
{
    RWHEEL_CNT++;
    if(RWHEEL_CNT >= gRWheelExpCnt){
        if(NULL != gRWheelExpCB){
            gRWheelExpCB();
        }
    }
}

void ProximityDetectionProc(void)
{
    u8      i = 0;
    Msg_t   Msg;

    if((++gtmpCnt)%2){
        /* Update wheel floating sign */
        /* motor overloading sign */
        /* Save proximity condition in Tx off */
        for(i = 0; i < IFRD_TxRx_CHAN_NUM; i++){
            gIFRDTxOffRxVal[i] = ADCConvertedLSB[i];
        }
        IFRD_TX_ENABLE();

        /* Save last wheel counter */
        gLastWheelCnt[WHEEL_IDX_F] = FWHEEL_CNT;
        gLastWheelCnt[WHEEL_IDX_L] = LWHEEL_CNT;
        gLastWheelCnt[WHEEL_IDX_R] = RWHEEL_CNT;
    }
    else{
        if(IS_MOTION_PROC_FINISH()){
            if( /* Update front and bottom of LEFT proximity condition in Tx on */
                ((gIFRDTxOffRxVal[IFRD_CHAN_FRONT_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_L-1] > gProximityDetectionThreshold[IFRD_CHAN_FRONT_L])&&(gRobotMode!=ROBOT_WORK_MODE_HOMING))
                ||
                (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_L-1] < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_L])
                ||
                (!COLLISION_SIGN_LEFT)
    //                ||
    //                (!COLLISION_SIGN_FL)
                ){
                /* Send Left front and bottom path fault message */
                Msg.expire = 0;
                Msg.prio = MSG_PRIO_HIGH;
                Msg.type = MSG_TYPE_MOTION;
                Msg.MsgCB = NULL;
                Msg.Data.MEvt = MOTION_EVT_PATH_FAULT_L;
                SweepRobot_SendMsg(&Msg);
            }
            else if(/* Update front and bottom of RIGHT proximity condition in Tx on */
                ((gIFRDTxOffRxVal[IFRD_CHAN_FRONT_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_R-1] > gProximityDetectionThreshold[IFRD_CHAN_FRONT_R])&&(gRobotMode!=ROBOT_WORK_MODE_HOMING))
                ||
                (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_R-1] < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_R])
                ||
                (!COLLISION_SIGN_RIGHT)
                ||
                (!COLLISION_SIGN_FR)
                ){
                /* Send Right front and bottom path fault message */
                Msg.expire = 0;
                Msg.prio = MSG_PRIO_HIGH;
                Msg.type = MSG_TYPE_MOTION;
                Msg.MsgCB = NULL;
                Msg.Data.MEvt = MOTION_EVT_PATH_FAULT_R;
                SweepRobot_SendMsg(&Msg);
            }
        }

        if( /* Update LEFT side proximity condition in Tx on */
            (gIFRDTxOffRxVal[IFRD_CHAN_SIDE_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_L-1] > gProximityDetectionThreshold[IFRD_CHAN_SIDE_L])
            ){
            /* Send Left proximity condition message */
            Msg.expire = 0;
            Msg.prio = MSG_PRIO_LOW;
            Msg.type = MSG_TYPE_MOTION;
            Msg.MsgCB = NULL;
            Msg.Data.MEvt = MOTION_EVT_PROXIMITY_SL;
            SweepRobot_SendMsg(&Msg);
        }
        else if(/* Update RIGHT side proximity condition in Tx on */
            (gIFRDTxOffRxVal[IFRD_CHAN_SIDE_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_R-1] > gProximityDetectionThreshold[IFRD_CHAN_SIDE_R])
            ){
            /* Send Right proximity condition message */
            Msg.expire = 0;
            Msg.prio = MSG_PRIO_LOW;
            Msg.type = MSG_TYPE_MOTION;
            Msg.MsgCB = NULL;
            Msg.Data.MEvt = MOTION_EVT_PROXIMITY_SR;
            SweepRobot_SendMsg(&Msg);
        }
        IFRD_TX_DISABLE();

        /* Wheel speed adjust */
        gDeltaWheelCnt[WHEEL_IDX_F] = FWHEEL_CNT - gLastWheelCnt[WHEEL_IDX_F];
        gDeltaWheelCnt[WHEEL_IDX_L] = LWHEEL_CNT - gLastWheelCnt[WHEEL_IDX_L];
        gDeltaWheelCnt[WHEEL_IDX_R] = RWHEEL_CNT - gLastWheelCnt[WHEEL_IDX_R];

        if( (gDeltaWheelCnt[WHEEL_IDX_L] - gCurLWheelSpeed) > 0 ){
            MotorCtrl_ChanSpeedDec(MOTOR_CTRL_CHAN_LWHEEL);
        }
        else if( (gCurLWheelSpeed - gDeltaWheelCnt[WHEEL_IDX_L]) > 0 ){
            MotorCtrl_ChanSpeedInc(MOTOR_CTRL_CHAN_LWHEEL);
        }
        if( (gDeltaWheelCnt[WHEEL_IDX_R] - gCurRWheelSpeed) > 0 ){
            MotorCtrl_ChanSpeedDec(MOTOR_CTRL_CHAN_RWHEEL);
        }
        else if( (gCurRWheelSpeed - gDeltaWheelCnt[WHEEL_IDX_R]) > 0 ){
            MotorCtrl_ChanSpeedInc(MOTOR_CTRL_CHAN_RWHEEL);
        }
    }

}

void IFRD_PathDetectInit(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;

    /* Infrared Tx control GPIO init */
    RCC_APB2PeriphClockCmd(IFRD_TX_CTRL_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = IFRD_LEFT_TX_CTRL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IFRD_LEFT_TX_CTRL_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = IFRD_RIGHT_TX_CTRL_PIN;
    GPIO_Init(IFRD_RIGHT_TX_CTRL_GPIO, &GPIO_InitStructure);
    IFRD_TX_DISABLE();

    /* Infrared based Proximity detection init */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
    TIM_DeInit(TIM5);
    TIM_TimeBaseStructure.TIM_Period = 2000-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    plat_int_reg_cb(STM32F10x_INT_TIM5, (void*)ProximityDetectionProc);
}

static void IFRD_PathDetectStart(void)
{
    gtmpCnt = 0;
    TIM_SetCounter(TIM5, 0);
    TIM_Cmd(TIM5, ENABLE);
}

static void IFRD_PathDetectStop(void)
{
    TIM_SetCounter(TIM5, 0);
    TIM_Cmd(TIM5, DISABLE);
    IFRD_TX_DISABLE();
}

static void WheelCntMach_Start(void)
{
    EXTI_InitTypeDef            EXTI_InitStructure;

    LWHEEL_CNT_CLR();
    RWHEEL_CNT_CLR();

    EXTI_InitStructure.EXTI_Line = WHEEL_CNT_EXTI_LINES;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

static void WheelCntMach_Stop(void)
{
    EXTI_InitTypeDef            EXTI_InitStructure;

    LWHEEL_CNT_CLR();
    RWHEEL_CNT_CLR();

    EXTI_InitStructure.EXTI_Line = WHEEL_CNT_EXTI_LINES;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(EXTI_InitStructure.EXTI_Line);
}

void MotionCtrl_Init(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    /* Wheel motor counter init */
	RCC_APB2PeriphClockCmd(WHEEL_CNT_GPIO_PERIPH_ID|RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = FWHEEL_COUNTER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_Init(FWHEEL_COUNTER_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LWHEEL_COUNTER_PIN;
    GPIO_Init(LWHEEL_COUNTER_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = RWHEEL_COUNTER_PIN;
    GPIO_Init(RWHEEL_COUNTER_GPIO, &GPIO_InitStructure);

//    GPIO_EXTILineConfig(FWHEEL_COUNTER_EXTI_GPIO_SOURCE, FWHEEL_COUNTER_EXTI_PIN_SOURCE);
    GPIO_EXTILineConfig(LWHEEL_COUNTER_EXTI_GPIO_SOURCE, LWHEEL_COUNTER_EXTI_PIN_SOURCE);
    GPIO_EXTILineConfig(RWHEEL_COUNTER_EXTI_GPIO_SOURCE, RWHEEL_COUNTER_EXTI_PIN_SOURCE);

    NVIC_InitStructure.NVIC_IRQChannel = LWHEEL_COUNTER_EXTI_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = RWHEEL_COUNTER_EXTI_IRQN;
    NVIC_Init(&NVIC_InitStructure);
//    NVIC_InitStructure.NVIC_IRQChannel = FWHEEL_COUNTER_EXTI_IRQN;
//    NVIC_Init(&NVIC_InitStructure);

//    plat_int_reg_cb(FWHEEL_COUNTER_INT_INDEX, (void*)FWheelCounterISR);
    plat_int_reg_cb(LWHEEL_COUNTER_INT_INDEX, (void*)LWheelCounterISR);
    plat_int_reg_cb(RWHEEL_COUNTER_INT_INDEX, (void*)RWheelCounterISR);

    /* Wheel floating detection init */
    RCC_APB2PeriphClockCmd(WHEEL_FLOAT_DETECT_GPIO_PERIPH_ID, ENABLE);

	GPIO_InitStructure.GPIO_Pin = WHEEL_FLOAT_DETECT_L_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WHEEL_FLOAT_DETECT_L_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = WHEEL_FLOAT_DETECT_R_PIN;
    GPIO_Init(WHEEL_FLOAT_DETECT_R_GPIO, &GPIO_InitStructure);

    /* Collision detection init */
    RCC_APB2PeriphClockCmd(COLLISION_DETECT_GPIO_PERIPH_ID, ENABLE);

	GPIO_InitStructure.GPIO_Pin = COLLISION_DETECT_LEFT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COLLISION_DETECT_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = COLLISION_DETECT_FL_PIN;
    GPIO_Init(COLLISION_DETECT_FL_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = COLLISION_DETECT_RIGHT_PIN;
    GPIO_Init(COLLISION_DETECT_RIGHT_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = COLLISION_DETECT_FR_PIN;
    GPIO_Init(COLLISION_DETECT_FR_GPIO, &GPIO_InitStructure);

    /* Infrared based Proximity detection init */
    IFRD_PathDetectInit();
}

static inline void MotionCtrl_LWheelDefProc(void)
{
    LWHEEL_EXP_CB_REG(NULL);
    gActSeqDepLIndicator--;
}

static inline void MotionCtrl_RWheelDefProc(void)
{
    RWHEEL_EXP_CB_REG(NULL);
    gActSeqDepRIndicator--;
}

static inline void MotionCtrl_RWheelSubProc(void)
{
    u8  idx = 0;

    gActSeqDepRIndicator--;

    idx = gActSeqDepth-gActSeqDepRIndicator;

    if(gActSequence[idx].RWheelDir!=MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_RWHEEL)){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);
        MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_RWHEEL, gActSequence[idx].RWheelDir);
    }

    RWHEEL_CNT_CLR();

    RWHEEL_EXP_CNT_SET(gActSequence[idx].RWheelExpCnt);

    if(gActSeqDepRIndicator > 1){
        RWHEEL_EXP_CB_REG(MotionCtrl_RWheelSubProc);
    }
    else{
        RWHEEL_EXP_CB_REG(gActSequence[idx].RWheelExpSpeed > 0 ? MotionCtrl_RWheelDefProc : MotionCtrl_Stop);
    }

    RWHEEL_EXP_SPEED_SET(gActSequence[idx].RWheelExpSpeed);

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, gActSequence[idx].RWheelSpeed);
}

static inline void MotionCtrl_LWheelSubProc(void)
{
    u8  idx = 0;

    gActSeqDepLIndicator--;

    idx = gActSeqDepth-gActSeqDepLIndicator;

    if(gActSequence[idx].LWheelDir!=MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_LWHEEL)){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
        MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_LWHEEL, gActSequence[idx].LWheelDir);
    }

    LWHEEL_CNT_CLR();

    LWHEEL_EXP_CNT_SET(gActSequence[idx].LWheelExpCnt);

    if(gActSeqDepLIndicator > 1){
        LWHEEL_EXP_CB_REG(MotionCtrl_LWheelSubProc);
    }
    else{
        LWHEEL_EXP_CB_REG(gActSequence[idx].LWheelExpSpeed > 0 ? MotionCtrl_LWheelDefProc : MotionCtrl_Stop);
    }

    LWHEEL_EXP_SPEED_SET(gActSequence[idx].LWheelExpSpeed);

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, gActSequence[idx].LWheelSpeed);
}

void MotionCtrl_Proc(void)
{
    gActSeqDepLIndicator = gActSeqDepth;
    gActSeqDepRIndicator = gActSeqDepth;

    IFRD_PathDetectStop();

    if(gActSequence[0].LWheelDir!=MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_LWHEEL)
     ||gActSequence[0].RWheelDir!=MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_RWHEEL)
        ){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);

        /* direction */
        MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_LWHEEL, gActSequence[0].LWheelDir);
        MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_RWHEEL, gActSequence[0].RWheelDir);
    }

    LWHEEL_CNT_CLR();
    RWHEEL_CNT_CLR();

    LWHEEL_EXP_CNT_SET(gActSequence[0].LWheelExpCnt);
    RWHEEL_EXP_CNT_SET(gActSequence[0].RWheelExpCnt);

    if(gActSeqDepLIndicator > 1){
        LWHEEL_EXP_CB_REG(MotionCtrl_LWheelSubProc);
    }
    else{
        LWHEEL_EXP_CB_REG(MotionCtrl_LWheelDefProc);
    }

    if(gActSeqDepRIndicator > 1){
        RWHEEL_EXP_CB_REG(MotionCtrl_RWheelSubProc);
    }
    else{
        RWHEEL_EXP_CB_REG(MotionCtrl_RWheelDefProc);
    }

    LWHEEL_EXP_SPEED_SET(gActSequence[0].LWheelExpSpeed);
    RWHEEL_EXP_SPEED_SET(gActSequence[0].RWheelExpSpeed);

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, gActSequence[0].RWheelSpeed);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, gActSequence[0].LWheelSpeed);

    IFRD_PathDetectStart();
}

void MotionCtrl_Start(void)
{
    WheelCntMach_Start();

    IFRD_PathDetectStart();

    gActSeqDepth = 0;
    gActSeqDepLIndicator = 0;
    gActSeqDepRIndicator = 0;

    /* Running light */
    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, 1);

    gRobotState = ROBOT_STATE_RUNNING;
}

void MotionCtrl_Stop(void)
{
    WheelCntMach_Stop();

    IFRD_PathDetectStop();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);

    gActSeqDepth = 0;
    gActSeqDepLIndicator = 0;
    gActSeqDepRIndicator = 0;

    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, 1);

    gRobotState = ROBOT_STATE_IDLE;
}

void MotionCtrl_LeftPathFaulProc(u16 backcnt, u16 turncnt, u8 StopOnFinish)
{
    gActSequence[0].LWheelDir = 0;
    gActSequence[0].RWheelDir = 0;
    gActSequence[0].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpCnt = backcnt;
    gActSequence[0].RWheelExpCnt = backcnt;
    gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[1].LWheelDir = 1;
    gActSequence[1].RWheelDir = 0;
    gActSequence[1].LWheelSpeed = 10;
    gActSequence[1].RWheelSpeed = 10;
    gActSequence[1].LWheelExpCnt = turncnt;
    gActSequence[1].RWheelExpCnt = turncnt;
    gActSequence[1].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[1].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[2].LWheelDir = 1;
    gActSequence[2].RWheelDir = 1;
    gActSequence[2].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[2].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[2].LWheelExpCnt = 0x1;
    gActSequence[2].RWheelExpCnt = 0x2;
    gActSequence[2].LWheelExpSpeed = StopOnFinish ? 0 : WHEEL_CRUISE_SPEED;
    gActSequence[2].RWheelExpSpeed = StopOnFinish ? 0 : WHEEL_CRUISE_SPEED;
    gActSeqDepth = 3;
    MotionCtrl_Proc();
}

void MotionCtrl_RightPathFaulProc(u16 backcnt, u16 turncnt, u8 StopOnFinish)
{
    gActSequence[0].LWheelDir = 0;
    gActSequence[0].RWheelDir = 0;
    gActSequence[0].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpCnt = backcnt;
    gActSequence[0].RWheelExpCnt = backcnt;
    gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[1].LWheelDir = 0;
    gActSequence[1].RWheelDir = 1;
    gActSequence[1].LWheelSpeed = 10;
    gActSequence[1].RWheelSpeed = 10;
    gActSequence[1].LWheelExpCnt = turncnt;
    gActSequence[1].RWheelExpCnt = turncnt;
    gActSequence[1].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[1].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[2].LWheelDir = 1;
    gActSequence[2].RWheelDir = 1;
    gActSequence[2].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[2].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[2].LWheelExpCnt = 0x1;
    gActSequence[2].RWheelExpCnt = 0x2;
    gActSequence[2].LWheelExpSpeed = StopOnFinish ? 0 : WHEEL_CRUISE_SPEED;
    gActSequence[2].RWheelExpSpeed = StopOnFinish ? 0 : WHEEL_CRUISE_SPEED;
    gActSeqDepth = 3;
    MotionCtrl_Proc();
}

void MotionCtrl_MoveLeftSlowly(u8 dec)
{
    if(!IS_MOTION_PROC_FINISH())
        return;

    printf("L\r\n");

//    if(dec){
        LWHEEL_EXP_SPEED_SET(dec);
        RWHEEL_EXP_SPEED_SET(WHEEL_HOMING_SPEED);
//    }
//    else{
//        gActSequence[0].LWheelDir = 0;
//        gActSequence[0].RWheelDir = 1;
//        gActSequence[0].LWheelSpeed = 0;
//        gActSequence[0].RWheelSpeed = 0;
//        gActSequence[0].LWheelExpCnt = 5;
//        gActSequence[0].RWheelExpCnt = 5;
//        gActSequence[0].LWheelExpSpeed = WHEEL_HOMING_SPEED;
//        gActSequence[0].RWheelExpSpeed = WHEEL_HOMING_SPEED;
//        gActSeqDepth = 1;
//        MotionCtrl_Proc();
//    }
}

void MotionCtrl_MoveRightSlowly(u8 dec)
{
    if(!IS_MOTION_PROC_FINISH())
        return;

   printf("R\r\n");

//    if(dec){
        LWHEEL_EXP_SPEED_SET(WHEEL_HOMING_SPEED);
        RWHEEL_EXP_SPEED_SET(dec);
//    }
//    else{
//        gActSequence[0].LWheelDir = 1;
//        gActSequence[0].RWheelDir = 0;
//        gActSequence[0].LWheelSpeed = 0;
//        gActSequence[0].RWheelSpeed = 0;
//        gActSequence[0].LWheelExpCnt = 5;
//        gActSequence[0].RWheelExpCnt = 5;
//        gActSequence[0].LWheelExpSpeed = WHEEL_HOMING_SPEED;
//        gActSequence[0].RWheelExpSpeed = WHEEL_HOMING_SPEED;
//        gActSeqDepth = 1;
//        MotionCtrl_Proc();
//    }
}

void MotionCtrl_MoveDirectly(void)
{
    if(!IS_MOTION_PROC_FINISH())
        return;

    printf("S\r\n");

    gActSequence[0].LWheelDir = 1;
    gActSequence[0].RWheelDir = 1;
    gActSequence[0].LWheelSpeed = 20;
    gActSequence[0].RWheelSpeed = 20;
    gActSequence[0].LWheelExpCnt = 5;
    gActSequence[0].RWheelExpCnt = 5;
    gActSequence[0].LWheelExpSpeed = WHEEL_HOMING_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_HOMING_SPEED;
    gActSeqDepth = 1;
    MotionCtrl_Proc();
}

void MotionCtrl_AutoMotionInit(void)
{
    if(gRobotState == ROBOT_STATE_HOME){
        MotionCtrl_Start();
        gActSequence[0].LWheelDir = 0;
        gActSequence[0].RWheelDir = 0;
        gActSequence[0].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].LWheelExpCnt = WHEEL_FAULT_BACK_CNT*2;
        gActSequence[0].RWheelExpCnt = WHEEL_FAULT_BACK_CNT*2;
        gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[1].LWheelDir = 0;
        gActSequence[1].RWheelDir = 1;
        gActSequence[1].LWheelSpeed = 10;
        gActSequence[1].RWheelSpeed = 10;
        gActSequence[1].LWheelExpCnt = WHEEL_TURN_180_CNT;
        gActSequence[1].RWheelExpCnt = WHEEL_TURN_180_CNT;
        gActSequence[1].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[1].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[2].LWheelDir = 1;
        gActSequence[2].RWheelDir = 1;
        gActSequence[2].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[2].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[2].LWheelExpCnt = 0x1;
        gActSequence[2].RWheelExpCnt = 0x2;
        gActSequence[2].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[2].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSeqDepth = 3;
    }
    else{
        MotionCtrl_Start();
        gActSequence[0].LWheelDir = 1;
        gActSequence[0].RWheelDir = 1;
        gActSequence[0].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].LWheelExpCnt = 0x1;
        gActSequence[0].RWheelExpCnt = 0x2;
        gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSeqDepth = 1;
    }
    MotionCtrl_Proc();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
}

void MotionCtrl_ManualCtrlProc(enum MotionCtrlManualAct act)
{
    MotionCtrl_Start();

    switch(act){
        case MANUAL_ACT_UP:
            gActSequence[0].LWheelDir = 1;
            gActSequence[0].RWheelDir = 1;
            gActSequence[0].LWheelExpCnt = 0x1;
            gActSequence[0].RWheelExpCnt = 0x1;
            gActSeqDepth = 1;
            break;
        case MANUAL_ACT_LEFT:
            gActSequence[0].LWheelDir = 0;
            gActSequence[0].RWheelDir = 1;
            gActSequence[0].LWheelExpCnt = WHEEL_TURN_45_CNT;
            gActSequence[0].RWheelExpCnt = WHEEL_TURN_45_CNT;
            gActSeqDepth = 2;
            break;
        case MANUAL_ACT_DOWN:
            gActSequence[0].LWheelDir = 0;
            gActSequence[0].RWheelDir = 1;
            gActSequence[0].LWheelExpCnt = WHEEL_TURN_180_CNT;
            gActSequence[0].RWheelExpCnt = WHEEL_TURN_180_CNT;
            gActSeqDepth = 2;
            break;
        case MANUAL_ACT_RIGHT:
            gActSequence[0].LWheelDir = 1;
            gActSequence[0].RWheelDir = 0;
            gActSequence[0].LWheelExpCnt = WHEEL_TURN_45_CNT;
            gActSequence[0].RWheelExpCnt = WHEEL_TURN_45_CNT;
            gActSeqDepth = 2;
            break;
    }

    gActSequence[0].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[1].LWheelExpCnt = 0x1;
    gActSequence[1].RWheelExpCnt = 0x2;
    gActSequence[1].LWheelExpSpeed = 0;
    gActSequence[1].RWheelExpSpeed = 0;
    MotionCtrl_Proc();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
}

void MotionCtrl_RoundedSlowly(void)
{
    MotionCtrl_Stop();
    mDelay(1);
    MotionCtrl_Start();

    gActSequence[0].LWheelDir = 1;
    gActSequence[0].RWheelDir = 1;
    gActSequence[0].LWheelSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpCnt = 0x1; //WHEEL_TURN_360_CNT;
    gActSequence[0].RWheelExpCnt = 0x2; //WHEEL_TURN_360_CNT;
    gActSequence[0].LWheelExpSpeed = WHEEL_HOMING_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_HOMING_SPEED;
    gActSeqDepth = 1;
    MotionCtrl_Proc();
}