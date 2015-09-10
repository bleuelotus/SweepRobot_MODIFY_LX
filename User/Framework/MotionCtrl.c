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

#define LBRUSH_CUR_THRESHOLD        300                                         //  0.5A
#define RBRUSH_CUR_THRESHOLD        300                                         //  0.5A
#define MBRUSH_CUR_THRESHOLD        1000                                        //  1.6A
#define FUN_CUR_THRESHOLD           1000                                        //  1.6A
#define ASH_TRAY_INSTALL_CUR        3500                                        //  2.8V


enum _PathFaultProcMode {

    PATH_FAULT_PROC_MODE_NORMAL,
    PATH_FAULT_PROC_MODE_EDGE,
    PATH_FAULT_PROC_MODE_EDGE_L,
    PATH_FAULT_PROC_MODE_EDGE_R,
};

/* Infrared based proximity detection sensitivity */
const u16 gProximityDetectionThreshold[IFRD_TxRx_CHAN_NUM] = { 800, 800, 250, 250, 150, 150 };

static u16 gWheelCnt[WHEEL_NUM] = {0};
static u16 gLWheelExpCnt = 0xFFFF, gRWheelExpCnt = 0xFFFF;

typedef void (*WheelExpFunc_t)(void);

static WheelExpFunc_t  gLWheelExpCB = NULL;
static WheelExpFunc_t  gRWheelExpCB = NULL;

static u16 gLastWheelCnt[WHEEL_NUM] = {0};
static u16 gDeltaWheelCnt[WHEEL_NUM] = {0};
static u8 gCurLWheelSpeed = WHEEL_CRUISE_SPEED, gCurRWheelSpeed = WHEEL_CRUISE_SPEED;
static s8 gLastPathFault = 0; /* 0: none, 1: right, -1: left */
static u8 gConsecutivePathFaultCnt = 0;
static enum _PathFaultProcMode gPathFaultProcMode = PATH_FAULT_PROC_MODE_NORMAL;
static s16 gLastPathCondSLDiff = 0, gLastPathCondSRDiff = 0;
static s16 gCurPathCondSLDiff = 0, gCurPathCondSRDiff = 0;
//static s8 gPathCondSideProxmityStrength = 0; /* 0: stable, 1: far, -1: closer */

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
#define LWHEEL_EXP_CNT_INC(n)                       do{gLWheelExpCnt+=n;}while(0)
#define RWHEEL_EXP_CNT_INC(n)                       do{gRWheelExpCnt+=n;}while(0)
#define LWHEEL_EXP_CB_REG(f)                        do{gLWheelExpCB=f;}while(0)
#define RWHEEL_EXP_CB_REG(f)                        do{gRWheelExpCB=f;}while(0)
#define LWHEEL_EXP_SPEED_SET(s)                     do{gCurLWheelSpeed=s;}while(0)
#define RWHEEL_EXP_SPEED_SET(s)                     do{gCurRWheelSpeed=s;}while(0)
#define LWHEEL_CUR_SPEED                            (gCurLWheelSpeed)
#define RWHEEL_CUR_SPEED                            (gCurRWheelSpeed)

static u8 gtmpCnt = 0;
static u16 gIFRDTxOffRxVal[IFRD_TxRx_CHAN_NUM] = {0};
static u32 gPathCondMap = 0;

void FWheelCounterISR(void)
{
    FWHEEL_CNT++;
}

void LWheelCounterISR(void)
{
    LWHEEL_CNT++;
    if(LWHEEL_CNT > WHEEL_BODY_THROUGH_CNT){
        gLastPathFault = 0;
        gConsecutivePathFaultCnt = 0;
    }
    if(LWHEEL_CNT >= gLWheelExpCnt){
        if(NULL != gLWheelExpCB){
            gLWheelExpCB();
        }
    }
}

void RWheelCounterISR(void)
{
    RWHEEL_CNT++;
    if(RWHEEL_CNT > WHEEL_BODY_THROUGH_CNT){
        gLastPathFault = 0;
        gConsecutivePathFaultCnt = 0;
    }
    if(RWHEEL_CNT >= gRWheelExpCnt){
        if(NULL != gRWheelExpCB){
            gRWheelExpCB();
        }
    }
}

void MotionStateProc(void)
{
    u8      i = 0;
    Msg_t   Msg;

    if((++gtmpCnt)%2){
        /* Update exception sign: left, right, middle brush over loading, wheel floationg and ash tray exist or not */
//        if(
////           WHEEL_FLOAT_SIGN_ALL
////           ||
////           (ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1] > LBRUSH_CUR_THRESHOLD)
////           ||
////           (ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1] > RBRUSH_CUR_THRESHOLD)
////           ||
////           (ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1] > MBRUSH_CUR_THRESHOLD)
////           ||
////           (ADCConvertedLSB[MEAS_CHAN_FUN_CUR-1] > FUN_CUR_THRESHOLD)
////           ||
////           (ADCConvertedLSB[MEAS_CHAN_ASH_TRAY_LVL-1] < ASH_TRAY_INSTALL_CUR)
//           ){
//            /* Send exception message */
//            Msg.expire = 0;
//            Msg.prio = MSG_PRIO_HIGHEST;
//            Msg.type = MSG_TYPE_MOTION;
//            Msg.MsgCB = NULL;
//            Msg.Data.MEvt = MOTION_EVT_EXCEPTION;
//            SweepRobot_SendMsg(&Msg);
//        }

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
        /* Update front, side and bottom of LEFT proximity condition in Tx on */
        if( (gIFRDTxOffRxVal[IFRD_CHAN_FRONT_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_L-1] > gProximityDetectionThreshold[IFRD_CHAN_FRONT_L]) && (gHomingStage < ROBOT_HOMING_STAGE3) ) {
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_FL_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_FL_POS);
        }
        if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_L-1] < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_L]) ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BL_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_BL_POS);
        }

        gCurPathCondSLDiff = gIFRDTxOffRxVal[IFRD_CHAN_SIDE_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_L-1];
        if( gCurPathCondSLDiff > gProximityDetectionThreshold[IFRD_CHAN_SIDE_L] ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_SL_POS);
//            printf("L side: %d\r\n", gCurPathCondSLDiff);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_SL_POS);
        }
        /* Path adjust for edge mode */
        if( IS_MOTION_PROC_FINISH() && (gPathFaultProcMode == PATH_FAULT_PROC_MODE_EDGE_L) ){
            if( (gLastPathCondSLDiff - gCurPathCondSLDiff) >= 50 ){
                /* faraway */
                LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-3);
                RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
            }
            else if( (gLastPathCondSLDiff - gCurPathCondSLDiff) <= -50 ){
                /* closer */
                LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-3);
            }
            else{
                /* no change */
                if( gPathCondMap & (1 << PATH_COND_PROXIMITY_FLAG_SL_POS) ){
                    LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                    RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                }
                else{
                    LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-5);
                    RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                }
            }
        }
        gLastPathCondSLDiff = gCurPathCondSLDiff;

        /* Update left side collision state */
        if( (!COLLISION_SIGN_LEFT) ){
            gPathCondMap |= (1 << PATH_COND_COLLISION_FLAG_SL_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_COLLISION_FLAG_SL_POS);
        }
        if( !COLLISION_SIGN_FL ){
            gPathCondMap |= (1 << PATH_COND_COLLISION_FLAG_FL_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_COLLISION_FLAG_FL_POS);
        }

        /* Update front, side and bottom of RIGHT proximity condition in Tx on */
        if( (gIFRDTxOffRxVal[IFRD_CHAN_FRONT_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_R-1] > gProximityDetectionThreshold[IFRD_CHAN_FRONT_R]) && (gHomingStage < ROBOT_HOMING_STAGE3) ) {
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_FR_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_FR_POS);
        }
        if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_R-1] < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_R]) ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BR_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_BR_POS);
        }

        gCurPathCondSRDiff = gIFRDTxOffRxVal[IFRD_CHAN_SIDE_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_R-1];
        if( gCurPathCondSRDiff > gProximityDetectionThreshold[IFRD_CHAN_SIDE_R] ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_SR_POS);
//            printf("R side: %d\r\n", gCurPathCondSRDiff);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_SR_POS);
        }
        /* Path adjust for edge mode */
        if( IS_MOTION_PROC_FINISH() && (gPathFaultProcMode == PATH_FAULT_PROC_MODE_EDGE_R) ){
            if( (gLastPathCondSRDiff - gCurPathCondSRDiff) >= 50 ){
                /* faraway */
                LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-3);
            }
            else if( (gLastPathCondSRDiff - gCurPathCondSRDiff) <= -50 ){
                /* closer */
                LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-3);
                RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
            }
            else{
                /* no change */
                if( gPathCondMap & (1 << PATH_COND_PROXIMITY_FLAG_SR_POS) ){
                    LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                    RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                }
                else{
                    LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                    RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-5);
                }
            }
        }
        gLastPathCondSRDiff = gCurPathCondSRDiff;

        /* Update right side collision state */
        if( (!COLLISION_SIGN_RIGHT) ){
            gPathCondMap |= (1 << PATH_COND_COLLISION_FLAG_SR_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_COLLISION_FLAG_SR_POS);
        }
        if( (!COLLISION_SIGN_FR) ){
            gPathCondMap |= (1 << PATH_COND_COLLISION_FLAG_FR_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_COLLISION_FLAG_FR_POS);
        }
        IFRD_TX_DISABLE();

//        printf("map1: 0x%X\r\n", gPathCondMap);

        if(IS_MOTION_PROC_FINISH()){
            if( ((gPathFaultProcMode == PATH_FAULT_PROC_MODE_NORMAL) && (gPathCondMap & (PATH_FAULT_LEFT_MASK | PATH_FAULT_RIGHT_MASK)))
                ||
                ((gPathFaultProcMode > PATH_FAULT_PROC_MODE_EDGE) && (gPathCondMap & (PATH_FAULT_COLLISION_MASK | PATH_FAULT_FRONT_MASK)))
                ){
                    /* Send Left front and bottom path fault message */
                    Msg.expire = 0;
                    Msg.prio = MSG_PRIO_HIGH;
                    Msg.type = MSG_TYPE_MOTION;
                    Msg.MsgCB = NULL;
                    Msg.Data.MEvt = MOTION_EVT_PATH_FAULT;
                    if(!SweepRobot_SendMsg(&Msg)){
                        MOTION_PROC_STATE_SET();
                    }
            }
        }

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

    plat_int_reg_cb(STM32F10x_INT_TIM5, (void*)MotionStateProc);
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

static inline void MotionCtrl_LWheelProcExitOn(void)
{
    LWHEEL_EXP_CB_REG(NULL);
    gActSeqDepLIndicator--;
}

static inline void MotionCtrl_RWheelProcExitOn(void)
{
    RWHEEL_EXP_CB_REG(NULL);
    gActSeqDepRIndicator--;
}

static inline void MotionCtrl_LWheelProcExitOff(void)
{
    LWHEEL_EXP_CB_REG(NULL);
//    LWHEEL_EXP_SPEED_SET(0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
    gActSeqDepLIndicator--;
    if(gActSeqDepRIndicator==0 && gActSeqDepLIndicator==0){
        MotionCtrl_Stop();
    }
}

static inline void MotionCtrl_RWheelProcExitOff(void)
{
    RWHEEL_EXP_CB_REG(NULL);
//    RWHEEL_EXP_SPEED_SET(0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);
    gActSeqDepRIndicator--;
    if(gActSeqDepRIndicator==0 && gActSeqDepLIndicator==0){
        MotionCtrl_Stop();
    }
}

static inline void MotionCtrl_RWheelSubProc(void)
{
    u8  idx = 0;

    idx = gActSeqDepth-gActSeqDepRIndicator;

    gActSequence[idx].RWheelSync = 1;

    if(gActSequence[idx].LWheelSync==gActSequence[idx].RWheelSync){
        if(NULL!=gActSequence[idx].PostAct){
            if(gActSequence[idx].PostAct()){
                gActSeqDepRIndicator--;
            }
            else{
                RWHEEL_CNT_CLR();
                return;
            }
        }
        else{
            gActSeqDepRIndicator--;
        }
    }
    else{
        RWHEEL_EXP_CNT_INC(16);
        return;
    }

    idx = gActSeqDepth-gActSeqDepRIndicator;

    RWHEEL_CNT_CLR();

    /* FIXME: L/R wheel sync */
    if(NULL!=gActSequence[idx].PreAct){
        gActSequence[idx].PreAct(&gActSequence[idx]);
    }

    if(gActSequence[idx].RWheelDefDir!=MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_RWHEEL)){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);
        MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_RWHEEL, gActSequence[idx].RWheelDefDir);
    }

    RWHEEL_EXP_CNT_SET(gActSequence[idx].RWheelExpCnt);

    if(gActSeqDepRIndicator > 1){
        RWHEEL_EXP_CB_REG(MotionCtrl_RWheelSubProc);
    }
    else{
        RWHEEL_EXP_CB_REG(gActSequence[idx].RWheelExpSpeed > 0 ? MotionCtrl_RWheelProcExitOn : MotionCtrl_RWheelProcExitOff);
    }

    RWHEEL_EXP_SPEED_SET(gActSequence[idx].RWheelExpSpeed);

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, gActSequence[idx].RWheelInitSpeed);
}

static inline void MotionCtrl_LWheelSubProc(void)
{
    u8  idx = 0;

    idx = gActSeqDepth-gActSeqDepLIndicator;

    gActSequence[idx].LWheelSync = 1;

    if(gActSequence[idx].LWheelSync==gActSequence[idx].RWheelSync){
        if(NULL!=gActSequence[idx].PostAct){
            if(gActSequence[idx].PostAct()){
                gActSeqDepLIndicator--;
            }
            else{
                LWHEEL_CNT_CLR();
                return;
            }
        }
        else{
            gActSeqDepLIndicator--;
        }
    }
    else{
        LWHEEL_EXP_CNT_INC(16);
        return;
    }

    idx = gActSeqDepth-gActSeqDepLIndicator;

    LWHEEL_CNT_CLR();

    /* FIXME: L/R wheel sync */
    if(NULL!=gActSequence[idx].PreAct){
        gActSequence[idx].PreAct(&gActSequence[idx]);
    }

    if(gActSequence[idx].LWheelDefDir!=MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_LWHEEL)){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
        MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_LWHEEL, gActSequence[idx].LWheelDefDir);
    }

    LWHEEL_EXP_CNT_SET(gActSequence[idx].LWheelExpCnt);

    if(gActSeqDepLIndicator > 1){
        LWHEEL_EXP_CB_REG(MotionCtrl_LWheelSubProc);
    }
    else{
        LWHEEL_EXP_CB_REG(gActSequence[idx].LWheelExpSpeed > 0 ? MotionCtrl_LWheelProcExitOn : MotionCtrl_LWheelProcExitOff);
    }

    LWHEEL_EXP_SPEED_SET(gActSequence[idx].LWheelExpSpeed);

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, gActSequence[idx].LWheelInitSpeed);
}

void MotionCtrl_Proc(void)
{
    gActSeqDepLIndicator = gActSeqDepth;
    gActSeqDepRIndicator = gActSeqDepth;

    IFRD_PathDetectStop();

    if(gActSequence[0].LWheelDefDir!=MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_LWHEEL)
     ||gActSequence[0].RWheelDefDir!=MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_RWHEEL)
        ){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);

        /* direction */
        MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_LWHEEL, gActSequence[0].LWheelDefDir);
        MotorCtrl_ChanDirSet(MOTOR_CTRL_CHAN_RWHEEL, gActSequence[0].RWheelDefDir);
    }

    LWHEEL_CNT_CLR();
    RWHEEL_CNT_CLR();

    LWHEEL_EXP_CNT_SET(gActSequence[0].LWheelExpCnt);
    RWHEEL_EXP_CNT_SET(gActSequence[0].RWheelExpCnt);

    if(gActSeqDepLIndicator > 1){
        LWHEEL_EXP_CB_REG(MotionCtrl_LWheelSubProc);
    }
    else{
        LWHEEL_EXP_CB_REG(MotionCtrl_LWheelProcExitOn);
    }

    if(gActSeqDepRIndicator > 1){
        RWHEEL_EXP_CB_REG(MotionCtrl_RWheelSubProc);
    }
    else{
        RWHEEL_EXP_CB_REG(MotionCtrl_RWheelProcExitOn);
    }

    LWHEEL_EXP_SPEED_SET(gActSequence[0].LWheelExpSpeed);
    RWHEEL_EXP_SPEED_SET(gActSequence[0].RWheelExpSpeed);

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, gActSequence[0].RWheelInitSpeed);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, gActSequence[0].LWheelInitSpeed);

    IFRD_PathDetectStart();
}

void MotionCtrl_Start(void)
{
    WheelCntMach_Start();

    IFRD_PathDetectStart();

    gActSeqDepth = 0;
    gActSeqDepLIndicator = 0;
    gActSeqDepRIndicator = 0;
    gLastPathFault = 0;
//    gPathFaultProcEdgeModeCnt = 0;
    gPathFaultProcMode = PATH_FAULT_PROC_MODE_NORMAL;

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
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    0);

    gActSeqDepth = 0;
    gActSeqDepLIndicator = 0;
    gActSeqDepRIndicator = 0;
    gLastPathFault = 0;
//    gPathFaultProcEdgeModeCnt = 0;
    gPathFaultProcMode = PATH_FAULT_PROC_MODE_NORMAL;

    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, 1);

    gRobotState = ROBOT_STATE_IDLE;
}

u8 MotionCtrl_PathFaultBackProcCompleteCondTest(void)
{
    return ((!(gPathCondMap & PATH_PROXIMITY_SIDE_L_MASK)) || (!(gPathCondMap & PATH_PROXIMITY_SIDE_R_MASK)));
}

void MotionCtrl_PathFaultTryTurnCondTest(struct MotionCtrl_Action_s *node)
{
    if(gPathCondMap & (PATH_PROXIMITY_SIDE_L_MASK | PATH_PROXIMITY_SIDE_R_MASK)){
        if(!(gPathCondMap & PATH_PROXIMITY_SIDE_L_MASK)){
            node->LWheelDefDir = 0;
            node->RWheelDefDir = 1;
            gLastPathFault = -1;
        }
        else{
            node->LWheelDefDir = 1;
            node->RWheelDefDir = 0;
            gLastPathFault = 1;
        }
    }
    else{
        if(gLastPathFault){
            if(gLastPathFault > 0){
                node->LWheelDefDir = 1;
                node->RWheelDefDir = 0;
            }
            else{
                node->LWheelDefDir = 0;
                node->RWheelDefDir = 1;
            }
        }
    }
}

void MotionCtrl_PathFaultEdgeModeProcAct(struct MotionCtrl_Action_s *node)
{
    if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_L){
    }
    else if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_R){
    }
}

u8 MotionCtrl_PathFaultTurnProcCompleteCondTest(void)
{
    return (!(gPathCondMap & PATH_FAULT_PROXIMITY_MASK));
}

void MotionCtrl_PathFaultProc(u8 StopOnFinish)
{
    MCtrl_Act_t *pActSequence = gActSequence;
    u16 backCntL = WHEEL_FAULT_BACK_CNT, turnCntL = WHEEL_TURN_30_CNT;
    u16 backCntR = WHEEL_FAULT_BACK_CNT, turnCntR = WHEEL_TURN_30_CNT;

    gActSeqDepth = 0;

    if(gPathCondMap & PATH_FAULT_FRONT_MASK){

        if( (gPathFaultProcMode == PATH_FAULT_PROC_MODE_NORMAL) ||
            ((gPathFaultProcMode > PATH_FAULT_PROC_MODE_EDGE) && (gPathCondMap & (PATH_FAULT_COLLISION_MASK & PATH_FAULT_FRONT_MASK))) ){

            pActSequence->LWheelDefDir = 0;
            pActSequence->RWheelDefDir = 0;
            pActSequence->LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
            pActSequence->RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
            pActSequence->LWheelExpCnt = backCntL;
            pActSequence->RWheelExpCnt = backCntR;
            pActSequence->LWheelExpSpeed = WHEEL_FAULT_PROC_SPEED;
            pActSequence->RWheelExpSpeed = WHEEL_FAULT_PROC_SPEED;
            pActSequence->LWheelSync = 0;
            pActSequence->RWheelSync = 0;
            pActSequence->PreAct = NULL;
            pActSequence->PostAct = MotionCtrl_PathFaultBackProcCompleteCondTest;
            gActSeqDepth++;
            pActSequence++;
        }

        if(gPathFaultProcMode == PATH_FAULT_PROC_MODE_NORMAL){
            if( gConsecutivePathFaultCnt++ > 3 ){
                gPathFaultProcMode = PATH_FAULT_PROC_MODE_EDGE;
                printf("Edge mode on\r\n");
            }
        }
        else if(gPathFaultProcMode > PATH_FAULT_PROC_MODE_EDGE){
            if( !gConsecutivePathFaultCnt ){
                gPathFaultProcMode = PATH_FAULT_PROC_MODE_NORMAL;
                printf("Edge mode off\r\n");
            }
            else{
                gConsecutivePathFaultCnt--;
            }
        }
    }

    if(gPathFaultProcMode <= PATH_FAULT_PROC_MODE_EDGE){
        if( gPathCondMap & (PATH_FAULT_SIDE_MASK & PATH_FAULT_COLLISION_MASK) ){
            turnCntL = WHEEL_TURN_15_CNT;
            turnCntR = WHEEL_TURN_15_CNT;
            pActSequence->PreAct = NULL;
            pActSequence->PostAct = NULL;
        }
        else{
            pActSequence->PreAct = MotionCtrl_PathFaultTryTurnCondTest;
            pActSequence->PostAct = MotionCtrl_PathFaultTurnProcCompleteCondTest;
        }

        /* turn direction judge */
        if( (gPathCondMap & PATH_FAULT_LEFT_MASK) && (gPathCondMap & PATH_FAULT_RIGHT_MASK) ){
            turnCntL = WHEEL_TURN_90_CNT;
            turnCntR = WHEEL_TURN_90_CNT;
        }
        if(gPathCondMap & PATH_FAULT_LEFT_MASK){
            if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE){
                gPathFaultProcMode = PATH_FAULT_PROC_MODE_EDGE_L;
                printf("Edge mode L\r\n");
            }
            pActSequence->LWheelDefDir = 1;
            pActSequence->RWheelDefDir = 0;
            if(0==gLastPathFault){
                gLastPathFault = 1;
            }
        }
        else{
            if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE){
                gPathFaultProcMode = PATH_FAULT_PROC_MODE_EDGE_R;
                printf("Edge mode R\r\n");
            }
            pActSequence->LWheelDefDir = 0;
            pActSequence->RWheelDefDir = 1;
            if(0==gLastPathFault){
                gLastPathFault = -1;
            }
        }
    }
    else{
        if( gPathCondMap & (PATH_FAULT_SIDE_MASK & PATH_FAULT_COLLISION_MASK) ){
            if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_L){
                turnCntL = WHEEL_TURN_15_CNT;
                turnCntR = 5;
            }
            else{
                turnCntL = 5;
                turnCntR = WHEEL_TURN_15_CNT;
            }
        }
        if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_L){
            pActSequence->LWheelDefDir = 1;
            pActSequence->RWheelDefDir = 0;
        }
        else{
            pActSequence->LWheelDefDir = 0;
            pActSequence->RWheelDefDir = 1;
        }

        pActSequence->PreAct = NULL;
        pActSequence->PostAct = NULL;
    }
    pActSequence->LWheelInitSpeed = 10;
    pActSequence->RWheelInitSpeed = 10;
    pActSequence->LWheelExpCnt = turnCntL;
    pActSequence->RWheelExpCnt = turnCntR;
    pActSequence->LWheelExpSpeed = WHEEL_FAULT_PROC_SPEED;
    pActSequence->RWheelExpSpeed = WHEEL_FAULT_PROC_SPEED;
    pActSequence->LWheelSync = 0;
    pActSequence->RWheelSync = 0;
    gActSeqDepth++;
    pActSequence++;

    pActSequence->LWheelDefDir = 1;
    pActSequence->RWheelDefDir = 1;
    pActSequence->LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    pActSequence->RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    pActSequence->LWheelExpCnt = 0x1;
    pActSequence->RWheelExpCnt = 0x2;
    if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_L){
        pActSequence->LWheelExpSpeed = (WHEEL_FAULT_PROC_SPEED-6);
        pActSequence->RWheelExpSpeed = WHEEL_FAULT_PROC_SPEED;
    }
    else if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_R){
        pActSequence->LWheelExpSpeed = WHEEL_FAULT_PROC_SPEED;
        pActSequence->RWheelExpSpeed = (WHEEL_FAULT_PROC_SPEED-6);
    }
    else{
        pActSequence->LWheelExpSpeed = StopOnFinish ? 0 : WHEEL_CRUISE_SPEED;
        pActSequence->RWheelExpSpeed = StopOnFinish ? 0 : WHEEL_CRUISE_SPEED;
    }
    pActSequence->LWheelSync = 0;
    pActSequence->RWheelSync = 0;
    pActSequence->PreAct = NULL;
    pActSequence->PostAct = NULL;
    gActSeqDepth++;
    pActSequence++;
    MotionCtrl_Proc();
}

void MotionCtrl_MoveDirTune(u8 l, u8 r)
{
    if(!IS_MOTION_PROC_FINISH())
        return;

//    if(dec){
        LWHEEL_EXP_SPEED_SET(l);
        RWHEEL_EXP_SPEED_SET(r);
//    }
//    else{
//        gActSequence[0].LWheelDefDir = 1;
//        gActSequence[0].RWheelDefDir = 0;
//        gActSequence[0].LWheelInitSpeed = MotorCtrl_ChanSpeedLevelGet(MOTOR_CTRL_CHAN_LWHEEL);;
//        gActSequence[0].RWheelInitSpeed = MotorCtrl_ChanSpeedLevelGet(MOTOR_CTRL_CHAN_RWHEEL);;
//        gActSequence[0].LWheelExpCnt = 20;
//        gActSequence[0].RWheelExpCnt = 20;
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

    gActSequence[0].LWheelDefDir = 1;
    gActSequence[0].RWheelDefDir = 1;
    gActSequence[0].LWheelInitSpeed = MotorCtrl_ChanSpeedLevelGet(MOTOR_CTRL_CHAN_LWHEEL);;
    gActSequence[0].RWheelInitSpeed = MotorCtrl_ChanSpeedLevelGet(MOTOR_CTRL_CHAN_RWHEEL);;
    gActSequence[0].LWheelExpCnt = 2;
    gActSequence[0].RWheelExpCnt = 2;
    gActSequence[0].LWheelExpSpeed = WHEEL_HOMING_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_HOMING_SPEED;
    gActSequence[0].LWheelSync = 0;
    gActSequence[0].RWheelSync = 0;
    gActSequence[0].PreAct = NULL;
    gActSequence[0].PostAct = NULL;
    gActSeqDepth = 1;
    MotionCtrl_Proc();
}

void MotionCtrl_AutoMotionInit(void)
{
    if(gRobotState == ROBOT_STATE_HOME){
        MotionCtrl_Start();
        gActSequence[0].LWheelDefDir = 0;
        gActSequence[0].RWheelDefDir = 0;
        gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].LWheelExpCnt = WHEEL_FAULT_BACK_CNT*2;
        gActSequence[0].RWheelExpCnt = WHEEL_FAULT_BACK_CNT*2;
        gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[0].LWheelSync = 0;
        gActSequence[0].RWheelSync = 0;
        gActSequence[0].PreAct = NULL;
        gActSequence[0].PostAct = NULL;
        gActSequence[1].LWheelDefDir = 0;
        gActSequence[1].RWheelDefDir = 1;
        gActSequence[1].LWheelInitSpeed = 10;
        gActSequence[1].RWheelInitSpeed = 10;
        gActSequence[1].LWheelExpCnt = WHEEL_TURN_180_CNT;
        gActSequence[1].RWheelExpCnt = WHEEL_TURN_180_CNT;
        gActSequence[1].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[1].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[1].LWheelSync = 0;
        gActSequence[1].RWheelSync = 0;
        gActSequence[1].PreAct = NULL;
        gActSequence[1].PostAct = NULL;
        gActSequence[2].LWheelDefDir = 1;
        gActSequence[2].RWheelDefDir = 1;
        gActSequence[2].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[2].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[2].LWheelExpCnt = 0x1;
        gActSequence[2].RWheelExpCnt = 0x2;
        gActSequence[2].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[2].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[2].LWheelSync = 0;
        gActSequence[2].RWheelSync = 0;
        gActSequence[2].PreAct = NULL;
        gActSequence[2].PostAct = NULL;
        gActSeqDepth = 3;
    }
    else{
        MotionCtrl_Start();
        gActSequence[0].LWheelDefDir = 1;
        gActSequence[0].RWheelDefDir = 1;
        gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].LWheelExpCnt = 0x1;
        gActSequence[0].RWheelExpCnt = 0x2;
        gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
        gActSequence[0].LWheelSync = 0;
        gActSequence[0].RWheelSync = 0;
        gActSequence[0].PreAct = NULL;
        gActSequence[0].PostAct = NULL;
        gActSeqDepth = 1;
    }

    MotionCtrl_Proc();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    MOTOR_FUN_CHAN_STARTUP_SPEED);
}

void MotionCtrl_ManualCtrlProc(enum MotionCtrlManualAct act)
{
    MotionCtrl_Start();

    switch(act){
        case MANUAL_ACT_UP:
            gActSequence[0].LWheelDefDir = 1;
            gActSequence[0].RWheelDefDir = 1;
            gActSequence[0].LWheelExpCnt = 0x1;
            gActSequence[0].RWheelExpCnt = 0x2;
            gActSeqDepth = 1;
            break;
        case MANUAL_ACT_LEFT:
            gActSequence[0].LWheelDefDir = 0;
            gActSequence[0].RWheelDefDir = 1;
            gActSequence[0].LWheelExpCnt = WHEEL_TURN_15_CNT;
            gActSequence[0].RWheelExpCnt = WHEEL_TURN_15_CNT;
            gActSeqDepth = 2;
            break;
        case MANUAL_ACT_DOWN:
            gActSequence[0].LWheelDefDir = 0;
            gActSequence[0].RWheelDefDir = 1;
            gActSequence[0].LWheelExpCnt = WHEEL_TURN_180_CNT;
            gActSequence[0].RWheelExpCnt = WHEEL_TURN_180_CNT;
            gActSeqDepth = 2;
            break;
        case MANUAL_ACT_RIGHT:
            gActSequence[0].LWheelDefDir = 1;
            gActSequence[0].RWheelDefDir = 0;
            gActSequence[0].LWheelExpCnt = WHEEL_TURN_15_CNT;
            gActSequence[0].RWheelExpCnt = WHEEL_TURN_15_CNT;
            gActSeqDepth = 2;
            break;
    }

    gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].LWheelSync = 0;
    gActSequence[0].RWheelSync = 0;
    gActSequence[0].PreAct = NULL;
    gActSequence[0].PostAct = NULL;
    gActSequence[1].LWheelDefDir = gActSequence[0].LWheelDefDir;
    gActSequence[1].RWheelDefDir = gActSequence[0].RWheelDefDir;
    gActSequence[1].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[1].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[1].LWheelExpCnt = 0x1;
    gActSequence[1].RWheelExpCnt = 0x2;
    gActSequence[1].LWheelExpSpeed = 0;
    gActSequence[1].RWheelExpSpeed = 0;
    gActSequence[1].LWheelSync = 0;
    gActSequence[1].RWheelSync = 0;
    gActSequence[2].PreAct = NULL;
    gActSequence[1].PostAct = NULL;
    MotionCtrl_Proc();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_FAN,    MOTOR_FUN_CHAN_STARTUP_SPEED);
}

void MotionCtrl_RoundedSlowly(void)
{
    MotionCtrl_Stop();
    mDelay(1);
    MotionCtrl_Start();

    gActSequence[0].LWheelDefDir = 1;
    gActSequence[0].RWheelDefDir = 1;
    gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpCnt = 0x1; //WHEEL_TURN_360_CNT;
    gActSequence[0].RWheelExpCnt = 0x2; //WHEEL_TURN_360_CNT;
    gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].LWheelSync = 0;
    gActSequence[0].RWheelSync = 0;
    gActSequence[0].PreAct = NULL;
    gActSequence[0].PostAct = NULL;
    gActSeqDepth = 1;
    MotionCtrl_Proc();
}