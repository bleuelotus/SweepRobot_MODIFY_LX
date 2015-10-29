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
#include "Buzzer.h"

#define MOTION_MONITOR_TIM_PERIOD               200                                // 20ms

#define IFRD_CHAN_FRONT_L                       0
#define IFRD_CHAN_FRONT_R                       1
#define IFRD_CHAN_SIDE_L                        2
#define IFRD_CHAN_SIDE_R                        3
#define IFRD_CHAN_BOTTOM_L                      4
#define IFRD_CHAN_BOTTOM_R                      5
#ifdef REVISION_1_2
#define IFRD_CHAN_BOTTOM_SL                     6
#define IFRD_CHAN_BOTTOM_SR                     7
#define IFRD_CHAN_BOTTOM_BL                     8
#define IFRD_CHAN_BOTTOM_BR                     9
#endif

/* TODO: use deltaT, gLWheelTotalCnt and gRWheelTotalCnt to implement path planning */
/* deltaT = deltaRad * Pi * BASE_LEN / 180 / deltaV [ Pre-condition: Vinner != 0 ]; deltaV = Vouter - Vinner */

#define EDGE_MODE_EXIT_CNT                      10
#define SPOT_MODE_INIT_CIRCLE_CNT               120                             // 120 * 5 * 2ms  // 30 * 20 * 2ms
#define SPOT_MODE_CIRCLE_INC_STEP               220                             // for every 2 units of speed inc , 55 * 4

#define UNIVERSAL_WHEEL_DETECT_PERIOD           110                             // 110 * 20 = 2200ms
#define EDGE_MODE_ANGLE_360                     300                             // 300 * 20 ms == 300 * 20ms
#define EXCEPTION_CHECK_PERIOD                  2                               // 2 * 20 = 40ms
#define EXCEPTION_WHEEL_STUCK_CHECK_PERIOD      1                               // 1 * 20 = 20ms
#define UNIVERSAL_WHEEL_CHECK_PERIOD            1                               // 1 * 20 = 20ms

#define IFRD_CHAN_BOTTOM_SWITCH_TIME_PERIOD     10000
#define IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT        (BAT_MONITOR_TIM->CNT)

static void MotionCtrl_ExceptionProc(void);

enum _PathFaultProcMode {

    PATH_FAULT_PROC_MODE_NORMAL,
    PATH_FAULT_PROC_MODE_EDGE,
    PATH_FAULT_PROC_MODE_EDGE_L,
    PATH_FAULT_PROC_MODE_EDGE_R,
    PATH_FAULT_PROC_MODE_SPOT_L,
    PATH_FAULT_PROC_MODE_SPOT_R,
};

#ifdef REVISION_1_0
#define UNIVERSAL_WHEEL_ACTIVE_THRESHOLD    1
#elif defined REVISION_1_1
#define UNIVERSAL_WHEEL_ACTIVE_THRESHOLD    400
#elif defined REVISION_1_2
#define UNIVERSAL_WHEEL_ACTIVE_THRESHOLD    400
#endif

/* Infrared based proximity detection sensitivity */
/* FIXME: bottom detection threshold should set lower to adjust sheet, origin bottom detection threshold is 150 */
#ifdef REVISION_1_0
const u16 gProximityDetectionThreshold[IFRD_TxRx_CHAN_NUM] = { 800, 800, 250, 250, 120, 120 };
const u16 gHighLightDetectionThreshold[IFRD_TxRx_CHAN_NUM] = { 198, 198, 2500, 2500, 3000, 3000 };
#elif defined REVISION_1_1
const u16 gProximityDetectionThreshold[IFRD_TxRx_CHAN_NUM] = { 800, 800, 250, 250, 120, 120 };
const u16 gHighLightDetectionThreshold[IFRD_TxRx_CHAN_NUM] = { 198, 198, 2500, 2500, 3000, 3000 };
#elif defined REVISION_1_2
const u16 gProximityDetectionThreshold[IFRD_TxRx_CHAN_NUM] = { 1000, 1000, 250, 250, 120, 120, 120, 120 };
const u16 gHighLightDetectionThreshold[IFRD_TxRx_CHAN_NUM] = { 198, 198, 2500, 2500, 150, 150, 150, 150 };
#endif

static u32 gLWheelTotalCnt = 4, gRWheelTotalCnt = 4, gLastTotalLWheelCnt = 0, gLastTotalRWheelCnt = 0;
static u16 gWheelCnt[WHEEL_NUM] = {0};
static u16 gLWheelExpCnt = 0xFFFF, gRWheelExpCnt = 0xFFFF;

typedef void (*WheelExpFunc_t)(void);

static WheelExpFunc_t  gLWheelExpCB = NULL;
static WheelExpFunc_t  gRWheelExpCB = NULL;
static WheelExpFunc_t  gWheelProcExitCB = NULL;
static WheelExpFunc_t  gWheelProcCB = NULL;
static u16 gLastWheelCnt[WHEEL_NUM] = {0};
static u16 gDeltaWheelCnt[WHEEL_NUM] = {0};
static u8 gCurLWheelSpeed = WHEEL_CRUISE_SPEED, gCurRWheelSpeed = WHEEL_CRUISE_SPEED;

static s8 gLastPathFault = 0; /* 0: none, 1: right, -1: left */
static u8 gConsecutivePathFaultCnt = 0;
static u8 gPathFaultProcEdgeModeCnt = 0;
static u16 gSpotModeAngleCnt = 0, gSpotModeCircleCnt = SPOT_MODE_INIT_CIRCLE_CNT;
static enum _PathFaultProcMode gPathFaultProcMode = PATH_FAULT_PROC_MODE_NORMAL;
static s16 gLastPathCondSLDiff = 0, gLastPathCondSRDiff = 0;
static s16 gCurPathCondSLDiff = 0, gCurPathCondSRDiff = 0;
static u16 gEdgeModeAngleCnt = 0;

static u8 gHomingSigCapturedFlag = 0;

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
#define WHEEL_PROC_EXIT_CB_REG(f)                   do{gWheelProcExitCB=f;}while(0)
#define WHEEL_PROC_CB_REG(f)                        do{gWheelProcCB=f;}while(0)
#define LWHEEL_EXP_SPEED_SET(s)                     do{gCurLWheelSpeed=s;}while(0)
#define RWHEEL_EXP_SPEED_SET(s)                     do{gCurRWheelSpeed=s;}while(0)
#define LWHEEL_EXP_SPEED_INC(s)                     do{gCurLWheelSpeed+=s;}while(0)
#define RWHEEL_EXP_SPEED_INC(s)                     do{gCurRWheelSpeed+=s;}while(0)
#define LWHEEL_EXP_SPEED_DEC(s)                     do{gCurLWheelSpeed-=s;}while(0)
#define RWHEEL_EXP_SPEED_DEC(s)                     do{gCurRWheelSpeed-=s;}while(0)
#define LWHEEL_CUR_SPEED                            (gCurLWheelSpeed)
#define RWHEEL_CUR_SPEED                            (gCurRWheelSpeed)



static u16 gtmpAvoidencePeriodCnt = 0;
static u16 gtmpExceptionPeriodCnt = 0;
static u16 gtmpUniversalWheelPeriodCnt = 0;

static u16 gIFRDTxOffRxVal[IFRD_TxRx_CHAN_NUM] = {0};
static u32 gPathCondMap = 0;
static u8 gLastExceptionMask = 0, gExceptionMask = 0, gIsExceptionHandling = 0;
static MotionException_ErrCnt_t gMotionExceptionErrCnt;
static u16 gUniversalWheelActiveVal = 0, gUniversalWheelActiveValLast = 0, gUniveralWheelDetectPeriodCnt = 0;
static Msg_t gMsg;

//void FWheelCounterISR(void)
//{
//    FWHEEL_CNT++;
//}

void LWheelCounterISR(void)
{
    LWHEEL_CNT++;
    gLWheelTotalCnt++;
    if(NULL != gWheelProcCB){
        gWheelProcCB();
    }
    if(LWHEEL_CNT == WHEEL_BODY_THROUGH_CNT){
        gLastPathFault = 0;
        gConsecutivePathFaultCnt = 0;
        gHomingSigCapturedFlag = 0;
        gMotionExceptionErrCnt.AshTrayInsErrCnt = 0;
        gMotionExceptionErrCnt.WheelFloatErrCnt = 0;
        gMotionExceptionErrCnt.FanOCErrCnt = 0;
        gMotionExceptionErrCnt.MBrushOCErrCnt = 0;
        gMotionExceptionErrCnt.LBrushOCErrCnt = 0;
        gMotionExceptionErrCnt.RBrushOCErrCnt = 0;
        gMotionExceptionErrCnt.WheelStuckErrCnt = 0;
    }
    if(LWHEEL_CNT == gLWheelExpCnt){
        if(NULL != gLWheelExpCB){
            gLWheelExpCB();
        }
    }
}

void RWheelCounterISR(void)
{
    RWHEEL_CNT++;
    gRWheelTotalCnt++;
    if(NULL != gWheelProcCB){
        gWheelProcCB();
    }
    if(RWHEEL_CNT == WHEEL_BODY_THROUGH_CNT){
        gLastPathFault = 0;
        gConsecutivePathFaultCnt = 0;
        gHomingSigCapturedFlag = 0;
        gMotionExceptionErrCnt.AshTrayInsErrCnt = 0;
        gMotionExceptionErrCnt.WheelFloatErrCnt = 0;
        gMotionExceptionErrCnt.FanOCErrCnt = 0;
        gMotionExceptionErrCnt.MBrushOCErrCnt = 0;
        gMotionExceptionErrCnt.LBrushOCErrCnt = 0;
        gMotionExceptionErrCnt.RBrushOCErrCnt = 0;
        gMotionExceptionErrCnt.WheelStuckErrCnt = 0;
    }
    if(RWHEEL_CNT == gRWheelExpCnt){
        if(NULL != gRWheelExpCB){
            gRWheelExpCB();
        }
    }
}

u8 ExceptionStateCheck(void)
{
    u8 ExceptionMask = 0;

    /* Update exception sign: left, right, middle brush over loading, wheel floating and ash tray exist or not */
//    ExceptionMask |= (ASH_TRAY_INSTALL_SIGN ? 1 : 0) << EXCEPTION_MASK_ASHTRAY_INS_POS;
    ExceptionMask |= (WHEEL_FLOAT_SIGN_ALL ? 1 : 0) << EXCEPTION_MASK_WHEEL_FLOAT_POS;

    /* FIXME: change ADC value compare to actual value compare, use 1.2V internal voltage referance*/
    ADC2ValueConvertedLSB[MEAS_CHAN_FAN_CUR-1] = 1.2f*((float)ADCConvertedLSB[MEAS_CHAN_FAN_CUR-1]/(float)ADCConvertedLSB[MEAS_CHAN_INTERNAL_REFVOL-1]);
    ExceptionMask |= ((ADC2ValueConvertedLSB[MEAS_CHAN_FAN_CUR-1] > FAN_CUR_THRESHOLD) ? 1 : 0) << EXCEPTION_MASK_FAN_OC_POS;

    ADC2ValueConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1] = 1.2f*((float)ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1]/(float)ADCConvertedLSB[MEAS_CHAN_INTERNAL_REFVOL-1]);
    ExceptionMask |= ((ADC2ValueConvertedLSB[MEAS_CHAN_BRUSH_CUR_LEFT-1] > LBRUSH_CUR_THRESHOLD) ? 1 : 0) << EXCEPTION_MASK_LBRUSH_OC_POS;

    ADC2ValueConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1] =  1.2f*((float)ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1]/(float)ADCConvertedLSB[MEAS_CHAN_INTERNAL_REFVOL-1]);
    ExceptionMask |= ((ADC2ValueConvertedLSB[MEAS_CHAN_BRUSH_CUR_RIGHT-1] > RBRUSH_CUR_THRESHOLD) ? 1 : 0) << EXCEPTION_MASK_RBRUSH_OC_POS;

    ADC2ValueConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1] =  1.2f*((float)ADCConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1]/(float)ADCConvertedLSB[MEAS_CHAN_INTERNAL_REFVOL-1]);
    ExceptionMask |= ((ADC2ValueConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1] > MBRUSH_CUR_THRESHOLD) ? 1 : 0) << EXCEPTION_MASK_MBRUSH_OC_POS;

    ExceptionMask |= (((30 < MotorCtrl_ChanSpeedLevelGet(MOTOR_CTRL_CHAN_LWHEEL)) && ((gLWheelTotalCnt - gLastTotalLWheelCnt) < 4)) ? 1 : 0) << EXCEPTION_MASK_LWHEEL_STUCK_POS;
    ExceptionMask |= (((30 < MotorCtrl_ChanSpeedLevelGet(MOTOR_CTRL_CHAN_RWHEEL)) && ((gRWheelTotalCnt - gLastTotalRWheelCnt) < 4)) ? 1 : 0) << EXCEPTION_MASK_RWHEEL_STUCK_POS;

    gLastTotalLWheelCnt = gLWheelTotalCnt;
    gLastTotalRWheelCnt = gRWheelTotalCnt;

//    printf("%f,%f,\r\n",ADC2ValueConvertedLSB[MEAS_CHAN_FAN_CUR-1], ADC2ValueConvertedLSB[MEAS_CHAN_BRUSH_CUR_MIDDLE-1]);
    return ExceptionMask;
}

void MotionStateProc(void)
{
    u8      i = 0;
    static u16 gLastTimeCnt = 0;

    if( !( (++gtmpExceptionPeriodCnt) % EXCEPTION_CHECK_PERIOD) ){
        gExceptionMask = ExceptionStateCheck();
        if( (gLastExceptionMask & gExceptionMask) && (!gIsExceptionHandling) ){
            /* Send exception message */
            gMsg.expire = 0;
            gMsg.prio = MSG_PRIO_HIGHEST;
            gMsg.type = MSG_TYPE_MOTION;
            gMsg.MsgCB = NULL;
            gMsg.Data.MEvt = MOTION_EVT_EXCEPTION;
            if(!SweepRobot_SendMsg(&gMsg)){
                gIsExceptionHandling = 1;
            }
        }
        gLastExceptionMask = gExceptionMask;
    }

    /* Phase 1 */
    if((++gtmpAvoidencePeriodCnt)%2){

        /* Save proximity condition in Tx off */
        for(i = 0; i < IFRD_TxRx_ACTUAL_CHAN_NUM; i++){
            gIFRDTxOffRxVal[i] = ADCConvertedLSB[i];
        }

        /* FIXME: add bottom sensor switch process here */
        Meas_IFRD_BOTTOM_RX_SWITCH(AD_IFRD_BOTTOM_RX_SWITCH_GPIO, AD_IFRD_BOTTOM_RX_SWITCH_PIN, 1);
//        gLastTimeCnt = IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT;
//        while( ( (gLastTimeCnt > IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT) ? gLastTimeCnt - IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT : IFRD_CHAN_BOTTOM_SWITCH_TIME_PERIOD - gLastTimeCnt + IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT ) < 10 );
        uDelay(100);

        for(i = IFRD_TxRx_ACTUAL_CHAN_NUM; i < IFRD_TxRx_CHAN_NUM; i++){
            gIFRDTxOffRxVal[i] = ADCConvertedLSB[i-(IFRD_TxRx_CHAN_NUM-IFRD_TxRx_ACTUAL_CHAN_NUM)];
        }

        IFRD_TX_ENABLE();
    }
    /* Phase 2 */
    else{
        /* Update front, side and bottom of LEFT proximity condition in Tx on */
        if( (gIFRDTxOffRxVal[IFRD_CHAN_FRONT_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_FRONT_RX_L-1] > gProximityDetectionThreshold[IFRD_CHAN_FRONT_L]) && (gHomingStage < ROBOT_HOMING_STAGE3) ) {
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_FL_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_FL_POS);
        }

#ifdef REVISION_1_2

        if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_SL] < gHighLightDetectionThreshold[IFRD_CHAN_BOTTOM_SL]) ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BSL_POS);
        }else {
            if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_SL] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_L-1] < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_SL]) ){
                gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BSL_POS);
            }
            else {
                gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_BSL_POS);
            }
        }

        /* FIXME: add bottom sensor switch process here */
        Meas_IFRD_BOTTOM_RX_SWITCH(AD_IFRD_BOTTOM_RX_SWITCH_GPIO, AD_IFRD_BOTTOM_RX_SWITCH_PIN, 0);
//        gLastTimeCnt = IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT;
//        while( ( (gLastTimeCnt > IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT) ? gLastTimeCnt - IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT : IFRD_CHAN_BOTTOM_SWITCH_TIME_PERIOD - gLastTimeCnt + IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT ) < 10 );
        uDelay(100);

        if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_L] < gHighLightDetectionThreshold[IFRD_CHAN_BOTTOM_L]) ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BL_POS);
        }else {
            if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_L-1] < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_L]) ){
                gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BL_POS);
            }
            else {
                gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_BL_POS);
            }
        }
#else
        if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_L] < gHighLightDetectionThreshold[IFRD_CHAN_BOTTOM_L]) ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BL_POS);
        }else {
            if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_L-1] < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_L]) ){
                gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BL_POS);
            }
            else {
                gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_BL_POS);
            }
        }
#endif
        gCurPathCondSLDiff = gIFRDTxOffRxVal[IFRD_CHAN_SIDE_L] - ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_L-1];
        if( gCurPathCondSLDiff > gProximityDetectionThreshold[IFRD_CHAN_SIDE_L] ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_SL_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_SL_POS);
        }

        /* Path adjust for edge mode */
        if( IS_MOTION_PROC_FINISH() ){
            if( (gRobotMode == ROBOT_WORK_MODE_EDGE) && (gPathFaultProcMode == PATH_FAULT_PROC_MODE_EDGE_L) ){
                if( (gLastPathCondSLDiff - gCurPathCondSLDiff) >= 50 ){
                    /* faraway */
                    LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-3);
                    RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                    gEdgeModeAngleCnt = 0;
                }
                else if( (gLastPathCondSLDiff - gCurPathCondSLDiff) <= -50 ){
                    /* closer */
                    LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                    RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-3);
                    gEdgeModeAngleCnt = 0;
                }
                else{
                    /* no change */
                    if( gPathCondMap & (1 << PATH_COND_PROXIMITY_FLAG_SL_POS) ){
                        LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                        RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                        gEdgeModeAngleCnt = 0;
                    }
                    else{
                        LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-6);
                        RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                        if((gEdgeModeAngleCnt++) > EDGE_MODE_ANGLE_360){
                            LWHEEL_EXP_SPEED_SET(WHEEL_CRUISE_SPEED);
                            RWHEEL_EXP_SPEED_SET(WHEEL_CRUISE_SPEED);
                            gRobotMode = gRobotModeLast;
                            gEdgeModeAngleCnt = 0;
                        }
                    }
                }
            }
            else if( (gRobotMode == ROBOT_WORK_MODE_SPOT) && (gPathFaultProcMode == PATH_FAULT_PROC_MODE_SPOT_L) ){
                if((gSpotModeAngleCnt++) >= gSpotModeCircleCnt){
                    if(LWHEEL_CUR_SPEED < (WHEEL_CRUISE_SPEED/2)){
                        gSpotModeAngleCnt = 0;
                        gSpotModeCircleCnt += SPOT_MODE_CIRCLE_INC_STEP;
                        LWHEEL_EXP_SPEED_INC(1);
                    }
                    else{
                        gSpotModeCircleCnt -= SPOT_MODE_CIRCLE_INC_STEP;
                        /* change clock direction */
                        gMsg.expire = 0;
                        gMsg.prio = MSG_PRIO_HIGH;
                        gMsg.type = MSG_TYPE_MOTION;
                        gMsg.MsgCB = NULL;
                        gMsg.Data.MEvt = MOTION_EVT_PATH_FAULT;
                        SweepRobot_SendMsg(&gMsg);
                    }
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

#ifdef REVISION_1_2
        if( gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_R] < gHighLightDetectionThreshold[IFRD_CHAN_BOTTOM_R] ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BR_POS);
        }else {
            if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_R-1]) < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_R] ){
                gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BR_POS);
            }
            else {
                gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_BR_POS);
            }
        }

        /* FIXME: add bottom sensor switch process here */
        Meas_IFRD_BOTTOM_RX_SWITCH(AD_IFRD_BOTTOM_RX_SWITCH_GPIO, AD_IFRD_BOTTOM_RX_SWITCH_PIN, 1);
//        gLastTimeCnt = IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT;
//        while( ( (gLastTimeCnt > IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT) ? gLastTimeCnt - IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT : IFRD_CHAN_BOTTOM_SWITCH_TIME_PERIOD - gLastTimeCnt + IFRD_CHAN_BOTTOM_SWITCH_TIME_CNT ) < 10 );
        uDelay(100);

        if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_SR] < gHighLightDetectionThreshold[IFRD_CHAN_BOTTOM_SR]) ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BSR_POS);
        }else {
            if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_SR] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_R-1] < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_SR]) ){
                gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BSR_POS);
            }
            else {
                gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_BSR_POS);
            }
        }
#else
        /* XXX: add bottom high light detection function */
        if( gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_R] < gHighLightDetectionThreshold[IFRD_CHAN_BOTTOM_R] ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BR_POS);
        }else {
            if( (gIFRDTxOffRxVal[IFRD_CHAN_BOTTOM_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_BOTTOM_RX_R-1]) < gProximityDetectionThreshold[IFRD_CHAN_BOTTOM_R] ){
                gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_BR_POS);
            }
            else {
                gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_BR_POS);
            }
        }
#endif

        gCurPathCondSRDiff = gIFRDTxOffRxVal[IFRD_CHAN_SIDE_R] - ADCConvertedLSB[MEAS_CHAN_IFRD_SIDE_RX_R-1];
        if( gCurPathCondSRDiff > gProximityDetectionThreshold[IFRD_CHAN_SIDE_R] ){
            gPathCondMap |= (1 << PATH_COND_PROXIMITY_FLAG_SR_POS);
        }
        else {
            gPathCondMap &= ~(1 << PATH_COND_PROXIMITY_FLAG_SR_POS);
        }
        /* Path adjust for edge mode */
        if( IS_MOTION_PROC_FINISH() ){
            if( (gRobotMode == ROBOT_WORK_MODE_EDGE) && (gPathFaultProcMode == PATH_FAULT_PROC_MODE_EDGE_R) ) {
                if( (gLastPathCondSRDiff - gCurPathCondSRDiff) >= 50 ){
                    /* faraway */
                    LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                    RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-3);
                    gEdgeModeAngleCnt = 0;
                }
                else if( (gLastPathCondSRDiff - gCurPathCondSRDiff) <= -50 ){
                    /* closer */
                    LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-3);
                    RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                    gEdgeModeAngleCnt = 0;
                }
                else{
                    /* no change */
                    if( gPathCondMap & (1 << PATH_COND_PROXIMITY_FLAG_SR_POS) ){
                        LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                        RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                        gEdgeModeAngleCnt = 0;
                    }
                    else{
                        LWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED);
                        RWHEEL_EXP_SPEED_SET(WHEEL_FAULT_PROC_SPEED-6);
                        if((gEdgeModeAngleCnt++) > EDGE_MODE_ANGLE_360){
                            LWHEEL_EXP_SPEED_SET(WHEEL_CRUISE_SPEED);
                            RWHEEL_EXP_SPEED_SET(WHEEL_CRUISE_SPEED);
                            gRobotMode = gRobotModeLast;
                            gEdgeModeAngleCnt = 0;
                        }
                    }
                }
            }
            else if( (gRobotMode == ROBOT_WORK_MODE_SPOT) && (gPathFaultProcMode == PATH_FAULT_PROC_MODE_SPOT_R) ){
                if((gSpotModeAngleCnt--) <= gSpotModeCircleCnt){
                    if(RWHEEL_CUR_SPEED > 0){
                        gSpotModeAngleCnt = 0;
                        gSpotModeCircleCnt -= SPOT_MODE_CIRCLE_INC_STEP;
                        RWHEEL_EXP_SPEED_DEC(1);
                    }
                    else{
                        gSpotModeCircleCnt += SPOT_MODE_CIRCLE_INC_STEP;
                        /* change clock direction */
                        gMsg.expire = 0;
                        gMsg.prio = MSG_PRIO_HIGH;
                        gMsg.type = MSG_TYPE_MOTION;
                        gMsg.MsgCB = NULL;
                        gMsg.Data.MEvt = MOTION_EVT_PATH_FAULT;
                        SweepRobot_SendMsg(&gMsg);
                    }
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

#ifdef REVISION_1_2
        Meas_IFRD_BOTTOM_RX_SWITCH(AD_IFRD_BOTTOM_RX_SWITCH_GPIO, AD_IFRD_BOTTOM_RX_SWITCH_PIN, 0);
#endif

        if( IS_MOTION_PROC_FINISH() && (gPathCondMap & (PATH_FAULT_LEFT_MASK | PATH_FAULT_RIGHT_MASK)) ){
            if( ((gRobotMode != ROBOT_WORK_MODE_EDGE) && (gRobotMode != ROBOT_WORK_MODE_SPOT))
               ||
               (((gRobotMode == ROBOT_WORK_MODE_EDGE) || (gRobotMode == ROBOT_WORK_MODE_SPOT)) && (gPathCondMap & ((PATH_FAULT_PROXIMITY_MASK & PATH_FAULT_BOTTOM_MASK) | PATH_FAULT_COLLISION_MASK)))
              ){
                /* Send path fault message */
                gMsg.expire = 0;
                gMsg.prio = MSG_PRIO_HIGH;
                gMsg.type = MSG_TYPE_MOTION;
                gMsg.MsgCB = NULL;
                gMsg.Data.MEvt = MOTION_EVT_PATH_FAULT;
                if(!SweepRobot_SendMsg(&gMsg)){
                    MOTION_PROC_STATE_SET();
                }
            }
        }
    }

    if( (++gtmpUniversalWheelPeriodCnt) == UNIVERSAL_WHEEL_CHECK_PERIOD ){
        /* Universal wheel signal check */
        gUniversalWheelActiveVal = FWHEEL_ACTIVE_VAL;
    #ifdef DEBUG_LOG
    //        printf(":%d\r\n", gUniversalWheelActiveVal);
    #endif
        if( (LWHEEL_CUR_SPEED > 1) && (LWHEEL_CUR_SPEED==RWHEEL_CUR_SPEED) && (MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_LWHEEL)==1) && (MotorCtrl_ChanDirGet(MOTOR_CTRL_CHAN_RWHEEL)==1) ){
            if(abs(gUniversalWheelActiveVal-gUniversalWheelActiveValLast) > UNIVERSAL_WHEEL_ACTIVE_THRESHOLD){
                FWHEEL_CNT++;
                gUniversalWheelActiveValLast = gUniversalWheelActiveVal;
            }
            gUniveralWheelDetectPeriodCnt++;
            if(gUniveralWheelDetectPeriodCnt > UNIVERSAL_WHEEL_DETECT_PERIOD){
                gUniveralWheelDetectPeriodCnt = 0;
    #ifdef DEBUG_LOG
    //                printf("%d\r\n", FWHEEL_CNT);
    #endif
                gDeltaWheelCnt[WHEEL_IDX_F] = FWHEEL_CNT - gLastWheelCnt[WHEEL_IDX_F];
                if( (gDeltaWheelCnt[WHEEL_IDX_F] < 5) || (gDeltaWheelCnt[WHEEL_IDX_F] > 150) ){
                    /* Send block message */
    #ifdef DEBUG_LOG
                    printf("[%d]Robot is trapped.\r\n", gDeltaWheelCnt[WHEEL_IDX_F]);
    #endif
                    gMsg.expire = 0;
                    gMsg.prio = MSG_PRIO_NORMAL;
                    gMsg.type = MSG_TYPE_MOTION;
                    gMsg.MsgCB = NULL;
                    gMsg.Data.MEvt = MOTION_EVT_TRAPPED;
                    SweepRobot_SendMsg(&gMsg);
                }
                FWHEEL_CNT_CLR();
                gLastWheelCnt[WHEEL_IDX_F] = FWHEEL_CNT;
            }
        }
        else{
            FWHEEL_CNT_CLR();
            gUniveralWheelDetectPeriodCnt = 0;
        }
        gLastWheelCnt[WHEEL_IDX_L] = LWHEEL_CNT;
        gLastWheelCnt[WHEEL_IDX_R] = RWHEEL_CNT;
    }else if(gtmpUniversalWheelPeriodCnt == (UNIVERSAL_WHEEL_CHECK_PERIOD*2) ){
        /* Wheel speed adjust */
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
        gtmpUniversalWheelPeriodCnt = 0;
    }else
        ;
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
    NVIC_InitStructure.NVIC_IRQChannel = MOTION_MONITOR_TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MOTION_MONITOR_TIM_IRQ_PP;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = MOTION_MONITOR_TIM_IRQ_SP;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(MOTION_MONITOR_TIM_PERIPH_ID , ENABLE);
    TIM_DeInit(MOTION_MONITOR_TIM);
    TIM_TimeBaseStructure.TIM_Period = MOTION_MONITOR_TIM_PERIOD-1;             //1ms
    TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(MOTION_MONITOR_TIM, &TIM_TimeBaseStructure);
    TIM_ClearFlag(MOTION_MONITOR_TIM, TIM_FLAG_Update);
    TIM_ITConfig(MOTION_MONITOR_TIM, TIM_IT_Update, ENABLE);
}

static void IFRD_PathDetectStart(void)
{
    gtmpAvoidencePeriodCnt = 0;
    gtmpExceptionPeriodCnt = 0;
    gtmpUniversalWheelPeriodCnt = 0;

    TIM_SetCounter(MOTION_MONITOR_TIM, 0);
    TIM_ITConfig(MOTION_MONITOR_TIM, TIM_IT_Update, DISABLE);
    TIM_SetAutoreload(MOTION_MONITOR_TIM, MOTION_MONITOR_TIM_PERIOD);
    TIM_ClearFlag(MOTION_MONITOR_TIM, TIM_FLAG_Update);
    TIM_ITConfig(MOTION_MONITOR_TIM, TIM_IT_Update, ENABLE);
    plat_int_reg_cb(MOTION_MONITOR_TIM_INT_IDX, (void*)MotionStateProc);

    TIM_Cmd(MOTION_MONITOR_TIM, ENABLE);
}

static void IFRD_PathDetectStop(void)
{
    TIM_SetCounter(MOTION_MONITOR_TIM, 0);
    TIM_Cmd(MOTION_MONITOR_TIM, DISABLE);

    IFRD_TX_DISABLE();
}

static void WheelCntMach_Start(void)
{
    EXTI_InitTypeDef            EXTI_InitStructure;

    LWHEEL_CNT_CLR();
    RWHEEL_CNT_CLR();
    LWHEEL_EXP_CB_REG(NULL);
    RWHEEL_EXP_CB_REG(NULL);
    LWHEEL_EXP_CNT_SET(0xFFFF);
    RWHEEL_EXP_CNT_SET(0xFFFF);

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

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
#ifdef REVISION_1_0
    GPIO_InitStructure.GPIO_Pin = FWHEEL_COUNTER_PIN;
    GPIO_Init(FWHEEL_COUNTER_GPIO, &GPIO_InitStructure);
#endif
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

    /* Asy tray detection init */
    RCC_APB2PeriphClockCmd(ASH_TRAY_DETECT_GPIO_PERIPH_ID, ENABLE);

    GPIO_InitStructure.GPIO_Pin = ASH_TRAY_DETECT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ASH_TRAY_DETECT_GPIO, &GPIO_InitStructure);

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

static void MotionCtrl_LWheelProcExitOn(void)
{
    LWHEEL_EXP_CB_REG(NULL);
    gActSeqDepLIndicator--;
    if((NULL!=gWheelProcExitCB) && (gActSeqDepRIndicator==0) && (gActSeqDepLIndicator==0)){
        gWheelProcExitCB();
        WHEEL_PROC_EXIT_CB_REG(NULL);
    }
}

static void MotionCtrl_RWheelProcExitOn(void)
{
    RWHEEL_EXP_CB_REG(NULL);
    gActSeqDepRIndicator--;
    if((NULL!=gWheelProcExitCB) && (gActSeqDepRIndicator==0) && (gActSeqDepLIndicator==0)){
        gWheelProcExitCB();
        WHEEL_PROC_EXIT_CB_REG(NULL);
    }
}

static void MotionCtrl_LWheelProcExitOff(void)
{
    LWHEEL_EXP_CB_REG(NULL);
//    LWHEEL_EXP_SPEED_SET(0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
    gActSeqDepLIndicator--;
    if((gActSeqDepRIndicator==0) && (gActSeqDepLIndicator==0)){
        MotionCtrl_Stop();
        /* Notify main process make the further stop */
        gMsg.expire = 0;
        gMsg.prio = MSG_PRIO_HIGHEST;
        gMsg.type = MSG_TYPE_MOTION;
        gMsg.MsgCB = NULL;
        gMsg.Data.MEvt = MOTION_EVT_IDLE_SYNC;
        SweepRobot_SendMsg(&gMsg);
    }
}

static void MotionCtrl_RWheelProcExitOff(void)
{
    RWHEEL_EXP_CB_REG(NULL);
//    RWHEEL_EXP_SPEED_SET(0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);
    gActSeqDepRIndicator--;
    if((gActSeqDepRIndicator==0) && (gActSeqDepLIndicator==0)){
        MotionCtrl_Stop();
        /* Notify main process make the further stop */
        gMsg.expire = 0;
        gMsg.prio = MSG_PRIO_HIGHEST;
        gMsg.type = MSG_TYPE_MOTION;
        gMsg.MsgCB = NULL;
        gMsg.Data.MEvt = MOTION_EVT_IDLE_SYNC;
        SweepRobot_SendMsg(&gMsg);
    }
}

static void MotionCtrl_RWheelSubProc(void)
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

static void MotionCtrl_LWheelSubProc(void)
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
    /* Running light */
    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, CTRL_PANEL_LED_BR_LVL);

    WheelCntMach_Start();

    gActSeqDepth = 0;
    gActSeqDepLIndicator = 0;
    gActSeqDepRIndicator = 0;
    gLastPathFault = 0;
    gPathFaultProcEdgeModeCnt = 0;
}

void MotionCtrl_Stop(void)
{
    WheelCntMach_Stop();

    IFRD_PathDetectStop();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);

    gActSeqDepth = 0;
    gActSeqDepLIndicator = 0;
    gActSeqDepRIndicator = 0;
    gLastPathFault = 0;
    gPathFaultProcEdgeModeCnt = 0;

    gIsExceptionHandling = 0;
    gMotionExceptionErrCnt.AshTrayInsErrCnt = 0;
    gMotionExceptionErrCnt.WheelFloatErrCnt = 0;
    gMotionExceptionErrCnt.FanOCErrCnt = 0;
    gMotionExceptionErrCnt.MBrushOCErrCnt = 0;
    gMotionExceptionErrCnt.LBrushOCErrCnt = 0;
    gMotionExceptionErrCnt.RBrushOCErrCnt = 0;
    gMotionExceptionErrCnt.WheelStuckErrCnt = 0;

    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, CTRL_PANEL_LED_BR_LVL);
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

u8 MotionCtrl_PathFaultTurnProcCompleteCondTest(void)
{
    /* FIXME: only use PATH_FAULT_PROXIMITY_MASK may cause avoidence stuck */
//    return (!(gPathCondMap & PATH_FAULT_PROXIMITY_MASK));
    if( gPathCondMap & PATH_FAULT_PROXIMITY_MASK ){
        if(gPathCondMap & PATH_FAULT_COLLISION_MASK){
            return 1;
        }else
            return 0;
    }else{
        return 1;
    }
}

void MotionCtrl_PathFaultEdgeModeProcAct(struct MotionCtrl_Action_s *node)
{
    if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_L){
    }
    else if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_R){
    }
}

void MotionCtrl_PathFaultProc(u8 StopOnFinish)
{
    MCtrl_Act_t *pActSequence = gActSequence;
    u16 backCntL = WHEEL_FAULT_BACK_CNT, turnCntL = WHEEL_TURN_45_CNT;
    u16 backCntR = WHEEL_FAULT_BACK_CNT, turnCntR = WHEEL_TURN_45_CNT;

    gActSeqDepth = 0;

    /* Stage 1 */
    if(gPathCondMap & (PATH_FAULT_FRONT_MASK | PATH_FAULT_BOTTOM_MASK & PATH_FAULT_FRONT_MASK)){

        if( ((gRobotMode != ROBOT_WORK_MODE_EDGE) && (gRobotMode != ROBOT_WORK_MODE_SPOT))
            ||
            (((gRobotMode == ROBOT_WORK_MODE_EDGE) || (gRobotMode == ROBOT_WORK_MODE_SPOT)) && (gPathCondMap & ( PATH_FAULT_PROXIMITY_MASK & (PATH_FAULT_BOTTOM_MASK & PATH_FAULT_FRONT_MASK)  | PATH_FAULT_COLLISION_MASK & PATH_FAULT_FRONT_MASK ) )
          ) ){
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

        if(gRobotMode != ROBOT_WORK_MODE_EDGE){
            if( gConsecutivePathFaultCnt < (EDGE_MODE_EXIT_CNT*2) ){
                gConsecutivePathFaultCnt += 2;
            }
            else{
                gRobotModeLast = gRobotMode;
                gRobotMode = ROBOT_WORK_MODE_EDGE;
                gPathFaultProcEdgeModeCnt = EDGE_MODE_EXIT_CNT;
                if(gPathCondMap & PATH_FAULT_LEFT_MASK){
                    gPathFaultProcMode = PATH_FAULT_PROC_MODE_EDGE_L;
                }
                else if(gPathCondMap & PATH_FAULT_RIGHT_MASK){
                    gPathFaultProcMode = PATH_FAULT_PROC_MODE_EDGE_R;
                }
            }
        }
        else{
            if(gPathFaultProcMode == PATH_FAULT_PROC_MODE_EDGE){
                gRobotModeLast = gRobotMode;
                if(gPathCondMap & PATH_FAULT_LEFT_MASK){
                    gPathFaultProcMode = PATH_FAULT_PROC_MODE_EDGE_L;
                }
                else if(gPathCondMap & PATH_FAULT_RIGHT_MASK){
                    gPathFaultProcMode = PATH_FAULT_PROC_MODE_EDGE_R;
                }
            }
            else{
                if( !gPathFaultProcEdgeModeCnt ){
                    gRobotMode = gRobotModeLast;
                }
                else{
                    gPathFaultProcEdgeModeCnt--;
                }
            }
        }
    }

    /* Stage 2 */
    if(gRobotMode == ROBOT_WORK_MODE_EDGE) {
        if( gPathCondMap & (PATH_FAULT_SIDE_MASK & PATH_FAULT_COLLISION_MASK) ){
            if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_L){
                turnCntL = WHEEL_TURN_30_CNT;
                turnCntR = 5;
            }
            else{
                turnCntL = 5;
                turnCntR = WHEEL_TURN_30_CNT;
            }
        }
        /* else user default turn angle */
        if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_EDGE_L){
            pActSequence->LWheelDefDir = 1;
            pActSequence->RWheelDefDir = 0;
        }
        /* Edge mode R */
        else{
            pActSequence->LWheelDefDir = 0;
            pActSequence->RWheelDefDir = 1;
        }

        pActSequence->PreAct = NULL;
        pActSequence->PostAct = NULL;
    }
    else if(gRobotMode == ROBOT_WORK_MODE_SPOT){
        turnCntL = WHEEL_TURN_180_CNT;
        turnCntR = WHEEL_TURN_180_CNT;

        if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_SPOT_L){
            pActSequence->LWheelDefDir = 0;
            pActSequence->RWheelDefDir = 1;
        }
        else if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_SPOT_R){
            pActSequence->LWheelDefDir = 1;
            pActSequence->RWheelDefDir = 0;
        }
        pActSequence->PreAct = NULL;
        pActSequence->PostAct = NULL;
    }
    else{
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
        if( (gPathCondMap & PATH_FAULT_LEFT_MASK) && (gPathCondMap & PATH_FAULT_RIGHT_MASK) ){
            turnCntL = WHEEL_TURN_90_CNT;
            turnCntR = WHEEL_TURN_90_CNT;
        }

        /* turn direction judge */
        if(gPathCondMap & PATH_FAULT_LEFT_MASK){
            pActSequence->LWheelDefDir = 1;
            pActSequence->RWheelDefDir = 0;
            if(0==gLastPathFault){
                gLastPathFault = 1;
            }
        }
        else{
            pActSequence->LWheelDefDir = 0;
            pActSequence->RWheelDefDir = 1;
            if(0==gLastPathFault){
                gLastPathFault = -1;
            }
        }
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

    /* Stage 3 */
    pActSequence->LWheelDefDir = 1;
    pActSequence->RWheelDefDir = 1;
    pActSequence->LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    pActSequence->RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    pActSequence->LWheelExpCnt = 0x1;
    pActSequence->RWheelExpCnt = 0x2;

    if(gRobotMode == ROBOT_WORK_MODE_EDGE){
        pActSequence->LWheelExpSpeed = WHEEL_FAULT_PROC_SPEED;
        pActSequence->RWheelExpSpeed = WHEEL_FAULT_PROC_SPEED;
    }
    else if(gRobotMode == ROBOT_WORK_MODE_SPOT){
        if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_SPOT_L){
            gPathFaultProcMode = PATH_FAULT_PROC_MODE_SPOT_R;
            pActSequence->LWheelExpSpeed = RWHEEL_CUR_SPEED;
            pActSequence->RWheelExpSpeed = LWHEEL_CUR_SPEED;
        }
        else if(gPathFaultProcMode==PATH_FAULT_PROC_MODE_SPOT_R){
            gPathFaultProcMode = PATH_FAULT_PROC_MODE_SPOT_L;
            pActSequence->LWheelExpSpeed = RWHEEL_CUR_SPEED;
            pActSequence->RWheelExpSpeed = LWHEEL_CUR_SPEED;
        }
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

void ExceptionProcStopCondTestCB(void)
{
    if( ( (gExceptionMask & (1<<EXCEPTION_MASK_WHEEL_FLOAT_POS) ) && (gPathCondMap & PATH_FAULT_BOTTOM_MASK) )\
        ||     (gExceptionMask & (1<<EXCEPTION_MASK_FAN_OC_POS))\
        ||     (gExceptionMask & (1<<EXCEPTION_MASK_LBRUSH_OC_POS))\
        ||     (gExceptionMask & (1<<EXCEPTION_MASK_RBRUSH_OC_POS))\
        ||     (gExceptionMask & (1<<EXCEPTION_MASK_MBRUSH_OC_POS))\
        ){
#ifdef DEBUG_LOG
        printf("ExceptionStopCond=0x%X,0x%X\r\n", gExceptionMask, gPathCondMap);
#endif
        Buzzer_Play(BUZZER_TRI_PULS, BUZZER_SND_NORMAL);
        LWHEEL_EXP_CB_REG(NULL);
        RWHEEL_EXP_CB_REG(NULL);
        WHEEL_PROC_CB_REG(NULL);
        WHEEL_PROC_EXIT_CB_REG(NULL);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);
        MotionCtrl_Stop();
        /* Notify main process make the further stop */
        gMsg.expire = 0;
        gMsg.prio = MSG_PRIO_HIGHEST;
        gMsg.type = MSG_TYPE_MOTION;
        gMsg.MsgCB = NULL;
        gMsg.Data.MEvt = MOTION_EVT_IDLE_SYNC;
        SweepRobot_SendMsg(&gMsg);
    }
}

void ExceptionHandleFinishCB(void)
{
    WHEEL_PROC_CB_REG(NULL);
    
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);

    gIsExceptionHandling = 0;
}

s8 MotionCtrl_ExceptionStopErrCntr(u8 *ErrCnt, u8 StopCnt)
{
    (*ErrCnt)++;
    if((*ErrCnt) > StopCnt){
        return 1;
    }else
        return 0;
}

s8 MotionCtrl_ExceptionHandle(void)
{
    if(gExceptionMask){
        printf("gExp=0x%X\r\n",gExceptionMask);
    }
    if(gExceptionMask & (1<<EXCEPTION_MASK_LBRUSH_OC_POS)){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, 0);
        if(MotionCtrl_ExceptionStopErrCntr(&(gMotionExceptionErrCnt.LBrushOCErrCnt), 1)){
            printf("LBRUSH_OC\r\n");
            return -1;
        }
    }
    if(gExceptionMask & (1<<EXCEPTION_MASK_RBRUSH_OC_POS)){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, 0);
        if(MotionCtrl_ExceptionStopErrCntr(&(gMotionExceptionErrCnt.RBrushOCErrCnt), 1)){
            printf("RBRUSH_OC\r\n");
            return -1;
        }
    }
    if(gExceptionMask & (1<<EXCEPTION_MASK_MBRUSH_OC_POS)){
        MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, 0);
        if(MotionCtrl_ExceptionStopErrCntr(&(gMotionExceptionErrCnt.MBrushOCErrCnt), 1)){
            printf("MBRUSH_OC\r\n");
            return -1;
        }
    }

    if( gExceptionMask & ((1<<EXCEPTION_MASK_LWHEEL_STUCK_POS)|(1<<EXCEPTION_MASK_RWHEEL_STUCK_POS)) ){
        if(MotionCtrl_ExceptionStopErrCntr(&(gMotionExceptionErrCnt.WheelStuckErrCnt), 1)){
            printf("WHEEL_STUCK\r\n");
            return -1;
        }
    }

    if( gExceptionMask & (1<<EXCEPTION_MASK_ASHTRAY_INS_POS) ){
        gIsExceptionHandling = 0;
        printf("ASH_TRAY_NOT_INS\r\n");
        return -1;
    }

    if( gExceptionMask & (1<<EXCEPTION_MASK_FAN_OC_POS) ){
        if(MotionCtrl_ExceptionStopErrCntr(&(gMotionExceptionErrCnt.FanOCErrCnt), 1)){
            printf("FAN_OC\r\n");
            return -1;
        }else
            return 0;
    }

    /* FIXME: add wheel floating avoidence process */
    /* left bottom float detected and left wheel float detected */
    if( (gExceptionMask & (1<<EXCEPTION_MASK_WHEEL_FLOAT_POS)) && (!LWHEEL_FLOAT_SIGN) && (gPathCondMap & (PATH_FAULT_BOTTOM_MASK & PATH_FAULT_LEFT_MASK) ) ){
        gIsExceptionHandling = 0;
        printf("WHEEL_FLOAT_L\r\n");
        return -1;
    }
    /* right bottom float detected and right wheel float detected */
    if( (gExceptionMask & (1<<EXCEPTION_MASK_WHEEL_FLOAT_POS)) && (!RWHEEL_FLOAT_SIGN) && (gPathCondMap & (PATH_FAULT_BOTTOM_MASK & PATH_FAULT_RIGHT_MASK) ) ){
        gIsExceptionHandling = 0;
        printf("WHEEL_FLOAT_R\r\n");
        return -1;
    }

    if ( (gExceptionMask & (1<<EXCEPTION_MASK_WHEEL_FLOAT_POS)) && (!(gPathCondMap & PATH_FAULT_BOTTOM_MASK)) ){
        if(MotionCtrl_ExceptionStopErrCntr(&(gMotionExceptionErrCnt.WheelFloatErrCnt), 1)){
            printf("WHEEL_FLOAT_WITH_NO_BOTTOM_FLOAT_DETECT\r\n");
            return -1;
        }
    }

    MotionCtrl_ExceptionProc();

    return 0;
}

void MotionCtrl_ExceptionProc_Speed_Set(MCtrl_Act_t *sActSequence)
{
    if ( (gExceptionMask & (1<<EXCEPTION_MASK_WHEEL_FLOAT_POS)) && (!(gPathCondMap & PATH_FAULT_BOTTOM_MASK)) ){
        sActSequence->LWheelExpSpeed = WHEEL_ESCAPE_SPEED;
        sActSequence->RWheelExpSpeed = WHEEL_ESCAPE_SPEED;
    }
    else {
        sActSequence->LWheelExpSpeed = WHEEL_ESCAPE_SPEED;
        sActSequence->RWheelExpSpeed = WHEEL_ESCAPE_SPEED;
    }
}

void MotionCtrl_ExceptionProc(void)
{
    gExceptionMask &= 0;
    
    MotionCtrl_Start();
    gActSequence[0].LWheelDefDir = 0;
    gActSequence[0].RWheelDefDir = 0;
    gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED*2;
    gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED*2;
    gActSequence[0].LWheelExpCnt = WHEEL_FAULT_BACK_CNT*2;
    gActSequence[0].RWheelExpCnt = WHEEL_FAULT_BACK_CNT*2;
    MotionCtrl_ExceptionProc_Speed_Set(gActSequence);
    gActSequence[0].LWheelSync = 0;
    gActSequence[0].RWheelSync = 0;
    gActSequence[0].PreAct = NULL;
    gActSequence[0].PostAct = NULL;

    gActSequence[1].LWheelDefDir = 0;
    gActSequence[1].RWheelDefDir = 1;
    gActSequence[1].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED*2;
    gActSequence[1].RWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED*2;
    gActSequence[1].LWheelExpCnt = WHEEL_TURN_45_CNT;
    gActSequence[1].RWheelExpCnt = WHEEL_TURN_45_CNT;
    gActSequence->LWheelExpSpeed = WHEEL_ESCAPE_SPEED;
    gActSequence->RWheelExpSpeed = WHEEL_ESCAPE_SPEED;
    gActSequence[1].LWheelSync = 0;
    gActSequence[1].RWheelSync = 0;
    gActSequence[1].PreAct = NULL;
    gActSequence[1].PostAct = NULL;

    gActSequence[2].LWheelDefDir = 1;
    gActSequence[2].RWheelDefDir = 0;
    gActSequence[2].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED*2;
    gActSequence[2].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED*2;
    gActSequence[2].LWheelExpCnt = WHEEL_TURN_45_CNT;
    gActSequence[2].RWheelExpCnt = WHEEL_TURN_45_CNT;
    gActSequence->LWheelExpSpeed = WHEEL_ESCAPE_SPEED;
    gActSequence->RWheelExpSpeed = WHEEL_ESCAPE_SPEED;
    gActSequence[2].LWheelSync = 0;
    gActSequence[2].RWheelSync = 0;
    gActSequence[2].PreAct = NULL;
    gActSequence[2].PostAct = NULL;

    gActSequence[3].LWheelDefDir = 0;
    gActSequence[3].RWheelDefDir = 0;
    gActSequence[3].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED*2;
    gActSequence[3].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED*2;
    gActSequence[3].LWheelExpCnt = WHEEL_FAULT_BACK_CNT;
    gActSequence[3].RWheelExpCnt = WHEEL_FAULT_BACK_CNT;
    MotionCtrl_ExceptionProc_Speed_Set(gActSequence+3);
    gActSequence[3].LWheelSync = 0;
    gActSequence[3].RWheelSync = 0;
    gActSequence[3].PreAct = NULL;
    gActSequence[3].PostAct = NULL;

    gActSequence[4].LWheelDefDir = 1;
    gActSequence[4].RWheelDefDir = 0;
    gActSequence[4].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[4].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[4].LWheelExpCnt = WHEEL_TURN_180_CNT;
    gActSequence[4].RWheelExpCnt = WHEEL_TURN_180_CNT;
    gActSequence[4].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[4].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[4].LWheelSync = 0;
    gActSequence[4].RWheelSync = 0;
    gActSequence[4].PreAct = NULL;
    gActSequence[4].PostAct = NULL;

    gActSequence[5].LWheelDefDir = 1;
    gActSequence[5].RWheelDefDir = 1;
    gActSequence[5].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[5].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[5].LWheelExpCnt = 0x1;
    gActSequence[5].RWheelExpCnt = 0x2;
    gActSequence[5].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[5].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[5].LWheelSync = 0;
    gActSequence[5].RWheelSync = 0;
    gActSequence[5].PreAct = NULL;
    gActSequence[5].PostAct = NULL;
    gActSeqDepth = 6;

    MotionCtrl_Proc();

    WHEEL_PROC_CB_REG(ExceptionProcStopCondTestCB);
    WHEEL_PROC_EXIT_CB_REG(ExceptionHandleFinishCB);
}

void MotionCtrl_TrapProc(void)
{
    WheelCntMach_Stop();
    IFRD_PathDetectStop();

    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LWHEEL, 0);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RWHEEL, 0);

//    gActSeqDepLIndicator = 0;
//    gActSeqDepRIndicator = 0;

    IFRD_PathDetectStart();
    WheelCntMach_Start();

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
    gActSequence[1].LWheelExpCnt = WHEEL_TURN_90_CNT;
    gActSequence[1].RWheelExpCnt = WHEEL_TURN_90_CNT;
    gActSequence[1].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[1].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[1].LWheelSync = 0;
    gActSequence[1].RWheelSync = 0;
    gActSequence[1].PreAct = MotionCtrl_PathFaultTryTurnCondTest;
    gActSequence[1].PostAct = MotionCtrl_PathFaultTurnProcCompleteCondTest;
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

    MotionCtrl_Proc();
}

void MotionCtrl_ChargeStationAvoid(u8 dir, u8 turnCnt, u8 StopOnFinish)
{
    gActSequence[0].LWheelDefDir = dir ? 1 : 0;
    gActSequence[0].RWheelDefDir = dir ? 0 : 1;
    gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpCnt = turnCnt;
    gActSequence[0].RWheelExpCnt = turnCnt;
    gActSequence[0].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].LWheelSync = 0;
    gActSequence[0].RWheelSync = 0;
    gActSequence[0].PreAct = NULL;
    gActSequence[0].PostAct = NULL;
    gActSequence[1].LWheelDefDir = 1;
    gActSequence[1].RWheelDefDir = 1;
    gActSequence[1].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[1].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[1].LWheelExpCnt = 0x1;
    gActSequence[1].RWheelExpCnt = 0x2;
    gActSequence[1].LWheelExpSpeed = StopOnFinish ? 0 : WHEEL_CRUISE_SPEED;
    gActSequence[1].RWheelExpSpeed = StopOnFinish ? 0 : WHEEL_CRUISE_SPEED;
    gActSequence[1].LWheelSync = 0;
    gActSequence[1].RWheelSync = 0;
    gActSequence[1].PreAct = NULL;
    gActSequence[1].PostAct = NULL;
    gActSeqDepth = 2;

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

void MotionCtrl_HomingSigCaptureCondTest(void){
    gHomingSigCapturedFlag = 1;
}

void MotionCtrl_HomingSigCapturedProc(void)
{
    if(gHomingSigCapturedFlag){
#ifdef DEBUG_LOG
        printf("HomingSignalCaptured.\r\n");
#endif
        WHEEL_PROC_CB_REG(NULL);
        LWHEEL_EXP_CB_REG(NULL);
        RWHEEL_EXP_CB_REG(NULL);
        MotionCtrl_Stop();

        MotionCtrl_Start();

        gActSequence[0].LWheelDefDir = 1;
        gActSequence[0].RWheelDefDir = 1;
        gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
        gActSequence[0].LWheelExpCnt = 0x1;
        gActSequence[0].RWheelExpCnt = 0x2;
        gActSequence[0].LWheelExpSpeed = WHEEL_HOMING_SPEED;
        gActSequence[0].RWheelExpSpeed = WHEEL_HOMING_SPEED;
        gActSequence[0].LWheelSync = 0;
        gActSequence[0].RWheelSync = 0;
        gActSequence[0].PreAct = NULL;
        gActSequence[0].PostAct = NULL;
        gActSeqDepth = 1;

        MotionCtrl_Proc();
    }
}

void MotionCtrl_HomingMotionInit(void)
{
    gRobotState = ROBOT_STATE_RUNNING;
    gRobotMode = ROBOT_WORK_MODE_HOMING;

    /* DO NOT FORGET THIS */
    SweepRobot_StartupComplete();

    MotionCtrl_Start();

    gHomingSigCapturedFlag = 0;

    gActSequence[0].LWheelDefDir = 0;
    gActSequence[0].RWheelDefDir = 1;
    gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpCnt = WHEEL_TURN_360_CNT;
    gActSequence[0].RWheelExpCnt = WHEEL_TURN_360_CNT;
    gActSequence[0].LWheelExpSpeed = WHEEL_HOMING_SPEED-1;
    gActSequence[0].RWheelExpSpeed = WHEEL_HOMING_SPEED-1;
    gActSequence[0].LWheelSync = 0;
    gActSequence[0].RWheelSync = 0;
    gActSequence[0].PreAct = NULL;
    gActSequence[0].PostAct = NULL;
    gActSequence[1].LWheelDefDir = 1;
    gActSequence[1].RWheelDefDir = 1;
    gActSequence[1].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[1].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[1].LWheelExpCnt = 0x1;
    gActSequence[1].RWheelExpCnt = 0x2;
    gActSequence[1].LWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[1].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[1].LWheelSync = 0;
    gActSequence[1].RWheelSync = 0;
    gActSequence[1].PreAct = NULL;
    gActSequence[1].PostAct = NULL;
    gActSeqDepth = 2;

    MotionCtrl_Proc();

    WHEEL_PROC_CB_REG(MotionCtrl_HomingSigCapturedProc);
}

void MotionCtrl_DishomingMotionInit(void)
{
    gRobotState = ROBOT_STATE_RUNNING;
    gRobotMode = ROBOT_WORK_MODE_DISHOMING;

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
    gActSequence[1].LWheelDefDir = 1;
    gActSequence[1].RWheelDefDir = 0;
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
    gActSequence[2].LWheelExpSpeed = 0;
    gActSequence[2].RWheelExpSpeed = 0;
    gActSequence[2].LWheelSync = 0;
    gActSequence[2].RWheelSync = 0;
    gActSequence[2].PreAct = NULL;
    gActSequence[2].PostAct = NULL;
    gActSeqDepth = 3;

    MotionCtrl_Proc();
}


u8 MotionCtrl_RtrnHomingWheelForwardProc(void)
{
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_MBRUSH, MOTOR_MBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_LBRUSH, MOTOR_LBRUSH_CHAN_STARTUP_SPEED);
    MotorCtrl_ChanSpeedLevelSet(MOTOR_CTRL_CHAN_RBRUSH, MOTOR_RBRUSH_CHAN_STARTUP_SPEED);
    
    return 1;
}

void MotionCtrl_RtrnHomingMotionInit(void)
{
    gRobotState = ROBOT_STATE_RUNNING;
    gRobotMode = ROBOT_WORK_MODE_HOMING;

    MotionCtrl_Start();

    gActSequence[0].LWheelDefDir = 0;
    gActSequence[0].RWheelDefDir = 0;
    gActSequence[0].LWheelInitSpeed = 10;
    gActSequence[0].RWheelInitSpeed = 10;
    gActSequence[0].LWheelExpCnt = WHEEL_1QUARTER_CYCLE_CNT;
    gActSequence[0].RWheelExpCnt = WHEEL_1QUARTER_CYCLE_CNT;
    gActSequence[0].LWheelExpSpeed = WHEEL_HOMING_SPEED;
    gActSequence[0].RWheelExpSpeed = WHEEL_HOMING_SPEED;
    gActSequence[0].LWheelSync = 0;
    gActSequence[0].RWheelSync = 0;
    gActSequence[0].PreAct = NULL;
    gActSequence[0].PostAct = MotionCtrl_RtrnHomingWheelForwardProc;
    gActSequence[1].LWheelDefDir = 1;
    gActSequence[1].RWheelDefDir = 1;
    gActSequence[1].LWheelInitSpeed = 10;
    gActSequence[1].RWheelInitSpeed = 10;
    gActSequence[1].LWheelExpCnt = 0x1;
    gActSequence[1].RWheelExpCnt = 0x2;
    gActSequence[1].LWheelExpSpeed = WHEEL_HOMING_SPEED-1;
    gActSequence[1].RWheelExpSpeed = WHEEL_HOMING_SPEED-1;
    gActSequence[1].LWheelSync = 0;
    gActSequence[1].RWheelSync = 0;
    gActSequence[1].PreAct = NULL;
    gActSequence[1].PostAct = NULL;
    gActSeqDepth = 2;

    MotionCtrl_Proc();
}

void MotionCtrl_AutoMotionInit(void)
{
    gRobotState = ROBOT_STATE_RUNNING;
    gRobotMode = ROBOT_WORK_MODE_AUTO;
    /* Don't forget this */
    SweepRobot_StartupComplete();

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

    MotionCtrl_Proc();
}


void MotionCtrl_SpotMotionInit(void)
{
    gRobotState = ROBOT_STATE_RUNNING;
    gRobotMode = ROBOT_WORK_MODE_SPOT;
    /* Don't forget this */
    SweepRobot_StartupComplete();

    gPathFaultProcMode = PATH_FAULT_PROC_MODE_SPOT_L;
    gSpotModeAngleCnt = 0;
    gSpotModeCircleCnt = SPOT_MODE_INIT_CIRCLE_CNT;
    MotionCtrl_Start();

    gActSequence[0].LWheelDefDir = 1;
    gActSequence[0].RWheelDefDir = 1;
    gActSequence[0].LWheelInitSpeed = MOTOR_LWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].RWheelInitSpeed = MOTOR_RWHEEL_CHAN_STARTUP_SPEED;
    gActSequence[0].LWheelExpCnt = 0x1;
    gActSequence[0].RWheelExpCnt = 0x2;
    gActSequence[0].LWheelExpSpeed = 0;
    gActSequence[0].RWheelExpSpeed = WHEEL_CRUISE_SPEED;
    gActSequence[0].LWheelSync = 0;
    gActSequence[0].RWheelSync = 0;
    gActSequence[0].PreAct = NULL;
    gActSequence[0].PostAct = NULL;
    gActSeqDepth = 1;

    MotionCtrl_Proc();
}

void MotionCtrl_EdgeMotionInit(void)
{
    gRobotState = ROBOT_STATE_RUNNING;
    gRobotMode = ROBOT_WORK_MODE_EDGE;
    /* Don't forget this */
    SweepRobot_StartupComplete();

    gPathFaultProcMode = PATH_FAULT_PROC_MODE_EDGE;
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

    MotionCtrl_Proc();
}

void MotionCtrl_ManualCtrlProc(enum MotionCtrlManualAct act)
{
    gRobotState = ROBOT_STATE_RUNNING;
    gRobotMode = ROBOT_WORK_MODE_MANUAL;
    /* Don't forget this */
    SweepRobot_StartupComplete();

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
    gActSequence[1].PreAct = NULL;
    gActSequence[1].PostAct = NULL;
    MotionCtrl_Proc();
}
