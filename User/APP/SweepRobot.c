/******************** (C) COPYRIGHT 2007 EJE ********************
* File Name          : SweepRobot.c
* Author             : Reason Chen
* Version            : V1.0
* Date               : 5-May-2015
* Description        : Sweeping Robot demo
*******************************************************************************/
#include <stdlib.h>
#include "math.h"
#include "string.h"
#include "boardcfg.h"
#include "stm32f10x_conf.h"
#include "SweepRobot.h"
#include "MotorCtrl.h"
#include "delay.h"
#include "BatteryManage.h"
#include "PWM.h"
#include "PwrManagement.h"
#include "MotionCtrl.h"
#include "IrDA.h"
#include "Measurement.h"
#include "CtrlPanel.h"
#include "RTC.h"
#include "Buzzer.h"

enum RobotState     gRobotState;
enum RobotWorkMode  gRobotMode;

#define ROBOT_MAIN_MSG_Q_SIZE               10

struct RobotHomingState_s {

    u8      Pos;
    s16     Angle;
};

static MsgQueue_t *MainMsgQ = NULL;
static const s16 HomingSigSrc2AngleMap[] = { 180, 90, 45, 315, 270 };
//static struct RobotHomingState_s LastHomingState;
enum _RobotHomingStage gHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
static enum _RobotHomingStage LastHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;

void SweepRobot_PMMsgProc(enum PM_Mode mode);
void SweepRobot_BMMsgProc(enum BatteryEvt evt);
void SweepRobot_CtrlMsgProc(u8 CtrlCode);
void SweepRobot_PwrStationMsgProc(PwrStationSigData_t *PwrSig);
void SweepRobot_MotionMsgProc(enum MotionEvt evt);

s8 SweepRobot_Init(void)
{
    s8      err = 0;

    /* Power management init */
    PM_Init();
    /* Real time clock init */
    RTC_Init();
    /* Message Queue for processing robot state */
    MainMsgQ = MsgQueue_Create(ROBOT_MAIN_MSG_Q_SIZE);
    if(MainMsgQ == NULL){
        err = -1;
        goto SWEEPROBOT_INIT_FAIL;
    }
    /* Physical state measurement */
    Meas_Init();
    /* Start measurement */
    Meas_Start();
    /* PWM resource init */
    PWM_ControllerInit();
    /* PWM controller start */
    PWM_ControllerStart();
    /* Battery management init */
    BM_Init();
    /* Buzzer init */
    Buzzer_Init();
    /* Motor controller init */
    err = MotorCtrl_Init();
    if(err)
        goto SWEEPROBOT_INIT_FAIL;
    /* Robot motion control */
    MotionCtrl_Init();
    /* IrDA Rx init */
    IrDA_Init();
    /* CtrlPanel config */
    CtrlPanel_Init();

    gRobotState = ROBOT_STATE_IDLE;
#ifdef DEBUG_LOG
    printf("Robot init OK !\r\n");
#endif
    return err;
SWEEPROBOT_INIT_FAIL:
#ifdef DEBUG_LOG
    printf("Robot init failed !\r\n");
#endif
    return err;
}

void SweepRobot_Start(void)
{
    u8  i = 0;

    while(1){
        /* Message loop */
        for(i = 0; i < ROBOT_MAIN_MSG_Q_SIZE; i++){
            if(!MainMsgQ->Msg.expire){
                if(MainMsgQ->Msg.prio == MSG_PRIO_HIGHEST){
                    switch(MainMsgQ->Msg.type){
                        case MSG_TYPE_PM:
#ifdef DEBUG_LOG
                            printf("PM msg %d.\r\n", MainMsgQ->Msg.Data.PMEvt);
#endif
                            SweepRobot_PMMsgProc(MainMsgQ->Msg.Data.PMEvt);
                            break;
                        case MSG_TYPE_BM:
                            PM_ResetSysIdleState();
#ifdef DEBUG_LOG
                            printf("BM msg %d.\r\n", MainMsgQ->Msg.Data.BatEvt);
#endif
                            SweepRobot_BMMsgProc(MainMsgQ->Msg.Data.BatEvt);
                            break;
                        case MSG_TYPE_CTRL:
                            PM_ResetSysIdleState();
#ifdef DEBUG_LOG
                            printf("CTRL msg code: 0x%X.\r\n", MainMsgQ->Msg.Data.ByteVal);
#endif
                            SweepRobot_CtrlMsgProc(MainMsgQ->Msg.Data.ByteVal);
                            break;
                        case MSG_TYPE_PWR_STATION:
#ifdef DEBUG_LOG
//                            printf("PWR_STATION msg Pos: %d, code: 0x%X.\r\n", MainMsgQ->Msg->Data.PSSigDat.src, MainMsgQ->Msg->Data.PSSigDat.sig);
#endif
                            SweepRobot_PwrStationMsgProc(&MainMsgQ->Msg.Data.PSSigDat);
                            break;
                        case MSG_TYPE_MOTION:
                            PM_ResetSysIdleState();
#ifdef DEBUG_LOG
                            printf("MOTION msg 0x%X.\r\n", MainMsgQ->Msg.Data.MEvt);
#endif
                            SweepRobot_MotionMsgProc(MainMsgQ->Msg.Data.MEvt);
                            break;
                    }
                    if(MainMsgQ->Msg.MsgCB!=NULL){
                        MainMsgQ->Msg.MsgCB();
                    }
                    MainMsgQ->Msg.expire = 1;
                }
                else{
                    MainMsgQ->Msg.prio--;
                }
            }
            MainMsgQ = MainMsgQ->Next;
        }
    }
}

void SweepRobot_AutoModeProc(void)
{
    if(gRobotState != ROBOT_STATE_RUNNING){
        MotionCtrl_AutoMotionInit();
        gRobotMode = ROBOT_WORK_MODE_AUTO;
    }
    else{
        MotionCtrl_Stop();
    }
}

void SweepRobot_SpotModeProc(void)
{
    if(gRobotMode != ROBOT_WORK_MODE_SPOT){
        MotionCtrl_SpotMotionInit();
        gRobotState = ROBOT_STATE_RUNNING;
    }
}

void SweepRobot_ManualModeProc(enum MotionCtrlManualAct act)
{
    if(gRobotState == ROBOT_STATE_HOME){
        return;
    }

    MotionCtrl_ManualCtrlProc(act);
    gRobotMode = ROBOT_WORK_MODE_MANUAL;
}

void SweepRobot_HomingInit(void)
{
    if(gRobotState == ROBOT_STATE_HOME){
#ifdef DEBUG_LOG
        printf("Already in charging.\r\n");
#endif
        return;
    }

    MotionCtrl_RoundedSlowly();
    gRobotMode = ROBOT_WORK_MODE_HOMING;
    /* Turn around for search power station signal */
    gHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
    LastHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
}

void SweepRobot_HomingSuccess(void)
{
    MotionCtrl_Stop();

    gHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
    LastHomingStage = ROBOT_HOMING_STAGE_UNKNOWN;
}

void SweepRobot_PMMsgProc(enum PM_Mode mode)
{
    if(gRobotState==ROBOT_STATE_IDLE){
        PM_EnterPwrMode(mode);
    }
    else{
        PM_ResetSysIdleState();
    }
}

void SweepRobot_BMMsgProc(enum BatteryEvt evt)
{
    switch(evt){
        case BM_EVT_POWER_LOSS:
#ifdef DEBUG_LOG
            printf("Robot disconnect from power station.\r\n");
#endif
            if(gRobotState == ROBOT_STATE_HOME){
                gRobotState = ROBOT_STATE_IDLE;
                CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, 1);
            }
            break;
        case BM_EVT_POWER_LINK:
#ifdef DEBUG_LOG
            printf("Robot connect to power station.\r\n");
#endif
            if(gRobotState == ROBOT_STATE_RUNNING && gRobotMode == ROBOT_WORK_MODE_HOMING){
                SweepRobot_HomingSuccess();
            }
            gRobotState = ROBOT_STATE_HOME;
            gRobotMode  = ROBOT_WORK_MODE_HOMING;
            break;
        case BM_EVT_LOW_LEVEL:
            /* Low battery condition, try to home and get charged */
#ifdef DEBUG_LOG
            printf("Robot low battery condition.\r\n");
#endif
            if(gRobotMode != ROBOT_WORK_MODE_HOMING){
                CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, 1);
                SweepRobot_HomingInit();
            }
            break;
        case BM_EVT_CHARGE_COMPLETE:
            CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, 1);
#ifdef DEBUG_LOG
            printf("Robot finish charging.\r\n");
#endif
            break;
    }
}

void SweepRobot_CtrlMsgProc(u8 CtrlCode)
{
    switch(CtrlCode){
        case REMOTE_CMD_RUN_STOP:
            SweepRobot_AutoModeProc();
            break;
        case REMOTE_CMD_SPOT:
            SweepRobot_SpotModeProc();
            break;
        case REMOTE_CMD_MODE_1:
            /* work mode switch */
            break;
        case REMOTE_CMD_MODE_2:
            /* work mode switch */
            break;
        case REMOTE_CMD_MODE_3:
            /* work mode switch */
            break;
        case REMOTE_CMD_MODE_4:
            /* work mode switch */
            break;
        case REMOTE_CMD_UP:
            SweepRobot_ManualModeProc(MANUAL_ACT_UP);
            break;
        case REMOTE_CMD_DOWN:
            SweepRobot_ManualModeProc(MANUAL_ACT_DOWN);
            break;
        case REMOTE_CMD_LEFT:
            SweepRobot_ManualModeProc(MANUAL_ACT_LEFT);
            break;
        case REMOTE_CMD_RIGHT:
            SweepRobot_ManualModeProc(MANUAL_ACT_RIGHT);
            break;
        case REMOTE_CMD_CHARGE:
            SweepRobot_HomingInit();
            break;
    }
}

void SweepRobot_MotionMsgProc(enum MotionEvt evt)
{
    switch(evt){
        case MOTION_EVT_PATH_FAULT:
            if(gRobotState == ROBOT_STATE_RUNNING){
                if(gRobotMode == ROBOT_WORK_MODE_MANUAL){
                    MotionCtrl_PathFaultProc(1);
                }
                else{
                    MotionCtrl_PathFaultProc(0);
                }
            }
            break;
        case MOTION_EVT_EXCEPTION:
#ifdef DEBUG_LOG
            printf("Exception state.\r\n");
#endif
            MotionCtrl_Stop();
            break;
        case MOTION_EVT_TRAPPED:
            MotionCtrl_TrapProc();
            break;
    }
}

#define IS_PWR_STATION_BACKOFF_SIG(sig)     ((sig==PWR_STATION_BACKOFF_SIG_L)||(sig==PWR_STATION_BACKOFF_SIG_R))
#define IS_PWR_STATION_SIG_LONG(sig)        ((sig==PWR_STATION_HOME_SIG_LL)||(sig==PWR_STATION_HOME_SIG_RL))
#define IS_PWR_STATION_SIG_CENTER(sig)      (sig==PWR_STATION_HOME_SIG_CENTER)
#if 0
#define IS_PWR_STATION_SIG_SHORT(sig)       ((sig==PWR_STATION_HOME_SIG_LS)||(sig==PWR_STATION_HOME_SIG_RS))
#define IS_PWR_STATION_SIG_LEFT(sig)        ((sig==PWR_STATION_HOME_SIG_LL)||(sig==PWR_STATION_HOME_SIG_LS))
#define IS_PWR_STATION_SIG_RIGHT(sig)       ((sig==PWR_STATION_HOME_SIG_RL)||(sig==PWR_STATION_HOME_SIG_RS))
#else
#define IS_PWR_STATION_SIG_SHORT(sig)       ((sig==PWR_STATION_BACKOFF_SIG_R)||(sig==PWR_STATION_BACKOFF_SIG_L))
#define IS_PWR_STATION_SIG_LEFT(sig)        ((sig==PWR_STATION_HOME_SIG_LL)||(sig==PWR_STATION_BACKOFF_SIG_L))
#define IS_PWR_STATION_SIG_RIGHT(sig)       ((sig==PWR_STATION_HOME_SIG_RL)||(sig==PWR_STATION_BACKOFF_SIG_R))
#endif

void SweepRobot_PwrStationMsgProc(PwrStationSigData_t *PwrSig)
{
    static PwrStationSigData_t RobotHomingData[10];
    static u8 i, j, k, l, HomingDataCnt = 0, RepeatCodeFound = 0, HomingStateConfirmCnt = 0;
    struct RobotHomingState_s HomingState, LastHomingState = {0};

    if(gRobotState != ROBOT_STATE_RUNNING){
        return;
    }

    if(gRobotMode == ROBOT_WORK_MODE_HOMING){
#if 0
        /* Ignore backoff signal */
        if(IS_PWR_STATION_BACKOFF_SIG(PwrSig->sig)){
            return;
        }
#endif
        for(i = 0; i < HomingDataCnt; i++){
            if(RobotHomingData[i].sig == PwrSig->sig && RobotHomingData[i].src == PwrSig->src){
                break;
            }
        }

        if(i < HomingDataCnt){
            /* all received signal in one loop */
            l = HomingDataCnt-i;
            if(l >= 2){
                /* find repeative code */
                for(j = 0; j < l; j++){
                    if(RepeatCodeFound)
                        break;
                    for(k = j+1; k < l; k++){
                        if(RobotHomingData[j].sig == RobotHomingData[k].sig){
                            HomingState.Pos = RobotHomingData[j].sig;
                            HomingState.Angle = (HomingSigSrc2AngleMap[RobotHomingData[j].src] + HomingSigSrc2AngleMap[RobotHomingData[k].src]);
                            HomingState.Angle = (HomingState.Angle == 360) ? 0 : (HomingState.Angle / 2);
                            RepeatCodeFound = 1;
                            break;
                        }
                    }
                }
                /* priority */
                if(RepeatCodeFound){
                    /* Aleady get foucs */
                    if(IS_PWR_STATION_SIG_LONG(HomingState.Pos)){
                        /* Last postion is stage2(short distance area) */
                        if(LastHomingStage==ROBOT_HOMING_STAGE_UNKNOWN||LastHomingStage==ROBOT_HOMING_STAGE2){
                            for(i = 0; i < l; i++){
                                if(IS_PWR_STATION_SIG_SHORT(RobotHomingData[i].sig)){
                                    /* found short signal */
                                    break;
                                }
                            }
                            if(i>=l){
                                for(i = 0; i < l; i++){
                                    if(IS_PWR_STATION_SIG_CENTER(RobotHomingData[i].sig)){
                                        /* found center signal */
                                        break;
                                    }
                                }
                            }
                            if(i<l){
                                HomingState.Pos = RobotHomingData[i].sig;
                                HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[i].src];
                            }
                        }
                        /* come from stage1 or stage3, stage ok */
                        else {
                            for(i = 0; i < l; i++){
                                if(IS_PWR_STATION_SIG_CENTER(RobotHomingData[i].sig)){
                                    /* found center signal */
                                    break;
                                }
                            }
                            if(i>=l){
                                for(i = 0; i < l; i++){
                                    if(IS_PWR_STATION_SIG_SHORT(RobotHomingData[i].sig)){
                                        /* found short signal */
                                        break;
                                    }
                                }
                            }
                            if(i<l){
                                HomingState.Pos = RobotHomingData[i].sig;
                                HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[i].src];
                            }
                        }
                    }
                }
                else{
                    if(LastHomingStage==ROBOT_HOMING_STAGE_UNKNOWN||LastHomingStage==ROBOT_HOMING_STAGE2){
                        for(i = 0; i < l; i++){
                            if(IS_PWR_STATION_SIG_SHORT(RobotHomingData[i].sig)){
                                /* found short signal */
                                break;
                            }
                        }
                        if(i>=l){
                            for(i = 0; i < l; i++){
                                if(IS_PWR_STATION_SIG_CENTER(RobotHomingData[i].sig)){
                                    /* found center signal */
                                    break;
                                }
                            }
                        }
                        if(i<l){
                            HomingState.Pos = RobotHomingData[i].sig;
                            HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[i].src];
                        }
                        else{
                            HomingState.Pos = RobotHomingData[0].sig;
                            HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[0].src];
                        }
                    }
                    /* come from stage1 or stage3, stage ok */
                    else {
                        for(i = 0; i < l; i++){
                            if(IS_PWR_STATION_SIG_CENTER(RobotHomingData[i].sig)){
                                /* found center signal */
                                break;
                            }
                        }
                        if(i>=l){
                            for(i = 0; i < l; i++){
                                if(IS_PWR_STATION_SIG_SHORT(RobotHomingData[i].sig)){
                                    /* found short signal */
                                    break;
                                }
                            }
                        }
                        if(i<l){
                            HomingState.Pos = RobotHomingData[i].sig;
                            HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[i].src];
                        }
                        else{
                            HomingState.Pos = RobotHomingData[0].sig;
                            HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[0].src];
                        }
                    }
                }
            }
            else {
                HomingState.Pos = RobotHomingData[HomingDataCnt-1].sig;
                HomingState.Angle = HomingSigSrc2AngleMap[RobotHomingData[HomingDataCnt-1].src];
            }

            HomingState.Angle = (HomingState.Angle > 180) ? (HomingState.Angle - 360) : HomingState.Angle;
#ifdef DEBUG_LOG
            printf("Pos: 0x%X, Ang: %d\r\n", HomingState.Pos, HomingState.Angle);
#endif
            HomingDataCnt = 0;
            RepeatCodeFound = 0;
        }

        if(HomingDataCnt >= 5){
            HomingDataCnt = 0;
        }

        RobotHomingData[HomingDataCnt].sig = PwrSig->sig;
        RobotHomingData[HomingDataCnt].src = PwrSig->src;
        HomingDataCnt++;

        if(LastHomingState.Pos!=HomingState.Pos){
            HomingStateConfirmCnt++;
            if(HomingStateConfirmCnt>5){
                LastHomingState.Pos = HomingState.Pos;
                LastHomingState.Angle = HomingState.Angle;
            }
            else{
                HomingState.Pos = LastHomingState.Pos;
                HomingState.Angle = LastHomingState.Angle;
            }
        }

        /* plan path to power station */
        if(IS_PWR_STATION_SIG_LONG(HomingState.Pos)){
            gHomingStage = ROBOT_HOMING_STAGE1;
            if(IS_PWR_STATION_SIG_LEFT(HomingState.Pos)){
                if(HomingState.Angle <= -90){
                    MotionCtrl_MoveDirTune(0, WHEEL_HOMING_SPEED);
                }
                else if(HomingState.Angle < 90){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 0);
                }
                else if(HomingState.Angle == 90){
                    MotionCtrl_MoveDirTune(4, WHEEL_HOMING_SPEED);
                    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, 1);
                }
                else{
                    MotionCtrl_MoveDirTune(1, WHEEL_HOMING_SPEED);
                }
            }
            else {
                if(HomingState.Angle >= 90){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 0);
                }
                else if(HomingState.Angle > -90){
                    MotionCtrl_MoveDirTune(0, WHEEL_HOMING_SPEED);
                }
                else if(HomingState.Angle == -90){
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 4);
                    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_BLUE, 1);
                }
                else{
                    MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 1);
                }
            }
        }
        else if(IS_PWR_STATION_SIG_SHORT(HomingState.Pos)){
            gHomingStage = ROBOT_HOMING_STAGE2;
            if(IS_PWR_STATION_SIG_LEFT(HomingState.Pos)){
                if(HomingState.Angle <= 0 && HomingState.Angle >= -135){
                    MotionCtrl_MoveDirTune(0, WHEEL_CRUISE_SPEED);
                }
                else if(HomingState.Angle > 0 && HomingState.Angle < 180){
                    MotionCtrl_MoveDirTune(WHEEL_CRUISE_SPEED, 0);
                }
                else {
                    MotionCtrl_MoveDirTune(WHEEL_CRUISE_SPEED, WHEEL_CRUISE_SPEED);
                    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, 1);
                }
            }
            else{
                if(HomingState.Angle <= 0 && HomingState.Angle >= -135){
                    MotionCtrl_MoveDirTune(0, WHEEL_CRUISE_SPEED);
                }
                else if(HomingState.Angle > 0 && HomingState.Angle < 180){
                    MotionCtrl_MoveDirTune(WHEEL_CRUISE_SPEED, 0);
                }
                else {
                    MotionCtrl_MoveDirTune(WHEEL_CRUISE_SPEED, WHEEL_CRUISE_SPEED);
                    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, 1);
                }
            }
        }
        /* Center area */
        else if(PWR_STATION_HOME_SIG_CENTER==HomingState.Pos){
            gHomingStage = ROBOT_HOMING_STAGE3;
            if(HomingState.Angle > 45){
                MotionCtrl_MoveDirTune(0, WHEEL_HOMING_SPEED);
            }
            else if(HomingState.Angle > 0){
                MotionCtrl_MoveDirTune(2, WHEEL_HOMING_SPEED-1);
            }
            else if(HomingState.Angle < -45){
                MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED, 0);
            }
            else if(HomingState.Angle < 0){
                MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED-1, 2);
            }
            else{
                gHomingStage = ROBOT_HOMING_STAGE_OK;
                MotionCtrl_MoveDirTune(WHEEL_HOMING_SPEED-2, WHEEL_HOMING_SPEED-2);
                CtrlPanel_LEDCtrl(CTRL_PANEL_LED_RED, 1);
            }
        }
        LastHomingStage = gHomingStage;
    }
    else {
        if( IS_MOTION_PROC_FINISH() && (PwrSig->sig == (u8)PWR_STATION_BACKOFF_SIG_L || PwrSig->sig == (u8)PWR_STATION_BACKOFF_SIG_R) ){
            if( (PwrSig->src==IRDA_RECV_POS_L || PwrSig->src==IRDA_RECV_POS_FL) ){
                if(gRobotMode == ROBOT_WORK_MODE_MANUAL){
                    MotionCtrl_ChargeStationAvoid(1, WHEEL_TURN_30_CNT, 1);
                }
                else{
                    MotionCtrl_ChargeStationAvoid(1, WHEEL_TURN_30_CNT, 0);
                }
            }
            else if( (PwrSig->src==IRDA_RECV_POS_R || PwrSig->src==IRDA_RECV_POS_FR) ){
                if(gRobotMode == ROBOT_WORK_MODE_MANUAL){
                    MotionCtrl_ChargeStationAvoid(0, WHEEL_TURN_30_CNT, 1);
                }
                else{
                    MotionCtrl_ChargeStationAvoid(0, WHEEL_TURN_30_CNT, 0);
                }
            }
        }
    }
}

s8 SweepRobot_SendMsg(Msg_t *Msg)
{
    if(MsgQueue_InQueue(MainMsgQ, ROBOT_MAIN_MSG_Q_SIZE, Msg)){
#ifdef DEBUG_LOG
        printf("Msg inqueue failed !\r\n");
#endif
        return -1;
    }
    return 0;
}