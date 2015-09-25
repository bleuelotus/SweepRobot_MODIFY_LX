/******************** (C) COPYRIGHT 2007 EJE ********************
* File Name          : SweepRobot.c
* Author             : Reason Chen
* Version            : V1.0
* Date               : 5-May-2015
* Description        : Sweeping Robot demo
*******************************************************************************/

#ifndef __SWEEP_ROBOT_H__
#define __SWEEP_ROBOT_H__

#include "stm32f10x_conf.h"
#include "MsgQueue.h"

extern enum RobotState     gRobotState;
extern enum RobotWorkMode  gRobotMode, gRobotModeLast;
extern enum _RobotHomingStage gHomingStage;

enum RobotState {

    ROBOT_STATE_STARTUP,
    ROBOT_STATE_IDLE,
    ROBOT_STATE_RUNNING,
    ROBOT_STATE_HOME,
};

enum RobotWorkMode{

    ROBOT_WORK_MODE_AUTO,
    ROBOT_WORK_MODE_MANUAL,
    ROBOT_WORK_MODE_SPOT,
    ROBOT_WORK_MODE_EDGE,
    ROBOT_WORK_MODE_HOMING,
    ROBOT_WORK_MODE_DISHOMING,
};

enum _RobotHomingStage {

    ROBOT_HOMING_STAGE_UNKNOWN,
    ROBOT_HOMING_STAGE1,
    ROBOT_HOMING_STAGE2,
    ROBOT_HOMING_STAGE3,
    ROBOT_HOMING_STAGE_OK
};

s8 SweepRobot_Init(void);
void SweepRobot_Start(void);
void SweepRobot_StartupComplete(void);
void SweepRobot_Stop(void);
s8 SweepRobot_SendMsg(Msg_t *Msg);


#endif /* !__SWEEP_ROBOT_H__ */

