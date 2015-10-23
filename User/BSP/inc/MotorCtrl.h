/**
  ******************************************************************************
  * @file    MotorCtrl.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Brushless DC motor control drvier
  ******************************************************************************
  */

#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__


#include "stm32f10x_conf.h"

#define MOTOR_FAN_CHAN_MAX_SPEED                90
#define MOTOR_BRUSH_CHAN_MAX_SPEED              90
#define MOTOR_WHEEL_CHAN_MAX_SPEED              80

/* original motor speed definition */
/*
#define MOTOR_FAN_CHAN_STARTUP_SPEED            70
#define MOTOR_LWHEEL_CHAN_STARTUP_SPEED         30
#define MOTOR_RWHEEL_CHAN_STARTUP_SPEED         30
#define MOTOR_LBRUSH_CHAN_STARTUP_SPEED         80
#define MOTOR_RBRUSH_CHAN_STARTUP_SPEED         80
#define MOTOR_MBRUSH_CHAN_STARTUP_SPEED         80
*/

#define MOTOR_FAN_CHAN_STARTUP_SPEED            50
#define MOTOR_LWHEEL_CHAN_STARTUP_SPEED         30
#define MOTOR_RWHEEL_CHAN_STARTUP_SPEED         30
#define MOTOR_LBRUSH_CHAN_STARTUP_SPEED         75
#define MOTOR_RBRUSH_CHAN_STARTUP_SPEED         75
#define MOTOR_MBRUSH_CHAN_STARTUP_SPEED         80

enum MotorCtrlChannel {

    MOTOR_CTRL_CHAN_FAN,
    MOTOR_CTRL_CHAN_RWHEEL,
    MOTOR_CTRL_CHAN_LWHEEL,
    MOTOR_CTRL_CHAN_LBRUSH,
    MOTOR_CTRL_CHAN_RBRUSH,
    MOTOR_CTRL_CHAN_MBRUSH,
};

enum MotorAction {

    MOTOR_ACTION_UP,
    MOTOR_ACTION_DOWN,
    MOTOR_ACTION_BRAKE
};

enum MotorStatus {

    MOTOR_STATUS_UNKNOWN,
    MOTOR_STATUS_STOP,
    MOTOR_STATUS_UP,
    MOTOR_STATUS_DOWN
};

s8 MotorCtrl_Init(void);
s8 MotorCtrl_ChanSpeedLevelSet(enum MotorCtrlChannel Channel, u16 level);
u16 MotorCtrl_ChanSpeedLevelGet(enum MotorCtrlChannel Channel);
s8 MotorCtrl_ChanDirSet(enum MotorCtrlChannel Channel, u8 Dir);
u8 MotorCtrl_ChanDirGet(enum MotorCtrlChannel Channel);
enum MotorStatus MotorCtrl_ChanStateGet(enum MotorCtrlChannel Channel);
s8 MotorCtrl_ChanSpeedDec(enum MotorCtrlChannel Channel);
s8 MotorCtrl_ChanSpeedInc(enum MotorCtrlChannel Channel);

#endif /* __MOTOR_CTRL_H__ */
