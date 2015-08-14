/**
  ******************************************************************************
  * @file    PWM.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   PWM resource manage drvier
  ******************************************************************************
  */

#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f10x_conf.h"

enum PWMChannelRes {

    PWM_CHAN_RWHEEL,
    PWM_CHAN_RBRUSH,
    PWM_CHAN_CHARGE,
    PWM_CHAN_MBRUSH,
    PWM_CHAN_LBRUSH,
    PWM_CHAN_LWHEEL,
    PWM_CHAN_FAN,
    PWM_CHAN_NULL,
};

s8 PWM_ControllerInit(void);
s8 PWM_ChanInit(enum PWMChannelRes ChanResIdx, u8 Polarity, u16 DutyCycle);
s8 PWM_DutyCycleSet(enum PWMChannelRes ChanResIdx, u16 DutyCycle);
s8 PWM_DutyCycleGet(enum PWMChannelRes ChanResIdx, u16 *DutyCycle);
void PWM_ControllerStart(void);
void PWM_ControllerStop(void);

#endif /* __PWM_H__ */
