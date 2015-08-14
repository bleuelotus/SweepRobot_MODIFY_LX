/**
  ******************************************************************************
  * @file    MotorCtrl.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Brushless DC motor controller drvier
  ******************************************************************************
  */

#include "MotorCtrl.h"
#include "PWM.h"

#define WHEEL_MOTOR_CTRL_GPIO           GPIOE
#define WHEEL_MOTOR_CTRL_PIN            GPIO_Pin_10

typedef struct _MotorCtrlChannel_s {

    enum PWMChannelRes      PWMResIdx;
    GPIO_TypeDef*           PhaseGPIO;
    u16                     PhasePin;
    u16                     SpeedMax;
} MotorCtrlChannel_t;

                                                       /* Chan                  PhaseGPIO   PhasePin    */
static const MotorCtrlChannel_t     MotorCtrlChan[] = {
                                                        { PWM_CHAN_FAN,         NULL,       0,              MOTOR_FUN_CHAN_MAX_SPEED    },                              /* Dust cleaner fan     */
                                                        { PWM_CHAN_RWHEEL,      GPIOD,      GPIO_Pin_13,    MOTOR_WHEEL_CHAN_MAX_SPEED  },                              /* Left wheel           */
                                                        { PWM_CHAN_LWHEEL,      GPIOE,      GPIO_Pin_5,     MOTOR_WHEEL_CHAN_MAX_SPEED  },                              /* Right wheel          */
                                                        { PWM_CHAN_LBRUSH,      NULL,       0,              MOTOR_BRUSH_CHAN_MAX_SPEED  },                              /* Left brush           */
                                                        { PWM_CHAN_RBRUSH,      NULL,       0,              MOTOR_BRUSH_CHAN_MAX_SPEED  },                              /* Rgith brush          */
                                                        { PWM_CHAN_MBRUSH,      NULL,       0,              MOTOR_BRUSH_CHAN_MAX_SPEED  },                              /* Middle brush         */
                                                      };

s8 MotorCtrl_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    u8                  i = 0, ret = 0;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);

    /* Left/Right wheel motor control gpio init */
    GPIO_InitStructure.GPIO_Pin = WHEEL_MOTOR_CTRL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WHEEL_MOTOR_CTRL_GPIO, &GPIO_InitStructure);
    GPIO_WriteBit(WHEEL_MOTOR_CTRL_GPIO, WHEEL_MOTOR_CTRL_PIN, (BitAction)1);

    for(i = 0; i < sizeof(MotorCtrlChan)/sizeof(MotorCtrlChan[0]); i++){

        if(MotorCtrlChan[i].PhasePin != 0 && MotorCtrlChan[i].PhaseGPIO != NULL){
            GPIO_InitStructure.GPIO_Pin = MotorCtrlChan[i].PhasePin;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(MotorCtrlChan[i].PhaseGPIO, &GPIO_InitStructure);
        }

        if( PWM_CHAN_NULL != MotorCtrlChan[i].PWMResIdx ){
            ret = PWM_ChanInit(MotorCtrlChan[i].PWMResIdx, 1, 0);
            if(ret){
                return -1;
            }
        }
    }
    return 0;
}

/* Timer should be stop before calling this func */
s8 MotorCtrl_ChanSpeedLevelSet(enum MotorCtrlChannel Channel, u16 level)
{
    if(level > MotorCtrlChan[Channel].SpeedMax)
        level = MotorCtrlChan[Channel].SpeedMax;
    return PWM_DutyCycleSet(MotorCtrlChan[Channel].PWMResIdx, level);
}

u16 MotorCtrl_ChanSpeedLevelGet(enum MotorCtrlChannel Channel)
{
    u16  DutyCycle = 0;

    PWM_DutyCycleGet(MotorCtrlChan[Channel].PWMResIdx, &DutyCycle);

    return DutyCycle;
}

s8 MotorCtrl_ChanSpeedDec(enum MotorCtrlChannel Channel)
{
    u16  DutyCycle = 0;

    DutyCycle = MotorCtrl_ChanSpeedLevelGet(Channel);
    if(DutyCycle > 1){
        DutyCycle -= 2;
    }
    return MotorCtrl_ChanSpeedLevelSet(Channel, DutyCycle);
}

s8 MotorCtrl_ChanSpeedInc(enum MotorCtrlChannel Channel)
{
    u16  DutyCycle = 0;

    DutyCycle = MotorCtrl_ChanSpeedLevelGet(Channel);

    if(DutyCycle < MotorCtrlChan[Channel].SpeedMax - 1)
        DutyCycle += 2;

    return MotorCtrl_ChanSpeedLevelSet(Channel, DutyCycle);
}

/* Brake should be taken before changing direction */
s8 MotorCtrl_ChanDirSet(enum MotorCtrlChannel Channel, u8 Dir)
{
    if(MotorCtrlChan[Channel].PhasePin != 0 && MotorCtrlChan[Channel].PhaseGPIO != NULL){
        /* Phase */
        GPIO_WriteBit(MotorCtrlChan[Channel].PhaseGPIO, MotorCtrlChan[Channel].PhasePin, (BitAction)(Dir));
    }
    return 0;
}

u8 MotorCtrl_ChanDirGet(enum MotorCtrlChannel Channel)
{
    return GPIO_ReadOutputDataBit(MotorCtrlChan[Channel].PhaseGPIO, MotorCtrlChan[Channel].PhasePin);
}

enum MotorStatus MotorCtrl_ChanStateGet(enum MotorCtrlChannel Channel)
{
    u16  DutyCycle = 0;

    PWM_DutyCycleGet(MotorCtrlChan[Channel].PWMResIdx, &DutyCycle);

    if( DutyCycle != 0 ){
        if( GPIO_ReadOutputDataBit(MotorCtrlChan[Channel].PhaseGPIO, MotorCtrlChan[Channel].PhasePin) )
            return MOTOR_STATUS_UP;
        else
            return MOTOR_STATUS_DOWN;
    }
    return MOTOR_STATUS_STOP;
}

