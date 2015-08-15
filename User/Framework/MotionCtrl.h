/**
  ******************************************************************************
  * @file    MotionCtrl.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Robot motion control drvier
  ******************************************************************************
  */

#ifndef __MITION_CTRL_H__
#define __MITION_CTRL_H__

#include "stm32f10x_conf.h"
#include "MotorCtrl.h"

#define IFRD_TxRx_CHAN_NUM                  15                                   // Total:8, but 2 back lights of bottom are not used
#define IFRD_FRONT_CHAN_NUM                 2
#define IFRD_SIDE_CHAN_NUM                  2
#define IFRD_BOTTOM_CHAN_NUM                2
#define MCTRL_ACT_MAX_DEPTH                 3

typedef struct MotionCtrl_Action_s{
    u8              LWheelDir;
    u8              RWheelDir;
    u8              LWheelSpeed;
    u8              RWheelSpeed;
    u8              LWheelExpSpeed;
    u8              RWheelExpSpeed;
    u16             LWheelExpCnt;
    u16             RWheelExpCnt;
} MCtrl_Act_t;

extern u8 gProximitySign[IFRD_TxRx_CHAN_NUM];
extern u8 gActSeqDepth;
extern u8 gActSeqDepLIndicator;
extern u8 gActSeqDepRIndicator;
extern MCtrl_Act_t gActSequence[MCTRL_ACT_MAX_DEPTH];

#define IS_MOTION_PROC_FINISH()             ((gActSeqDepLIndicator==0)&&(gActSeqDepRIndicator==0))

#define IFRD_TX_CTRL_GPIO_PERIPH_ID         (RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE)
#define IFRD_LEFT_TX_CTRL_GPIO              GPIOC
#define IFRD_RIGHT_TX_CTRL_GPIO             GPIOE
#define IFRD_LEFT_TX_CTRL_PIN               GPIO_Pin_7
#define IFRD_RIGHT_TX_CTRL_PIN              GPIO_Pin_0
#define IFRD_TX_ENABLE()                    do{GPIO_SetBits(IFRD_LEFT_TX_CTRL_GPIO, IFRD_LEFT_TX_CTRL_PIN);GPIO_SetBits(IFRD_RIGHT_TX_CTRL_GPIO, IFRD_RIGHT_TX_CTRL_PIN);}while(0)
#define IFRD_TX_DISABLE()                   do{GPIO_ResetBits(IFRD_LEFT_TX_CTRL_GPIO, IFRD_LEFT_TX_CTRL_PIN);GPIO_ResetBits(IFRD_RIGHT_TX_CTRL_GPIO, IFRD_RIGHT_TX_CTRL_PIN);}while(0)

#define COLLISION_DETECT_GPIO_PERIPH_ID     (RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE)
#define COLLISION_DETECT_LEFT_GPIO          GPIOE
#define COLLISION_DETECT_FL_GPIO            GPIOE
#define COLLISION_DETECT_RIGHT_GPIO         GPIOC
#define COLLISION_DETECT_FR_GPIO            GPIOC
#define COLLISION_DETECT_LEFT_PIN           GPIO_Pin_3
#define COLLISION_DETECT_FL_PIN             GPIO_Pin_2
#define COLLISION_DETECT_RIGHT_PIN          GPIO_Pin_9
#define COLLISION_DETECT_FR_PIN             GPIO_Pin_8
#define COLLISION_SIGN_LEFT                 GPIO_ReadInputDataBit(COLLISION_DETECT_LEFT_GPIO, COLLISION_DETECT_LEFT_PIN)
#define COLLISION_SIGN_FL                   GPIO_ReadInputDataBit(COLLISION_DETECT_FL_GPIO, COLLISION_DETECT_FL_PIN)
#define COLLISION_SIGN_RIGHT                GPIO_ReadInputDataBit(COLLISION_DETECT_RIGHT_GPIO, COLLISION_DETECT_RIGHT_PIN)
#define COLLISION_SIGN_FR                   GPIO_ReadInputDataBit(COLLISION_DETECT_FR_GPIO, COLLISION_DETECT_FR_PIN)

#define WHEEL_FLOAT_DETECT_GPIO_PERIPH_ID   (RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE)
#define WHEEL_FLOAT_DETECT_L_GPIO           GPIOD
#define WHEEL_FLOAT_DETECT_R_GPIO           GPIOE
#define WHEEL_FLOAT_DETECT_L_PIN            GPIO_Pin_15
#define WHEEL_FLOAT_DETECT_R_PIN            GPIO_Pin_8
#define LWHEEL_FLOAT_SIGN                   GPIO_ReadInputDataBit(WHEEL_FLOAT_DETECT_L_GPIO, WHEEL_FLOAT_DETECT_L_PIN)
#define RWHEEL_FLOAT_SIGN                   GPIO_ReadInputDataBit(WHEEL_FLOAT_DETECT_R_GPIO, WHEEL_FLOAT_DETECT_R_PIN)
#define WHEEL_FLOAT_SIGN_ALL                (LWHEEL_FLOAT_SIGN||RWHEEL_FLOAT_SIGN)

#define WHEEL_CNT_GPIO_PERIPH_ID            (RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE)
#define WHEEL_NUM                           3
#define FWHEEL_COUNTER_GPIO                 GPIOA
#define RWHEEL_COUNTER_GPIO                 GPIOD
#define LWHEEL_COUNTER_GPIO                 GPIOE
#define FWHEEL_COUNTER_PIN                  GPIO_Pin_12
#define RWHEEL_COUNTER_PIN                  GPIO_Pin_14
#define LWHEEL_COUNTER_PIN                  GPIO_Pin_7
#define FWHEEL_COUNTER_EXTI_GPIO_SOURCE     GPIO_PortSourceGPIOA
#define RWHEEL_COUNTER_EXTI_GPIO_SOURCE     GPIO_PortSourceGPIOD
#define LWHEEL_COUNTER_EXTI_GPIO_SOURCE     GPIO_PortSourceGPIOE
#define FWHEEL_COUNTER_EXTI_PIN_SOURCE      GPIO_PinSource12
#define RWHEEL_COUNTER_EXTI_PIN_SOURCE      GPIO_PinSource14
#define LWHEEL_COUNTER_EXTI_PIN_SOURCE      GPIO_PinSource7
#define FWHEEL_COUNTER_INT_INDEX            STM32F10x_INT_EXTI15_10_12
#define RWHEEL_COUNTER_INT_INDEX            STM32F10x_INT_EXTI15_10_14
#define LWHEEL_COUNTER_INT_INDEX            STM32F10x_INT_EXTI9_5_7
#define FWHEEL_COUNTER_EXTI_IRQN            EXTI15_10_IRQn
#define RWHEEL_COUNTER_EXTI_IRQN            EXTI15_10_IRQn
#define LWHEEL_COUNTER_EXTI_IRQN            EXTI9_5_IRQn
#define WHEEL_CNT_EXTI_LINES                (EXTI_Line7|EXTI_Line14)            //(EXTI_Line7|EXTI_Line12|EXTI_Line14)

#define WHEEL_GEAR_RATIO                    62.5
#define WHEEL_1CYCLE_CNT                    500                                 //(WHEEL_GEAR_RATIO*8)
#define WHEEL_BASE_LEN                      204                                 // mm
#define WHEEL_DIAMETER                      68                                  // mm
#define WHEEL_TURN_45_CNT                   188
#define WHEEL_TURN_90_CNT                   375
#define WHEEL_TURN_180_CNT                  750
#define WHEEL_TURN_360_CNT                  1500
#define WHEEL_FAULT_BACK_CNT                240

#define WHEEL_CRUISE_SPEED                  10
#define WHEEL_HOMING_SPEED                  5

enum MotionEvt {

    MOTION_EVT_PATH_FAULT_L,
    MOTION_EVT_PATH_FAULT_R,
    MOTION_EVT_PROXIMITY_SL,
    MOTION_EVT_PROXIMITY_SR,
};

enum MotionCtrlManualAct {

    MANUAL_ACT_UP,
    MANUAL_ACT_LEFT,
    MANUAL_ACT_DOWN,
    MANUAL_ACT_RIGHT,
};

void MotionCtrl_Init(void);
void MotionCtrl_Stop(void);
void MotionCtrl_Start(void);
void MotionCtrl_Proc(void);
void MotionCtrl_AutoMotionInit(void);
void MotionCtrl_ManualCtrlProc(enum MotionCtrlManualAct act);
void MotionCtrl_LeftPathFaulProc(u16 backcnt, u16 turncnt, u8 StopOnFinish);
void MotionCtrl_RightPathFaulProc(u16 backcnt, u16 turncnt, u8 StopOnFinish);
void MotionCtrl_RoundedSlowly(void);
void MotionCtrl_MoveDirTune(u8 l, u8 r);

#endif /* __MITION_CTRL_H__ */
