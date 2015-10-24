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

#ifdef REVISION_1_0
#define IFRD_TxRx_CHAN_NUM                  8                                   // Total:8, but 2 back lights of bottom are not used
#define IFRD_TxRx_ACTUAL_CHAN_NUM           8
#elif defined REVISION_1_1
#define IFRD_TxRx_CHAN_NUM                  6                                   // 2 back lights of bottom are not used
#define IFRD_TxRx_ACTUAL_CHAN_NUM           6
#elif defined REVISION_1_2
#define IFRD_TxRx_CHAN_NUM                  8                                  // Total:8, actual:6, 2 side lights of bottom are used with front lights of bottom through analog switch
#define IFRD_TxRx_ACTUAL_CHAN_NUM           6
#endif
#define IFRD_FRONT_CHAN_NUM                 2
#define IFRD_SIDE_CHAN_NUM                  2
#ifdef REVISION_1_1
#define IFRD_BOTTOM_CHAN_NUM                2
#elif defined REVISION_1_2
#define IFRD_BOTTOM_CHAN_NUM                4
#endif
#define MCTRL_ACT_MAX_DEPTH                 10

enum MotionEvt {

    MOTION_EVT_PATH_FAULT,
    MOTION_EVT_EXCEPTION,
    MOTION_EVT_TRAPPED,
    MOTION_EVT_IDLE_SYNC,
};

enum MotionCtrlManualAct {

    MANUAL_ACT_UP,
    MANUAL_ACT_LEFT,
    MANUAL_ACT_DOWN,
    MANUAL_ACT_RIGHT,
};

typedef struct MotionCtrl_Action_s{
    u8              LWheelDefDir;
    u8              RWheelDefDir;
    u8              LWheelInitSpeed;
    u8              RWheelInitSpeed;
    u8              LWheelExpSpeed;
    u8              RWheelExpSpeed;
    u16             LWheelExpCnt;
    u16             RWheelExpCnt;
    u8              LWheelSync;
    u8              RWheelSync;
    void            (*PreAct)(struct MotionCtrl_Action_s *node);
    void            (*ProcAct)(struct MotionCtrl_Action_s *node);
    u8              (*PostAct)(void);
} MCtrl_Act_t;

typedef struct MotionException_ErrCnt_s{    
    u8                AshTrayInsErrCnt;
    u8                WheelFloatErrCnt;
    u8                MBrushOCErrCnt;
    u8                FanOCErrCnt;
    u8                LBrushOCErrCnt;
    u8                RBrushOCErrCnt;
    u8                WheelStuckErrCnt;
}MotionException_ErrCnt_t;

extern u8 gProximitySign[IFRD_TxRx_CHAN_NUM];
extern u8 gActSeqDepth;
extern u8 gActSeqDepLIndicator;
extern u8 gActSeqDepRIndicator;
extern MCtrl_Act_t gActSequence[MCTRL_ACT_MAX_DEPTH];

//#define LBRUSH_CUR_THRESHOLD                300                             //  0.5A
//#define RBRUSH_CUR_THRESHOLD                300                             //  0.5A
//#define MBRUSH_CUR_THRESHOLD                1000                            //  1.6A
//#define FAN_CUR_THRESHOLD                   1000                            //  1.6A

#define LBRUSH_CUR_THRESHOLD                0.25f                           //  0.5A = 0.25*2
#define RBRUSH_CUR_THRESHOLD                0.25f                           //  0.5A = 0.25*2
#define MBRUSH_CUR_THRESHOLD                0.53f                           //  1.6A = 0.53*3
#define FAN_CUR_THRESHOLD                   0.53f                           //  1.6A = 0.53*3

#define IS_MOTION_PROC_FINISH()             ((gActSeqDepLIndicator==0)&&(gActSeqDepRIndicator==0))
#define MOTION_PROC_STATE_SET()             do{gActSeqDepLIndicator++;}while(0);

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
#define WHEEL_FLOAT_SIGN_ALL                (!(LWHEEL_FLOAT_SIGN && RWHEEL_FLOAT_SIGN))

#define ASH_TRAY_DETECT_GPIO_PERIPH_ID      (RCC_APB2Periph_GPIOE)
#define ASH_TRAY_DETECT_GPIO                GPIOE
#define ASH_TRAY_DETECT_PIN                 GPIO_Pin_9
#define ASH_TRAY_INSTALL_SIGN               GPIO_ReadInputDataBit(ASH_TRAY_DETECT_GPIO, ASH_TRAY_DETECT_PIN)

#define WHEEL_CNT_GPIO_PERIPH_ID            (RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE)
#define WHEEL_NUM                           3

#ifdef REVISION_1_0
#define FWHEEL_COUNTER_GPIO                 GPIOA
#define FWHEEL_COUNTER_PIN                  GPIO_Pin_12
#define FWHEEL_COUNTER_EXTI_GPIO_SOURCE     GPIO_PortSourceGPIOA
#define FWHEEL_COUNTER_EXTI_PIN_SOURCE      GPIO_PinSource12
#define FWHEEL_COUNTER_INT_INDEX            STM32F10x_INT_EXTI15_10_12
#define FWHEEL_COUNTER_EXTI_IRQN            EXTI15_10_IRQn
#elif defined REVISION_1_1
#define FWHEEL_COUNTER_GPIO                 GPIOC
#define FWHEEL_COUNTER_PIN                  GPIO_Pin_5
#define FWHEEL_COUNTER_EXTI_GPIO_SOURCE     GPIO_PortSourceGPIOC
#define FWHEEL_COUNTER_EXTI_PIN_SOURCE      GPIO_PinSource5
#define FWHEEL_COUNTER_INT_INDEX            STM32F10x_INT_EXTI9_5_5
#define FWHEEL_COUNTER_EXTI_IRQN            EXTI9_5_IRQn
#elif defined REVISION_1_2
#define FWHEEL_COUNTER_GPIO                 GPIOC
#define FWHEEL_COUNTER_PIN                  GPIO_Pin_5
#define FWHEEL_COUNTER_EXTI_GPIO_SOURCE     GPIO_PortSourceGPIOC
#define FWHEEL_COUNTER_EXTI_PIN_SOURCE      GPIO_PinSource5
#define FWHEEL_COUNTER_INT_INDEX            STM32F10x_INT_EXTI9_5_5
#define FWHEEL_COUNTER_EXTI_IRQN            EXTI9_5_IRQn
#endif

#define LWHEEL_COUNTER_GPIO                 GPIOE
#define LWHEEL_COUNTER_PIN                  GPIO_Pin_7
#define LWHEEL_COUNTER_EXTI_GPIO_SOURCE     GPIO_PortSourceGPIOE
#define LWHEEL_COUNTER_EXTI_PIN_SOURCE      GPIO_PinSource7
#define LWHEEL_COUNTER_INT_INDEX            STM32F10x_INT_EXTI9_5_7
#define LWHEEL_COUNTER_EXTI_IRQN            EXTI9_5_IRQn

#define RWHEEL_COUNTER_GPIO                 GPIOD
#define RWHEEL_COUNTER_PIN                  GPIO_Pin_14
#define RWHEEL_COUNTER_EXTI_GPIO_SOURCE     GPIO_PortSourceGPIOD
#define RWHEEL_COUNTER_EXTI_PIN_SOURCE      GPIO_PinSource14
#define RWHEEL_COUNTER_INT_INDEX            STM32F10x_INT_EXTI15_10_14
#define RWHEEL_COUNTER_EXTI_IRQN            EXTI15_10_IRQn

#define WHEEL_CNT_EXTI_LINES                (EXTI_Line7|EXTI_Line14)            //(EXTI_Line7|EXTI_Line12|EXTI_Line14)
#ifdef REVISION_1_0
#define FWHEEL_ACTIVE_VAL                   GPIO_ReadInputDataBit(FWHEEL_COUNTER_GPIO, FWHEEL_COUNTER_PIN)
#elif REVISION_1_1
#define FWHEEL_ACTIVE_VAL                   ADCConvertedLSB[MEAS_CHAN_UNIVERSAL_WHEEL_SIG-1]
#elif REVISION_1_2
#define FWHEEL_ACTIVE_VAL                   ADCConvertedLSB[MEAS_CHAN_UNIVERSAL_WHEEL_SIG-1]
#endif

#define WHEEL_GEAR_RATIO                    62.5
#define WHEEL_1CYCLE_CNT                    500                                 //(WHEEL_GEAR_RATIO*8)
#define WHEEL_BASE_LEN                      204                                 // mm
#define WHEEL_DIAMETER                      68                                  // mm
#define WHEEL_TURN_15_CNT                   60//63
#define WHEEL_TURN_30_CNT                   120//128
#define WHEEL_TURN_45_CNT                   175//188
#define WHEEL_TURN_90_CNT                   350//375
#define WHEEL_TURN_180_CNT                  700//750
#define WHEEL_BODY_THROUGH_CNT              1400
#define WHEEL_TURN_360_CNT                  1400//1500
#define WHEEL_FAULT_BACK_CNT                180

#define WHEEL_ESCAPE_SPEED                  20
#define WHEEL_CRUISE_SPEED                  12
#define WHEEL_MODE_SPOT_SPEED               10
#define WHEEL_FAULT_PROC_SPEED              9
#define WHEEL_HOMING_SPEED                  5

#define PATH_COND_PROXIMITY_FLAG_FL_POS     0
#define PATH_COND_PROXIMITY_FLAG_BL_POS     1
#define PATH_COND_PROXIMITY_FLAG_SL_POS     2
#define PATH_COND_COLLISION_FLAG_FL_POS     3
#define PATH_COND_COLLISION_FLAG_SL_POS     4
#ifdef REVISION_1_2
#define PATH_COND_PROXIMITY_FLAG_BSL_POS    5
#define PATH_COND_PROXIMITY_FLAG_BBL_POS    6
#endif

#define PATH_COND_PROXIMITY_FLAG_FR_POS     8
#define PATH_COND_PROXIMITY_FLAG_BR_POS     9
#define PATH_COND_PROXIMITY_FLAG_SR_POS     10
#define PATH_COND_COLLISION_FLAG_FR_POS     11
#define PATH_COND_COLLISION_FLAG_SR_POS     12
#ifdef REVISION_1_2
#define PATH_COND_PROXIMITY_FLAG_BSR_POS    13
#define PATH_COND_PROXIMITY_FLAG_BBR_POS    14
#endif

#define PATH_FAULT_LEFT_MASK                0x003B
#define PATH_FAULT_RIGHT_MASK               0x3B00
#define PATH_FAULT_PROXIMITY_MASK           0x2323
#define PATH_FAULT_COLLISION_MASK           0x1818
#define PATH_PROXIMITY_SIDE_L_MASK          0x0004
#define PATH_PROXIMITY_SIDE_R_MASK          0x0400
#define PATH_FAULT_FRONT_MASK               0x0B0B
#define PATH_FAULT_BOTTOM_MASK              0x2222
#define PATH_FAULT_SIDE_MASK                0x1414

#define EXCEPTION_MASK_WHEEL_FLOAT_POS      0
#define EXCEPTION_MASK_ASHTRAY_INS_POS      1
#define EXCEPTION_MASK_LWHEEL_STUCK_POS     2
#define EXCEPTION_MASK_RWHEEL_STUCK_POS     3
#define EXCEPTION_MASK_LBRUSH_OC_POS        4
#define EXCEPTION_MASK_RBRUSH_OC_POS        5
#define EXCEPTION_MASK_MBRUSH_OC_POS        6
#define EXCEPTION_MASK_FAN_OC_POS           7

#define MOTION_MONITOR_TIM_PERIPH_ID        RCC_APB1Periph_TIM2
#define MOTION_MONITOR_TIM                  TIM2
#define MOTION_MONITOR_TIM_IRQn             TIM2_IRQn
#define MOTION_MONITOR_TIM_IRQ_PP           2
#define MOTION_MONITOR_TIM_IRQ_SP           1
#define MOTION_MONITOR_TIM_INT_IDX          STM32F10x_INT_TIM2

void MotionCtrl_Init(void);
void MotionCtrl_Stop(void);
void MotionCtrl_Start(void);
void MotionCtrl_Proc(void);
void MotionCtrl_AutoMotionInit(void);
void MotionCtrl_SpotMotionInit(void);
void MotionCtrl_EdgeMotionInit(void);
void MotionCtrl_DishomingMotionInit(void);
void MotionCtrl_HomingMotionInit(void);
void MotionCtrl_ManualCtrlProc(enum MotionCtrlManualAct act);
void MotionCtrl_PathFaultProc(u8 StopOnFinish);
s8 MotionCtrl_ExceptionHandle(void);
void MotionCtrl_TrapProc(void);
void MotionCtrl_ChargeStationAvoid(u8 dir, u8 turnCnt, u8 StopOnFinish);
void MotionCtrl_MoveDirTune(u8 l, u8 r);

#endif /* __MITION_CTRL_H__ */
