/**
  ******************************************************************************
  * @file    Irda.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   IrDA 1838 receive demodulation
  ******************************************************************************
  */

#ifndef __IRDA_H__
#define	__IRDA_H__

#include "stm32f10x_conf.h"

#define IRDA_MODE_EJE


#define IRDA_BACK_LIGHT_GPIO_PERIPH_ID              RCC_APB2Periph_GPIOD
#define IRDA_BACK_LIGHT_GPIO                        GPIOD
#define IRDA_BACK_LIGHT_PIN                         GPIO_Pin_12
#define IRDA_BACK_LIGHT_EXTI_GPIO                   GPIO_PortSourceGPIOD
#define IRDA_BACK_LIGHT_EXTI_PIN                    GPIO_PinSource12
#define IRDA_BACK_LIGHT_EXTI_IRQ                    EXTI15_10_IRQn
#define IRDA_BACK_LIGHT_EXTI_LINE                   EXTI_Line12
#define IRDA_BACK_LIGHT_INT_INDEX                   STM32F10x_INT_EXTI15_10_12

#define IRDA_LEFT_LIGHT_GPIO_PERIPH_ID              RCC_APB2Periph_GPIOE
#define IRDA_LEFT_LIGHT_GPIO                        GPIOE
#define IRDA_LEFT_LIGHT_PIN                         GPIO_Pin_4
#define IRDA_LEFT_LIGHT_EXTI_GPIO                   GPIO_PortSourceGPIOE
#define IRDA_LEFT_LIGHT_EXTI_PIN                    GPIO_PinSource4
#define IRDA_LEFT_LIGHT_EXTI_IRQ                    EXTI4_IRQn
#define IRDA_LEFT_LIGHT_EXTI_LINE                   EXTI_Line4
#define IRDA_LEFT_LIGHT_INT_INDEX                   STM32F10x_INT_EXTI4

#define IRDA_FRONT_L_LIGHT_GPIO_PERIPH_ID           RCC_APB2Periph_GPIOD
#define IRDA_FRONT_L_LIGHT_GPIO                     GPIOD
#define IRDA_FRONT_L_LIGHT_PIN                      GPIO_Pin_5
#define IRDA_FRONT_L_LIGHT_EXTI_GPIO                GPIO_PortSourceGPIOD
#define IRDA_FRONT_L_LIGHT_EXTI_PIN                 GPIO_PinSource5
#define IRDA_FRONT_L_LIGHT_EXTI_IRQ                 EXTI9_5_IRQn
#define IRDA_FRONT_L_LIGHT_EXTI_LINE                EXTI_Line5
#define IRDA_FRONT_L_LIGHT_INT_INDEX                STM32F10x_INT_EXTI9_5_5

#define IRDA_FRONT_R_LIGHT_GPIO_PERIPH_ID           RCC_APB2Periph_GPIOD
#define IRDA_FRONT_R_LIGHT_GPIO                     GPIOD
#define IRDA_FRONT_R_LIGHT_PIN                      GPIO_Pin_1
#define IRDA_FRONT_R_LIGHT_EXTI_GPIO                GPIO_PortSourceGPIOD
#define IRDA_FRONT_R_LIGHT_EXTI_PIN                 GPIO_PinSource1
#define IRDA_FRONT_R_LIGHT_EXTI_IRQ                 EXTI1_IRQn
#define IRDA_FRONT_R_LIGHT_EXTI_LINE                EXTI_Line1
#define IRDA_FRONT_R_LIGHT_INT_INDEX                STM32F10x_INT_EXTI1

#ifdef REVISION_1_0
#define IRDA_RIGHT_LIGHT_GPIO_PERIPH_ID             RCC_APB2Periph_GPIOA
#define IRDA_RIGHT_LIGHT_GPIO                       GPIOA
#define IRDA_RIGHT_LIGHT_PIN                        GPIO_Pin_8
#define IRDA_RIGHT_LIGHT_EXTI_GPIO                  GPIO_PortSourceGPIOA
#define IRDA_RIGHT_LIGHT_EXTI_PIN                   GPIO_PinSource8
#define IRDA_RIGHT_LIGHT_EXTI_IRQ                   EXTI9_5_IRQn
#define IRDA_RIGHT_LIGHT_EXTI_LINE                  EXTI_Line8
#define IRDA_RIGHT_LIGHT_INT_INDEX                  STM32F10x_INT_EXTI9_5_8
#elif defined REVISION_1_1
#define IRDA_RIGHT_LIGHT_GPIO_PERIPH_ID             RCC_APB2Periph_GPIOD
#define IRDA_RIGHT_LIGHT_GPIO                       GPIOD
#define IRDA_RIGHT_LIGHT_PIN                        GPIO_Pin_6
#define IRDA_RIGHT_LIGHT_EXTI_GPIO                  GPIO_PortSourceGPIOD
#define IRDA_RIGHT_LIGHT_EXTI_PIN                   GPIO_PinSource6
#define IRDA_RIGHT_LIGHT_EXTI_IRQ                   EXTI9_5_IRQn
#define IRDA_RIGHT_LIGHT_EXTI_LINE                  EXTI_Line6
#define IRDA_RIGHT_LIGHT_INT_INDEX                  STM32F10x_INT_EXTI9_5_6
#endif

#define IRDA_LIGHT_GPIO_PERIPH_ID                   (IRDA_BACK_LIGHT_GPIO_PERIPH_ID|        \
                                                     IRDA_LEFT_LIGHT_GPIO_PERIPH_ID|        \
                                                     IRDA_FRONT_L_LIGHT_GPIO_PERIPH_ID|     \
                                                     IRDA_FRONT_R_LIGHT_GPIO_PERIPH_ID|     \
                                                     IRDA_RIGHT_LIGHT_GPIO_PERIPH_ID)

#define IRDA_LIGHT_EXTI_LINES                       (IRDA_BACK_LIGHT_EXTI_LINE|             \
                                                     IRDA_LEFT_LIGHT_EXTI_LINE|             \
                                                     IRDA_FRONT_L_LIGHT_EXTI_LINE|          \
                                                     IRDA_FRONT_R_LIGHT_EXTI_LINE|          \
                                                     IRDA_RIGHT_LIGHT_EXTI_LINE)

#define IRDA_HOMING_CODE_BOUND_UPPER                0x47
#define IRDA_HOMING_CODE_BOUND_LOWER                0x39
#define IS_IRDA_HOMING_CODE(code)                   ((code<IRDA_HOMING_CODE_BOUND_UPPER) && (code>IRDA_HOMING_CODE_BOUND_LOWER))
//#define IS_IRDA_HOMING_CODE(code)                   (code<IRDA_HOMING_CODE_BOUND_UPPER)

enum IRAD_Light {

    IRDA_BACK_LIGHT = 0,
    IRDA_LEFT_LIGHT,
    IRDA_FRONT_L_LIGHT,
    IRDA_FRONT_R_LIGHT,
    IRDA_RIGHT_LIGHT,
    IRDA_LIGHT_NUM,
};

enum IrDARecvPos {

    IRDA_RECV_POS_B = IRDA_BACK_LIGHT,
    IRDA_RECV_POS_L = IRDA_LEFT_LIGHT,
    IRDA_RECV_POS_FL = IRDA_FRONT_L_LIGHT,
    IRDA_RECV_POS_FR = IRDA_FRONT_R_LIGHT,
    IRDA_RECV_POS_R = IRDA_RIGHT_LIGHT,
};

enum PwrStationSignal {

    PWR_STATION_BACKOFF_SIG_L   = 0x46,
    PWR_STATION_BACKOFF_SIG_R   = 0x45,
    PWR_STATION_HOME_SIG_LL     = 0x41,
    PWR_STATION_HOME_SIG_LS     = 0x44,
    PWR_STATION_HOME_SIG_RL     = 0x43,
    PWR_STATION_HOME_SIG_RS     = 0x42,
    PWR_STATION_HOME_SIG_CENTER = 0x40,
};

typedef struct PwrStationSignalData{

    enum IrDARecvPos        src;
    enum PwrStationSignal   sig;
} PwrStationSigData_t;

void IrDA_Init(void);
void IrDA_DeInit(void);
u8 IrDA_ProcessNEC(u8 idx);
void IrDA_ProcessEJE(void (*RemoteCB)(u8 code));
void PwrStationHomingSigProc(u8 idx, u8 code);

#define IRDA_ID             0

#endif /* __IRDA_H__ */