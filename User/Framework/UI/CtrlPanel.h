/**
  ******************************************************************************
  * @file    CtrlPanel.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Control Panel driver
  ******************************************************************************
  */

#ifndef __CTRL_PANEL_H__
#define __CTRL_PANEL_H__

#include "stm32f10x_conf.h"
#include "SweepRobot.h"
#include "IrDA.h"


#define CTRL_BTN_ALL_IN_ONE_GPIO_PERIPH_ID  RCC_APB2Periph_GPIOD
#define CTRL_BTN_ALL_IN_ONE_GPIO            GPIOD
#ifdef REVISION_1_0
#define CTRL_BTN_ALL_IN_ONE_PIN             GPIO_Pin_6
#elif defined REVISION_1_1
#define CTRL_BTN_ALL_IN_ONE_PIN             GPIO_Pin_7
#elif defined REVISION_1_2
#define CTRL_BTN_ALL_IN_ONE_PIN             GPIO_Pin_7
#endif
#define CTRL_BTN_ALL_IN_ONE_EXTI_GPIO       GPIO_PortSourceGPIOD
#define CTRL_BTN_ALL_IN_ONE_EXTI_PIN        GPIO_PinSource6
#define CTRL_BTN_ALL_IN_ONE_EXTI_IRQ        EXTI9_5_IRQn
#define CTRL_BTN_ALL_IN_ONE_INT_PP          1
#define CTRL_BTN_ALL_IN_ONE_INT_SP          1
#define CTRL_BTN_ALL_IN_ONE_EXTI_LINES      EXTI_Line6
#define CTRL_BTN_ALL_IN_ONE_INT_INDEX       STM32F10x_INT_EXTI9_5_6
#define CTRL_BTN_ALL_IN_ONE_INT_CB          CtrlPanel_AllInOneBtnProc

#define CTRL_LED_CTRL_GPIO_PERIPH_ID        RCC_APB2Periph_GPIOD
#define CTRL_LED_R_CTRL_GPIO                GPIOD
#define CTRL_LED_R_CTRL_PIN                 GPIO_Pin_2
#define CTRL_LED_G_CTRL_GPIO                GPIOD
#define CTRL_LED_G_CTRL_PIN                 GPIO_Pin_3
#define CTRL_LED_B_CTRL_GPIO                GPIOD
#define CTRL_LED_B_CTRL_PIN                 GPIO_Pin_4


#define REMOTE_CMD_PLAN_STATE               0x2A
#define REMOTE_CMD_PLAN_CANCEL              0xB6
#define REMOTE_CMD_PLAN_SET                 0xC8

#define REMOTE_CMD_PLAN_0P5H                0x75
#define REMOTE_CMD_PLAN_1P0H                0x74
#define REMOTE_CMD_PLAN_1P5H                0x73
#define REMOTE_CMD_PLAN_2P0H                0x72
#define REMOTE_CMD_PLAN_2P5H                0x71
#define REMOTE_CMD_PLAN_3P0H                0x70
#define REMOTE_CMD_PLAN_3P5H                0x6F
#define REMOTE_CMD_PLAN_4P0H                0x6E
#define REMOTE_CMD_PLAN_4P5H                0x6D
#define REMOTE_CMD_PLAN_5P0H                0x6C
#define REMOTE_CMD_PLAN_5P5H                0x6B
#define REMOTE_CMD_PLAN_6P0H                0x6A
/*...*/
#define REMOTE_CMD_PLAN_23P5H               0x47

#define REMOTE_CMD_CHARGE                   0xAC
#define REMOTE_CMD_UP                       0xBE
#define REMOTE_CMD_LEFT                     0xBA
#define REMOTE_CMD_DOWN                     0xBC
#define REMOTE_CMD_RIGHT                    0xB8
#define REMOTE_CMD_RUN_STOP                 0x8A
#define REMOTE_CMD_MODE_AUTO                0xAF
#define REMOTE_CMD_MODE_1                   0xB4
#define REMOTE_CMD_MODE_2                   0xB3
#define REMOTE_CMD_MODE_3                   0xB2
#define REMOTE_CMD_MODE_4                   0xB0
#define REMOTE_CMD_SPOT                     0xB5

enum CtrlPanelLED {

    CTRL_PANEL_LED_NONE     = 0x00,
    CTRL_PANEL_LED_RED      = 0x01,
    CTRL_PANEL_LED_GREEN    = 0x02,
    CTRL_PANEL_LED_RG       = 0x03,
    CTRL_PANEL_LED_BLUE     = 0x04,
    CTRL_PANEL_LED_RB       = 0x05,
    CTRL_PANEL_LED_GB       = 0x06,
    CTRL_PANEL_LED_RGB      = 0x07,
};

#define CTRL_PANEL_LED_BR_LVL               5

void CtrlPanel_Init(void);
void CtrlPanel_LEDCtrl(enum CtrlPanelLED, u8 Brightness);
void CtrlPanel_RemoteCB(u8 code);
void CtrlPanel_LEDCtrlProc(void);

#endif /* !__CTRL_PANEL_H__ */
