/**
  ******************************************************************************
  * @file    CtrlPanel.c
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Control Panel driver
  ******************************************************************************
  */
#include <stdlib.h>
#include "delay.h"
#include "CtrlPanel.h"
#include "SweepRobot.h"
#include "key.h"
#include "IrDA.h"

struct CtrlPanel_LEDObj_s {

    enum CtrlPanelLED   led;
    GPIO_TypeDef*       GPIOx;
    u16                 GPIO_Pin;
};

const struct CtrlPanel_LEDObj_s LEDS[] = {
                                            { CTRL_PANEL_LED_RED,          CTRL_LED_R_CTRL_GPIO,       CTRL_LED_R_CTRL_PIN },
                                            { CTRL_PANEL_LED_GREEN,        CTRL_LED_G_CTRL_GPIO,       CTRL_LED_G_CTRL_PIN },
                                            { CTRL_PANEL_LED_BLUE,         CTRL_LED_B_CTRL_GPIO,       CTRL_LED_B_CTRL_PIN },
                                         };

static u8 gLED = CTRL_PANEL_LED_NONE;
static u8 gBrightness = 0, gBrightnessCnt = 0;
static u8 gHighCnt[3] = {0}, gLowCnt[3] = {0};

void CtrlPanel_LEDCtrl(enum CtrlPanelLED led, u8 Brightness)
{
    if(Brightness > 10)
        return;

    gLED = led;
    gBrightness = Brightness;
}

void CtrlPanel_LEDCtrlProc(void)
{
    u8  i = 0;

    if(!(gBrightnessCnt%10)){
        gBrightnessCnt = 0;
        gHighCnt[0] = gBrightness;
        gHighCnt[1] = gBrightness;
        gHighCnt[2] = gBrightness;
        gLowCnt[0]  = 10 - gBrightness;
        gLowCnt[1]  = 10 - gBrightness;
        gLowCnt[2]  = 10 - gBrightness;
    }

    for(i=0;i<sizeof(LEDS)/sizeof(LEDS[0]);i++){
        if(((gLED>>i)&0x01) && (gHighCnt[i] > 0)){
            if(gBrightnessCnt%2 || (gLowCnt[i] == 0)) {
                GPIO_SetBits(LEDS[i].GPIOx, LEDS[i].GPIO_Pin);
                gHighCnt[i]--;
            }
            else{
                GPIO_ResetBits(LEDS[i].GPIOx, LEDS[i].GPIO_Pin);
                gLowCnt[i]--;
            }
        }
        else{
            GPIO_ResetBits(LEDS[i].GPIOx, LEDS[i].GPIO_Pin);
            gLowCnt[i]--;
        }
    }
    gBrightnessCnt++;
}

void CtrlPanel_RemoteCB(u8 code)
{
    Msg_t   Msg;

    Msg.expire = 0;
    Msg.type = MSG_TYPE_CTRL;
    Msg.prio = MSG_PRIO_HIGHEST;
    Msg.MsgCB = NULL;
    Msg.Data.ByteVal = code;
    SweepRobot_SendMsg(&Msg);
}

void CtrlPanel_AllInOneBtnUpProc(void)
{
    Msg_t   Msg;

    Msg.expire = 0;
    Msg.type = MSG_TYPE_CTRL;
    Msg.prio = MSG_PRIO_HIGHEST;
    Msg.MsgCB = NULL;
    Msg.Data.ByteVal = REMOTE_CMD_RUN_STOP;
    SweepRobot_SendMsg(&Msg);

    PM_ResetSysIdleState();
}

void CtrlPanel_AllInOneBtnDownProc(void)
{
    if(gSystemIdleCnt > 3){
        gSystemIdleCnt = 3;
    }
}

void CtrlPanel_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    Key_t                   AllInOnekey;

    RCC_APB2PeriphClockCmd(CTRL_LED_CTRL_GPIO_PERIPH_ID, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = CTRL_LED_R_CTRL_PIN;
    GPIO_Init(CTRL_LED_R_CTRL_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = CTRL_LED_G_CTRL_PIN;
    GPIO_Init(CTRL_LED_G_CTRL_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = CTRL_LED_B_CTRL_PIN;
    GPIO_Init(CTRL_LED_B_CTRL_GPIO, &GPIO_InitStructure);

    CtrlPanel_LEDCtrl(CTRL_PANEL_LED_GREEN, CTRL_PANEL_LED_BR_LVL);

    Key_CoreInit();

    AllInOnekey.RCCResID = CTRL_BTN_ALL_IN_ONE_GPIO_PERIPH_ID;
    AllInOnekey.GPIOx = CTRL_BTN_ALL_IN_ONE_GPIO;
    AllInOnekey.GPIO_Pin = CTRL_BTN_ALL_IN_ONE_PIN;
    AllInOnekey.EvtCB.Down = CtrlPanel_AllInOneBtnDownProc;
    AllInOnekey.EvtCB.Up = CtrlPanel_AllInOneBtnUpProc;
    Key_Register(&AllInOnekey);

    Key_StartListen();
}
