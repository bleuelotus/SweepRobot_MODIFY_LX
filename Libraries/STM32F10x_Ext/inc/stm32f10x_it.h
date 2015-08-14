/**
  ******************************************************************************
  * @file   stm32f10x_it.h
  * @author  lschen@miramems.com
  * @version  V1.0.0
  * @date   17-April-2009
  * @brief  This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

typedef void (*stm32f10x_int_cb_t)(void);

enum stm32f10x_int_type {

    STM32F10x_INT_NMI,
    STM32F10x_INT_HARDFAULT,
    STM32F10x_INT_MEMMANAGE,
    STM32F10x_INT_BUSFAULT,
    STM32F10x_INT_USAEAGEFAULT,
    STM32F10x_INT_SVC,
    STM32F10x_INT_DEBUGMON,
    STM32F10x_INT_PENDSV,
    STM32F10x_INT_SYSTICK,

    STM32F10x_INT_WWDG,
    STM32F10x_INT_PVD,
    STM32F10x_INT_TAMPER,
    STM32F10x_INT_RTC_SEC,
    STM32F10x_INT_RTC_ALR,
    STM32F10x_INT_FLASH,
    STM32F10x_INT_RCC,
    STM32F10x_INT_EXTI0,
    STM32F10x_INT_EXTI1,
    STM32F10x_INT_EXTI2,
    STM32F10x_INT_EXTI3,
    STM32F10x_INT_EXTI4,
    STM32F10x_INT_DMA1_Channel1,
    STM32F10x_INT_DMA1_Channel2,
    STM32F10x_INT_DMA1_Channel3,
    STM32F10x_INT_DMA1_Channel4,
    STM32F10x_INT_DMA1_Channel5,
    STM32F10x_INT_DMA1_Channel6,
    STM32F10x_INT_DMA1_Channel7,

#ifdef STM32F10X_HD
    STM32F10x_INT_ADC1_2,
    STM32F10x_INT_USB_HP_CAN1_TX,
    STM32F10x_INT_USB_LP_CAN1_RX0,
    STM32F10x_INT_CAN1_RX1,
    STM32F10x_INT_CAN1_SCE,
    STM32F10x_INT_EXTI9_5_5,
    STM32F10x_INT_EXTI9_5_6,
    STM32F10x_INT_EXTI9_5_7,
    STM32F10x_INT_EXTI9_5_8,
    STM32F10x_INT_EXTI9_5_9,
    STM32F10x_INT_TIM1_BRK,
    STM32F10x_INT_TIM1_UP,
    STM32F10x_INT_TIM1_TRG_COM,
    STM32F10x_INT_TIM1_CC,
    STM32F10x_INT_TIM2,
    STM32F10x_INT_TIM3,
    STM32F10x_INT_TIM4,
    STM32F10x_INT_I2C1_EV,
    STM32F10x_INT_I2C1_ER,
    STM32F10x_INT_I2C2_EV,
    STM32F10x_INT_I2C2_ER,
    STM32F10x_INT_SPI1,
    STM32F10x_INT_SPI2,
    STM32F10x_INT_USART1,
    STM32F10x_INT_USART2,
    STM32F10x_INT_USART3,
    STM32F10x_INT_EXTI15_10_10,
    STM32F10x_INT_EXTI15_10_11,
    STM32F10x_INT_EXTI15_10_12,
    STM32F10x_INT_EXTI15_10_13,
    STM32F10x_INT_EXTI15_10_14,
    STM32F10x_INT_EXTI15_10_15,
    STM32F10x_INT_RTCAlarm,
    STM32F10x_INT_USBWakeUp,
    STM32F10x_INT_TIM8_BRK,
    STM32F10x_INT_TIM8_UP,
    STM32F10x_INT_TIM8_TRG_COM,
    STM32F10x_INT_TIM8_CC,
    STM32F10x_INT_ADC3,
    STM32F10x_INT_FSMC,
    STM32F10x_INT_SDIO,
    STM32F10x_INT_TIM5,
    STM32F10x_INT_SPI3,
    STM32F10x_INT_UART4,
    STM32F10x_INT_UART5,
    STM32F10x_INT_TIM6,
    STM32F10x_INT_TIM7,
    STM32F10x_INT_DMA2_Channel1,
    STM32F10x_INT_DMA2_Channel2,
    STM32F10x_INT_DMA2_Channel3,
    STM32F10x_INT_DMA2_Channel4_5,
#endif /* STM32F10X_HD */

    STM32F10x_INT_BOUND
};

s8 plat_int_reg_cb(enum stm32f10x_int_type type, void *cb);
s8 plat_int_dereg_cb(enum stm32f10x_int_type type);

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
