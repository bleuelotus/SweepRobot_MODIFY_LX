/*******************************************************************************/
 /**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @author  lschen@miramems.com
  * @version V1.0
  * @date    17-April-2014
  * @brief
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MiraMEMS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 MiraMEMS</center></h2>
  */
/*******************************************************************************/
/*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

static stm32f10x_int_cb_t stm32f10x_int_cb_tbl[STM32F10x_INT_BOUND] = {(stm32f10x_int_cb_t)0};

s8 plat_int_reg_cb(enum stm32f10x_int_type type, void *cb)
{
    if( (type >= STM32F10x_INT_BOUND) || (!cb) )
        return -1;
    stm32f10x_int_cb_tbl[type] = (stm32f10x_int_cb_t)cb;

    return 0;
}

s8 plat_int_dereg_cb(enum stm32f10x_int_type type)
{
    if( type >= STM32F10x_INT_BOUND )
        return -1;
    stm32f10x_int_cb_tbl[type] = (stm32f10x_int_cb_t)0;

    return 0;
}

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
//    stm32f10x_int_cb_tbl[STM32F10x_INT_NMI]();
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
//    stm32f10x_int_cb_tbl[STM32F10x_INT_SVC]();
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
//    stm32f10x_int_cb_tbl[STM32F10x_INT_DEBUGMON]();
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
//    stm32f10x_int_cb_tbl[STM32F10x_INT_PENDSV]();
}

/*******************************************************************************
* ��������  : SysTickHandler (1ms�ж�һ��)
* ��������  : This function handles SysTick Handler.
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void SysTick_Handler(void)
{
    if(stm32f10x_int_cb_tbl[STM32F10x_INT_SYSTICK]){
        stm32f10x_int_cb_tbl[STM32F10x_INT_SYSTICK]();
    }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_ALR) != RESET){
        RTC_ClearITPendingBit(RTC_IT_ALR);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_RTC_ALR]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_RTC_ALR]();
        }
        RTC_WaitForLastTask();
    }
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET){
        RTC_ClearITPendingBit(RTC_IT_SEC);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_RTC_SEC]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_RTC_SEC]();
        }
        RTC_WaitForLastTask();
    }
}

/*******************************************************************************
* ��������  : TIM1_UP_IRQHandler
* ��������  : TIMER1�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update)==SET){
        TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_TIM1_UP]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_TIM1_UP]();
        }
    }
}
/*******************************************************************************
* ��������  : TIM2_IRQHandler
* ��������  : TIMER2�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2 , TIM_IT_Update)!= RESET){
        TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_TIM2]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_TIM2]();
        }
    }
}

/*******************************************************************************
* ��������  : TIM3_IRQHandler
* ��������  : TIMER3�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3 , TIM_IT_Update)!= RESET){
        TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_TIM3]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_TIM3]();
        }
    }
}

/*******************************************************************************
* ��������  : TIM4_IRQHandler
* ��������  : TIMER4�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4 , TIM_IT_Update)!= RESET){
        TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_TIM4]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_TIM4]();
        }
    }
}

/*******************************************************************************
* ��������  : TIM5_IRQHandler
* ��������  : TIMER5�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void TIM5_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM5 , TIM_IT_Update)!= RESET){
        TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_TIM5]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_TIM5]();
        }
    }
}

/*******************************************************************************
* ��������  : TIM6_IRQHandler
* ��������  : TIMER6�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void TIM6_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6 , TIM_IT_Update)!= RESET){
        TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_TIM6]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_TIM6]();
        }
    }
}

/*******************************************************************************
* ��������  : TIM7_IRQHandler
* ��������  : TIMER7�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM7 , TIM_IT_Update)!= RESET){
        TIM_ClearITPendingBit(TIM7 , TIM_FLAG_Update);
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_TIM7]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_TIM7]();
        }
    }
}

/*******************************************************************************
* ��������  : USART1_IRQHandler
* ��������  : USART1�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
    if(stm32f10x_int_cb_tbl[STM32F10x_INT_USART1]){
        stm32f10x_int_cb_tbl[STM32F10x_INT_USART1]();
    }
}

/*******************************************************************************
* ��������  : USART2_IRQHandler
* ��������  : USART2�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
    if(stm32f10x_int_cb_tbl[STM32F10x_INT_USART2]){
        stm32f10x_int_cb_tbl[STM32F10x_INT_USART2]();
    }
}

/*******************************************************************************
* ��������  : UART4_IRQHandler
* ��������  : UART4�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void UART4_IRQHandler(void)
{
    if(stm32f10x_int_cb_tbl[STM32F10x_INT_UART4]){
        stm32f10x_int_cb_tbl[STM32F10x_INT_UART4]();
    }
}

/*******************************************************************************
* ��������  : EXTI0_IRQHandler
* ��������  : �ⲿ�жϿ���0�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI0]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI0]();
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
/*******************************************************************************
* ��������  : EXTI1_IRQHandler
* ��������  : �ⲿ�жϿ���0�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI1]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI1]();
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
/*******************************************************************************
* ��������  : EXTI2_IRQHandler
* ��������  : �ⲿ�жϿ���0�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI2]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI2]();
        }
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
/*******************************************************************************
* ��������  : EXTI0_IRQHandler
* ��������  : �ⲿ�жϿ���0�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI3]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI3]();
        }
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
/*******************************************************************************
* ��������  : EXTI4_IRQHandler
* ��������  : �ⲿ�жϿ���0�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI4]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI4]();
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}
/*******************************************************************************
* ��������  : EXTI15_10_IRQHandler
* ��������  : �ⲿ�жϿ���14�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_5]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_5]();
        }
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
    if(EXTI_GetITStatus(EXTI_Line6) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_6]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_6]();
        }
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
    if(EXTI_GetITStatus(EXTI_Line7) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_7]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_7]();
        }
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
    if(EXTI_GetITStatus(EXTI_Line8) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_8]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_8]();
        }
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
    if(EXTI_GetITStatus(EXTI_Line9) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_9]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI9_5_9]();
        }
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

/*******************************************************************************
* ��������  : EXTI15_10_IRQHandler
* ��������  : �ⲿ�жϿ���14�жϷ����ӳ���
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line10) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_10]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_10]();
        }
        EXTI_ClearITPendingBit(EXTI_Line10);
    }
    if(EXTI_GetITStatus(EXTI_Line11) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_11]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_11]();
        }
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
    if(EXTI_GetITStatus(EXTI_Line12) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_12]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_12]();
        }
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
    if(EXTI_GetITStatus(EXTI_Line13) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_13]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_13]();
        }
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
    if(EXTI_GetITStatus(EXTI_Line14) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_14]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_14]();
        }
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
    if(EXTI_GetITStatus(EXTI_Line15) != RESET){
        if(stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_15]){
            stm32f10x_int_cb_tbl[STM32F10x_INT_EXTI15_10_15]();
        }
        EXTI_ClearITPendingBit(EXTI_Line15);
    }
}

/*******************************************************************************
* ��������  : USB_LP_CAN1_RX0_IRQHandler
* ��������  : USB�����ж�
* �������  : None
* �������  : None
* ���ز���  : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if(stm32f10x_int_cb_tbl[STM32F10x_INT_USB_LP_CAN1_RX0]){
        stm32f10x_int_cb_tbl[STM32F10x_INT_USB_LP_CAN1_RX0]();
    }
}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
