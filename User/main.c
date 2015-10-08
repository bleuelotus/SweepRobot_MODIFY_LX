/**
  ******************************************************************************
  * @file    main.c
  * @author  Reason Chen
  * @version V1.0.0
  * @date    5-May-2015
  * @brief   main program body
  ******************************************************************************
  */
#include <stdlib.h>
#include "stm32f10x.h"
#include <stdio.h>
#include "Usart.h"
#include "delay.h"
#include "boardcfg.h"
#include "SweepRobot.h"

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    /* Use systick as delay  */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    /* Priority Group devide */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

#ifdef DEBUG_LOG
    UART4_Config();
#endif

    if(SweepRobot_Init())
        goto HALT; 

    SweepRobot_Start();

HALT:
    while (1)
    {
    }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
