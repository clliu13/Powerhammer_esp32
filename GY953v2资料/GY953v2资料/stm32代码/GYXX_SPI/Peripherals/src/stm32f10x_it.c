/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usart.h"
#include "spi.h"

/** @addtogroup STM32F10x_StdPeriph_Template
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
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
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
  * @retval None
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
  * @retval None
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
  * @retval None
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
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}
////////////////
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
		if(BIT==0)
		{
			Spi_read_buf(0xc1,ACM_BUF,41);//��ȡGY953����
			BIT=1;
		}
    EXTI_ClearITPendingBit(EXTI_Line5);
  }
}

//////////////
void USART1_IRQHandler(void)
{
	static uint8_t i=0,rebuf[3]={0};
	uint8_t sum=0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		rebuf[i++]=USART_ReceiveData(USART1);
		if(!(rebuf[0]==0xa5))
		i=0;
		if(i==3)
		{
			sum=(uint8_t)(rebuf[0]+rebuf[1]);
			if(sum==rebuf[2])
			{	
				switch(rebuf[1])
				{
					case 0x45:stata_reg^=RPY;break;//�ϴ�ŷ����
					case 0x15:stata_reg^=ACC;break;//�ϴ����ٶ�
					case 0x25:stata_reg^=GYR;break;//�ϴ�������
					case 0x35:stata_reg^=MAG;break;//�ϴ��ų�
					case 0x65:stata_reg^=Q4;break;//�ϴ���Ԫ��
					case 0xa4:KEY=1;break;//50hz�������
					case 0xa5:KEY=2;break;//100hz�������
					case 0xa6:KEY=3;break;//200hz�������
					case 0x57:KEY=4;break;//����У׼
					case 0x58:KEY=5;break;//�ų�У׼
					case 0x59:KEY=6;break;//�ָ��������ã�����ձ����У׼����
					case 0x75:read_key=1;break;//��ȡУ׼����
					case 0x85: read_key=2;break;//��ȡ����
					case 0x50:KEY=7;break;//�������д�����
					case 0x51:KEY=8;break;//�رռӼ�
					case 0x52:KEY=9;break;//�ر�������
					case 0x53:KEY=10;break;//�رմų�
					default:KEY=0;read_key=0;break;	
				}
			}
			i=0;
		}
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
	}	
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
