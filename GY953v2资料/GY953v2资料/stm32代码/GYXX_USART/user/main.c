#include "stm32f10x.h"
#include "string.h"
#include "delay.h"
#include "LED.h"
#include "usart.h"
/*
Keil: MDK5.10.0.2
MCU:stm32f103c8
本程序采用串口1读取GY-953模块数据
硬件接法：
GY-953_TX--STM32_RX
STM32_TX--FT232RX
当把该程序下载进stm32后，再将stm32的TX引脚接到GY-953模块的RX引脚，
然后复位stm32，复位成功后，该程序会发送输出指令给模块，然后将stm32
的TX接到FT232串口模块的RX引脚，打开上位机，选择波特率115200，选择
型号GYXX，点击打开串口按钮，此时，便能看到上传的数据
程序说明：
程序将对外发送一次输出指令，在串口接收中断接收GY-953模块输出的数据
，在主循环处理接收到的数据并发送到上位机显示，该程序支持GY-953串口
数据的读取
注：中断处理程序位于stm32f10x_it.c
该程序仅做参考，如有问题请与我们联系
http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.HuqW76&v=1
版本:2015.1.26
*/
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_X;
  
  /* 4个抢占优先级，4个响应优先级 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*抢占优先级可打断中断级别低的中断*/
	/*响应优先级按等级执行*/
	NVIC_X.NVIC_IRQChannel = USART1_IRQn;//中断向量
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//响应优先级
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//使能中断响应
  NVIC_Init(&NVIC_X);
}
void send_Instruction(void)
{
	uint8_t send_data[3]={0};
	send_data[0]=0xa5;
	send_data[1]=0x15;//加计功能字节
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);//指令校验和
	USART_Send_bytes(send_data,3);//发送加速度输出指令
	delay_ms(1);
	send_data[0]=0xa5;
	send_data[1]=0x25;
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);
	USART_Send_bytes(send_data,3);//发送陀螺仪数据输出指令
	delay_ms(1);
	send_data[0]=0xa5;
	send_data[1]=0x35;
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);
	USART_Send_bytes(send_data,3);//发送磁场数据输出指令
	delay_ms(1);
	send_data[0]=0xa5;
	send_data[1]=0x45;
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);
	USART_Send_bytes(send_data,3);//发送欧拉角数据输出指令
	delay_ms(1);
	send_data[0]=0xa5;
	send_data[1]=0x65;
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);
	USART_Send_bytes(send_data,3);//发送四元数数据输出指令
}
int main(void)
{
	uint8_t data_buf[9]={0};
	int16_t ROLL=0,PITCH=0,YAW=0;
	int16_t rpy[3]={0},Acc[3]={0},Gyr[3]={0},Mag[3]={0},Q[4]={0};
	delay_init(72);//72M 
	LED_Int(GPIOB,GPIO_Pin_9,RCC_APB2Periph_GPIOB);//led
	Usart_Int(115200);//115200波特率
	NVIC_Configuration();//串口中断优先级配置
	send_Instruction();//向模块发送5个指令
	GPIO_SetBits(GPIOB,GPIO_Pin_9);
	while(1)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_9);//LED灭
		switch(CHeck(data_buf))//检查有无数据接收完毕，并返回输出指令
		{
			case 0x15:{//加数度数据输出
				  Acc[0]=(data_buf[0]<<8)|data_buf[1];
					Acc[1]=(data_buf[2]<<8)|data_buf[3];
					Acc[2]=(data_buf[4]<<8)|data_buf[5];
					send_out(Acc,3,0x15);
			}
			break;
			case 0x25:{//陀螺仪数据输出
				  Gyr[0]=(data_buf[0]<<8)|data_buf[1];
					Gyr[1]=(data_buf[2]<<8)|data_buf[3];
					Gyr[2]=(data_buf[4]<<8)|data_buf[5];
					send_out(Gyr,3,0x25);
			}
				break;
			case 0x35:{//磁场数据输出
				  Mag[0]=(data_buf[0]<<8)|data_buf[1];
					Mag[1]=(data_buf[2]<<8)|data_buf[3];
					Mag[2]=(data_buf[4]<<8)|data_buf[5];
					send_out(Mag,3,0x35);
			}
				break;
			case 0x45:{//欧拉角数据输出
				  ROLL=(data_buf[0]<<8)|data_buf[1];
					PITCH=(data_buf[2]<<8)|data_buf[3];
					YAW=(data_buf[4]<<8)|data_buf[5];
					rpy[0]=ROLL;
				  rpy[1]=PITCH;
				  rpy[2]=YAW;
					send_out(rpy,3,0x45);
			}
				break;
			case 0x65:{//四元数数据输出
				  Q[0]=(data_buf[0]<<8)|data_buf[1];
					Q[1]=(data_buf[2]<<8)|data_buf[3];
					Q[2]=(data_buf[4]<<8)|data_buf[5];
					Q[3]=(data_buf[6]<<8)|data_buf[7];
					send_out(Q,4,0x65);
			}
				break;
			default:break;
		}
	}
}
