#include "stm32f10x.h"
#include "string.h"
#include "math.h"
#include "LED.h"
#include "spi.h"
#include "exti.h"
#include "usart.h"
#include "delay.h"
/*
Keil: MDK5.10.0.2
MCU:stm32f103c8
本程序采用SPI读取GY-953模块
硬件接法：
将STM32的硬件SPI1的对应引脚接到模块，即PA5接到sck，PA6接到MISO,PA7接到
MOSI,PA2接到CS，PB5接到INT,RX--ft232TX;TX--ft232RX
程序说明：
通过数据更新引脚INT上升沿触发（GPIOB，GPIO_Pin_5）外部中断进行spi读取
0x01-0x29的所有寄存器（共41个），然后通过串口中断接收指令，对接收的数据
处理并将通过spi读取到的数据发送到上位机显示，该程序支持所有只读寄存器的
读取，支持上位机'加陀校准'、'磁场校准'、'读取量程'、‘精度、频率’按钮功能
注：中断处理程序位于stm32f10x_it.c
角度、加计、陀螺、磁场数据类型为有符号数据（正负）
该程序仅做参考，如有问题请与我们联系
http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.HuqW76&v=1
版本:2015.1.26
*/
//配置中断优先级
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_X;
  
  /* 4个抢占优先级，4个响应优先级 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*抢占优先级可打断中断级别低的中断*/
	/*响应优先级按等级执行*/
  NVIC_X.NVIC_IRQChannel = EXTI9_5_IRQn;//中断向量
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//响应优先级
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//使能中断响应
  NVIC_Init(&NVIC_X);
	
	NVIC_X.NVIC_IRQChannel = USART1_IRQn;//中断向量
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//响应优先级
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//使能中断响应
  NVIC_Init(&NVIC_X);
}
int main(void)
{
	int16_t ROLL=0,PITCH=0,YAW=0;
	int16_t rpy[3]={0},Acc[3]={0},Gyr[3]={0},Mag[3]={0},Q[4]={0};
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	float gx, gy, gz;
	uint8_t data_buf[41]={0},data[2]={0},sum=0;//缓存
	delay_init(72);
	LED_Int(GPIOB,GPIO_Pin_9,RCC_APB2Periph_GPIOB);//led
	Usart_Int(115200);//串口波特率115200
	Spi1_Int(SPI_BaudRatePrescaler_256);//SPI分频256，72M/256
	Exti_Int(GPIOB,GPIO_Pin_5,GPIO_PortSourceGPIOB);//外部上升沿中断B5
	NVIC_Configuration();//中断优先级
	//开所有传感器，输出频率为100hz（默认开所有传感器，输出50hz）
	//不能关所有传感器，如0x04，也为开所有传感器，输出100hz
	BIT=0;
	while(!BIT);//等待GY953模块初始化完成
	delay_ms(500);
	data[0]=0x74|RegisterA;
	Spi_write_buf(0x41,&data[0],1);//向寄存器1写data[0],设置模块数据更新频率为100hz
	Spi_read_buf(0x81,&data[1],1);//读寄存器1
	if(data[1]==data[0])//判断是否配置成功
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);//LED亮
	while(1)
	{
		if(BIT)//接收完毕标志
		{
			 memcpy(data_buf,ACM_BUF , sizeof(ACM_BUF));//转移数据
			 BIT=0;//清标志
			 CHeck(data_buf);//检查串口指令
			//接收欧拉角数据并上传
				if(stata_reg&RPY)
				{
					//注：此时的角度为实际角度的100倍
					sum=0;
					for(uint8_t k=0;k<6;k++)
						sum+=data_buf[20+k];
					if(sum==data_buf[39])//判断校验和，减少spi通信错误
					{
						ROLL=(data_buf[20]<<8)|data_buf[21];
						PITCH=(data_buf[22]<<8)|data_buf[23];
						YAW=(data_buf[24]<<8)|data_buf[25];
						rpy[0]=ROLL;
						rpy[1]=PITCH;
						rpy[2]=YAW;
						send_out(rpy,3,0x45);//加入上位机协议上传
					}
					
				}
				//接收加计数据并上传
				if(stata_reg&ACC)
				{
					sum=0;
					for(uint8_t k=0;k<6;k++)
						sum+=data_buf[2+k];
					if(sum==data_buf[36])//判断校验和，减少spi通信错误
					{
						Acc[0]=(data_buf[2]<<8)|data_buf[3];
						Acc[1]=(data_buf[4]<<8)|data_buf[5];
						Acc[2]=(data_buf[6]<<8)|data_buf[7];
						send_out(Acc,3,0x15);//加入上位机协议上传
					}
					
				}
				//接收陀螺仪数据并上传
				if(stata_reg&GYR)
				{
					sum=0;
					for(uint8_t k=0;k<6;k++)
						sum+=data_buf[8+k];
					if(sum==data_buf[37])//判断校验和，减少spi通信错误
					{
						Gyr[0]=(data_buf[8]<<8)|data_buf[9];
						Gyr[1]=(data_buf[10]<<8)|data_buf[11];
						Gyr[2]=(data_buf[12]<<8)|data_buf[13];
						send_out(Gyr,3,0x25);
					}
				}
				//接收磁场数据并上传
				if(stata_reg&MAG)
				{
					sum=0;
					for(uint8_t k=0;k<6;k++)
						sum+=data_buf[14+k];
					if(sum==data_buf[38])//判断校验和，减少spi通信错误
					{
						Mag[0]=(data_buf[14]<<8)|data_buf[15];
						Mag[1]=(data_buf[16]<<8)|data_buf[17];
						Mag[2]=(data_buf[18]<<8)|data_buf[19];
						send_out(Mag,3,0x35);//加入上位机协议上传
					}
				}
				//接收四元数并上传
				if(stata_reg&Q4)
				{
					sum=0;
					for(uint8_t k=0;k<8;k++)
						sum+=data_buf[26+k];
					if(sum==data_buf[40])//判断校验和，减少spi通信错误
					{
						Q[0]=(data_buf[26]<<8)|data_buf[27];
						Q[1]=(data_buf[28]<<8)|data_buf[29];
						Q[2]=(data_buf[30]<<8)|data_buf[31];
						Q[3]=(data_buf[32]<<8)|data_buf[33];
						send_out(Q,4,0x65);//加入上位机协议上传
					}
				}
//				if(stata_reg&RPY)//接收四元数并计算欧拉角上传
//				{
//					Q0=(data_buf[24]<<8)|data_buf[25];
//					Q1=(data_buf[26]<<8)|data_buf[27];
//					Q2=(data_buf[28]<<8)|data_buf[29];
//					Q3=(data_buf[30]<<8)|data_buf[31];
//		
//					q0=Q0/10000.0f;
//					q1=Q1/10000.0f;
//					q2=Q2/10000.0f;
//					q3=Q3/10000.0f;

//					gx = 2 * (q1*q3 - q0*q2);
//					gy = 2 * (q0*q1 + q2*q3);
//					gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
//					ROLL=asin(-2 * q1 * q3+2 * q0* q2)* 5730;;
//					PITCH= (int16_t)(atan2(gy, -2 * q1 * q1 - 2 * q2* q2 + 1)* 5730);
//					YAW = (int16_t)(atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1)*5730);
//					rpy[0]=(uint16_t)ROLL;
//					rpy[1]=(uint16_t)PITCH;
//					rpy[2]=(uint16_t)YAW;
//					send_out(rpy,3,0x45);//加入上位机协议上传
//				}
			
			
		}
	}
	
}

