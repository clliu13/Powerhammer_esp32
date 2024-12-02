#include "stc_it.h"
#include "usart.h"
#include "exti.h"
#include "spi.h"
//外部中断1
uint8_t stata=0;
void EXTI1_IRQHandler(void)interrupt 2
{
	uint8_t P3=0;
	if(!BIT)//数据处理完毕BIT=0
	{
		BIT=1;//下降沿标志
		EX1=0;//关闭外部中断
		LED_0=0;//指示灯亮
	}
}
//串口中断
void USART_IRQHandler(void)interrupt 4
{
	static uint8_t i=0,rebuf[3]={0};
	uint8_t sum=0;
    if(TI)//发送完成标志
	{
	  TI=0;//清发送完成标志
	  send_ok=0;//缓存标志置0 
	}
	if(RI)//接收完成标志
	{
		rebuf[i++]=SBUF;
		RI=0;//清中断接收标志
		if(!(rebuf[0]==0xa5))//判断帧头
		i=0;
		if(i==3)//判断功能字节
	 {
		sum=(uint8_t)(rebuf[1]+rebuf[0]);
		if(sum==rebuf[2])
		{
			switch(rebuf[1])
			{
				case 0x45:stata_reg^=RPY;break;//输出欧拉角指令
				case 0x15:stata_reg^=ACC;break;//输出加速度数据指令
				case 0x25:stata_reg^=GYR;break;//输出陀螺仪数据指令
				case 0x35:stata_reg^=MAG;break;//输出磁场数据指令
				case 0x65:stata_reg^=Q4;break;//输出四元数数据指令
				case 0xa4:KEY=1;break;//模块输出频率50hz设置
				case 0xa5:KEY=2;break;//模块输出频率100hz设置
				case 0xa6:KEY=3;break;//模块输出频率200hz设置
				case 0x57:KEY=4;break;//加陀校准
				case 0x58:KEY=5;break;//磁场校准
				case 0x59:KEY=6;break;//恢复出厂设置，即清除保存的校准数据
				case 0x75:read_key=1;break;//读取校准精度
				case 0x85:read_key=read_key|2;break;//读取传感器量程
				case 0x50:KEY=7;break;//开启所有传感器
				case 0x51:KEY=8;break;//关闭加计
				case 0x52:KEY=9;break;//关闭陀螺
				case 0x53:KEY=10;break;//关闭磁场
				default:KEY=0,read_key=0;break;	
			}
		}
		i=0;//清缓存计数
	  }	
	}

}
