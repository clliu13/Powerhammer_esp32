#include <reg52.h>
#include "string.h"
#include "spi.h"
#include "exti.h"
#include "usart.h"
/*
Keil:uVision2
MCU:STC90C516RD+
本程序采用SPI读取GY-953模块
硬件接法：P3^3-INT;P3^4-CS;P3^5-SCK;P3^6-MISO;P3^7-MOSI;51_RX--ft232TX;51_TX--ft232RX
外接晶振11.0592M
程序说明：
由于GY-953数据更新时,为低电平,所以待出现上升沿时为数据更新,而51MCU不支持
上升沿中断,所以本程序在INT出现下降沿后待INT出现高电平表示一个上升沿,然后
开始读取0x01-0x24的寄存器(共36个),本程序通过串口接收输出指令,可上传给
上位机直接显示,程序支持上位机'加陀校准'、'磁场校准'、'保存数据'、'读取量程'
'精度、频率'按钮功能
注:中断处理程序位于stc_it.c
在编译此程序前，请先设置一下文件路径，设置方法：
1、点击Project;2、点击Options for Target 'GYXX_SPI';3、点击C51；4、在include Paths
找到程序文件夹的路径，把..\GYXX_SPI_51\Peripherals\inc\加入即可，例如
F:\ARM\GYXX_SPI_51\Peripherals\inc\
该程序仅做参考，如有问题请联系我们
http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.HuqW76&v=1
版本:GY953.V2
*/
int main(void)
{
	uint8_t DATA_ok=0,Data=0,sum=0,i=0;	
	int16_t ROLL=0,PITCH=0,YAW=0;
    int16_t Q[4]={0};
	Spi_Int();//spi初始化
	Usart_Int(9600);//串口初始化，波特率9600
	Exti_Int(1,1);//外部中断1，下降沿触发
	IP=0x10;//串口优先级高，外部中断优先级低
	//开所有传感器，输出频率设置成100hz（默认开所有传感器，输出50hz，不能关所有传感器）
	//即写成0x04，并不关闭所有传感器，这样也是开所有传感器，输出频率为100hz
	while(!BIT);
	Data=0x74|RegisterA;
	Spi_write_buf(0x41,&Data,1);
	while(1)
	{	//INT为低电平时表示数据正在更新，所以在上升沿后表示更新完毕，可读取
		if(BIT&&(P3^3==1))
		{
		for(i=0;i<36;i++)
		 Spi_read_buf(0x81+i,&ACM_BUF[i],1);//读取数据
		 DATA_ok=1;	//读取标志置一
		 BIT=0;
		}
		if(DATA_ok)//数据读取完毕
	  {
		 LED_0=1;//指示灯灭
		 DATA_ok=0;	//清标志
		 CHeck(ACM_BUF);//串口指令检查
	    if(stata_reg&RPY)//欧拉角输出
		{
			sum=0;
			for(i=0;i<6;i++)
				sum+=ACM_BUF[20+i];
		  Spi_read_buf((0x80|0x28),&Data,1);//读取角度校验和数据
		  if(sum==Data)//判断收到的数据校验和和读取的是否相同，减少spi通讯错误
		 {	
			  ROLL=(ACM_BUF[20]<<8)|ACM_BUF[21];
			  PITCH=(ACM_BUF[22]<<8)|ACM_BUF[23];
			  YAW=(ACM_BUF[24]<<8)|ACM_BUF[25];
			  Q[0]=(uint16_t)ROLL;
			  Q[1]=(uint16_t)PITCH;
			  Q[2]=(uint16_t)YAW;
			  send_out(Q,3,0x45);
		 }
		}
		if(stata_reg&ACC)//加数度数据输出
		{
		   Q[0]=(ACM_BUF[2]<<8)|ACM_BUF[3];//ACC_X
		   Q[1]=(ACM_BUF[4]<<8)|ACM_BUF[5];//ACC_Y
		   Q[2]=(ACM_BUF[6]<<8)|ACM_BUF[7];//ACC_Z
		   send_out(Q,3,0x15);
		}
		if(stata_reg&GYR)//陀螺数据输出
		{
			Q[0]=(ACM_BUF[8]<<8)|ACM_BUF[9];//GYR_X
			Q[1]=(ACM_BUF[10]<<8)|ACM_BUF[11];//GYR_Y
			Q[2]=(ACM_BUF[12]<<8)|ACM_BUF[13];//GYR_Z
			send_out(Q,3,0x25);
		}
		if(stata_reg&MAG)//磁场数据输出
		{
			Q[0]=(ACM_BUF[14]<<8)|ACM_BUF[15];//MAG_X
			Q[1]=(ACM_BUF[16]<<8)|ACM_BUF[17];//MAG_Y
			Q[2]=(ACM_BUF[18]<<8)|ACM_BUF[19];//MAG_Z
			send_out(Q,3,0x35);
		}
		if(stata_reg&Q4)//四元数数据输出
		{
		    Q[0]=(ACM_BUF[26]<<8)|ACM_BUF[27];//q0
			Q[1]=(ACM_BUF[28]<<8)|ACM_BUF[29];//q1
			Q[2]=(ACM_BUF[30]<<8)|ACM_BUF[31];//q2
			Q[3]=(ACM_BUF[32]<<8)|ACM_BUF[33];//q3
			send_out(Q,4,0x65);
		}
		EX1=1;//数据处理完毕，开外部中断
	   }
			
	}
}
