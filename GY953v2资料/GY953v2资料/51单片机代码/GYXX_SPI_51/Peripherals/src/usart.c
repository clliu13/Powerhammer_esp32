#include "usart.h"
#include "string.h"
#include "spi.h"
#include "exti.h"

void Usart_Int(uint32_t BaudRatePrescaler)
{
	 SCON  = 0x50;//1位起始位，8位数据位，1位停止位，异步，使能接收
	 TMOD |= 0x20;//定时器1，工作方式2，8位自动重装计数值
	//例9600，28800/9600=3,TH1=253=FD；
     TH1   =256-(28800/BaudRatePrescaler); 
	 TL1   =256-(28800/BaudRatePrescaler);
     TCON |=0x40;//启动定时器1
	 IE   |=0x90;//打开串口中断，MCU总中断
}
//发送一个字节
uint8_t send_ok=0;
void USART_send_byte(uint8_t Tx_data)
{
	while(send_ok);//等待发送缓存为空
 	SBUF = Tx_data;
	send_ok=1;//缓存标志置1
}
//发送Length-1个数据+1个数据累加和
void USART_Send(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];
		USART_send_byte(Buffer[i++]);
	}
}
//发送一帧数据
void send_out(int16_t *Data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[13],i=0,k=0;
	memset(TX_DATA,0,(2*length+5));//清空发送数据
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=send;//功能字节
	TX_DATA[i++]=2*length;//数据个数
	for(k=0;k<length;k++)//存入数据到缓存TX_DATA数组
	{
		TX_DATA[i++]=(uint16_t)Data[k]>>8;
		TX_DATA[i++]=(uint16_t)Data[k];
	}
	USART_Send(TX_DATA,(2*length+5));//发送一帧数据	
}
//简单粗暴延时
void delay(uint16_t x)
{
	while(x--);
}
//检查串口指令
uint8_t KEY=0,read_key=0,stata_reg=0;

void CHeck(uint8_t *re_data)
{
	uint16_t num[4]={0};
	uint8_t Data;
	switch(read_key)
	{
		case 1://发送精度
		{
			num[0]=((re_data[35]>>4)&0x03);
			num[1]=((re_data[35]>>2)&0x03);
			num[2]=(re_data[35]&0x03);
			num[3]=(re_data[0]&0x07);
			send_out(num,4,0x75);
			read_key=0;
		}
			break;
		case 2://发送量程
		{
			num[0]=((re_data[34]>>4)&0x03);
			num[1]=((re_data[34]>>2)&0x03);
			num[2]=(re_data[34]&0x03);
			send_out(num,3,0x85);
			read_key=0;
		}
			
		break;
		default:read_key=0;break;
	}
	switch(KEY)
	{
		case 1:Data=0x73|RegisterA;Spi_write_buf(0x41,&Data,1);KEY=0;break;//50hz
		case 2:Data=0x74|RegisterA;Spi_write_buf(0x41,&Data,1);KEY=0;break;//100hz
		case 3:Data=0x75|RegisterA;Spi_write_buf(0x41,&Data,1);KEY=0;break;//200hz
		case 4:Data=0X04|RegisterB;Spi_write_buf(0x42,&Data,1);KEY=0;break;//加陀校准
		case 5:Data=0x08|RegisterB;Spi_write_buf(0x42,&Data,1);KEY=0;break;//磁场校准
		case 6:Data=0x80|RegisterB;Spi_write_buf(0x42,&Data,1);KEY=0;break;//恢复出厂设置
		case 7:Data=0x73|RegisterA;Spi_write_buf(0x41,&Data,1);KEY=0;break;//全开传感器
		case 8:Data=0x63|RegisterA;Spi_write_buf(0x41,&Data,1);KEY=0;break;//关加计
		case 9:Data=0x53|RegisterA;Spi_write_buf(0x41,&Data,1);KEY=0;break;//关陀螺
		case 10:;Data=0x33|RegisterA;Spi_write_buf(0x41,&Data,1);KEY=0;break;//关磁场
	  default:KEY=0;break;
	}
}

