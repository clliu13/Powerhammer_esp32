#include "usart.h"
#include "string.h"
#include "spi.h"
void Usart_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	  //USART1_TX   PA.9
  GPIO_usartx.GPIO_Pin = GPIO_Pin_9;
  GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_usartx); 
  //USART1_RX	  PA.10
  GPIO_usartx.GPIO_Pin = GPIO_Pin_10;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_usartx); 
	
	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;//8位数据格式
	Usart_X.USART_StopBits=USART_StopBits_1;//1位停止位
	Usart_X.USART_Parity = USART_Parity_No;//无校验
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &Usart_X);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启接收中断
  USART_Cmd(USART1, ENABLE);
}
//发送一个字节数据
//input:byte,待发送的数据
void USART_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//等待发送完成
	USART1->DR=byte;	
}
//发送多字节数据
void USART_Send_bytes(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART_send_byte(Buffer[i++]);
	}
}
//发送多字节数据+校验和
void USART_Send(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];//累加Length-1前的数据
		USART_send_byte(Buffer[i++]);
	}
}
//发送一帧数据
void send_out(int16_t *data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[20],i=0,k=0;
	memset(TX_DATA,0,(2*length+5));//清零缓存TX_DATA
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=send;//功能字节
	TX_DATA[i++]=2*length;//数据个数
	for(k=0;k<length;k++)//存入数据到缓存TX_DATA
	{
		TX_DATA[i++]=(uint16_t)data[k]>>8;
		TX_DATA[i++]=(uint16_t)data[k];
	}
	USART_Send(TX_DATA,2*length+5);	
}
//KEY 功能指令传递；read_key 读取量程，精度标志;
//stata_reg输出控制寄存器
uint8_t KEY=0,read_key=0,stata_reg=0;
uint8_t ACM_BUF[41]={0},BIT=0;//ACM_BUF--- spi读取的数据缓存，BIT读取完成标志

void CHeck(uint8_t *re_data)
{
	int16_t num[4]={0};
	uint8_t data;
	switch(read_key)
	{
		case 1://精度
		{
			num[0]=((re_data[35]>>4)&0x03);//ACC精度
			num[1]=((re_data[35]>>2)&0x03);//GYRO精度
			num[2]=(re_data[35]&0x03);//MAG精度
			num[3]=(re_data[0]&0x07);//数据更新频率
			send_out(num,4,0x75);//加入上位机协议上传
			read_key=0;
		}
			break;
		case 2://量程
		{
			num[0]=((re_data[34]>>4)&0x03);
			num[1]=((re_data[34]>>2)&0x03);
			num[2]=(re_data[34]&0x03);
			send_out(num,3,0x85);//加入上位机协议上传
			read_key=0;
		}		
		break;
		default:read_key=0;break;
	}
	switch(KEY)
	{
		case 1:data=0x73|RegisterA;Spi_write_buf(0x41,&data,1);KEY=0;break;//50hz
		case 2:data=0x74|RegisterA;Spi_write_buf(0x41,&data,1);KEY=0;break;//100hz
		case 3:data=0x75|RegisterA;Spi_write_buf(0x41,&data,1);KEY=0;break;//200hz
		case 4:data=0x04|RegisterB;Spi_write_buf(0x42,&data,1);KEY=0;break;//加陀校准
		case 5:data=0x08|RegisterB;Spi_write_buf(0x42,&data,1);KEY=0;break;//磁场校准
		case 6:data=0x80|RegisterB;Spi_write_buf(0x41,&data,1);KEY=0;break;//恢复出厂设置
		case 7:data=0x73|RegisterA;Spi_write_buf(0x41,&data,1);KEY=0;break;//全开传感器
		case 8:data=0x63|RegisterA;Spi_write_buf(0x41,&data,1);KEY=0;break;//关加计
		case 9:data=0x53|RegisterA;Spi_write_buf(0x41,&data,1);KEY=0;break;//关陀螺
		case 10:data=0x33|RegisterA;Spi_write_buf(0x41,&data,1);KEY=0;break;//关磁场
	  default:KEY=0;break;
	}
}
