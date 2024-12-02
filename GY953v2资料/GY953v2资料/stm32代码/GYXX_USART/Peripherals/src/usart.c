#include "usart.h"
#include "string.h"
void Usart_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	  //USART1_TX   PA.9
  GPIO_usartx.GPIO_Pin = GPIO_Pin_9;
  GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
  GPIO_Init(GPIOA, &GPIO_usartx); 
  //USART1_RX	  PA.10
  GPIO_usartx.GPIO_Pin = GPIO_Pin_10;
  GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_usartx); 
	
	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;//8λ���ݸ�ʽ
	Usart_X.USART_StopBits=USART_StopBits_1;//1λֹͣλ
	Usart_X.USART_Parity = USART_Parity_No;//��У��
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &Usart_X);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//���������ж�
  USART_Cmd(USART1, ENABLE);
}
//����һ���ֽ�����
//input:byte,�����͵�����
void USART_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//�ȴ��������
	USART1->DR=byte;	
}
//���Ͷ��ֽ�����
void USART_Send_bytes(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART_send_byte(Buffer[i++]);
	}
}
//���Ͷ��ֽ�����+У���
void USART_Send(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];//�ۼ�Length-1ǰ������
		USART_send_byte(Buffer[i++]);
	}
}
//����һ֡����
void send_out(int16_t *data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[20],i=0,k=0;
	memset(TX_DATA,0,(2*length+5));//���㻺��TX_DATA
	TX_DATA[i++]=0X5A;//֡ͷ
	TX_DATA[i++]=0X5A;//֡ͷ
	TX_DATA[i++]=send;//�����ֽ�
	TX_DATA[i++]=2*length;//���ݳ���
	for(k=0;k<length;k++)//�������ݵ�����TX_DATA
	{
		TX_DATA[i++]=(uint16_t)data[k]>>8;
		TX_DATA[i++]=(uint16_t)data[k];
	}
	USART_Send(TX_DATA,sizeof(TX_DATA));	
}
uint8_t stata_reg=0;//����״̬�Ĵ���
//5�����ݻ��棬��ӦAcc-Q�����泤��Ϊ7������6������+1��У�������
//���泤��Ϊ9������8����Ԫ������+1��У�������
uint8_t data1_buf[7]={0},data2_buf[7]={0},data3_buf[7]={0},
data4_buf[7]={0},data5_buf[9]={0};
uint8_t CHeck(uint8_t *data)
{
  uint8_t i=0,flag=0,length=0,sum=0x5a+0x5a;
	if(stata_reg)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);//LED��
		if(stata_reg&ACC)//�жϼ��������ݽ������
		{
			stata_reg^=ACC;//���־λ
			flag=0x15;//��¼�����ֽ�
			length=6;//��¼���ݳ���
			/*ת�����ݵ�data�����⴮���ж϶�data1_buf��Ӱ��*/
		  memcpy(data,data1_buf,sizeof(data1_buf));
		  sum=sum+flag+length;//�ۼ�֡ͷ�������ֽڡ����ݳ���
		}
		else if(stata_reg&GYR)
		{
			stata_reg^=GYR;
			flag=0x25;
			length=6;
		  memcpy(data,data2_buf,sizeof(data2_buf));
		  sum=sum+flag+length;
		}
		else if(stata_reg&MAG)
		{
			stata_reg^=MAG;
			flag=0x35;
			length=6;
		  memcpy(data,data3_buf,sizeof(data3_buf));
		  sum=sum+flag+length;
		}
		else if(stata_reg&RPY)
		{
			stata_reg^=RPY;
			flag=0x45;
			length=6;
		  memcpy(data,data4_buf,sizeof(data4_buf));
		  sum=sum+flag+length;
		}
		else if(stata_reg&Q4)
		{
			stata_reg^=Q4;
			flag=0x65;
			length=8;
		  memcpy(data,data5_buf,sizeof(data5_buf));
		  sum=sum+flag+length;
		}
		for(i=0;i<length;i++)//�ۼ�����
		{
		 sum+=data[i];
		}
		if(sum!=data[i])//�ж�У����Ƿ���ȷ
		return 0;
		else
			return flag;//���ع����ֽ�	
	}
	else
		return 0;
}

