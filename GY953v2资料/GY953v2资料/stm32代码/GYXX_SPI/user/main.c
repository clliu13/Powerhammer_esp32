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
���������SPI��ȡGY-953ģ��
Ӳ���ӷ���
��STM32��Ӳ��SPI1�Ķ�Ӧ���Žӵ�ģ�飬��PA5�ӵ�sck��PA6�ӵ�MISO,PA7�ӵ�
MOSI,PA2�ӵ�CS��PB5�ӵ�INT,RX--ft232TX;TX--ft232RX
����˵����
ͨ�����ݸ�������INT�����ش�����GPIOB��GPIO_Pin_5���ⲿ�жϽ���spi��ȡ
0x01-0x29�����мĴ�������41������Ȼ��ͨ�������жϽ���ָ��Խ��յ�����
������ͨ��spi��ȡ�������ݷ��͵���λ����ʾ���ó���֧������ֻ���Ĵ�����
��ȡ��֧����λ��'����У׼'��'�ų�У׼'��'��ȡ����'�������ȡ�Ƶ�ʡ���ť����
ע���жϴ������λ��stm32f10x_it.c
�Ƕȡ��Ӽơ����ݡ��ų���������Ϊ�з������ݣ�������
�ó�������ο���������������������ϵ
http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.HuqW76&v=1
�汾:2015.1.26
*/
//�����ж����ȼ�
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_X;
  
  /* 4����ռ���ȼ���4����Ӧ���ȼ� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*��ռ���ȼ��ɴ���жϼ���͵��ж�*/
	/*��Ӧ���ȼ����ȼ�ִ��*/
  NVIC_X.NVIC_IRQChannel = EXTI9_5_IRQn;//�ж�����
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 1;//��ռ���ȼ�
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//��Ӧ���ȼ�
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж���Ӧ
  NVIC_Init(&NVIC_X);
	
	NVIC_X.NVIC_IRQChannel = USART1_IRQn;//�ж�����
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//��Ӧ���ȼ�
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж���Ӧ
  NVIC_Init(&NVIC_X);
}
int main(void)
{
	int16_t ROLL=0,PITCH=0,YAW=0;
	int16_t rpy[3]={0},Acc[3]={0},Gyr[3]={0},Mag[3]={0},Q[4]={0};
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	float gx, gy, gz;
	uint8_t data_buf[41]={0},data[2]={0},sum=0;//����
	delay_init(72);
	LED_Int(GPIOB,GPIO_Pin_9,RCC_APB2Periph_GPIOB);//led
	Usart_Int(115200);//���ڲ�����115200
	Spi1_Int(SPI_BaudRatePrescaler_256);//SPI��Ƶ256��72M/256
	Exti_Int(GPIOB,GPIO_Pin_5,GPIO_PortSourceGPIOB);//�ⲿ�������ж�B5
	NVIC_Configuration();//�ж����ȼ�
	//�����д����������Ƶ��Ϊ100hz��Ĭ�Ͽ����д����������50hz��
	//���ܹ����д���������0x04��ҲΪ�����д����������100hz
	BIT=0;
	while(!BIT);//�ȴ�GY953ģ���ʼ�����
	delay_ms(500);
	data[0]=0x74|RegisterA;
	Spi_write_buf(0x41,&data[0],1);//��Ĵ���1дdata[0],����ģ�����ݸ���Ƶ��Ϊ100hz
	Spi_read_buf(0x81,&data[1],1);//���Ĵ���1
	if(data[1]==data[0])//�ж��Ƿ����óɹ�
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);//LED��
	while(1)
	{
		if(BIT)//������ϱ�־
		{
			 memcpy(data_buf,ACM_BUF , sizeof(ACM_BUF));//ת������
			 BIT=0;//���־
			 CHeck(data_buf);//��鴮��ָ��
			//����ŷ�������ݲ��ϴ�
				if(stata_reg&RPY)
				{
					//ע����ʱ�ĽǶ�Ϊʵ�ʽǶȵ�100��
					sum=0;
					for(uint8_t k=0;k<6;k++)
						sum+=data_buf[20+k];
					if(sum==data_buf[39])//�ж�У��ͣ�����spiͨ�Ŵ���
					{
						ROLL=(data_buf[20]<<8)|data_buf[21];
						PITCH=(data_buf[22]<<8)|data_buf[23];
						YAW=(data_buf[24]<<8)|data_buf[25];
						rpy[0]=ROLL;
						rpy[1]=PITCH;
						rpy[2]=YAW;
						send_out(rpy,3,0x45);//������λ��Э���ϴ�
					}
					
				}
				//���ռӼ����ݲ��ϴ�
				if(stata_reg&ACC)
				{
					sum=0;
					for(uint8_t k=0;k<6;k++)
						sum+=data_buf[2+k];
					if(sum==data_buf[36])//�ж�У��ͣ�����spiͨ�Ŵ���
					{
						Acc[0]=(data_buf[2]<<8)|data_buf[3];
						Acc[1]=(data_buf[4]<<8)|data_buf[5];
						Acc[2]=(data_buf[6]<<8)|data_buf[7];
						send_out(Acc,3,0x15);//������λ��Э���ϴ�
					}
					
				}
				//�������������ݲ��ϴ�
				if(stata_reg&GYR)
				{
					sum=0;
					for(uint8_t k=0;k<6;k++)
						sum+=data_buf[8+k];
					if(sum==data_buf[37])//�ж�У��ͣ�����spiͨ�Ŵ���
					{
						Gyr[0]=(data_buf[8]<<8)|data_buf[9];
						Gyr[1]=(data_buf[10]<<8)|data_buf[11];
						Gyr[2]=(data_buf[12]<<8)|data_buf[13];
						send_out(Gyr,3,0x25);
					}
				}
				//���մų����ݲ��ϴ�
				if(stata_reg&MAG)
				{
					sum=0;
					for(uint8_t k=0;k<6;k++)
						sum+=data_buf[14+k];
					if(sum==data_buf[38])//�ж�У��ͣ�����spiͨ�Ŵ���
					{
						Mag[0]=(data_buf[14]<<8)|data_buf[15];
						Mag[1]=(data_buf[16]<<8)|data_buf[17];
						Mag[2]=(data_buf[18]<<8)|data_buf[19];
						send_out(Mag,3,0x35);//������λ��Э���ϴ�
					}
				}
				//������Ԫ�����ϴ�
				if(stata_reg&Q4)
				{
					sum=0;
					for(uint8_t k=0;k<8;k++)
						sum+=data_buf[26+k];
					if(sum==data_buf[40])//�ж�У��ͣ�����spiͨ�Ŵ���
					{
						Q[0]=(data_buf[26]<<8)|data_buf[27];
						Q[1]=(data_buf[28]<<8)|data_buf[29];
						Q[2]=(data_buf[30]<<8)|data_buf[31];
						Q[3]=(data_buf[32]<<8)|data_buf[33];
						send_out(Q,4,0x65);//������λ��Э���ϴ�
					}
				}
//				if(stata_reg&RPY)//������Ԫ��������ŷ�����ϴ�
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
//					send_out(rpy,3,0x45);//������λ��Э���ϴ�
//				}
			
			
		}
	}
	
}

