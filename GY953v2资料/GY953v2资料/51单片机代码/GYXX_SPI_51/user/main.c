#include <reg52.h>
#include "string.h"
#include "spi.h"
#include "exti.h"
#include "usart.h"
/*
Keil:uVision2
MCU:STC90C516RD+
���������SPI��ȡGY-953ģ��
Ӳ���ӷ���P3^3-INT;P3^4-CS;P3^5-SCK;P3^6-MISO;P3^7-MOSI;51_RX--ft232TX;51_TX--ft232RX
��Ӿ���11.0592M
����˵����
����GY-953���ݸ���ʱ,Ϊ�͵�ƽ,���Դ�����������ʱΪ���ݸ���,��51MCU��֧��
�������ж�,���Ա�������INT�����½��غ��INT���ָߵ�ƽ��ʾһ��������,Ȼ��
��ʼ��ȡ0x01-0x24�ļĴ���(��36��),������ͨ�����ڽ������ָ��,���ϴ���
��λ��ֱ����ʾ,����֧����λ��'����У׼'��'�ų�У׼'��'��������'��'��ȡ����'
'���ȡ�Ƶ��'��ť����
ע:�жϴ������λ��stc_it.c
�ڱ���˳���ǰ����������һ���ļ�·�������÷�����
1�����Project;2�����Options for Target 'GYXX_SPI';3�����C51��4����include Paths
�ҵ������ļ��е�·������..\GYXX_SPI_51\Peripherals\inc\���뼴�ɣ�����
F:\ARM\GYXX_SPI_51\Peripherals\inc\
�ó�������ο���������������ϵ����
http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.HuqW76&v=1
�汾:GY953.V2
*/
int main(void)
{
	uint8_t DATA_ok=0,Data=0,sum=0,i=0;	
	int16_t ROLL=0,PITCH=0,YAW=0;
    int16_t Q[4]={0};
	Spi_Int();//spi��ʼ��
	Usart_Int(9600);//���ڳ�ʼ����������9600
	Exti_Int(1,1);//�ⲿ�ж�1���½��ش���
	IP=0x10;//�������ȼ��ߣ��ⲿ�ж����ȼ���
	//�����д����������Ƶ�����ó�100hz��Ĭ�Ͽ����д����������50hz�����ܹ����д�������
	//��д��0x04�������ر����д�����������Ҳ�ǿ����д����������Ƶ��Ϊ100hz
	while(!BIT);
	Data=0x74|RegisterA;
	Spi_write_buf(0x41,&Data,1);
	while(1)
	{	//INTΪ�͵�ƽʱ��ʾ�������ڸ��£������������غ��ʾ������ϣ��ɶ�ȡ
		if(BIT&&(P3^3==1))
		{
		for(i=0;i<36;i++)
		 Spi_read_buf(0x81+i,&ACM_BUF[i],1);//��ȡ����
		 DATA_ok=1;	//��ȡ��־��һ
		 BIT=0;
		}
		if(DATA_ok)//���ݶ�ȡ���
	  {
		 LED_0=1;//ָʾ����
		 DATA_ok=0;	//���־
		 CHeck(ACM_BUF);//����ָ����
	    if(stata_reg&RPY)//ŷ�������
		{
			sum=0;
			for(i=0;i<6;i++)
				sum+=ACM_BUF[20+i];
		  Spi_read_buf((0x80|0x28),&Data,1);//��ȡ�Ƕ�У�������
		  if(sum==Data)//�ж��յ�������У��ͺͶ�ȡ���Ƿ���ͬ������spiͨѶ����
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
		if(stata_reg&ACC)//�������������
		{
		   Q[0]=(ACM_BUF[2]<<8)|ACM_BUF[3];//ACC_X
		   Q[1]=(ACM_BUF[4]<<8)|ACM_BUF[5];//ACC_Y
		   Q[2]=(ACM_BUF[6]<<8)|ACM_BUF[7];//ACC_Z
		   send_out(Q,3,0x15);
		}
		if(stata_reg&GYR)//�����������
		{
			Q[0]=(ACM_BUF[8]<<8)|ACM_BUF[9];//GYR_X
			Q[1]=(ACM_BUF[10]<<8)|ACM_BUF[11];//GYR_Y
			Q[2]=(ACM_BUF[12]<<8)|ACM_BUF[13];//GYR_Z
			send_out(Q,3,0x25);
		}
		if(stata_reg&MAG)//�ų��������
		{
			Q[0]=(ACM_BUF[14]<<8)|ACM_BUF[15];//MAG_X
			Q[1]=(ACM_BUF[16]<<8)|ACM_BUF[17];//MAG_Y
			Q[2]=(ACM_BUF[18]<<8)|ACM_BUF[19];//MAG_Z
			send_out(Q,3,0x35);
		}
		if(stata_reg&Q4)//��Ԫ���������
		{
		    Q[0]=(ACM_BUF[26]<<8)|ACM_BUF[27];//q0
			Q[1]=(ACM_BUF[28]<<8)|ACM_BUF[29];//q1
			Q[2]=(ACM_BUF[30]<<8)|ACM_BUF[31];//q2
			Q[3]=(ACM_BUF[32]<<8)|ACM_BUF[33];//q3
			send_out(Q,4,0x65);
		}
		EX1=1;//���ݴ�����ϣ����ⲿ�ж�
	   }
			
	}
}
