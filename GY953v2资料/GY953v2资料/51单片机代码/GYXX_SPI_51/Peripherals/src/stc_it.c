#include "stc_it.h"
#include "usart.h"
#include "exti.h"
#include "spi.h"
//�ⲿ�ж�1
uint8_t stata=0;
void EXTI1_IRQHandler(void)interrupt 2
{
	uint8_t P3=0;
	if(!BIT)//���ݴ������BIT=0
	{
		BIT=1;//�½��ر�־
		EX1=0;//�ر��ⲿ�ж�
		LED_0=0;//ָʾ����
	}
}
//�����ж�
void USART_IRQHandler(void)interrupt 4
{
	static uint8_t i=0,rebuf[3]={0};
	uint8_t sum=0;
    if(TI)//������ɱ�־
	{
	  TI=0;//�巢����ɱ�־
	  send_ok=0;//�����־��0 
	}
	if(RI)//������ɱ�־
	{
		rebuf[i++]=SBUF;
		RI=0;//���жϽ��ձ�־
		if(!(rebuf[0]==0xa5))//�ж�֡ͷ
		i=0;
		if(i==3)//�жϹ����ֽ�
	 {
		sum=(uint8_t)(rebuf[1]+rebuf[0]);
		if(sum==rebuf[2])
		{
			switch(rebuf[1])
			{
				case 0x45:stata_reg^=RPY;break;//���ŷ����ָ��
				case 0x15:stata_reg^=ACC;break;//������ٶ�����ָ��
				case 0x25:stata_reg^=GYR;break;//�������������ָ��
				case 0x35:stata_reg^=MAG;break;//����ų�����ָ��
				case 0x65:stata_reg^=Q4;break;//�����Ԫ������ָ��
				case 0xa4:KEY=1;break;//ģ�����Ƶ��50hz����
				case 0xa5:KEY=2;break;//ģ�����Ƶ��100hz����
				case 0xa6:KEY=3;break;//ģ�����Ƶ��200hz����
				case 0x57:KEY=4;break;//����У׼
				case 0x58:KEY=5;break;//�ų�У׼
				case 0x59:KEY=6;break;//�ָ��������ã�����������У׼����
				case 0x75:read_key=1;break;//��ȡУ׼����
				case 0x85:read_key=read_key|2;break;//��ȡ����������
				case 0x50:KEY=7;break;//�������д�����
				case 0x51:KEY=8;break;//�رռӼ�
				case 0x52:KEY=9;break;//�ر�����
				case 0x53:KEY=10;break;//�رմų�
				default:KEY=0,read_key=0;break;	
			}
		}
		i=0;//�建�����
	  }	
	}

}
