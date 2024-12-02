#include "stm32f10x.h"
#include "string.h"
#include "delay.h"
#include "LED.h"
#include "usart.h"
/*
Keil: MDK5.10.0.2
MCU:stm32f103c8
��������ô���1��ȡGY-953ģ������
Ӳ���ӷ���
GY-953_TX--STM32_RX
STM32_TX--FT232RX
���Ѹó������ؽ�stm32���ٽ�stm32��TX���Žӵ�GY-953ģ���RX���ţ�
Ȼ��λstm32����λ�ɹ��󣬸ó���ᷢ�����ָ���ģ�飬Ȼ��stm32
��TX�ӵ�FT232����ģ���RX���ţ�����λ����ѡ������115200��ѡ��
�ͺ�GYXX������򿪴��ڰ�ť����ʱ�����ܿ����ϴ�������
����˵����
���򽫶��ⷢ��һ�����ָ��ڴ��ڽ����жϽ���GY-953ģ�����������
������ѭ��������յ������ݲ����͵���λ����ʾ���ó���֧��GY-953����
���ݵĶ�ȡ
ע���жϴ������λ��stm32f10x_it.c
�ó�������ο���������������������ϵ
http://shop62474960.taobao.com/?spm=a230r.7195193.1997079397.2.HuqW76&v=1
�汾:2015.1.26
*/
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_X;
  
  /* 4����ռ���ȼ���4����Ӧ���ȼ� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*��ռ���ȼ��ɴ���жϼ���͵��ж�*/
	/*��Ӧ���ȼ����ȼ�ִ��*/
	NVIC_X.NVIC_IRQChannel = USART1_IRQn;//�ж�����
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//��Ӧ���ȼ�
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж���Ӧ
  NVIC_Init(&NVIC_X);
}
void send_Instruction(void)
{
	uint8_t send_data[3]={0};
	send_data[0]=0xa5;
	send_data[1]=0x15;//�Ӽƹ����ֽ�
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);//ָ��У���
	USART_Send_bytes(send_data,3);//���ͼ��ٶ����ָ��
	delay_ms(1);
	send_data[0]=0xa5;
	send_data[1]=0x25;
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);
	USART_Send_bytes(send_data,3);//�����������������ָ��
	delay_ms(1);
	send_data[0]=0xa5;
	send_data[1]=0x35;
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);
	USART_Send_bytes(send_data,3);//���ʹų��������ָ��
	delay_ms(1);
	send_data[0]=0xa5;
	send_data[1]=0x45;
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);
	USART_Send_bytes(send_data,3);//����ŷ�����������ָ��
	delay_ms(1);
	send_data[0]=0xa5;
	send_data[1]=0x65;
	send_data[2]=(uint8_t)(send_data[0]+send_data[1]);
	USART_Send_bytes(send_data,3);//������Ԫ���������ָ��
}
int main(void)
{
	uint8_t data_buf[9]={0};
	int16_t ROLL=0,PITCH=0,YAW=0;
	int16_t rpy[3]={0},Acc[3]={0},Gyr[3]={0},Mag[3]={0},Q[4]={0};
	delay_init(72);//72M 
	LED_Int(GPIOB,GPIO_Pin_9,RCC_APB2Periph_GPIOB);//led
	Usart_Int(115200);//115200������
	NVIC_Configuration();//�����ж����ȼ�����
	send_Instruction();//��ģ�鷢��5��ָ��
	GPIO_SetBits(GPIOB,GPIO_Pin_9);
	while(1)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_9);//LED��
		switch(CHeck(data_buf))//����������ݽ�����ϣ����������ָ��
		{
			case 0x15:{//�������������
				  Acc[0]=(data_buf[0]<<8)|data_buf[1];
					Acc[1]=(data_buf[2]<<8)|data_buf[3];
					Acc[2]=(data_buf[4]<<8)|data_buf[5];
					send_out(Acc,3,0x15);
			}
			break;
			case 0x25:{//�������������
				  Gyr[0]=(data_buf[0]<<8)|data_buf[1];
					Gyr[1]=(data_buf[2]<<8)|data_buf[3];
					Gyr[2]=(data_buf[4]<<8)|data_buf[5];
					send_out(Gyr,3,0x25);
			}
				break;
			case 0x35:{//�ų��������
				  Mag[0]=(data_buf[0]<<8)|data_buf[1];
					Mag[1]=(data_buf[2]<<8)|data_buf[3];
					Mag[2]=(data_buf[4]<<8)|data_buf[5];
					send_out(Mag,3,0x35);
			}
				break;
			case 0x45:{//ŷ�����������
				  ROLL=(data_buf[0]<<8)|data_buf[1];
					PITCH=(data_buf[2]<<8)|data_buf[3];
					YAW=(data_buf[4]<<8)|data_buf[5];
					rpy[0]=ROLL;
				  rpy[1]=PITCH;
				  rpy[2]=YAW;
					send_out(rpy,3,0x45);
			}
				break;
			case 0x65:{//��Ԫ���������
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
