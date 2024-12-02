#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"
#define ACC 0X01//�Ӽƿ���λ
#define GYR 0X02//���ݿ���λ
#define MAG 0X04//�ų�����λ
#define RPY 0X08//ŷ���ǿ���λ
#define Q4  0X10//��Ԫ������λ
#define RegisterA 0X08//�Ĵ���1����ֵ
#define RegisterB 0X11//�Ĵ���2����ֵ
void USART_send_byte(uint8_t byte);
void Usart_Int(uint32_t BaudRatePrescaler);
void send_out(int16_t *data,uint8_t length,uint8_t send);
void CHeck(uint8_t *re_data);
extern uint8_t KEY,read_key,stata_reg;
extern uint8_t ACM_BUF[41],BIT;
#endif
