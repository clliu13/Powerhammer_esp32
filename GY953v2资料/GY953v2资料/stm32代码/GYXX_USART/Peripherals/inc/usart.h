#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"
#define ACC 0X01//�Ӽƿ���λ
#define GYR 0X02//���ݿ���λ
#define MAG 0X04//�ų�����λ
#define RPY 0X08//ŷ���ǿ���λ
#define Q4  0X10//��Ԫ������λ
void Usart_Int(uint32_t BaudRatePrescaler);
void USART_send_byte(uint8_t byte);
void USART_Send_bytes(uint8_t *Buffer, uint8_t Length);
void send_out(int16_t *data,uint8_t length,uint8_t send);
uint8_t CHeck(uint8_t *data_buf);
extern uint8_t stata_reg;
extern uint8_t data1_buf[7],data2_buf[7],data3_buf[7],
data4_buf[7],data5_buf[9];
#endif
