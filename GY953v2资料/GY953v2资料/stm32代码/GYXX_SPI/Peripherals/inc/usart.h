#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"
#define ACC 0X01//加计控制位
#define GYR 0X02//陀螺控制位
#define MAG 0X04//磁场控制位
#define RPY 0X08//欧拉角控制位
#define Q4  0X10//四元数控制位
#define RegisterA 0X08//寄存器1固有值
#define RegisterB 0X11//寄存器2固有值
void USART_send_byte(uint8_t byte);
void Usart_Int(uint32_t BaudRatePrescaler);
void send_out(int16_t *data,uint8_t length,uint8_t send);
void CHeck(uint8_t *re_data);
extern uint8_t KEY,read_key,stata_reg;
extern uint8_t ACM_BUF[41],BIT;
#endif
