/*
 * wave.h
 *
 *  Created on: Nov 29, 2014
 *      Author: ass
 */
#ifndef WAVE_H_
#define WAVE_H_

#include "usart.h"

// void Uart1_Send_AF(int16_t aa,int16_t bb,int16_t cc,int16_t dd,int16_t ee,int16_t ff,int16_t gg,int16_t hh);
void Uart1_send_custom_uint16(uint16_t aa);
void Uart1_send_custom_int16(int16_t aa);
void Uart1_send_custom_float(unsigned char fun,float aa,float bb,float cc);
// void Uart1_Send_AE(uint16_t throttle,uint16_t aa,uint16_t bb,uint16_t cc,uint16_t dd,uint16_t ee);
// void Uart1_Send_PID(uint16_t rol_p,uint16_t rol_i,uint16_t rol_d,uint16_t pit_p,uint16_t pit_i,uint16_t pit_d);
// void Uart1_send_custom_PID(uint8_t aa);
unsigned char UART_Putc(unsigned char data);
void send_wave(int tx_num);
void send_temp(int tx_num);
// void Uart1_send_temp(uint16_t aa);
void receive_Data(void);
void printhh(void); 
void print5n(unsigned int x); 
void print4n(unsigned int x);
void print3n(unsigned int x);
void print2n(unsigned int x);


void send_status(int16_t rol, int16_t pitch, int16_t yaw, int16_t alt_cbs, int32_t alt_prs, u8 armed);
void send_senser(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t mag_x, int16_t mag_y, int16_t mag_z);
void send_rcdata(int16_t throttle, int16_t yaw, int16_t rol, int16_t pitch, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6);

#endif /* WAVE_H_ */
