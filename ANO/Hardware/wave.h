/*
 * wave.h
 *
 *  Created on: Nov 29, 2014
 *      Author: ass
 */
#ifndef WAVE_H_
#define WAVE_H_

#include "usart.h"

void Uart1_Send_AF(signed int aa,signed int bb,signed int cc,signed int dd,signed int ee,signed int ff,signed int gg,signed int hh);
//void Uart1_send_custom(uint16_t data);
unsigned char Uart1_Put_float(float DataToSend);
void Uart1_send_custom(float data);
unsigned char UART_Putc(unsigned char data);
void send_wave(int tx_num);

void printhh(void); 
void print5n(unsigned int x); 
void print4n(unsigned int x);
void print3n(unsigned int x);
void print2n(unsigned int x);

#endif /* WAVE_H_ */
