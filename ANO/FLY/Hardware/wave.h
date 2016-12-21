/*
 * wave.h
 *
 *  Created on: Nov 29, 2014
 *      Author: ass
 */
#ifndef WAVE_H_
#define WAVE_H_

#include "usart.h"
#include "Rc.h"
//#define DATA_TRANSFER_USE_USART
#define DATA_TRANSFER_USE_SPI_NRF

#define ACK_OK 0
void NRF_Check(void);
void Data_Transfer(void);
unsigned char Uart1_Put_Int32(int32_t DataToSend);
void sendSenser(int16_t aa,int16_t bb,int16_t cc,int16_t dd,int16_t ee,int16_t ff,int16_t roll,int16_t pitch,int16_t yaw);
void Uart1_send_custom_uint16(uint16_t aa);
void Uart1_send_custom_int16(int16_t aa);
void Uart1_send_custom_three_int16(int16_t aa,int16_t bb,int16_t cc);
void zhen2_send_custom_four_int16(int16_t aa,int16_t bb,int16_t cc,int16_t dd);
void Uart1_send_custom_float(unsigned char fun,float aa,float bb,float cc);
void Uart1_send_custom_float_V2(unsigned char fun,float aa,float bb,float cc);
void sendPwmVoltage(Define_Rc_Data *rc_data,uint16_t aa,uint16_t bb,uint16_t cc,uint16_t dd);
void Uart1_Send_PID(uint16_t rol_p,uint16_t rol_i,uint16_t rol_d,uint16_t pit_p,uint16_t pit_i,uint16_t pit_d);
void Uart1_send_custom_PID(uint8_t aa);
unsigned char UART_Putc(unsigned char data);
void send_wave(int tx_num);

void receive_Data(void);

void sendSenserPackage(void);
void send_statusV2(int16_t rol, int16_t pitch, int16_t yaw, int16_t alt_cbs, int32_t alt_prs, u8 armed);
void send_senserV2(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t mag_x, int16_t mag_y, int16_t mag_z);
void send_rcdataV2(Define_Rc_Data *rc_data);
#endif /* WAVE_H_ */
