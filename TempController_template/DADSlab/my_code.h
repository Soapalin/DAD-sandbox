/*
 * my_code.h
 *
 *  Created on: 28/01/2014
 *      Author: local_admin
 */

#ifndef MY_CODE_H_
#define MY_CODE_H_

void InitADC(void);
void InitPWM(void);
void InitUART1(void);
void InitTimerPWM(void);

void InitialScreen(void);
void UpdateScreen(void);

void adc2ASCII(void);
//void V2Temp(void);
void IR_Read(void);
void PID_controller(void);
void SendPacket(void);

void InitI2C1(void);
int i2c_string(unsigned char slave_addr, unsigned char command);

#endif /* MY_CODE_H_ */
