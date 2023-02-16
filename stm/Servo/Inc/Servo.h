//
// Created by 35802 on 2022/7/17.
//
#ifndef UARTTEST_SERVO_H
#define UARTTEST_SERVO_H
#include "stm32f1xx_hal.h"
#define TOTAL_SERVOS_NUM 30
#define FRAME_HEADER 0x55    //帧头
#define CMD_SERVO_MOVE 0x03 //舵机移动指令
#define CMD_GET_BATTERY_VOLTAGE 0x0F //获取电池电压指令
#define CMD_SERVO_UNLOAD 0x14 //断掉舵机电力
#define CMD_GET_SERVO_ANGLE 0x15 //获取舵机角度

extern int isUartRxCompleted;
static UART_HandleTypeDef *ServoHuart;
static UART_HandleTypeDef *UpperServoHuart;

void Uart_Init(UART_HandleTypeDef *huart, UART_HandleTypeDef *UpperHuart);
void unloadServos(uint8_t Num,uint16_t Time,...);
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void unloadServosByArray(uint8_t Num,uint16_t Time, int* ID);
void moveServosByArray(uint8_t Num, uint16_t Time, uint16_t* Position);
void getServoAngleByArray(uint8_t Num,uint16_t Time, const int* ID);
void getBatteryVoltage(void);
void getServoAngle(uint8_t Num,uint16_t Time,...);
void convertAngleData(double * AngleBuf, uint16_t * ConvertAngleBuf);
void deConvertAngle(double * convertAngleBuf, uint16_t * angleBuf);
void lockServos(uint8_t Num, int * rx_lock,...);
#endif //UARTTEST_SERVO_H
