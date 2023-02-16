//
// Created by 35802 on 2022/7/17.
//
#include <stdarg.h>
#include "Servo.h"
#include "ServoDeviation.h"
#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位

static UART_HandleTypeDef * ServoHuart;
static UART_HandleTypeDef * UpperServoHuart;
static uint8_t LobotTxBuf[128];  //发送缓存


void Uart_Init(UART_HandleTypeDef *huart, UART_HandleTypeDef *UpperHuart){
    ServoHuart = huart;
    UpperServoHuart = UpperHuart;
}

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
    if (servoID > 31 || !(Time > 0)) {  //舵机ID不能打于31,可根据对应控制板修改
        return;
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;    //填充帧头
    LobotTxBuf[2] = 8;
    LobotTxBuf[3] = CMD_SERVO_MOVE;           //数据长度=要控制舵机数*3+5，此处=1*3+5//填充舵机移动指令
    LobotTxBuf[4] = 1;                        //要控制的舵机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);       //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);      //取得时间的高八位
    LobotTxBuf[7] = servoID;                  //舵机ID
    LobotTxBuf[8] = GET_LOW_BYTE(Position);   //取得目标位置的低八位
    LobotTxBuf[9] = GET_HIGH_BYTE(Position);  //取得目标位置的高八位
    HAL_UART_Transmit(ServoHuart,LobotTxBuf,10, HAL_MAX_DELAY);
    HAL_UART_Transmit(UpperServoHuart,LobotTxBuf,10, HAL_MAX_DELAY);
}


void moveServos(uint8_t Num, uint16_t Time, ...)
{
    uint8_t index = 7;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;  //

    va_start(arg_ptr, Time); //取得可变参数首地址
    if (Num < 1 || Num > 32) {
        return;               //舵机数不能为零和大与32，时间不能小于0
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                //数据长度 = 要控制舵机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_MOVE;             //舵机移动指令
    LobotTxBuf[4] = Num;                        //要控制舵机数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);         //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);        //取得时间的高八位


    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
        temp = va_arg(arg_ptr, int);  //可变参数中取得对应目标位置
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); //填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(temp);//填充目标位置高八位
    }
    va_end(arg_ptr);  //置空arg_ptr
    //发送
    HAL_UART_Transmit(ServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(UpperServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
}

void moveServosByArray(uint8_t Num, uint16_t Time, uint16_t* Position){
    uint8_t index = 7;
    uint16_t temp;
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                //数据长度 = 要控制舵机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_MOVE;             //舵机移动指令
    LobotTxBuf[4] = Num;                        //要控制舵机数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);         //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);        //取得时间的高八位
    for (uint8_t i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = i+1;//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
        temp = Position[i];  //可变参数中取得对应目标位置
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); //填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(temp);//填充目标位置高八位
    }
    HAL_UART_Transmit(ServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(UpperServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
}

void unloadServos(uint8_t Num,uint16_t Time,...){
    uint8_t index = 5;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;
    va_start(arg_ptr, Time); //取得可变参数首地址
    if (Num < 1 || Num > 32) {
        return;               //舵机数不能为零和大与32，时间不能小于0
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num + 3;                //数据长度 = 要控制舵机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_UNLOAD;             //舵机移动指令
    LobotTxBuf[4] = Num;                        //要控制舵机数
    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
    }
    va_end(arg_ptr);  //置空arg_ptr
    HAL_UART_Transmit(ServoHuart,LobotTxBuf, LobotTxBuf[2] + 2,HAL_MAX_DELAY);    //发送
    HAL_UART_Transmit(UpperServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
}

void unloadServosByArray(uint8_t Num,uint16_t Time, int* ID){
    uint8_t index = 5;
    uint8_t i = 0;
    uint16_t temp;

    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num + 3;                //数据长度 = 要控制舵机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_UNLOAD;             //舵机移动指令
    LobotTxBuf[4] = Num;                        //要控制舵机数

    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = ID[i];//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
    }
    HAL_UART_Transmit(ServoHuart,LobotTxBuf, LobotTxBuf[2] + 2,HAL_MAX_DELAY);    //发送
    HAL_UART_Transmit(UpperServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
}

void getBatteryVoltage(void)
{
//	uint16_t Voltage = 0;
    LobotTxBuf[0] = FRAME_HEADER;  //填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;             //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
    LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  //填充获取电池电压命令
    HAL_UART_Transmit_IT(ServoHuart,LobotTxBuf,4);
}

void getServoAngle(uint8_t Num,uint16_t Time,...){
    uint8_t index = 5;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;  //
    LobotTxBuf[0] = FRAME_HEADER;  //填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = Num+3;
    LobotTxBuf[3] = CMD_GET_SERVO_ANGLE;
    LobotTxBuf[4] = Num;
    va_start(arg_ptr, Time); //取得可变参数首地址
    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
    }
    va_end(arg_ptr);  //置空arg_ptr
    HAL_UART_Transmit(ServoHuart,LobotTxBuf, LobotTxBuf[2] + 2,1000);    //发送
    HAL_UART_Transmit(UpperServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
}

void getServoAngleByArray(uint8_t Num,uint16_t Time, const int* ID){
    uint8_t index = 5;
    uint8_t i = 0;
    uint16_t temp;
    LobotTxBuf[0] = FRAME_HEADER;  //填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = Num+3;
    LobotTxBuf[3] = CMD_GET_SERVO_ANGLE;
    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = ID[i];
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
    }
    HAL_UART_Transmit_IT(ServoHuart,LobotTxBuf, LobotTxBuf[2] + 2);    //发送
    HAL_UART_Transmit(UpperServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
}

void convertAngleData(double * convertAngleBuf,uint16_t * angleBuf){
    int iter = 0;
    for(iter = 0;iter < TOTAL_SERVOS_NUM ;++iter){
        switch (iter+1) {
            case 1:convertAngleBuf[iter] =
                            (((double)(angleBuf[iter]-SERVO_1_MIN_ANGLE))
                            /(SERVO_1_MAX_ANGLE-SERVO_1_MIN_ANGLE)) * 90.0;break;
            case 2:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_2_MIN_ANGLE))
                            /(SERVO_2_MAX_ANGLE-SERVO_2_MIN_ANGLE)) * 90.0;break;
            case 3:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_3_MIN_ANGLE))
                            /(SERVO_3_MAX_ANGLE-SERVO_3_MIN_ANGLE)) * 90.0;break;
            case 4:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_4_MIN_ANGLE))
                            /(SERVO_4_MAX_ANGLE-SERVO_4_MIN_ANGLE)) * 90.0;break;
            case 5:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_5_MIN_ANGLE))
                            /(SERVO_5_MAX_ANGLE-SERVO_5_MIN_ANGLE)) * 90.0;break;
            case 6:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_6_MIN_ANGLE))
                            /(SERVO_6_MAX_ANGLE-SERVO_6_MIN_ANGLE)) * 90.0;break;
            case 7:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_7_MIN_ANGLE))
                            /(SERVO_7_MAX_ANGLE-SERVO_7_MIN_ANGLE)) * 90.0;break;
            case 8:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_8_MIN_ANGLE))
                            /(SERVO_8_MAX_ANGLE-SERVO_8_MIN_ANGLE)) * 90.0;break;
            case 9:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_9_MIN_ANGLE))
                            /(SERVO_9_MAX_ANGLE-SERVO_9_MIN_ANGLE)) * 90.0;break;
            case 10:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_10_MIN_ANGLE))
                            /(SERVO_10_MAX_ANGLE-SERVO_10_MIN_ANGLE)) * 90.0;break;
            case 11:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_11_MIN_ANGLE))
                            /(SERVO_11_MAX_ANGLE-SERVO_11_MIN_ANGLE)) * 90.0;break;
            case 12:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_12_MIN_ANGLE))
                            /(SERVO_12_MAX_ANGLE-SERVO_12_MIN_ANGLE)) * 90.0;break;
            case 13:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_13_MIN_ANGLE))
                            /(SERVO_13_MAX_ANGLE-SERVO_13_MIN_ANGLE)) * 90.0;break;
            case 14:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_14_MIN_ANGLE))
                            /(SERVO_14_MAX_ANGLE-SERVO_14_MIN_ANGLE)) * 90.0;break;
            case 15:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_15_MIN_ANGLE))
                            /(SERVO_15_MAX_ANGLE-SERVO_15_MIN_ANGLE)) * 90.0;break;
            case 16:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_16_MIN_ANGLE))
                            /(SERVO_16_MAX_ANGLE-SERVO_16_MIN_ANGLE)) * 90.0;break;
            case 17:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_17_MIN_ANGLE))
                            /(SERVO_17_MAX_ANGLE-SERVO_17_MIN_ANGLE)) * 90.0;break;
            case 18:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_18_MIN_ANGLE))
                            /(SERVO_18_MAX_ANGLE-SERVO_18_MIN_ANGLE)) * 90.0;break;
            case 19:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_19_MIN_ANGLE))
                            /(SERVO_19_MAX_ANGLE-SERVO_19_MIN_ANGLE)) * 90.0;break;
            case 20:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_20_MIN_ANGLE))
                            /(SERVO_20_MAX_ANGLE-SERVO_20_MIN_ANGLE)) * 90.0;break;
            case 21:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_21_MIN_ANGLE))
                            /(SERVO_21_MAX_ANGLE-SERVO_21_MIN_ANGLE)) * 90.0;break;
            case 22:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_22_MIN_ANGLE))
                            /(SERVO_22_MAX_ANGLE-SERVO_22_MIN_ANGLE)) * 90.0;break;
            case 23:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_23_MIN_ANGLE))
                            /(SERVO_23_MAX_ANGLE-SERVO_23_MIN_ANGLE)) * 90.0;break;
            case 24:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_24_MIN_ANGLE))
                            /(SERVO_24_MAX_ANGLE-SERVO_24_MIN_ANGLE)) * 90.0;break;
            case 25:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_25_MIN_ANGLE))
                            /(SERVO_25_MAX_ANGLE-SERVO_25_MIN_ANGLE)) * 90.0;break;
            case 26:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_26_MIN_ANGLE))
                            /(SERVO_26_MAX_ANGLE-SERVO_26_MIN_ANGLE)) * 90.0;break;
            case 27:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_27_MIN_ANGLE))
                            /(SERVO_27_MAX_ANGLE-SERVO_27_MIN_ANGLE)) * 90.0;break;
            case 28:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_28_MIN_ANGLE))
                            /(SERVO_28_MAX_ANGLE-SERVO_28_MIN_ANGLE)) * 90.0;break;
            case 29:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_29_MIN_ANGLE))
                            /(SERVO_29_MAX_ANGLE-SERVO_29_MIN_ANGLE)) * 90.0;break;
            case 30:convertAngleBuf[iter] =
                           (((double)(angleBuf[iter]-SERVO_30_MIN_ANGLE))
                            /(SERVO_30_MAX_ANGLE-SERVO_30_MIN_ANGLE)) * 90.0;break;
            default:;break;
        }
    }
}

void deConvertAngle(double * convertAngleBuf, uint16_t * angleBuf){
    int iter = 0;
    for(iter = 0;iter < TOTAL_SERVOS_NUM;++iter){
        switch (iter+1) {
            case 1:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_1_MAX_ANGLE-SERVO_1_MIN_ANGLE)
                                      + SERVO_1_MIN_ANGLE);break;
            case 2:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_2_MAX_ANGLE-SERVO_2_MIN_ANGLE)
                                      + SERVO_2_MIN_ANGLE);break;
            case 3:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_3_MAX_ANGLE-SERVO_3_MIN_ANGLE)
                                      + SERVO_3_MIN_ANGLE);break;
            case 4:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_4_MAX_ANGLE-SERVO_4_MIN_ANGLE)
                                      + SERVO_4_MIN_ANGLE);break;
            case 5:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_5_MAX_ANGLE-SERVO_5_MIN_ANGLE)
                                      + SERVO_5_MIN_ANGLE);break;
            case 6:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_6_MAX_ANGLE-SERVO_6_MIN_ANGLE)
                                      + SERVO_6_MIN_ANGLE);break;
            case 7:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_7_MAX_ANGLE-SERVO_7_MIN_ANGLE)
                                      + SERVO_7_MIN_ANGLE);break;
            case 8:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_8_MAX_ANGLE-SERVO_8_MIN_ANGLE)
                                      + SERVO_8_MIN_ANGLE);break;
            case 9:angleBuf[iter] =
                           (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_9_MAX_ANGLE-SERVO_9_MIN_ANGLE)
                                      + SERVO_9_MIN_ANGLE);break;
            case 10:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_10_MAX_ANGLE-SERVO_10_MIN_ANGLE)
                                       + SERVO_10_MIN_ANGLE);break;
            case 11:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_11_MAX_ANGLE-SERVO_11_MIN_ANGLE)
                                       + SERVO_11_MIN_ANGLE);break;
            case 12:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_12_MAX_ANGLE-SERVO_12_MIN_ANGLE)
                                       + SERVO_12_MIN_ANGLE);break;
            case 13:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_13_MAX_ANGLE-SERVO_13_MIN_ANGLE)
                                       + SERVO_13_MIN_ANGLE);break;
            case 14:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_14_MAX_ANGLE-SERVO_14_MIN_ANGLE)
                                       + SERVO_14_MIN_ANGLE);break;
            case 15:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_15_MAX_ANGLE-SERVO_15_MIN_ANGLE)
                                       + SERVO_15_MIN_ANGLE);break;
            case 16:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_16_MAX_ANGLE-SERVO_16_MIN_ANGLE)
                                       + SERVO_16_MIN_ANGLE);break;
            case 17:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_17_MAX_ANGLE-SERVO_17_MIN_ANGLE)
                                       + SERVO_17_MIN_ANGLE);break;
            case 18:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_18_MAX_ANGLE-SERVO_18_MIN_ANGLE)
                                       + SERVO_18_MIN_ANGLE);break;
            case 19:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_19_MAX_ANGLE-SERVO_19_MIN_ANGLE)
                                       + SERVO_19_MIN_ANGLE);break;
            case 20:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_20_MAX_ANGLE-SERVO_20_MIN_ANGLE)
                                       + SERVO_20_MIN_ANGLE);break;
            case 21:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_21_MAX_ANGLE-SERVO_21_MIN_ANGLE)
                                       + SERVO_21_MIN_ANGLE);break;
            case 22:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_22_MAX_ANGLE-SERVO_22_MIN_ANGLE)
                                       + SERVO_22_MIN_ANGLE);break;
            case 23:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_23_MAX_ANGLE-SERVO_23_MIN_ANGLE)
                                       + SERVO_23_MIN_ANGLE);break;
            case 24:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_24_MAX_ANGLE-SERVO_24_MIN_ANGLE)
                                       + SERVO_24_MIN_ANGLE);break;
            case 25:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_25_MAX_ANGLE-SERVO_25_MIN_ANGLE)
                                       + SERVO_25_MIN_ANGLE);break;
            case 26:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_26_MAX_ANGLE-SERVO_26_MIN_ANGLE)
                                       + SERVO_26_MIN_ANGLE);break;
            case 27:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_27_MAX_ANGLE-SERVO_27_MIN_ANGLE)
                                       + SERVO_27_MIN_ANGLE);break;
            case 28:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_28_MAX_ANGLE-SERVO_28_MIN_ANGLE)
                                       + SERVO_28_MIN_ANGLE);break;
            case 29:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_29_MAX_ANGLE-SERVO_29_MIN_ANGLE)
                                       + SERVO_29_MIN_ANGLE);break;
            case 30:angleBuf[iter] =
                            (uint16_t)((convertAngleBuf[iter]/90) * (SERVO_30_MAX_ANGLE-SERVO_30_MIN_ANGLE)
                                       + SERVO_30_MIN_ANGLE);break;
            default:;break;
        }
    }
}

