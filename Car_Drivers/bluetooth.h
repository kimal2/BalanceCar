#ifndef _BLUETOOCH_H
#define _BLUETOOCH_H

#include "stm32f1xx_hal.h"

extern uint8_t bluetooth_buf[100]; //蓝牙接收缓冲区
extern uint8_t bluetooth_Rxflag; //蓝牙接收标志位 
extern uint8_t databyte; //蓝牙接收数据

typedef struct
{
    float speed;
    float turn;
} Bt_Cmd_t;



#endif
