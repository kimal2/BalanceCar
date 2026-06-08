#ifndef _PIDTASK_H
#define _PIDTASK_H
#include "pid.h"
#include "stm32f1xx_hal.h" 

extern PID_t AnglePID;
extern PID_t SpeedPID;
extern PID_t TurnPID;

extern uint8_t Run_Flag;

extern short ax, ay, az, gx, gy, gz;
extern float LeftSpeed;
extern    float RightSpeed;
extern    float AvgSpeed;
extern    float DifSpeed;

#endif

