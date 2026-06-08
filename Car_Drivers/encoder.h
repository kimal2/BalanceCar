#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f1xx_hal.h"

extern int Encoder_Left;
extern int Encoder_Right;

int Get_Encoder_Value(TIM_HandleTypeDef *htim);
void get_encoder(void);

#endif
