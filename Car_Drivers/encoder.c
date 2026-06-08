#include "encoder.h"
#include "tim.h"

int Encoder_Left,Encoder_Right;

int Get_Encoder_Value(TIM_HandleTypeDef *htim)
{
	int temp;
	temp=(short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
	return temp;
}

/// @brief 向前转为正数
void get_encoder(void)
{
  Encoder_Left = -Get_Encoder_Value(&htim2);
  Encoder_Right  = Get_Encoder_Value(&htim4);
}
