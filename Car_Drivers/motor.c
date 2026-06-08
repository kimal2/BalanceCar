#include "motor.h"
#include "tim.h"

int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}

/// @brief -7200~7200 取值范围，向前转为正数
/// @param moto1 
/// @param moto2 
void Set_Motor_Speed(int moto_left,int moto_right)			
{
	if(moto_left<0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(moto_left));
	if(moto_right<0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(moto_right));
}
