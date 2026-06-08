#include "bluetooth.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "PIDTask.h"
#include "stdlib.h"

uint8_t bluetooth_buf[100]; // 蓝牙接收缓冲区
uint8_t bluetooth_Rxflag;	// 蓝牙接收标志位
uint8_t databyte;			// 蓝牙接收数据



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		static uint8_t data_index = 0;
		static uint8_t RxState = 0;
		if (RxState == 0)
		{
			if (databyte == '[' && bluetooth_Rxflag == 0) // 如果数据确实是包头，并且上一个数据包已处理完毕
			{
				RxState = 1;
				data_index = 0;
			}
		}
		else if (RxState == 1)
		{
			if (databyte == ']') // 如果数据确实是包尾
			{
				RxState = 0;
				bluetooth_buf[data_index] = '\0';
				bluetooth_Rxflag = 1; // 接收完毕，标志位置1
			}
			else
			{
				bluetooth_buf[data_index] = databyte;
				data_index++;
			}
		}
		UART_Start_Receive_IT(&huart3, &databyte, 1);
	}
}

void StartBtCmd(void *argument)
{
	for (;;)
	{
		if (bluetooth_Rxflag)
		{
			char *Tag = strtok(bluetooth_buf, ",");
			if (strcmp(Tag, "slider") == 0) // Tag为slider，收到滑杆数据包
			{
				char *Name = strtok(NULL, ",");	 // 提取数据2，定义为滑杆名称
				char *Value = strtok(NULL, ","); // 提取数据3，定义为滑杆值
				/*执行滑杆操作*/
				if (strcmp(Name, "AngleKp") == 0) // 如果滑杆名称是AngleKp
				{
					AnglePID.Kp = atof(Value); // 则把滑杆值赋值给角度环Kp
				}
				else if (strcmp(Name, "AngleKi") == 0) // 如果滑杆名称是AngleKi
				{
					AnglePID.Ki = atof(Value); // 则把滑杆值赋值给角度环Ki
				}
				else if (strcmp(Name, "AngleKd") == 0) // 如果滑杆名称是AngleKd
				{
					AnglePID.Kd = atof(Value); // 则把滑杆值赋值给角度环Kd
				}
				else if (strcmp(Name, "SpeedKp") == 0) // 如果滑杆名称是SpeedKp
				{
					SpeedPID.Kp = atof(Value); // 则把滑杆值赋值给速度环Kp
				}
				else if (strcmp(Name, "SpeedKi") == 0) // 如果滑杆名称是SpeedKi
				{
					SpeedPID.Ki = atof(Value); // 则把滑杆值赋值给速度环Ki
				}
				else if (strcmp(Name, "SpeedKd") == 0) // 如果滑杆名称是SpeedKd
				{
					SpeedPID.Kd = atof(Value); // 则把滑杆值赋值给速度环Kd
				}
				else if (strcmp(Name, "TurnKp") == 0) // 如果滑杆名称是TurnKp
				{
					TurnPID.Kp = atof(Value); // 则把滑杆值赋值给转向环Kp
				}
				else if (strcmp(Name, "TurnKi") == 0) // 如果滑杆名称是TurnKi
				{
					TurnPID.Ki = atof(Value); // 则把滑杆值赋值给转向环Ki
				}
				else if (strcmp(Name, "TurnKd") == 0) // 如果滑杆名称是TurnKd
				{
					TurnPID.Kd = atof(Value); // 则把滑杆值赋值给转向环Kd
				}

				else if (strcmp(Name, "offset") == 0) // 如果滑杆名称是TurnKd
				{
					AnglePID.Out_Offset = atof(Value); // 则把滑杆值赋值给转向环Kd
				}
			}

			if (strcmp(Tag, "key") == 0) // Tag为key，收到按键数据包
			{
				char *Name = strtok(NULL, ",");	  // 提取数据2，定义为按键名称
				char *Action = strtok(NULL, ","); // 提取数据3，定义为按键动作

				/*此处可执行按键操作，目前程序暂时没用到按键*/
				if (strcmp(Name, "switch") == 0) // 如果按键名称是switch
				{
					if (strcmp(Action, "down") == 0) // 如果按键动作是
					{
						Run_Flag ^= 1; // 则把Run_Flag取反
					}
				}
			}

			else if (strcmp(Tag, "joystick") == 0) // Tag为joystick，收到摇杆数据包
			{
				int8_t LH = atoi(strtok(NULL, ",")); // 提取数据2，定义为摇杆值LH
				int8_t LV = atoi(strtok(NULL, ",")); // 提取数据3，定义为摇杆值LV
				int8_t RH = atoi(strtok(NULL, ",")); // 提取数据4，定义为摇杆值RH
				int8_t RV = atoi(strtok(NULL, ",")); // 提取数据5，定义为摇杆值RV

				/*执行摇杆操作*/
				// BtCmd.speed = LV / 25.0; // 把摇杆值LV乘以0.01赋值给角度环目标值
				// BtCmd.turn = RH / 25.0;	 // 把摇杆值RH除以2赋值给速度环差值
				
				//填入数据
				if(Run_Flag)
				{
					extern osMessageQueueId_t BtCmdQueueHandle;
					Bt_Cmd_t *BtCmd = pvPortMalloc(sizeof(Bt_Cmd_t));
					BtCmd->speed = LV / 25.0f; // 把摇杆值LV乘以0.01赋值给角度环目标值
					BtCmd->turn = RH / 25.0f;	 // 把摇杆值RH除以2赋值给速度环差值
					osMessageQueuePut(BtCmdQueueHandle, &BtCmd,0,0);
				}
				
			}

			bluetooth_Rxflag = 0;
		}
		    
	}
}