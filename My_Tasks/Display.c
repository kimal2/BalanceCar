#include "Display.h"
#include "cmsis_os.h"
#include "oled.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"

#include "PIDTask.h"


extern float roll_acc , roll_gyro , roll; //roll角的加速度计测量值，陀螺仪测量值，最终值

char disp_buf[50];
void DisplayTask(void *argument)
{

    for(;;)
    {
        sprintf(disp_buf,"%.2f",roll);
        OLED_ShowString(0,0,(uint8_t *)disp_buf,8);

        // sprintf(disp_buf,"%+06d",gz+170);
        // OLED_ShowString(0,1,disp_buf,8);

        sprintf(disp_buf,"%.1f,%.1f,%.1f",AnglePID.Kp,AnglePID.Ki,AnglePID.Kd);
        OLED_ShowString(0,2,(uint8_t *)disp_buf,8);
               
        // sprintf(disp_buf,"roll:%+04.2f",roll);
        // OLED_ShowString(0,0,disp_buf,8);
        // sprintf(disp_buf,"rola:%+04.2f",roll_acc);
        // OLED_ShowString(0,1,disp_buf,8);
        // sprintf(disp_buf,"rolg:%+04.2f",roll_gyro);
        // OLED_ShowString(0,2,disp_buf,8);

        // extern uint16_t AvgPWM;
        // sprintf(disp_buf,"AvgPWM: %d",AvgPWM);
        // OLED_ShowString(0,4,disp_buf,16);
        
        // sprintf(disp_buf,"roll:%.2f\n",roll);
        // HAL_UART_Transmit(&huart3,disp_buf,strlen(disp_buf),100);

        //测试PID计算时间
        extern uint32_t run_time;
        sprintf(disp_buf,"%lu,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n", HAL_GetTick(),
        0.0f,AnglePID.actual,AnglePID.output,AnglePID.error,AnglePID.Kp,
        AnglePID.Ki,AnglePID.Kd);
        // sprintf(disp_buf,"t,ac:%f,%f\n",0.0f,AnglePID.actual);
        //OLED_ShowString(0,3,disp_buf,8);
        HAL_UART_Transmit(&huart3,(const uint8_t *)disp_buf,strlen((char *)disp_buf),100);

        
        osDelay(50);
    }
}


