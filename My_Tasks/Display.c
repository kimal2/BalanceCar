#include "Display.h"
#include "cmsis_os.h"
#include "oled.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"

#include "PIDTask.h"

#include "math.h"

extern float roll_acc, roll_gyro, roll; // roll角的加速度计测量值，陀螺仪测量值，最终值

char disp_buf[50];
void DisplayTask(void *argument)
{

    for (;;)
    {
        sprintf(disp_buf, "%06.2fdeg", roll);
        OLED_ShowString(0, 0, (uint8_t *)disp_buf, 8);

        sprintf(disp_buf, "%+06d", gx);
        OLED_ShowString(0, 1, disp_buf, 8);

        sprintf(disp_buf, "tpos:%05.2f", target_position);
        OLED_ShowString(0, 2, (uint8_t *)disp_buf, 8);

        sprintf(disp_buf, "pos:%05.2f", AvgPosition);
        OLED_ShowString(0, 3, (uint8_t *)disp_buf, 8);

        sprintf(disp_buf, "spd:%05.2f", AvgSpeed);
        OLED_ShowString(0, 4, (uint8_t *)disp_buf, 8);

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

        // 测试PID计算时间
        extern uint32_t run_time;
        sprintf(disp_buf, "%04dus", run_time / 72);
        OLED_ShowString(0, 7, (uint8_t *)disp_buf, 8);

        extern float Bt_V;
        // sprintf(disp_buf, "[plot,%f,%f,%f,%f,%f]",
        //         SpeedPID.target, SpeedPID.actual,
        //         AnglePID.target, AnglePID.actual,
        //     Bt_V);
        sprintf(disp_buf, "%f,%f,%f,%f,%f\n",
                SpeedPID.target, SpeedPID.actual,
                AnglePID.target, AnglePID.actual,
            Bt_V);
        // OLED_ShowString(0,3,disp_buf,8);
        HAL_UART_Transmit_DMA(&huart3, (const uint8_t *)disp_buf, strlen((char *)disp_buf));

        osDelay(50);
    }
}
