#include "PIDTask.h"
#include "cmsis_os.h"
#include "MPU6050.h"
#include "math.h"
#include "pid.h"
#include "motor.h"
#include "bluetooth.h"
#include "encoder.h"

#define GX_OFFSET -85

PID_t AnglePID =
    {
        .Kp = 450.90f,
        .Ki = 2362.71f,
        .Kd = 669.5f,
        .dt = 0.01,
        .Out_Max = 7200,
        .Out_Min = -7200,
        .integ_limit = 7200,
        .Out_Offset = 363,
};

PID_t SpeedPID =
    {
        .Kp = 2.126f,
        .Ki = 2.66f,
        .Kd = 0.0,
        .dt = 0.05,
        .Out_Max = 15,
        .Out_Min = -15, // 速度环控制角度环，角度最大+-15
        .integ_limit = 15,
};

PID_t TurnPID =
    {
        .Kp = 1573.92f,
        .Ki = 6.29f,
        .Kd = 0.0,
        .dt = 0.05,
        .Out_Max = 7200,
        .Out_Min = -7200, // 转向环控制速度环，速度最大+-50
        .integ_limit = 7200,
};

short ax, ay, az, gx, gy, gz;    // 加速度计和陀螺仪的原始数据
float roll_acc, roll_gyro, roll; // roll角的加速度计测量值，陀螺仪测量值，最终值
uint8_t Run_Flag = 0;            // 运行标志位，0为不运行，1为运行

int16_t AvgPWM;
int16_t LeftPWM, RightPWM;
int16_t DifPWM = 0;

float LeftSpeed;
float RightSpeed;
float AvgSpeed;
float DifSpeed;

// 执行时间s
 uint32_t run_time;
 uint32_t run_time_gaplast, run_time_gapnow,run_time_gap;

void StartTaskBalance(void *argument)
{

  uint32_t tick = osKernelGetTickCount();
  for (;;)
  {
    tick += 10;
    // 计算时间
    uint32_t start = DWT->CYCCNT;

    MPU_Get_AccelAndGyro(&ax, &ay, &az, &gx, &gy, &gz);
    gx -= GX_OFFSET;
    roll_acc = atan2(ay, az) * 180 / 3.1415926;
    roll_acc -= 0.3f;
    roll_gyro = roll + (gx) / 32768.0f * 2000 * 0.01f;
    float Alpha = 0.02f;
    roll = Alpha * roll_acc + (1 - Alpha) * roll_gyro;

    if (roll > 50 || roll < -50)
    {
      Run_Flag = 0;
    }

    static float target_speed;
    static float target_turn;
    //取出指令
    extern osMessageQueueId_t BtCmdQueueHandle;
    Bt_Cmd_t *BtCmd;
    if (osMessageQueueGet(BtCmdQueueHandle, &BtCmd, 0, 0) == osOK)
    {
      // 把参数写入PID
      target_speed = BtCmd->speed;
      target_turn = BtCmd->turn;
      vPortFree(BtCmd);
    }
    //目标速度一阶低通滤波(10ms更新一次)
    static float Alpha_tgtSpeed = 0.9f;
    SpeedPID.target = Alpha_tgtSpeed*SpeedPID.target + (1-Alpha_tgtSpeed)*target_speed;

    static float Alpha_tgtTurn = 0.9f;
    TurnPID.target = Alpha_tgtTurn*TurnPID.target + (1-Alpha_tgtTurn)*target_turn;

    static uint8_t speed_max_time;
    if(AvgSpeed>5.0||AvgSpeed<-5.0)
    {
      //速度达到最大值，1200ms后自动停止
      //每10ms，如果进入if, speed_max_time+=10;
      speed_max_time+=10;
      if(speed_max_time>120)
      {
        Run_Flag=0;
        speed_max_time=0;
      }
    }
    



    


    if (Run_Flag)
    {
      // 速度环 50ms更新一次
      static uint8_t speed_updatecnt;
      speed_updatecnt++;
      if (speed_updatecnt > 5)
      {
        speed_updatecnt = 0;
        // 获取速度
        get_encoder();
        LeftSpeed = Encoder_Left / 44.0f / 0.05 / 21.3;
        RightSpeed = Encoder_Right / 44.0f / 0.05 / 21.3;
        AvgSpeed = (LeftSpeed + RightSpeed) / 2;
        DifSpeed = LeftSpeed - RightSpeed;

        SpeedPID.actual = AvgSpeed;
        PID_Calc(&SpeedPID);
        AnglePID.target = -SpeedPID.output;

        TurnPID.actual = DifSpeed;
        PID_Calc(&TurnPID);
        DifPWM = TurnPID.output;
      }

      // 角度环10ms
      AnglePID.actual = roll;
      PID_Calc(&AnglePID);
      AvgPWM = AnglePID.output;

      LeftPWM = AvgPWM + DifPWM / 2;
      RightPWM = AvgPWM - DifPWM / 2;

      Set_Motor_Speed(LeftPWM, RightPWM);

      uint32_t end = DWT->CYCCNT;
      run_time = end - start;
    }

    else
    {
      Set_Motor_Speed(0, 0);
      PID_Clear(&AnglePID);
      PID_Clear(&SpeedPID);
      PID_Clear(&TurnPID);
      speed_max_time=0;
    }

    // 测试延迟时间
    //  run_time_gapnow=tick;
    //  run_time_gap=run_time_gapnow-run_time_gaplast;
    //  run_time_gaplast=run_time_gapnow;
    // 绝对延迟10ms
    osDelayUntil(tick);
  }
}
