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
        .Kp = 253.19f,
        .Ki = 0,
        .Kd = 3.443f,
        .Out_Max = 3600,
        .Out_Min = -3600,
        .integ_limit = 2000,
        .Out_Offset = 130,
};

PID_t SpeedPID =
    {
        .Kp = 5.637f,
        .Ki = 0.47f,
        .Kd = 0,
        .Out_Max = 30,
        .Out_Min = -30, // 速度环控制角度环，角度最大+-15
        .integ_limit = 15,
};

PID_t TurnPID =
    {
        .Kp = 1000.0f,
        .Ki = 500.0f,
        .Kd = 0,
        .Out_Max = 3600,
        .Out_Min = -3600, // 转向环控制速度环，速度最大+-50
        .integ_limit = 3000,
};

PID_t PositionPID =
    {
        .Kp = 0.0508f, // 位置环比例系数（需实地调参）
        .Ki = 0,       // 位置环积分系数
        .Kd = 0.2244f,
        .Out_Max = 20, // 输出限幅 ±10（给速度环的目标速度）
        .Out_Min = -20,
};

short ax, ay, az, gx, gy, gz;    // 加速度计和陀螺仪的原始数据
float roll_acc, roll_gyro, roll; // roll角的加速度计测量值，陀螺仪测量值，最终值
uint8_t Run_Flag = 0;            // 运行标志位，0为不运行，1为运行

// ===== PWM相关变量 =====
int16_t AvgPWM;
int16_t LeftPWM, RightPWM;
int16_t DifPWM = 0;

// ===== 速度环相关变量 =====
float LeftSpeed;
float RightSpeed;
float AvgSpeed;
float DifSpeed;

// ===== 位置环相关变量 =====
float Position_Left = 0;   // 左轮累计位置（编码器脉冲数）
float Position_Right = 0;  // 右轮累计位置
float AvgPosition = 0;     // 平均位置
float target_position = 0; // 目标位置
// 前馈系数
float K_qk = 1.177f;

// 执行时间s
uint32_t run_time;
uint32_t run_time_gaplast, run_time_gapnow, run_time_gap;

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
    if(Run_Flag) LED_ON();
    else LED_OFF();

    static float target_speed;
    static float target_turn;
    // 取出指令
    extern osMessageQueueId_t BtCmdQueueHandle;
    Bt_Cmd_t *BtCmd;
    if (osMessageQueueGet(BtCmdQueueHandle, &BtCmd, 0, 0) == osOK)
    {
      // 把参数写入PID
      target_speed = BtCmd->speed;
      target_turn = BtCmd->turn;
      vPortFree(BtCmd);
    }

    static float Alpha_tgtTurn = 0.9f;
    TurnPID.target = Alpha_tgtTurn * TurnPID.target + (1 - Alpha_tgtTurn) * target_turn;

    // ==== 防止超速 =====
    static uint8_t speed_max_time;
    if (AvgSpeed > 4.5 || AvgSpeed < -4.5)
    {
      // 速度达到最大值，1200ms后自动停止
      // 每10ms，如果进入if, speed_max_time+=1;
      speed_max_time += 1;
      if (speed_max_time > 100)
      {
        Run_Flag = 0;
        speed_max_time = 0;
      }
    }

    //===== 控制逻辑 =====
    if (Run_Flag)
    {
      // 速度环 50ms更新一次
      static uint8_t speed_updatecnt;
      speed_updatecnt++;
      if (speed_updatecnt == 5)
      {
        speed_updatecnt = 0;
        // 获取速度
        get_encoder();
        LeftSpeed = Encoder_Left / 44.0f / 0.05 / 21.3;
        RightSpeed = Encoder_Right / 44.0f / 0.05 / 21.3;
        AvgSpeed = (LeftSpeed + RightSpeed) / 2;
        DifSpeed = LeftSpeed - RightSpeed;

        // 编码器累计得到位置
        Position_Left += LeftSpeed;
        Position_Right += RightSpeed;
        AvgPosition = (Position_Left + Position_Right) / 2.0f;

        /* 蓝牙速度一阶滤波 */
        static float Bt_V;
        static float Alpha_tgtSpeed = 0.9f;
        Bt_V = Alpha_tgtSpeed * Bt_V + (1 - Alpha_tgtSpeed) * target_speed;
        target_position += Bt_V*10 * 0.05f;
        PositionPID.target = target_position;
        PositionPID.actual = AvgPosition;
        PID_Calc(&PositionPID);
        SpeedPID.target = PositionPID.output;

        // 速度环计算
        SpeedPID.actual = AvgSpeed;
        PID_Calc(&SpeedPID);
        AnglePID.target = -SpeedPID.output - K_qk * (target_position - AvgPosition);

        // 角度环计算
        TurnPID.actual = DifSpeed;
        PID_Calc(&TurnPID);
        DifPWM = TurnPID.output;
      }

      // 角度环10ms
      AnglePID.actual = roll;
      PID_AngleCalc(&AnglePID,(gx) / 32768.0f * 2000);
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
      PID_Clear(&PositionPID);

      Position_Left = 0;
      Position_Right = 0;
      AvgPosition = 0;

      speed_max_time = 0;
      LeftSpeed = 0;
      RightSpeed = 0;
      AvgSpeed = 0;

      LeftPWM = 0;
      RightPWM = 0;
      AvgPWM = 0;
    }

    // 测试延迟时间
    //  run_time_gapnow=tick;
    //  run_time_gap=run_time_gapnow-run_time_gaplast;
    //  run_time_gaplast=run_time_gapnow;
    // 绝对延迟10ms
    osDelayUntil(tick);
  }
}

void LED_ON(void)
{
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
}

void LED_OFF(void)
{
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
}

