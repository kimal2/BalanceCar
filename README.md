# BalanceCar

基于 STM32F103C8T6 的两轮自平衡小车，采用**串级 PID 控制** + FreeRTOS 实时操作系统，支持蓝牙遥控。

## 硬件方案

| 组件 | 型号 | 接口 |
|------|------|------|
| 主控 | STM32F103C8T6 | — |
| 姿态传感器 | MPU6050 (6轴) | I2C1 (PB6 SCL, PB7 SDA) |
| 电机驱动 | TB6612 | TIM1 CH1/CH4 (PB12-15) |
| 编码器 | 霍尔编码器 | TIM2, TIM4 |
| 蓝牙 | HC-05 | USART3 (PB10 TX, PB11 RX) |
| OLED | 0.96" I2C | PB8 SCL, PB9 SDA |
| 超声波 | HC-SR04 | — |

## 软件架构

```
BalanceCar/
├── Core/               # STM32CubeMX 生成的外设初始化
│   ├── Inc/            # 头文件 (FreeRTOSConfig.h, gpio.h, i2c.h, tim.h, usart.h)
│   └── Src/            # 源文件 (main.c, freertos.c, 外设 HAL 初始化)
├── Drivers/            # HAL 库 & CMSIS
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
├── Middlewares/         # FreeRTOS
├── Car_Drivers/        # 外设驱动层
│   ├── mpu6050.c/.h    # MPU6050 初始化 & 数据读取
│   ├── encoder.c/.h    # 编码器读数
│   ├── motor.c/.h      # 电机 PWM 控制
│   ├── pid.c/.h        # PID 控制器通用实现
│   ├── bluetooth.c/.h  # 蓝牙串口收发 / 指令解析
│   ├── oled.c/.h       # OLED 显示驱动
│   ├── sr04.c/.h       # 超声波测距
│   └── IIC.c/.h        # 软件 I2C 实现
├── My_Tasks/           # FreeRTOS 任务层
│   ├── PIDTask.c/.h    # 平衡控制任务 (核心)
│   └── Display.c/.h    # 显示 & 调试输出任务
├── MDK-ARM/            # Keil MDK 工程文件
│   └── BalanceCar.uvprojx
└── BalanceCar.ioc      # STM32CubeMX 工程配置
```

## 控制算法 — 串级 PID

采用**三级串级 PID** 控制结构：

```
蓝牙目标速度/转向
       │
       ▼
  ┌─────────┐     ┌─────────┐
  │ 速度环 PID │ ──▶│ 角度环 PID │──▶ 电机 PWM
  │  (50ms)   │     │  (10ms)   │
  └─────────┘     └─────────┘
       ▲               ▲
  编码器反馈      MPU6050 姿态
       │
  ┌─────────┐
  │ 转向环 PID │──▶ 差速 PWM
  │  (50ms)   │
  └─────────┘
       ▲
  编码器差速反馈
```

- **角度环（内环，10ms）**：MPU6050 读取加速度计 + 陀螺仪 → 互补滤波得到 Roll 角 → PID 计算 PWM 输出，保持车身直立
- **速度环（外环，50ms）**：编码器读取左右轮速度 → PID 计算目标倾角，通过改变倾角来控制前进/后退速度
- **转向环（外环，50ms）**：左右轮差速 → PID 计算差速 PWM，叠加到电机输出实现转弯

### 姿态解算

- 互补滤波：`roll = α × roll_acc + (1-α) × roll_gyro`，α = 0.02
- 加速度计角度：`roll_acc = atan2(ay, az)`
- 陀螺仪积分：`roll_gyro += gx × dt`

### 安全保护

- 倾角超过 ±50° 自动停止
- 速度达到最大持续 1.2s 自动停止
- 目标值一阶低通滤波，避免指令突变

## FreeRTOS 任务

| 任务 | 优先级 | 周期 | 功能 |
|------|--------|------|------|
| `TaskBalance` | High | 10ms | PID 控制核心循环 |
| `TaskDisplay` | Normal | 50ms | OLED 显示 + UART 调试输出 |
| `TaskBtCmd` | Normal | — | 蓝牙指令接收与解析 |

任务间通过 `BtCmdQueue` 消息队列传递蓝牙指令。

## 编译与烧录

1. 使用 **Keil MDK 5** 打开 `MDK-ARM/BalanceCar.uvprojx`
2. 编译（Project → Build），生成 `BalanceCar.hex`
3. 通过 ST-Link / J-Link / 串口 ISP 烧录
4. 或使用 **STM32CubeMX** 打开 `BalanceCar.ioc` 重新生成工程

## 蓝牙控制协议

通过蓝牙串口发送速度/转向指令，格式由 `bluetooth.c` 解析（`Bt_Cmd_t { speed, turn }`），通过消息队列传入 PID 任务。

## License

MIT License
