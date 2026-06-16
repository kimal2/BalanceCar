# BalanceCar

基于 STM32F103C8T6 的两轮自平衡小车，采用**四级串级 PID** + FreeRTOS，支持蓝牙遥控与位置保持。

## 硬件

| 组件 | 型号 | 接口 |
|------|------|------|
| 主控 | STM32F103C8T6 | — |
| 姿态传感器 | MPU6050 | I2C1 (PB6 SCL, PB7 SDA) |
| 电机驱动 | TB6612 | TIM1 CH1/CH4 (PA8/PA11) |
| 编码器 | 霍尔编码器 | TIM2 (PA0/PA1), TIM4 (PB6/PB7) |
| 蓝牙 | HC-05 | USART3 (PB10 TX, PB11 RX) |
| OLED | 0.96" I2C | PB8 SCL, PB9 SDA |
## 软件架构

```
BalanceCar/
├── Core/               # STM32CubeMX 生成的外设初始化
├── Drivers/            # HAL 库 & CMSIS
├── Middlewares/        # FreeRTOS
├── Car_Drivers/        # 外设驱动层
│   ├── mpu6050.c/h     # MPU6050 初始化 & 数据读取
│   ├── encoder.c/h     # 编码器读数
│   ├── motor.c/h       # 电机 PWM 控制
│   ├── pid.c/h         # PID 控制器 (含角度环微分先行)
│   ├── bluetooth.c/h   # 蓝牙串口收发 & 指令解析
│   ├── oled.c/h        # OLED 显示驱动
│   └── IIC.c/h         # 软件 I2C
├── My_Tasks/           # FreeRTOS 任务层
│   ├── PIDTask.c/h     # 平衡控制任务 (核心)
│   └── Display.c/h     # OLED 显示 & UART 调试绘图
├── MDK-ARM/            # Keil MDK 工程
└── BalanceCar.ioc      # STM32CubeMX 配置
```

## 控制算法 — 四级串级 PID

```
蓝牙摇杆
   │
   ▼
┌──────────┐     ┌─────────┐     ┌─────────┐
│ 位置环 PID│───▶ │速度环 PID│───▶│角度环 PID│───▶ 电机 PWM
│  (50ms)  │     │  (50ms) │     │  (10ms) │
└──────────┘     └─────────┘     └─────────┘
     ▲               ▲               ▲
 编码器积分      编码器速度      MPU6050 姿态
     │
┌──────────┐
│ 转向环 PID │───▶ 差速 PWM
│  (50ms)   │
└──────────┘
     ▲
 编码器差速
```

### 各环说明

| 环路 | 周期 | 输入 | 输出 | 作用 |
|------|------|------|------|------|
| **位置环** | 50ms | 编码器积分位置 | 目标速度 | 位置保持、蓝牙速度积分生成虚拟目标 |
| **速度环** | 50ms | 编码器速度 | 目标倾角 | 控制前进/后退速度 |
| **角度环** | 10ms | MPU6050 Roll 角 | PWM | 保持车身直立 |
| **转向环** | 50ms | 左右轮差速 | 差速 PWM | 转弯控制 |

### 位置环设计

- **无模式切换**：蓝牙速度积分生成虚拟目标位置 `target_position += Bt_V × dt`
- **防积分发散**：`target_position` 限幅跟随实际位置 ±10
- **位置前馈**：`K_qk × (target_position - actual_position)` 直接叠加到角度环目标，提升响应
- **摇杆回中**：`Bt_V ≈ 0` 时 target_position 不动，自动位置保持，坡道也能停住

### 姿态解算

- 互补滤波：`roll = 0.02 × roll_acc + 0.98 × roll_gyro`
- 加速度计角度：`roll_acc = atan2(ay, az)`
- 陀螺仪积分融入互补滤波

### 角度环微分先行

`PID_AngleCalc` 直接用陀螺仪角速度做微分项，避免目标突变引发微分 kick：

```
output = Kp × error + Ki × integral - Kd × gyro_rate
```

### 安全保护

- 倾角超过 ±50° 自动停机
- 速度达到 4.5 rev/s 持续 1s 自动停机
- 目标速度一阶低通滤波（α=0.9）
- 停车时自动清零所有 PID 状态和位置累计

## FreeRTOS 任务

| 任务 | 优先级 | 周期 | 功能 |
|------|--------|------|------|
| `TaskBalance` | High | 10ms | PID 控制核心循环 |
| `TaskDisplay` | Normal | 50ms | OLED 显示 + plot 调试输出 |
| `TaskBtCmd` | Normal | 事件驱动 | 蓝牙指令接收与解析 |

任务间通过 `BtCmdQueue` 消息队列传递蓝牙指令。

## 蓝牙控制协议

### 摇杆数据 (Tag: `joystick`)

格式：`[joystick,LH,LV,RH,RV]`

- LV → `speed = LV / 25.0`（速度范围约 ±5）
- RH → `turn = RH / 25.0`（转向范围约 ±5）

### 滑杆调参 (Tag: `slider`)

格式：`[slider,Name,Value]`

| 滑杆名称 | 作用 |
|----------|------|
| `AngleKp/Ki/Kd` | 角度环 PID |
| `SpeedKp/Ki/Kd` | 速度环 PID |
| `TurnKp/Ki/Kd` | 转向环 PID |
| `PositionKp/Ki/Kd` | 位置环 PID |
| `Tpos` | 手动设置目标位置 |
| `offset` | 角度环输出死区偏移 |
| `kqk` | 位置前馈系数 |

### 按键 (Tag: `key`)

| 按键 | 动作 | 功能 |
|------|------|------|
| `switch` | down | 启停切换 (`Run_Flag ^= 1`) |

### UART 调试输出

Display 任务每 50ms 通过 USART3 DMA 发送 plot 数据：

```
[plot,SpeedPID.target,SpeedPID.actual,AnglePID.target,AnglePID.actual,Bt_V]
```

可用于上位机实时波形显示。

## 编译与烧录

1. **Keil MDK 5** 打开 `MDK-ARM/BalanceCar.uvprojx` → Build → 生成 hex
2. ST-Link / J-Link / 串口 ISP 烧录
3. 也可用 STM32CubeMX 打开 `BalanceCar.ioc` 重新生成工程

## License

MIT
