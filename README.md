# RM-Control-System (RoboMasters控制系统)

## 项目简介

本项目是一个基于 **STM32F4** 微控制器和 **FreeRTOS** 实时操作系统的RoboMasters机器人控制系统。项目采用了[湖南大学跃鹿战队](https://github.com/HNUYueLuRM/basic_framework)开源的代码架构，实现了步兵机器人的完整控制功能。

## ✨ 主要功能

### 🎯 已实现功能

- **底盘控制**
  - 底盘跟随云台模式（CHASSIS_FOLLOW_GIMBAL_YAW）
  - 小陀螺模式全向移动（CHASSIS_ROTATE）
  - 云台自由模式下全向移动（CHASSIS_NO_FOLLOW）
  - 全向轮/麦轮运动学解算

- **云台控制**
  - Yaw轴和Pitch轴独立控制
  - 使用IMU陀螺仪数据进行反馈（来自哈尔滨工程大学）
  - 支持自由模式和陀螺仪模式切换

- **发射机构控制**
  - 摩擦轮启停控制
  - 拨盘单发/三发/连发模式
  - 弹舱盖开关控制

- **姿态解算系统**
  - BMI088六轴IMU数据采集
  - 四元数扩展卡尔曼滤波（EKF）姿态融合算法
  - 实时输出Roll/Pitch/Yaw角度和角速度/加速度数据

- **视觉通信系统**
  - DM-IMU模块解析姿态数据并转发给上位机
  - C板作为消息中转站
  - 支持USB虚拟串口（VCP）或UART通信方式

- **裁判系统交互**
  - 裁判系统UI界面绘制与刷新
  - 实时接收比赛数据（血量、热量、弹量等）
  - 多机通信支持

- **安全保护机制**
  - 底盘功率限制与控制
  - 枪口超热量保护
  - 系统守护进程监控

## 🏗️ 系统架构

### 硬件平台

- **主控芯片**: STM32F4xx系列
- **开发环境**: STM32CubeMX + Keil MDK-ARM / EIDE
- **实时操作系统**: FreeRTOS
- **数学运算库**: ARM DSP Lib (Cortex-M4F)

### 外设接口

| 外设 | 用途 |
|------|------|
| CAN1/CAN2 | DJI电机通信、DM-IMU数据接收 |
| USART1/3/6 | 裁判系统、调试串口、遥控器 |
| I2C2/I3 | BMI088 IMU传感器 |
| SPI1/SPI | 其他外设扩展 |
| USB | 虚拟串口(VCP)视觉通信 |
| TIM1/4/5/8/10 | PWM输出、编码器输入、系统时基 |

### 软件架构

```
┌─────────────────────────────────────────────────────┐
│                    应用层 (Application)              │
│  ┌──────────┬──────────┬──────────┬──────────────┐  │
│  │ Chassis  │  Gimbal  │  Shoot   │ Referee/UI   │  │
│  │ 底盘控制  │ 云台控制  │ 发射控制  │ 裁判系统UI    │  │
│  └────┬─────┴────┬─────┴────┬─────┴──────┬───────┘  │
│       │          │          │             │          │
│  ┌────▼──────────▼──────────▼─────────────▼───────┐  │
│  │            CMD命令处理层                        │  │
│  │     (遥控器解析 + 上位机指令 + 状态机)           │  │
│  └──────────────────────┬────────────────────────┘  │
│                         │                           │
│  ┌──────────────────────▼────────────────────────┐  │
│  │              中间件层 (Middleware)              │  │
│  │  ┌─────────┬──────────┬──────────┬──────────┐  │  │
│  │  │Message  │  Motor   │   INS    │  Power   │  │  │
│  │  │ Center  │  Task    │  Task    │ Control  │  │  │
│  │  │消息中心 │ 电机任务  │ 姿态解算  │ 功率控制  │  │  │
│  │  └─────────┴──────────┴──────────┴──────────┘  │  │
│  └──────────────────────┬────────────────────────┘  │
│                         │                           │
│  ┌──────────────────────▼────────────────────────┐  │
│  │              驱动层 (BSP Drivers)               │  │
│  │  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌─────┐  │  │
│  │  │ BSP_ │ │ BSP_ │ │ BSP_ │ │ BSP_ │ │DJI  │  │  │
│  │  │ CAN  │ │USART │ │ PWM  │ │ USB  │ │Motor│  │  │  │
│  │  └──────┘ └──────┘ └──────┘ └──────┘ └─────┘  │  │
│  └──────────────────────┬────────────────────────┘  │
│                         │                           │
│  ┌──────────────────────▼────────────────────────┐  │
│  │         HAL层 + FreeRTOS + ARM DSP             │  │
│  └────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

## 📁 项目结构

```
RM-Control-System-main/
├── Core/                          # STM32CubeMX生成的核心代码
│   ├── Inc/                       # 头文件
│   │   ├── main.h                 # 主程序头文件
│   │   ├── FreeRTOSConfig.h       # FreeRTOS配置
│   │   ├── can.h, dma.h, gpio.h   # 外设头文件
│   │   └── ...
│   └── Src/                       # 源文件
│       ├── main.c                 # 主程序入口
│       ├── freertos.c             # FreeRTOS任务创建
│       ├── stm32f4xx_it.c         # 中断处理函数
│       └── ...                    # 其他HAL驱动源文件
│
├── other/                         # 业务逻辑代码（核心模块）
│   ├── robot.h/c                  # 机器人初始化入口
│   ├── robot_def.h                # 机器人类型定义、参数配置
│   ├── robot_cmd.h/c              # 遥控器命令解析
│   ├── robot_task.h               # 主控制任务
│   │
│   ├── chassis.h/c                # 底盘控制模块
│   ├── gimbal.h/c                 # 云台控制模块
│   ├── shoot.h/c                  # 发射机构控制模块
│   │
│   ├── dji_motor.h/c              # DJI电机驱动库
│   ├── motor_def.h                # 电机参数定义
│   ├── motor_task.h/c             # 电机控制任务
│   │
│   ├── ins_task.h/c               # INS姿态解算任务
│   ├── BMI088driver.h/c           # BMI088 IMU驱动
│   ├── BMI088Middleware.h/c       # BMI088中间件
│   ├── QuaternionEKF.h/c          # 四元数EKF滤波算法
│   ├── kalman_filter.h/c          # 卡尔曼滤波工具
│   │
│   ├── dm_imu.h/c                 # DM-IMU通信模块
│   │
│   ├── referee_task.h/c           # 裁判系统任务
│   ├── referee_protocol.h         # 裁判系统协议定义
│   ├── referee_communication.h/c  # 裁判系统通信
│   ├── referee_UI.h/c             # 裁判系统UI绘制
│   ├── rm_referee.h/c             # 裁判系统底层驱动
│   │
│   ├── bsp_can.h/c                # CAN总线驱动
│   ├── bsp_usart.h/c              # USART串口驱动
│   ├── bsp_usb.h/c                # USB虚拟串口驱动
│   ├── bsp_pwm.h/c                # PWM输出驱动
│   ├── bsp_dwt.h/c                # DWT周期计数器
│   ├── bsp_temperature.h/c        # 温度传感器驱动
│   │
│   ├── controller.h/c             # PID控制器实现
│   ├── message_center.h/c         # 消息中心（发布-订阅模式）
│   ├── master_process.h/c         # 主控流程处理
│   ├── daemon.h/c                 # 系统守护进程
│   ├── power_control.h/c          # 功率控制模块
│   │
│   ├── remote_control.h/c         # 遥控器数据接收
│   ├── seasky_protocol.h/c        # 上位机通信协议
│   ├── vofa.h/c                   # VOFA+调试工具接口
│   ├── buzzer.h/c                 # 蜂鸣器控制
│   ├── alg_slope.h/c              # 斜坡函数（平滑控制）
│   ├── user_lib.h/c               # 用户工具库
│   ├── crc16.h/c, crc8.h/c        # CRC校验算法
│   └── general_def.h              # 通用定义
│
├── Drivers/                       # STM32 HAL驱动库
│   ├── CMSIS/                     # CMSIS标准接口
│   │   ├── Core/                  # Cortex-M4核心定义
│   │   ├── DSP/                   # ARM DSP数学库
│   │   └── Device/ST/STM32F4xx/   # 设备特定文件
│   └── STM32F4xx_HAL_Driver/      # STM32F4 HAL库
│
├── Middlewares/                   # 中间件
│   └── Third_Party/
│       └── FreeRTOS/              # FreeRTOS实时操作系统
│
├── USB_DEVICE/                    # USB设备驱动
│   ├── App/                       # 应用层
│   │   ├── usbd_cdc_if.h/c        # CDC类接口（虚拟串口）
│   │   ├── usbd_desc.h/c          # USB描述符
│   │   └── usb_device.h           # USB设备核心
│   └── Target/                    # 目标板配置
│       └── usbd_conf.h            # USB配置
│
├── MDK-ARM/                       # Keil工程文件
│   ├── .eide/                     # EIDE配置（VSCode插件）
│   └── RTE_Components.h           # 运行时环境组件
│
├── .vscode/                       # VSCode工作区配置
│   └── settings.json
│
├── 06_my_yuntai_test_02.ioc       # STM32CubeMX工程文件
├── .mxproject                     # CubeMX项目文件
└── readme.txt                     # 原始说明文档
```

## ⚙️ FreeRTOS任务分配

系统使用FreeRTOS进行多任务调度，主要任务如下：

| 任务名 | 优先级 | 栈大小 | 功能描述 |
|--------|--------|--------|----------|
| **defaultTask** | Normal | 128 words | USB设备初始化与维护 |
| **MOTORTASK** | Normal | 256 words | 电机控制循环 + 功率控制（1ms周期） |
| **DAEMONTASK** | Normal | 521 words | 系统守护进程，监控各模块状态 |
| **ROBOTTASK** | Normal | 1024 words | 主控制任务，协调底盘/云台/发射 |
| **INS_TASK** | Normal | 1024 words | INS姿态解算（1ms周期） |
| **StartUI_TASK** | Normal | 512 words | 裁判系统UI交互任务 |

## 🔧 配置说明

### 机器人类型配置 ([robot_def.h](other/robot_def.h))

```c
// 开发板类型（三选一）
#define ONE_BOARD           // 单板控制整车（推荐）
// #define CHASSIS_BOARD      // 底盘板（双板方案）
// #define GIMBAL_BOARD       // 云台板（双板方案）

// 机器人底盘类型（二选一）
#define ROBOT_OMNIDIRECTIONAL  // 全向轮底盘（当前使用）
// #define ROBOT_MECANUM        // 麦轮底盘

// 视觉通信方式
#define VISION_USE_VCP        // 使用USB虚拟串口
// #define VISION_USE_UART     // 使用UART串口
```

### 关键机械参数

在 [robot_def.h](other/robot_def.h) 中可根据实际机械结构调整以下参数：

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `YAW_CHASSIS_ALIGN_ECD` | 7834 | 云台与底盘对齐时的编码器值（全向轮） |
| `PITCH_HORIZON_ECD` | 3750 | 云台水平位置编码器值 |
| `WHEEL_BASE` | 370mm | 底盘纵向轴距 |
| `TRACK_WIDTH` | 420mm | 底盘横向轮距 |
| `RADIUS_WHEEL` | 60mm | 轮子半径 |
| `REDUCTION_RATIO_WHEEL` | 19.0 | 电机减速比 |
| `ONE_BULLET_DELTA_ANGLE` | 45° | 单发拨盘转动角度 |

### 控制模式说明

#### 底盘模式 (`chassis_mode_e`)
- `CHASSIS_ZERO_FORCE`: 电流零输入（停止状态）
- `CHASSIS_ROTATE`: 小陀螺模式（原地旋转）
- `CHASSIS_NO_FOLLOW`: 不跟随云台，全向平移
- `CHASSIS_FOLLOW_GIMBAL_YAW`: 跟随云台yaw角
- `CHASSIS_ROTATE_REVERSE`: 反转小陀螺

#### 云台模式 (`gimbal_mode_e`)
- `GIMBAL_ZERO_FORCE`: 电流零输入
- `GIMBAL_FREE_MODE`: 自由模式（电机编码器反馈）
- `GIMBAL_GYRO_MODE`: 陀螺仪模式（IMU数据反馈）

#### 发射模式 (`shoot_mode_e`, `friction_mode_e`, `loader_mode_e`, `lid_mode_e`)
- 支持摩擦轮启停、弹舱盖开关
- 拨盘支持：停止/反转/单发/三发/连发

## 🎮 遥控器操作手册

### 遥控器布局说明

```
┌─────────────────────────────────────┐
│  ┌─────┐                    ┌─────┐ │
│  │ 左  │                    │ 右  │ │
│  │ 开  │                    │ 开  │ │
│  │ 关  │                    │ 关  │ │
│  └──┬──┘                    └──┬──┘ │
│     │                          │    │
│  ┌──▼──┐                   ┌──▼──┐ │
│  │ 左  │                   │ 右  │ │
│  │ 摇  │                   │ 摇  │ │
│  │ 杆  │                   │ 杆  │ │
│  └─────┘                   └─────┘ │
│                                     │
│              [ 拨轮 ]               │
└─────────────────────────────────────┘

左侧开关 (switch_left)     右侧开关 (switch_right)
左摇杆 (rocker_l)          右摇杆 (rocker_r)
侧边拨轮 (dial)
```

### 开关功能详解

#### 📍 左侧开关 - 控制模式切换

| 位置 | 状态值 | 功能 | 说明 |
|------|--------|------|------|
| **上** ⬆️ | `RC_SW_UP` (1) | **键鼠控制模式** | 使用键盘和鼠标控制机器人（配合上位机） |
| **中** ➡️ | `RC_SW_MID` (3) | **急停状态** | 所有电机停止输出，云台保持当前位置 |
| **下** ⬇️ | `RC_SW_DOWN` (2) | **遥控器调试模式** | 使用遥控器摇杆进行手动控制和调试 |

> ⚠️ **安全提示**: 左侧开关拨到中间位置会立即触发急停，所有电机停止工作！

#### 📍 右侧开关 - 底盘运动模式（仅遥控器模式下生效）

| 位置 | 状态值 | 功能 | 运动特性 |
|------|--------|------|----------|
| **上** ⬆️ | `RC_SW_UP` (1) | **自由模式** (`CHASSIS_NO_FOLLOW`) | 底盘不跟随云台，可全向平移移动 |
| **中** ➡️ | `RC_SW_MID` (3) | **跟随模式** (`CHASSIS_FOLLOW_GIMBAL_YAW`) | 底盘yaw轴跟随云台方向，适合正常行驶 |
| **下** ⬇️ | `RC_SW_DOWN` (2) | **小陀螺模式** (`CHASSIS_ROTATE`) | 原地高速旋转，用于躲避弹道或展示 |

### 摇杆操作说明

#### 🕹️ 左摇杆（遥控器模式下）

- **左右拨动** (`rocker_l_`): 控制云台Yaw轴转动
- **上下拨动** (`rocker_l1`): 控制云台Pitch轴俯仰（当前代码中被视觉数据覆盖）

#### 🕹️ 右摇杆（底盘速度控制）

- **左右拨动** (`rocker_r_`): 控制底盘横向速度 `vx`
  - 向右 → 正值（向右平移）
  - 向左 → 负值（向左平移）
  - 速度系数: `40.0` （即满量程对应 ±660 * 40 = ±26400）

- **上下拨动** (`rocker_r1`): 控制底盘纵向速度 `vy`
  - 向上 → 正值（前进）
  - 向下 → 负值（后退）
  - 速度系数: `40.0`

#### 📊 摇杆数值范围

```
最小值: 364   (最左/最下)
中位值: 1024  (居中)
最大值: 1684  (最右/最上)

有效范围: [-660, +660] (相对于中位的偏移量)
```

### 侧边拨轮功能

#### 🔘 拨轮向上（正值区域）- 摩擦轮控制

| 操作 | 效果 | 说明 |
|------|------|------|
| **从下往上拨过500** | 切换摩擦轮开关 | 边沿检测，每次越过阈值切换一次状态 |

**工作流程**:
```
拨轮 > 500 (上升沿) → Friction_signal 取反 → 摩擦轮开启/关闭
```

#### 🔫 拨轮向下（负值区域）- 发射射速控制（需摩擦轮已开启且转速>30000）

| 拨轮范围 | 射击模式 | 射速 (发/秒) | 适用场景 |
|----------|----------|-------------|---------|
| `-50 ~ 0` | `LOAD_STOP` | 停止发射 | 待机/停止 |
| `-250 ~ -50` | `LOAD_BURSTFIRE` | 4 Hz | 低速点射 |
| `-450 ~ -250` | `LOAD_BURSTFIRE` | 8 Hz | 中速连射 |
| `-660 ~ -450` | `LOAD_BURSTFIRE` | 21 Hz | 高速连射/狂暴模式 |

> ⚠️ **安全保护**: 只有当摩擦轮转速 > 30000 时才允许拨弹，防止卡弹！

### 键鼠操作模式（左侧开关在上时）

虽然当前代码中键鼠模式的部分功能被注释掉，但预留了以下接口：

#### ⌨️ 键盘按键定义

| 按键 | 定义宏 | 预留功能 | 当前状态 |
|------|--------|----------|---------|
| **W** | `Key_W` | 前进 | 已注释 |
| **S** | `Key_S` | 后退 | 已注释 |
| **A** | `Key_A` | 左移 | 已注释 |
| **D** | `Key_D` | 右移 | 已注释 |
| **Q** | `Key_Q` | 切换小陀螺模式 | 已注释 |
| **E** | `Key_E` | 切换反转小陀螺 | 已注释 |
| **R** | `Key_R` | 预留 | 未使用 |
| **F/G** | `Key_F/G` | 预留 | 未使用 |
| **Z/X/C/V/B** | - | 预留 | 部分使用 |
| **Shift** | `Key_Shift` | 加速/功能键 | 已注释 |
| **Ctrl** | `Key_Ctrl` | 组合键 | ✅ **Ctrl+Z = 急停** |

#### 🖱️ 鼠标操作

| 操作 | 预留功能 | 当前状态 |
|------|---------|---------|
| **鼠标移动X/Y** | 云台Yaw/Pitch微调 | 已注释 |
| **左键按下** | 发射（连发） | 已注释 |
| **右键按下** | 预留 | 未使用 |

### 组合键与特殊操作

#### 🚨 急停功能（多种方式）

| 触发方式 | 条件 | 效果 | 恢复方式 |
|---------|------|------|---------|
| **左侧开关拨到中间** | 任意时刻 | 所有电机立即停止 | 拨离中间位置即可恢复 |
| **Ctrl + Z** | 左侧开关在【上】(键鼠模式) | 进入急停状态 | 再次按 Ctrl+Z 切换回正常 |
| **遥控器断开连接** | Daemon监控检测到 | 自动急停 | 重新连接遥控器 |

#### 🔄 系统复位（已预留，当前注释）

```c
// Ctrl + X → NVIC_SystemReset()  // 系统软复位
// 注意：此功能当前被注释，如需启用请取消 robot_cmd.c 中相关注释
```

### 实际操作示例

#### 场景1：遥控器调试底盘运动

```
步骤：
1. 左侧开关拨到 [下] → 进入遥控器模式
2. 右侧开关拨到 [中] → 选择跟随模式（正常行驶）
3. 右摇杆:
   - 向上推 → 机器人前进
   - 向下拉 → 机器人后退
   - 向左推 → 机器人向左平移
   - 向右推 → 机器人向右平移
4. 左摇杆左右拨 → 云台Yaw转动
5. 侧边拨轮向上拨一次 → 开启摩擦轮
6. 侧边拨轮向下拨到不同位置 → 不同射速射击
```

#### 场景2：小陀螺模式躲避

```
步骤：
1. 左侧开关拨到 [下]
2. 右侧开关拨到 [下] → 小陀螺模式激活
3. 机器人开始原地旋转（可通过右摇杆叠加移动）
4. 配合摩擦轮和发射进行旋转射击
```

#### 场景3：紧急情况处理

```
发现异常时：
方法1（最快）：立即将左侧开关拨到 [中间位置] → 全部停止
方法2（键鼠模式）：按住 Ctrl + Z → 进入急停

恢复正常：
- 将左侧开关拨回 [上] 或 [下] → 机器人重新启动控制
```

---

## 🔧 改车调参指南

当你的机器人机械结构发生变化或更换零件时，需要修改以下参数。本章节将详细列出所有需要调整的地方。

### 一、机械参数配置

#### 📏 文件位置：[robot_def.h](other/robot_def.h)

##### 1. 底盘尺寸参数（必改！）

```c
// ==================== 底盘几何参数 ====================
#define WHEEL_BASE           370    // 纵向轴距 (mm) ⚠️ 必须实测
#define TRACK_WIDTH          420    // 横向轮距 (mm) ⚠️ 必须实测
#define RADIUS_WHEEL         60     // 轮子半径 (mm) ⚠️ 根据轮胎型号确认
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比 ⚠️ 根据电机型号确认
```

**如何测量**：
- **WHEEL_BASE**: 前轮中心连线到后轮中心连线的垂直距离
- **TRACK_WIDTH**: 左轮中心连线到右轮中心连线的垂直距离
- **RADIUS_WHEEL**: 轮胎外径的一半（查看轮胎规格或用卡尺测量）
- **REDUCTION_RATIO_WHEEL**: 
  - M3508电机 → 19:1 (默认值)
  - 如使用其他型号电机，请查阅电机手册

**影响范围**：
- 运动学解算的准确性（直接影响全向移动轨迹）
- 速度闭环控制的精度
- 功率计算的基准值

##### 2. 云台零点参数（必改！）

```c
// ==================== 云台机械零点 ====================
#ifdef ROBOT_OMNIDIRECTIONAL
#define YAW_CHASSIS_ALIGN_ECD   7834   // ⚠️ Yaw电机编码器零点值
#endif

#ifdef ROBOT_MECANUM
#define YAW_CHASSIS_ALIGN_ECD   5470   // ⚠️ Yaw电机编码器零点值（麦轮车）
#endif

#define PITCH_HORIZON_ECD      3750   // ⚠️ Pitch水平位置编码器值
#define YAW_ECD_GREATER_THAN_4096  1   // 编码器值是否大于4096（0或1）
```

**如何校准**：

**Step 1 - 校准Yaw零点**：
```
1. 将云台Yaw轴机械对准底盘正前方（车头方向）
2. 读取当前Yaw电机编码器原始值（通过调试串口或VOFA+观察）
3. 将该值填入 YAW_CHASSIS_ALIGN_ECD
4. 如果该值 > 4096，设置 YAW_ECD_GREATER_THAN_4096 = 1
   如果该值 < 4096，设置 YAW_ECD_GREATER_THAN_4096 = 0
```

**Step 2 - 校准Pitch零点**：
```
1. 将云台Pitch轴调整到完全水平状态（使用水平仪）
2. 读取当前Pitch电机编码器原始值
3. 将该值填入 PITCH_HORIZON_ECD
```

**验证方法**：
- 上电后云台应自动归中到正前方水平位置
- 如果Yaw偏转，调整 `YAW_CHASSIS_ALIGN_ECD`
- 如果Pitch抬头/低头，调整 `PITCH_HORIZON_ECD`

##### 3. 发射机构参数

```c
// ==================== 发射机构参数 ====================
#define ONE_BULLET_DELTA_ANGLE   45    // 单发拨盘转动角度 (度)
#define REDUCTION_RATIO_LOADER   36.0f // 拨盘减速比（英雄用19.0f）
#define NUM_PER_CIRCLE           10    // 拨盘一圈装载子弹数
```

**何时需要修改**：
- 更换了拨盘电机型号 → 修改 `REDUCTION_RATIO_LOADER`
- 机械结构改动导致拨盘行程变化 → 修改 `ONE_BULLET_DELTA_ANGLE`
- 更改了弹仓设计 → 修改 `NUM_PER_CIRCLE`

##### 4. IMU安装误差补偿

```c
// ==================== IMU方向配置 ====================
#define GYRO2GIMBAL_DIR_YAW    1   // Yaw陀螺仪方向（1相同, -1相反）
#define GYRO2GIMBAL_DIR_PITCH  1   // Pitch陀螺仪方向
#define GYRO2GIMBAL_DIR_ROLL   1   // Roll陀螺仪方向
```

**如何判断方向**：
```
测试方法：
1. 让云台Yaw轴向正方向（顺时针）转动
2. 观察IMU输出的Yaw角度变化
   - 如果角度增加 → 设置为 1
   - 如果角度减小 → 设置为 -1

3. 同理测试Pitch和Roll轴
```

**典型问题现象**：
- 云台向右转但反馈显示向左 → `GYRO2GIMBAL_DIR_YAW` 设为 -1
- Pitch控制反向（抬头变低头）→ `GYRO2GIMBAL_DIR_PITCH` 设为 -1

##### 5. 云台限位角度

```c
#define PITCH_MAX_ANGLE  0   // Pitch最大角度 (度) ⚠️ 需要设置实际值
#define PITCH_MIN_ANGLE  0   // Pitch最小角度 (度) ⚠️ 需要设置实际值
```

**如何确定**：
- 测量云台Pitch轴机械限位（避免撞限位损坏电机）
- 典型步兵机器人：约 ±25° 或根据机械设计图纸
- **重要**: 这两个值当前都是0，需要根据实际情况填写！

---

### 二、电机配置

#### 🔧 文件位置：[chassis.c](other/chassis.c) 和 [gimbal.c](other/gimbal.c)

##### 1. 底盘电机配置 ([chassis.c](other/chassis.c#L80-L120))

```c
Motor_Init_Config_s chassis_motor_config = {
    .can_init_config.can_handle = &hcan1,        // CAN总线句柄
    .controller_param_init_config = {
        .speed_PID = { ... },                     // 速度环PID参数
        .current_PID = { ... },                   // 电流环PID参数
    },
    .motor_type = M3508,                         // 电机型号
};

// 四个电机的CAN ID和转向配置
chassis_motor_config.can_init_config.tx_id = 2;   // 左前电机 CAN ID
chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE; // 转向

chassis_motor_config.can_init_config.tx_id = 1;   // 右前电机 CAN ID
chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

chassis_motor_config.can_init_config.tx_id = 3;   // 左后电机 CAN ID
chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;

chassis_motor_config.can_init_config.tx_id = 4;   // 右后电机 CAN ID
chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
```

**需要修改的情况**：

| 场景 | 修改内容 |
|------|---------|
| **更换电机型号** | `.motor_type` 改为 M2006/M3508/GM6020 等 |
| **更改CAN ID分配** | 修改每个电机的 `tx_id` (1-4) |
| **电机转向错误** | 修改 `motor_reverse_flag`: `NORMAL` ↔ `REVERSE` |
| **车轮装反** | 对应电机的 `motor_reverse_flag` 取反 |

**电机转向调试方法**：
```
1. 给底盘发送前进指令（vy为正值）
2. 观察四个轮子的转动方向：
   - 左前、右后应向前转
   - 右前、左后应向后转（全向轮）
3. 如果某个轮子方向错误，修改对应的 motor_reverse_flag
```

##### 2. 云台电机配置 ([gimbal.c](other/gimbal.c#L70-L150))

```c
// Yaw电机配置
Motor_Init_Config_s yaw_config = {
    .can_init_config = {
        .can_handle = &hcan2,
        .tx_id = 4,                        // Yaw电机 CAN ID
    },
    .controller_param_init_config = {
        .angle_PID = { Kp, Ki, Kd, ... },  // 角度环PID
        .speed_PID = { Kp, Ki, Kd, ... },  // 速度环PID
        .other_angle_feedback_ptr = &DM_IMU_DATA->yaw,  // IMU反馈源
        .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
    },
    .controller_setting_init_config = {
        .angle_feedback_source = OTHER_FEED,  // 使用IMU数据
        .speed_feedback_source = OTHER_FEED,
        .outer_loop_type = ANGLE_LOOP,        // 外环为角度环
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,  // 双闭环
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,       // 电机方向
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL, // 反馈方向
    },
    .motor_type = GM6020,                    // 云台电机型号
};

// Pitch电机配置类似...
```

**需要修改的情况**：

| 参数 | 修改时机 | 影响因素 |
|------|---------|---------|
| `tx_id` | 更换CAN口或ID分配 | 硬件接线 |
| `motor_type` | 更换云台电机 | GM6020是常用型号 |
| `motor_reverse_flag` | 云台转向错误 | 安装方向 |
| `feedback_reverse_flag` | 反馈数据方向错误 | IMU安装方向 |
| `other_angle_feedback_ptr` | 更换IMU数据源 | BMI088 vs DM-IMU |
| PID参数 | 控制效果不佳 | 机械惯量、摩擦等 |

---

### 三、PID控制器调参

#### 📊 文件位置：各模块的 .c 文件

##### 1. 底盘速度环/电流环PID ([chassis.c](other/chassis.c))

```c
static float chassis_SPEED_PID[3] = {4.5f, 0.0f, 0.0f};   // Kp, Ki, Kd
static float chassis_CURRENT_PID[3] = {0.5f, 0.0f, 0.0f};
```

**调参建议**：

| 问题 | 调整方向 | 方法 |
|------|---------|------|
| 响应太慢 | 增大Kp | 先从较小值逐步增加 |
| 震荡/超调 | 减小Kp 或增大Kd | 引入微分抑制震荡 |
| 稳态误差大 | 增加Ki | 注意积分饱和 |
| 噪声太大 | 增大Kd 或添加低通滤波 | `Output_LPF_RC` 参数 |

##### 2. 云台Yaw轴PID ([gimbal.c](other/gimbal.c))

```c
// 角度环 (外环)
float Yaw_Angle_Kp = 0.9, Yaw_Angle_Ki = 0.1, Yaw_Angle_Kd = 0.08;
// 速度环 (内环)
float Yaw_speed_Kp = 1300, Yaw_Speed_Ki = 300, Yaw_Speed_Kd = 0;
```

**典型调参流程**：
```
1. 先调速度环（内环）：
   - Ki=0, Kd=0, 从小增大Kp直到出现轻微震荡
   - 加入Kd消除震荡
   - 最后加入Ki消除稳态误差

2. 再调角度环（外环）：
   - 同样先调Kp，再调Kd，最后Ki
   - 角度环响应应比速度环慢3-5倍
```

##### 3. 云台Pitch轴PID ([gimbal.c](other/gimbal.c))

```c
float Pitch_Angle_Kp = 0.3, Pitch_Angle_Ki = 0, Pitch_Angle_Kd = 0;
float Pitch_speed_Kp = 4200, Pitch_Speed_Ki = 0, Pitch_Speed_Kd = 0;

float pitch_Gravity_Compensation = 0;  // 重力补偿（待整定）
```

**特殊注意**：
- Pitch轴受重力影响，可能需要重力补偿（`Gravity_Compensation`）
- 如果Pitch轴总是向下掉，增大重力补偿值或检查机械平衡

##### 4. 前馈增益 ([gimbal.c](other/gimbal.c))

```c
static float yaw_gyro_feedforward = -0.472267985;   // Yaw陀螺仪前馈
static float pitch_gyro_feedforward = 0.5;            // Pitch陀螺仪前馈
```

**作用**: 提高动态响应速度，减少跟踪误差  
**调整**: 如果云台跟踪迟缓，适当增大前馈系数

---

### 四、通信与硬件配置

#### 🔌 文件位置：[robot_def.h](other/robot_def.h)、[freertos.c](Core/Src/freertos.c)

##### 1. 开发板类型选择

```c
// ⚠️ 只能选择一个！否则编译报错
#define ONE_BOARD          // 单板控制整车（推荐，当前使用）
// #define CHASSIS_BOARD   // 底盘板（双板方案）
// #define GIMBAL_BOARD    // 云台板（双板方案）
```

##### 2. 机器人底盘类型

```c
#define ROBOT_OMNIDIRECTIONAL  // 全向轮底盘（当前使用）
// #define ROBOT_MECANUM       // 麦轮底盘
```

**影响范围**：
- 运动学解算公式（全向轮 vs 麦轮不同）
- 默认的Yaw零点编码器值
- 电机转向配置

##### 3. 视觉通信方式

```c
#define VISION_USE_VCP     // USB虚拟串口（推荐，当前使用）
// #define VISION_USE_UART  // 硬件串口
```

**切换影响**：
- VCP: 使用USB口，需要初始化USB_DEVICE，在 `defaultTask` 中调用 `MX_USB_DEVICE_Init()`
- UART: 使用USART1，需要修改 `VisionInit(&huart1)` 的参数

##### 4. 外设引脚映射

**文件**: STM32CubeMX生成的文件 (`Core/Src/*.c`, `Core/Inc/*.h`)

如果更换了MCU型号或引脚分配，需要：
1. 打开 `06_my_yuntai_test_02.ioc` 文件（STM32CubeMX工程）
2. 重新配置引脚
3. 生成代码
4. 或者直接修改对应的 `mx_*_init()` 函数

**常用外设对应关系**：

| 外设 | 引脚 | 用途 | 修改文件 |
|------|------|------|---------|
| CAN1 | - | 底盘电机通信 | `can.c`, `can.h` |
| CAN2 | - | 云台电机 + DM-IMU | `can.c`, `can.h` |
| USART3 | PA10/PA11 | 遥控器接收 | `usart.c`, `usart.h` |
| USART6 | - | 裁判系统 | `usart.c`, `usart.h` |
| I2C2/I3 | - | BMI088 IMU | `i2c.c`, `i2c.h` |
| USB | PA11/PA12 | 视觉通信(VCP) | USB_DEVICE目录 |

---

### 五、改车检查清单 ✅

当你完成以下任何一项机械改装后，请按此清单逐一检查：

#### 📋 机械结构改动

- [ ] **更换/移动了轮子**
  - [ ] 测量新的 `WHEEL_BASE` 和 `TRACK_WIDTH` 并更新
  - [ ] 确认 `RADIUS_WHEEL` 是否变化
  - [ ] 测试全向移动轨迹是否正确（直线是否走直线）
  - [ ] 检查四个轮子转向是否正确

- [ ] **调整了云台安装**
  - [ ] 重新校准 `YAW_CHASSIS_ALIGN_ECD`（Yaw零点）
  - [ ] 重新校准 `PITCH_HORIZON_ECD`（Pitch零点）
  - [ ] 更新 `YAW_ECD_GREATER_THAN_4096` 标志
  - [ ] 测试云台归中是否准确
  - [ ] 检查 `PITCH_MAX_ANGLE` / `PITCH_MIN_ANGLE` 限位

- [ ] **更换了IMU传感器**
  - [ ] 确认驱动程序兼容性（BMI088 vs 其他型号）
  - [ ] 检查 `GYRO2GIMBAL_DIR_*` 方向配置
  - [ ] 验证姿态解算输出是否正确（静态时Roll/Pitch应接近0）
  - [ ] 如使用DM-IMU，确认CAN通信正常

- [ ] **更换了电机**
  - [ ] 更新 `.motor_type` 型号定义
  - [ ] 确认 `REDUCTION_RATIO_WHEEL` 减速比
  - [ ] 检查 `tx_id` CAN ID分配
  - [ ] 测试每个电机的转向（`motor_reverse_flag`）
  - [ ] 重新调节PID参数（不同电机惯量不同）

- [ ] **修改了发射机构**
  - [ ] 更新 `ONE_BULLET_DELTA_ANGLE` 单发角度
  - [ ] 更新 `REDUCTION_RATIO_LOADER` 减速比
  - [ ] 更新 `NUM_PER_CIRCLE` 载弹量
  - [ ] 测试单发/连发功能是否正常

#### 📋 电子系统改动

- [ ] **更换了主控板**
  - [ ] 重新配置STM32CubeMX工程（引脚、时钟树）
  - [ ] 重新生成HAL层代码
  - [ ] 检查所有外设初始化顺序
  - [ ] 确认FreeRTOS任务栈大小足够

- [ ] **更改了CAN总线分配**
  - [ ] 更新 `can_init_config.can_handle` (&hcan1 / &hcan2)
  - [ ] 重新分配各个电机的 `tx_id`
  - [ ] 检查DM-IMU使用的CAN总线和滤波器ID

- [ ] **更改了串口分配**
  - [ ] 更新遥控器串口：`RemoteControlInit(&huartX)`
  - [ ] 更新裁判系统串口：`Referee_Interactive_init(&huartX)`
  - [ ] 更新视觉通信串口：`VisionInit(&huartX)` 或改为VCP

- [ ] **添加了新模块**
  - [ ] 在 `robot_init()` 中添加初始化调用
  - [ ] 在 `freertos.c` 中创建新任务（如需要）
  - [ ] 通过消息中心注册发布者/订阅者
  - [ ] 在守护进程(daemon)中注册监控

#### 📋 性能优化相关

- [ ] **控制效果不佳**
  - [ ] 重新整定底盘速度环/电流环PID
  - [ ] 重新整定云台角度环/速度环PID
  - [ ] 调整前馈增益（feedforward）
  - [ ] 检查低通滤波器参数（`Output_LPF_RC`）
  - [ ] 检查死区设置（`DeadBand`）

- [ ] **功率控制问题**
  - [ ] 检查功率限制算法参数（`power_control.c`）
  - [ ] 确认裁判系统功率数据接收正常
  - [ ] 调整缓冲能量利用策略

- [ ] **通信延迟/丢包**
  - [ ] 检查CAN总线波特率和终端电阻
  - [ ] 检查UART波特率配置
  - [ ] 优化任务优先级和调度频率
  - [ ] 检查消息中心发布-订阅机制是否阻塞

---

### 六、常见问题排查

#### ❓ 云台上电后乱转或抖动

**可能原因及解决**：
1. **零点参数错误** → 重新校准 `YAW_CHASSIS_ALIGN_ECD` 和 `PITCH_HORIZON_ECD`
2. **PID参数过大** → 减小Kp，特别是角度环Kp
3. **IMU方向配置错误** → 检查 `GYRO2GIMBAL_DIR_*` 
4. **反馈源配置错误** → 确认使用的是IMU还是编码器（`OTHER_FEED` vs `MOTOR_FEED`）
5. **电机转向配置错误** → 检查 `motor_reverse_flag`

#### ❓ 底盘移动方向错误

**可能原因及解决**：
1. **轮子转向错误** → 修改对应电机的 `motor_reverse_flag`
2. **运动学公式选错** → 确认使用的是 `ROBOT_OMNIDIRECTIONAL` 还是 `ROBOT_MECANUM`
3. **摇杆方向相反** → 在 `RemoteControlSet()` 中修改vx/vy的正负号

#### ❓ 遥控器无反应

**可能原因及解决**：
1. **串口接线错误** → 确认遥控器接在USART3（或你配置的串口）
2. **未初始化** → 检查 `RobotCMDInit()` 是否被调用
3. **Daemon检测离线** → 检查守护进程是否误判
4. **开关在中间位置** → 左侧开关在中间是急停状态！

#### ❓ 摩擦轮无法开启或拨弹卡住

**可能原因及解决**：
1. **拨轮阈值未达到** → 需要将拨轮拨过500的位置
2. **摩擦轮转速不足** → 检查摩擦轮电机是否正常，需要>30000才能拨弹
3. **机械卡死** → 检查拨盘是否有异物堵塞
4. **电流限制过低** → 检查摩擦轮电机的电流环MaxOut设置

#### ❓ 视觉数据不通

**可能原因及解决**：
1. **VCP未初始化** → 确保 `MX_USB_DEVICE_Init()` 在 `defaultTask` 中被调用
2. **USB线问题** → 使用数据线而非充电线
3. **上位机未打开对应端口** → 检查COM口号
4. **协议格式错误** → 用示波器/逻辑分析仪抓包检查数据格式

---

### 七、调试工具推荐

#### 🔧 软件调试工具

| 工具 | 用途 | 使用位置 |
|------|------|---------|
| **VOFA+** | 实时波形显示 | 代码中已有 `vofa.h/c` 接口 |
| **Serial Port Tool** | 串口数据监视 | 查看调试打印信息 |
| **STM32CubeMonitor** | 变量实时监控 | 通过ST-Link SWD接口 |
| **Logic Analyzer** | 信号时序分析 | I2C/SPI/UART/CAN协议分析 |

#### 📊 关键调试变量

代码中已经定义了大量debug数组，可以通过VOFA+实时观察：

```c
// chassis.c
extern float debug[10];           // 底盘调试数据
float debug_speed;                // 底盘速度
float Chassis_follow_lapse;       // 跟随延迟

// gimbal.c
int16_t debug_yaw1[8];            // Yaw调试数据
float debug_gyro[10];             // 陀螺仪数据
float debug_pitch_data[30];       // Pitch历史数据
float debug_yaw_data[30];         // Yaw历史数据

// robot_cmd.c
float debug_yaw[3];               // 命令调试数据
```

**使用方法**：
1. 在代码中将需要观察的变量赋值给debug数组
2. 连接VOFA+软件（通过串口或VCP）
3. 实时绘制波形图，分析控制效果

---

## 🚀 快速开始

### 环境要求

- **IDE**: Keil MDK-ARM v5.x 或 VSCode + EIDE插件
- **编译器**: ARM Compiler v5/v6
- **工具链**: STM32CubeMX v6.x（用于配置外设）
- **下载器**: ST-Link / J-Link / DAP-Link

### 编译步骤

1. **克隆项目**
   ```bash
   git clone <repository-url>
   ```

2. **打开工程**
   - Keil用户：打开 `MDK-ARM/` 目录下的 `.uvprojx` 文件
   - VSCode用户：确保安装了EIDE插件，直接打开项目文件夹

3. **配置参数**
   - 编辑 `other/robot_def.h` 选择开发板类型和机器人型号
   - 根据实际机械结构调整关键参数

4. **编译项目**
   - Keil: 点击 Build 按钮 (F7)
   - VSCode EIDE: 使用快捷键 `F7` 或点击Build

5. **下载程序**
   - 连接ST-Link/J-Link下载器
   - 点击 Download 按钮 (F8)

### 注意事项

⚠️ **重要提醒**：
- 烧录前务必确认 `robot_def.h` 中的开发板类型定义正确
- 云台和底盘的机械零点参数需要根据实际装配调整
- 首次使用建议先在无负载情况下测试各模块功能
- 修改宏定义后需要重新编译整个项目

## 📊 数据流图

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   遥控器      │     │   上位机      │     │  裁判系统     │
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │                    │                    │
       ▼                    ▼                    ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ remote_      │     │ seasky_      │     │ referee_     │
│ control      │     │ protocol     │     │ communication│
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │                    │                    │
       └────────┬───────────┘─────────────────────┘
                │
                ▼
       ┌────────────────┐
       │ master_process │ ◄── 命令解析与状态机
       └───────┬────────┘
               │
       ┌───────▼────────┐
       │ message_center │ ◄── 发布-订阅消息中心
       └───┬───────┬────┘
           │       │
     ┌─────▼──┐ ┌──▼─────┐ ┌──────────┐
     │Chassis │ │ Gimbal │ │  Shoot   │
     └────┬───┘ └──┬─────┘ └────┬─────┘
          │        │           │
     ┌────▼────────▼───────────▼────┐
     │        motor_task           │ ◄── 电机统一调度
     └──────────────┬───────────────┘
                    │
           ┌────────▼────────┐
           │   dji_motor     │ ◄── PID控制 + 电流环
           └────────┬────────┘
                    │
           ┌────────▼────────┐
           │    bsp_can      │ ◄── CAN总线发送
           └─────────────────┘

┌──────────────┐
│   BMI088     │
│   IMU传感器  │
└──────┬───────┘
       │
       ▼
┌──────────────┐     ┌──────────────┐
│ Quaternion   │────▶│   gimbal     │
│ EKF滤波器    │ 姿态│  (反馈数据)   │
└──────────────┘     └──────┬───────┘
                            │
                   ┌────────▼────────┐
                   │     dm_imu      │────▶ 上位机（视觉）
                   └─────────────────┘
```

## 🔌 关键模块说明

### 1. 消息中心 (message_center)

采用发布-订阅（Publish-Subscribe）模式，实现模块间解耦通信：
- **CMD模块**: 发布遥控器和上位机的控制指令
- **Chassis/Gimbal/Shoot**: 订阅各自的控制指令
- **Referee**: 订阅裁判系统数据并发布UI更新请求

### 2. 电机控制 (dji_motor)

统一的DJI电机驱动框架，支持：
- M2006/M3508等常用型号
- 位置环/速度环/电流环多级PID控制
- 低通滤波器平滑控制量
- 自动识别电机类型并应用对应参数

### 3. 姿态解算 (ins_task)

基于BMI088的高精度姿态解算系统：
- **原始数据采集**: SPI/I2C读取加速度计和陀螺仪数据
- **四元数EKF**: 扩展卡尔曼滤波融合多传感器数据
- **温度补偿**: 减少温漂影响
- **输出**: Roll/Pitch/Yaw欧拉角 + 角速度 + 加速度

### 4. 功率控制 (power_control)

智能功率管理系统：
- 实时监测底盘总功耗
- 根据裁判系统限功率动态调整输出
- 枪口超热量自动降功率保护
- 平滑过渡避免突变

### 5. 裁判系统 (referee_task)

完整的裁判系统交互方案：
- 协议解析：接收比赛状态、血量、热量等信息
- UI绘制：自定义操作界面图形显示
- 数据上报：发送机器人状态给裁判系统
- 错误处理：CRC校验和数据包完整性检查

## 🛠️ 技术栈

- **语言**: C (C99标准)
- **架构**: 基于HNU YueLu RM基础框架
- **RTOS**: FreeRTOS v10.x (CMSIS-OS2封装)
- **控制算法**: PID控制器 + 卡尔曼滤波
- **数学库**: ARM CMSIS-DSP (Cortex-M4F优化)
- **通信协议**: CAN 2.0B / UART / USB CDC
- **版本控制**: Git

## 📝 开发日志

### 当前版本特性 (v0.1)

✅ 完成基础框架搭建
✅ 底盘全向运动控制（跟随/小陀螺/自由模式）
✅ 云台双轴控制（电机反馈/IMU反馈）
✅ 发射机构完整控制流程
✅ BMI088姿态解算（四元数EKF）
✅ DM-IMU视觉数据转发
✅ 裁判系统UI与数据交互
✅ 底盘功率限制与热量保护
✅ 系统守护进程

## 🤝 致谢

- **[湖南大学跃鹿战队](https://github.com/HNUYueLuRM/basic_framework)**: 提供优秀的代码架构基础
- **哈尔滨工程大学**: IMU数据处理相关算法参考
- **RoboMasters组委会**: 提供比赛平台和技术规范
- **DJI大疆创新**: 提供高性能机器人套件

## 📄 许可证

本项目仅供学习和RoboMasters比赛使用。请遵守相关法律法规和比赛规则。

---

**最后更新**: 2026-05-14  
**适用比赛**: RoboMasters 2026 赛季  
**维护团队**: [您的战队名称]
