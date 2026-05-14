# RoboMaster 2026 全向轮底盘机器人 —— 完整使用教程（面向初学者）

> **适用对象**：刚接触 RoboMaster 电控的初学者
> **项目版本**：Release Version 7.0
> **硬件平台**：STM32F407IG + DJI 电机 + BMI088 IMU

---

## 目录

1. [项目简介](#1-项目简介)
2. [你需要准备什么](#2-你需要准备什么)
3. [工程结构总览](#3-工程结构总览)
4. [核心概念通俗讲解](#4-核心概念通俗讲解)
5. [如何编译和烧录](#5-如何编译和烧录)
6. [各模块详解](#6-各模块详解)
7. [如何调试和调参](#7-如何调试和调参)
8. [常见问题 FAQ](#8-常见问题-faq)

---

## 1. 项目简介

### 这是什么？

这是一个**RoboMaster 比赛用的步兵机器人底盘控制程序**。简单来说，它能让你的机器人：

- 🚗 **全向移动**：像螃蟹一样横着走、斜着走、原地转圈（小陀螺）
- 🎯 **云台瞄准**：自动跟随目标，用鼠标/遥控器控制瞄准方向
- 🔫 **发射弹丸**：摩擦轮加速 + 拨盘供弹，实现射击功能
- 📡 **接收指令**：从遥控器或电脑键盘鼠标获取操作指令
- ⚡ **功率管理**：防止机器人因为用电过猛被裁判系统惩罚

### 技术栈一览

| 技术 | 说明 |
|------|------|
| 主控芯片 | STM32F407IGT6（ARM Cortex-M4） |
| 实时系统 | FreeRTOS（多任务调度） |
| 通信协议 | CAN 总线（控制电机）、串口（遥控器/视觉） |
| 姿态解算 | 四元数扩展卡尔曼滤波（EKF） |
| 控制算法 | 三环 PID（位置环 → 速度环 → 电流环） |
| 开发工具 | Keil MDK-ARM + STM32CubeMX |

---

## 2. 你需要准备什么

### 硬件清单

```
🖥️ 主控板：STM32F407IG 核心板
⚙️ 底盘：4个 M3508 电机 + 全向轮
🎯 云台：2个 GM6020 电机（Yaw轴 + Pitch轴）
🔫 发射：2个 M3508（摩擦轮）+ 1个 M2006（拨盘）
🧭 传感器：BMI088 六轴 IMU（陀螺仪+加速度计）
🎮 遥控器：DJI DBUS 遥控器
🔌 通信：CAN 总线、SPI、USART 等
⚡ 电源：电池 + 电源管理模块
💻 调试：ST-Link / J-Link 调试器
```

### 软件环境

- **Keil uVision5**（MDK-ARM 版本）：写代码、编译、下载
- **STM32CubeMX**：配置芯片引脚和外设（可选，已有 `.ioc` 文件）
- **串口助手**（如 SSCOM、VOFA+）：查看调试数据
- **RoboMaster 上位机**：查看机器人状态（可选）

---

## 3. 工程结构总览

打开工程文件夹后，你会看到这样的目录结构：

```
01_Release_Version_7.0/
├── Core/                    # STM32 HAL 库生成的核心代码
│   ├── Src/                 #   main.c（主函数）、freertos.c（任务创建）等 .c 文件
│   └── Inc/                 #   对应的头文件
├── Drivers/                 # STM32 官方驱动库（不用改）
├── Middlewares/             # 中间件（FreeRTOS、USB等）（不用改）
├── USB_DEVICE/              # USB 虚拟串口驱动（不用改）
├── other/                   # ★★★ 这是核心业务代码 ★★★
│   ├── robot_def.h          #   📋 机器人参数配置（最重要！）
│   ├── robot_cmd.c          #   🧠 指挥中心（处理遥控器/键鼠输入）
│   ├── chassis.c            #   🚗 底盘控制（运动解算、功率限制）
│   ├── gimbal.c             #   🎯 云台控制（PID、角度跟踪）
│   ├── shoot.c              #   🔫 发射控制（摩擦轮、拨弹）
│   ├── ins_task.c           #   🧭 姿态解算（IMU数据处理）
│   ├── dji_motor.c          #   ⚙️ 电机驱动（PID计算、CAN发送）
│   ├── power_control.c      #   ⚡ 功率控制（能量缓冲管理）
│   ├── controller.c         #   🎛️ PID算法库
│   ├── message_center.c     #   📮 消息中心（模块间通信）
│   ├── bsp_can.c            #   CAN总线底层驱动
│   ├── bsp_usart.c          #   串口底层驱动
│   ├── remote_control.c     #   遥控器数据解析
│   ├── referee_*.c          #   裁判系统通信
│   └── ...                  #   其他辅助文件
├── MDK-ARM/                 # Keil 工程文件（在这里打开 .uvprojx）
├── 06_my_yuntai_test_02.ioc # CubeMX 配置文件
└── CLAUDE.md                # 项目说明文档
```

> 💡 **新手提示**：你 90% 的时间都会在 `other/` 目录下工作，其他目录基本不需要修改。

---

## 4. 核心概念通俗讲解

### 4.1 什么是"任务"？（FreeRTOS）

想象一下你在做饭：
- 👨‍🍳 你（主厨）在炒菜 → **这是主任务**
- 🥬 帮手在切菜 → **另一个任务**
- 🔥 另一个帮手在煮汤 → **第三个任务**

在嵌入式系统中，**FreeRTOS 就是这个"厨房管理者"**，它让多个"任务"同时运行。

本工程的 6 个主要任务：

| 任务名 | 运行频率 | 干什么的？ | 生活类比 |
|--------|---------|-----------|---------|
| `INS_TASK` | 1000Hz (1ms) | 读取 IMU 数据，计算机器人姿态 | 🧭 导航员：每秒看1000次指南针 |
| `MOTORTASK` | 1000Hz (1ms) | 计算所有电机的 PID 输出，发送 CAN 指令 | ⚙️ 机械师：每秒调整1000次电机 |
| `ROBOTTASK` | 200Hz (5ms) | 处理遥控器输入，协调底盘/云台/发射 | 🧠 大脑：做决策 |
| `DAEMONTASK` | 100Hz (10ms) | 监测各模块是否在线（看门狗） | 👀 安保：检查大家是否正常 |
| `StartUI_TASK` | - | 更新裁判系统 UI 显示 | 📺 屏幕：显示状态 |
| `defaultTask` | - | 空闲任务（系统自带） | 😴 休息的人 |

### 4.2 什么是 PID 控制？

**PID = 比例(Proportional) + 积分(Integral) + 微分(Derivative)**

举个开空调的例子：
- 🎯 **目标**：房间温度 25°C
- 📏 **当前**：现在是 30°C（误差 = 5°C）

**P（比例）**："误差越大，用力越大"
→ 温差 5°C，就把空调开到 5 档

**I（积分）**："如果一直有误差，就加大力度"
→ 如果开了很久还是没降到25°C，就再加大一点

**D（微分）**："如果变化太快，就减缓一点"
→ 如果温度降得太猛了，就减小空调力度避免 overshoot

在本项目中，电机使用**三环 PID**：
```
目标角度 → [角度环PID] → 目标速度 → [速度环PID] → 目标电流 → [电流环PID] → 最终输出
```

### 4.3 什么是消息中心（Pub/Sub）？

想象一下公司里的邮件系统：
- 📤 **发布者(Publisher)**：发邮件的人
- 📥 **订阅者(Subscriber)**：收邮件的人
- 📧 **主题(Topic)**：邮件的标题

例如：
- `gimbal` 模块发布了一封邮件，主题叫 `"gimbal_feed"`，内容是"当前云台角度是 180°"
- `robot_cmd` 模块订阅了这个主题，收到邮件后就知道云台的角度了

这种设计让各个模块**互不干扰**，只需要关心自己订阅的数据。

### 4.4 运动学解算是什么？

**问题**：你想让机器人向"右前方 45°"移动，四个轮子该怎么转？

**答案**：运动学解算就是做这道数学题！

对于**全向轮（Omni Wheel）**：
```
        LF（左前）─── RF（右前）
         ╱              ╲
        ╱    机器人       ╲
       ╱                  ╲
      LB（左后）─── RB（右后）

输入：vx（前后速度）、vy（左右速度）、wz（旋转速度）
输出：四个轮子各自的目标转速
```

代码中的 `Kinematics_calculation()` 函数就是干这件事的！

---

## 5. 如何编译和烧录

### 第一步：打开 Keil 工程

1. 打开 `MDK-ARM/` 文件夹
2. 双击 `06_my_yuntai_test_02.uvprojx`（Keil 会自动打开）

### 第二步：编译项目

- 按 **F7** 或点击工具栏的 **Build** 按钮（锤子图标）
- 等待底部 Build Output 窗口显示：
  ```
  "06_my_yuntai_test_02" - 0 Error(s), 1 Warning(s).
  ```
- ✅ 看到 `0 Error(s)` 就说明编译成功！

### 第三步：连接下载器

1. 用 ST-Link/J-Link 连接板子的 SWD 接口
2. Keil 中点击 **Options for Target**（魔法棒图标）
3. 在 **Debug** 选项卡中选择你的下载器（ST-Link Debugger / J-Link）
4. 点击 **Settings** → 确认能检测到芯片（SW Device 显示芯片型号）

### 第四步：下载程序

- 按 **F8** 或点击 **Download** 按钮
- 等待显示 "Application running ..."
- 🎉 程序已经跑起来了！

### 第五步：连接外设并测试

1. 插上遥控器接收机（接到 USART3）
2. 接上 CAN 总线（连好所有电机）
3. 接上 IMU（SPI1 接口）
4. 给主板通电
5. 观察电机是否有反应

---

## 6. 各模块详解

### 6.1 参数配置文件 —— robot_def.h（必读！）

这是**最重要的配置文件**，所有的机器人都在这里定义参数：

```c
// ========== 选择你的机器人类型（只能选一个！）==========
#define ROBOT_OMNIDIRECTIONAL 3    // 全向轮步兵
// #define ROBOT_MECANUM 3         // 麦克纳姆轮步兵

// ========== 选择开发板类型（只能选一个！）==========
#define ONE_BOARD                   // 单板控制整车
// #define CHASSIS_BOARD           // 仅底盘板
// #define GIMBAL_BOARD            // 仅云台板

// ========== 机械参数（根据你的车修改！）==========
#define WHEEL_BASE 370              // 纵向轴距（毫米）
#define TRACK_WIDTH 420             // 横向轮距（毫米）
#define RADIUS_WHEEL 60             // 轮子半径（毫米）
#define YAW_CHASSIS_ALIGN_ECD 3700  // 云台中值编码器值（关键！）
```

> ⚠️ **注意**：YAW_CHASSIS_ALIGN_ECD 必须根据你的实际机械结构调整！这个值决定了云台"正前方"对应编码器的哪个位置。

### 6.2 指挥中心 —— robot_cmd.c

**作用**：处理操作者的输入，翻译成各模块能懂的命令

#### 工作流程：

```
遥控器/键鼠输入
       ↓
  RemoteControlSet()  或  MouseKeySet()
       ↓
  EmergencyHandler()  （紧急停止检测）
       ↓
  发布消息给 chassis / gimbal / shoot
```

#### 遥控器按键说明：

| 操作 | 遥控器动作 |
|------|-----------|
| 切换底盘模式 | 右侧拨杆：下=小陀螺 / 中=跟随 / 上=自由 |
| 切换控制方式 | 左侧拨杆：下=遥控器模式 / 上=键鼠模式 |
| 急停 | 左侧拨杆中位，或 Ctrl+Z |
| 摩擦轮开关 | 遥控器旋钮向上拨（遥控器模式）/ B键（键鼠模式） |
| 小陀螺模式 | Q键（键鼠模式） |
| 反向小陀螺 | E键（键鼠模式） |
| 系统复位 | Ctrl+X |

### 6.3 底盘控制 —— chassis.c

**作用**：让底盘按照指令运动

#### 三种运动模式：

| 模式 | 效果 | 适用场景 |
|------|------|---------|
| CHASSIS_FOLLOW_GIMBAL_YAW | 底盘始终朝向云台指向的方向 | 正常比赛、瞄准射击 |
| CHASSIS_ROTATE | 底盘原地旋转（小陀螺） | 快速转向、躲避攻击 |
| CHASSIS_NO_FOLLOW | 底盘自由移动，不跟随云台 | 手动调整位置 |
| CHASSIS_ROTATE_REVERSE | 反向小陀螺 | 特殊战术动作 |

#### 关键函数说明：

```c
ChassisInit()           // 初始化：注册4个底盘电机
ChassisTask()          // 主循环（200Hz执行）
Kinematics_calculation() // 运动学解算：vx/vy/wz → 4个轮速
LimitPowerCalculate()   // 功率限制：防止超功率被裁判惩罚
update_chassis_follow() // 跟随角度平滑过渡
```

### 6.4 云台控制 —— gimbal.c

**作用**：控制云台的 Yaw（水平）和 Pitch（俯仰）轴

#### PID 参数（可实时调整的全局变量）：

```c
// Yaw 轴（水平转动）
float Yaw_Angle_Kp = 0.9;    // 角度环比例
float Yaw_Angle_Ki = 0.1;    // 角度环积分
float Yaw_Angle_Kd = 0.08;   // 角度环微分
float Yaw_speed_Kp = 1300;   // 速度环比例
float Yaw_Speed_Ki = 300;    // 速度环积分

// Pitch 轴（上下俯仰）
float Pitch_Angle_Kp = 0.3;
float Pitch_speed_Kp = 4200;
```

> 💡 这些变量可以在调试时通过 Keil Watch 窗口实时修改，方便调参！

#### 软限位：

```c
if (pitch <= 140) pitch = 140;   // 最高仰角
if (pitch >= 205) pitch = 205;   // 最低俯角
```

### 6.5 发射控制 —— shoot.c

**作用**：控制摩擦轮转速和拨弹

#### 工作逻辑：

```
收到"开启摩擦轮"命令
       ↓
  缓启动：逐渐加速到目标转速（防电流冲击）
       ↓
  收到"拨弹"命令
       ↓
  检查摩擦轮是否达到安全转速（>20000）
       ↓
  是 → 启动拨弹电机发射
  否 → 不拨弹（防止卡弹）
```

#### 安全机制：

- ✅ 摩擦轮未达转速不拨弹（防卡弹）
- ✅ 缓启缓停（保护电机和电源）
- ✅ 过热自动降速

### 6.6 姿态解算 —— ins_task.c

**作用**：读取 IMU 数据，计算机器人的 Roll/Pitch/Yaw 角度

#### 工作原理（简化版）：

```
BMI088 传感器
    ↓ （读取原始数据：加速度 + 角速度）
四元数 EKF 滤波
    ↓ （融合数据，去除噪声）
输出：Roll / Pitch / Yaw / YawTotalAngle
    ↓
提供给云台使用（作为反馈）
```

#### 关键点：

- 运行频率：**1000Hz**（每毫秒更新一次）
- 使用 **四元数**表示姿态（避免万向锁）
- **Pitch 角度做了特殊处理**：负值加 360°（补偿安装反向）

### 6.7 电机驱动 —— dji_motor.c & power_control.c

#### 支持的电机类型：

| 型号 | 用途 | 编码器分辨率 |
|------|------|------------|
| M3508 | 底盘轮子、摩擦轮 | 8192（13位） |
| M2006 | 拨盘 | 8192（13位） |
| GM6020 | 云台电机 | 8192（13位） |

#### CAN 通信分组：

```
CAN1 总线：
  - 0x1FF: GM6020 电机组1（ID 1-4，电压控制）
  - 0x200: M3508/M2006 电机组1（ID 1-4）
  - 0x1FE: GM6020 电机组2（ID 5-7）
  - 0x2FE: GM6020 电压控制组2（ID 5-7）

CAN2 总线：
  - 同上结构，用于另一条总线
```

#### 功率控制原理：

当总功率超过裁判系统限制时：
1. 计算每个电机的实际消耗功率
2. 按比例缩减每个电机的电流输出
3. 保证总功率不超限

---

## 7. 如何调试和调参

### 7.1 使用 Debug 数组

工程中大量使用了 debug[] 数组来输出调试数据：

```c
// chassis.c 中
float debug[10];           // 可在 Keil 中观察
debug[0] = chassis_vx;     // 填入你想看的变量
debug[1] = chassis_vy;

// gimbal.c 中
float debug_pitch_data[30]; // Pitch 相关调试数据
float debug_yaw_data[30];   // Yaw 相关调试数据
```

**如何使用**：
1. 进入 Debug 模式（Ctrl+F5）
2. 打开 **Watch** 窗口（View → Watch Windows → Watch 1）
3. 输入变量名即可实时观察数值

### 7.2 使用串口打印（高级）

如果想把数据发到电脑上看波形：

1. 使用 VOFA+ 或上位机软件
2. 在代码中添加 printf() 或自定义协议
3. 通过 USB 虚拟串口发送数据

### 7.3 PID 调参步骤（以云台为例）

**第一步：先调速度环**
1. 把角度环 Kp 设为 0（暂时关闭角度环）
2. 给速度环一个阶跃信号（突然设定一个目标速度）
3. 观察响应曲线，调整 speed_Kp / Ki / Kd
4. 目标：快速稳定，无超调或小幅超调

**第二步：再调角度环**
1. 恢复角度环，从小 Kp 开始
2. 给一个角度阶跃（比如从 170° 突变到 190°）
3. 调整 Angle_Kp / Ki / Kd
4. 目标：快速到达目标，无振荡

**经验公式**（仅供参考）：
```
先 P：增大直到出现振荡，然后取一半
再加 I：消除稳态误差，不要太大（会超调）
最后 D：抑制超调，噪声大时要加滤波
```

### 7.4 常用调试快捷键

| 快捷键 | 功能 |
|--------|------|
| F5 | 运行/继续 |
| F10 | 单步跳过（不进入函数） |
| F11 | 单步进入（进入函数内部） |
| Ctrl+F5 | 启动调试会话 |
| Shift+F5 | 停止调试 |
| F9 | 设置/取消断点 |
| Ctrl+S | 保存文件 |

---

## 8. 常见问题 FAQ

### Q1: 编译报错怎么办？

**常见原因及解决方法**：
- ❌ #define 冲突：检查 robot_def.h 是否同时定义了多个机器人类型
- ❌ 头文件缺失：确认 #include 路径正确
- ❌ 内存不足：减少数组大小或优化堆栈设置
- ✅ 清理重新编译：Project → Clean Targets，然后重新 Build

### Q2: 电机不转怎么办？

**排查步骤**：
1. 检查 CAN 总线是否接好（H/L 不能接反）
2. 检查电机 ID 配置是否正确（tx_id 对应物理 ID）
3. 检查是否触发了急停（左侧拨杆中位）
4. 用示波器/逻辑分析仪检查 CAN 波形
5. 在 DJIMotorControl() 函数处打断点，检查是否执行到了

### Q3: 云台抖动/震荡怎么办？

**可能的原因**：
- PID 参数过大（尤其是 Kp）
- 反馈数据噪声大（需要滤波）
- 机械松动或间隙
- 采样频率不稳定

**尝试的解决方案**：
1. 降低 PID 的 Kp 值
2. 增加低通滤波（Output_LPF_RC）
3. 检查机械安装
4. 确保 INS_Task 以稳定的 1kHz 运行

### Q4: 底盘走直线偏移怎么办？

**检查项**：
1. 四个轮子的 MOTOR_DIRECTION_REVERSE 方向是否正确
2. 运动学解算公式符号是否正确
3. sin_theta 和 cos_theta 的补偿系数是否一致（当前版本不一致是个已知 Bug）
4. 机械上轮子是否装正

### Q5: 如何切换麦克纳姆轮和全向轮？

编辑 robot_def.h：
```c
// 取消注释你要的类型
#define ROBOT_OMNIDIRECTIONAL 3    // 全向轮
// #define ROBOT_MECANUM 3         // 麦克纳姆轮
```

然后**重新编译整个工程**（不是增量编译）。

### Q6: 功率限制怎么调？

在 chassis.c 的 LimitPowerCalculate() 函数中：
- power_buff >= 60：缓冲能量充足，可以放开限制
- power_buff <= 40：能量不足，需要降低功率
- limit_error：动态误差调整量

可以根据实际比赛情况微调这些阈值。

### Q7: 如何添加新的调试变量？

```c
// 1. 在对应的 .c 文件顶部声明
float my_debug_var;

// 2. 在函数中赋值
my_debug_var = 你想观测的值;

// 3. 在 Keil Debug 模式的 Watch 窗口中添加 my_debug_var
```

### Q8: FreeRTOS 任务优先级怎么改？

在 freertos.c 的 osThreadDef() 中修改：
```c
// 第三个参数是优先级，数字越大优先级越高
osThreadDef(INS_TASK, ins_task, osPriorityRealtime, ...);  // 最高优先级
osThreadDef(MOTORTASK, StartMOTORTASK, osPriorityHigh, ...); // 高优先级
osThreadDef(ROBOTTASK, StartROBOTTASK, osPriorityNormal, ...); // 普通优先级
```

> ⚠️ 注意：INS 和 MOTORTASK 应该保持最高优先级，否则会影响控制性能！

---

## 附录 A：文件依赖关系图

```
main.c
  ├─→ ChassisInit()     ← chassis.h
  ├─→ GimbalInit()      ← gimbal.h
  ├─→ ShootInit()       ← shoot.h
  ├─→ RobotCMDInit()    ← robot_cmd.h
  └─→ MX_FREERTOS_Init() → 创建任务
       ├─→ INS_TASK     ← ins_task.c（IMU姿态）
       ├─→ MOTORTASK    ← dji_motor.c / power_control.c（电机控制）
       ├─→ ROBOTTASK    ← robot_cmd.c（指挥中心）
       │    ├─→ ChassisTask()   ← chassis.c
       │    ├─→ GimbalTask()    ← gimbal.c
       │    └─→ ShootTask()     ← shoot.c
       ├─→ DAEMONTASK   ← daemon.c（看门狗）
       └─→ StartUI_TASK  ← referee_UI.c（UI显示）
```

## 附录 B：推荐学习资源

- **RoboMaster 官方文档**：https://www.robomaster.com
- **FreeRTOS 中文教程**：网上搜索"FreeRTOS入门"
- **PID 控制原理**：B站搜索"PID控制 直观理解"
- **CAN 协议**：了解 CAN2.0B 标准
- **四元数与欧拉角**：了解 3D 旋转数学基础

## 附录 C：术语对照表

| 英文 | 中文 | 解释 |
|------|------|------|
| Chassis | 底盘 | 机器人的移动部分 |
| Gimbal | 云台 | 机器人的瞄准装置 |
| Shoot | 发射 | 摩擦轮+拨弹机构 |
| Omni Wheel | 全向轮 | 可以多方向滚动的轮子 |
| Mecanum Wheel | 麦克纳姆轮 | 斜辊轮，可实现全向移动 |
| Kinematics | 运动学 | 研究运动的几何关系 |
| PID | 比例-积分-微分 | 经典控制算法 |
| EKF | 扩展卡尔曼滤波 | 用于传感器数据融合 |
| Quaternion | 四元数 | 表示三维旋转的数学工具 |
| CAN | 控制器局域网 | 工业现场总线协议 |
| Daemon | 守护进程 | 监控程序运行状态的机制 |
| Pub/Sub | 发布-订阅 | 一种消息传递模式 |

---

> 📝 **最后更新**：2026年4月28日
> 🤖 **适用版本**：Release Version 7.0
> 📧 **如有问题**：欢迎在项目 Issue 中反馈

**祝你调试顺利，比赛加油！🎉**
