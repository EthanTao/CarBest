# STC32G小车项目 - AI助手指导

## 项目概述

这是一个**STC32G单片机自动循迹小车**项目，用于大学生电子设计竞赛。主要功能包括电机控制、循迹传感器、OLED显示和超声波测距。

**当前分支**: `PID+状态机` — PID比例控制 + 状态机兜底

## 架构和模块

### 核心功能模块

| 模块 | 文件 | 功能 |
|------|------|------|
| **循迹控制** | `main.c` | PID循迹算法 + 状态机兜底，电机控制 |
| **PWM驱动** | `STC32G_PWM.*` | 双电机速度控制（PWM5/6） |
| **定时器** | `STC32G_Timer.*` | 1ms周期中断，控制循环 |
| **GPIO** | `STC32G_GPIO.*` | 引脚模式配置 |
| **中断管理** | `STC32G_NVIC.*` | 中断优先级配置 |
| **I2C通信** | `iic.*` + `STC32G_Soft_I2C.*` | 软件I2C实现 |
| **OLED显示** | `oled.*` + `font.*` | 128x64 OLED显示 |
| **超声波** | `main.c` (Timer1) | 测距功能 |
| **延时** | `STC32G_Delay.*` | 毫秒延时 |

### 硬件配置

**主频**: 24MHz
**编译器**: Keil uVision

#### 引脚映射

```
电机控制：
  AIN1=P4^5, AIN2=P2^7  (左电机方向)
  BIN1=P2^5, BIN2=P2^6  (右电机方向)
  PWM5(速度)=P2.0, PWM6(速度)=P2.1

循迹传感器(5路):
  左2=P0^3, 左1=P0^4, 中=P0^2, 右1=P0^1, 右2=P0^0

I2C(OLED):
  SCL=P3.3, SDA=P5.4

超声波:
  TRIG=P0^6, ECHO=P1^4

按键: KEY1-5 (P2^2-4, P1^3, P1^7)
LED:  P4^1, P4^2, P4^4
```

#### PWM配置

- **PWM模块**: PWMB
- **周期**: 2000 (100Hz@24MHz)
- **输出通道**: PWM5(右电机), PWM6(左电机)
- **占空比**: 0-100%表示，PWM_Period为100时转换为实际值

#### 定时器

| 定时器 | 用途 | 周期 | 中断 |
|--------|------|------|------|
| Timer0 | 1ms基础时钟 | 1ms | Priority_0 |
| Timer1 | 超声波测距计时 | 1us | Priority_0 |
| Timer2 | 控制循环分频 | 100us | Priority_0 |

## 编码规范

### 命名约定

- **函数**: `Module_FunctionName()` 或 `module_function_name()`
- **变量**: `camelCase` 或 `snake_case`
- **常量**: `UPPER_CASE`
- **SFR/寄存器**: `UPPERCASE` (来自stc32g.h)

### 类型定义

来自 `type_def.h`:
```c
u8, u16, u32    // unsigned
bit, sbit       // 位定义和位引脚（51内核）
```

## 关键参数 (main.c)

### 速度控制

```c
#define BASE_FAST      100  // 直行基础速度
#define BASE_NODE      100  // 十字路口直行速度（与直行同速）
#define MAX_PWM        100  // PWM最大值
#define MIN_RUN_PWM    70   // 非停车正转最低PWM
#define STALL_MARGIN   10   // 低压防停转余量（PID路径有效最低=MIN_RUN_PWM+STALL_MARGIN=80）
#define LOST_TURN_PWM  95   // 丢线转向速度
#define UTURN_PWM      95   // 掉头左右轮等速反向速度
```

> **防停转机制**: `MIN_RUN_PWM=70` 保证急转弯时不卡死；`STALL_MARGIN=10` 在 PID 路径额外抬高最低速，防止低压时电机停转。PWM 低于 `MIN_RUN_PWM+STALL_MARGIN(80)` 时自动抬到 80。
> 
> **提速策略**: 微修和急转弯内侧轮不降速，仅靠差速产生转向。所有状态 PWM 均在 90%+（除 PID 深度转弯时内侧轮可能降至 80%）。

### 机械偏置（补偿电机不对称）

```c
#define LEFT_TRIM     1    // 左轮平衡偏置，范围-10~10
#define RIGHT_TRIM    3    // 右轮平衡偏置，范围-10~10
```

> **调整原则**: 车往左偏 → 左轮慢/右轮快 → 加大LEFT_TRIM或减小RIGHT_TRIM
> 当前值来自实测：右轮偏快，经过多轮测试调整为 LEFT_TRIM=1, RIGHT_TRIM=3

### 微型修正参数

```c
#define MICRO_OUTER_PWM         98   // 微型修正外侧轮速度
#define MICRO_INNER_PWM         92   // 微型修正内侧轮速度
#define MICRO_TO_SHARP_TICKS    20   // 小修后允许大修抢占的时间
#define CENTER_SHARP_WINDOW_TICKS 20 // 中线和外侧传感器组合触发大弯的时间窗口
```

> 微型修正用于传感器内侧触发时的精细调整（ZUO1触发且ZUO2未触发 → 微修左；YOU1触发且YOU2未触发 → 微修右）。两轮均保持高速(92%/98%)，仅靠差速产生转向。窗口期内若外侧传感器也被触发则升级为急转弯。

### 急转弯参数

```c
#define SHARP_FORWARD_PWM      95   // 急转弯正转轮速度（前进侧）
#define SHARP_REVERSE_PWM      100  // 急转弯反转轮速度（倒退侧）
#define SHARP_MIN_PIVOT_TICKS  70   // 急转弯最短保持时间（约70ms）
```

> **急转弯逻辑**: SHARP_LEFT 时左轮反转(100) + 右轮正转(95)，SHARP_RIGHT 反之。满足 `SHARP_MIN_PIVOT_TICKS` 且目标侧传感器不再触发才退出。
> 进入急转弯时自动清零 `pid_integral`、`lost_ticks`，设置 `last_error=±4`。

### 丢线/死区处理

```c
#define LOST_STRAIGHT_TICKS   50   // 阶段1: 直行冲过死区（转弯/十字路口后常见）
#define LOST_CONFIRM_TICKS    150  // 阶段2: 按last_error偏转找线；阶段3: 原地掉头
```

**三段式死区处理** (`Track_HandleLostUturn`):

| 阶段 | 条件 | 行为 | 目的 |
|------|------|------|------|
| 阶段1 | `lost_ticks < 50` | `Motor_RunSafe(100,100)` 直行 | 冲过转弯后/十字路口后短死区 |
| 阶段2 | `50 ≤ lost_ticks < 150` | 按 `last_error` 方向 ±15 偏转 | 记忆方向微偏找线 |
| 阶段3 | `lost_ticks ≥ 150` | U-turn 掉头 | 完全丢线兜底 |

> **急转弯/掉头不截断**: `Track_HandleLostUturn` 检测到当前状态为 SHARP_LEFT/SHARP_RIGHT/UTURN_RIGHT/UTURN_STOP 时直接返回 0，让各自状态机继续控制电机。
> 
> **急转弯/CROSS_HOLD 出口清零**: 急转弯和十字路口退出时清零 `last_error`，确保后续死区处理阶段 2 直行而非朝旧方向偏转。CROSS_HOLD 入口/出口均清除 center-sharp 窗口，防止十字路口后被误判为急转弯。

### PID 循迹参数

```c
#define PID_KP              5    // 比例系数 - 基础修正力度
#define PID_KI              2    // 积分系数 - 消除稳态误差
#define PID_KD              15   // 微分系数 - 抑制震荡
#define PID_INTEGRAL_MAX    20   // 积分限幅，防止积分饱和
#define PID_OUTPUT_MAX      20   // PID单侧最大输出限幅
```

### PID 误差计算

5路传感器加权质心法：
```
传感器:    左2   左1   中   右1   右2
位置权重:   -4    -2    0    +2    +4

error = 加权总和 / 触发传感器数量   (范围 -4 ~ +4)
```

PID输出直接叠加到BASE_FAST上：
```
left_pwm  = BASE_FAST - pid_output + LEFT_TRIM
right_pwm = BASE_FAST + pid_output + RIGHT_TRIM
```

> **当前注意**: `BASE_FAST=100` 且 `MAX_PWM=100`，外侧轮叠加 PID 后会被限幅到 100。PID 实际通过**降低内侧轮速度**来产生差速转向。内侧轮最低至 80%（受 `MIN_RUN_PWM+STALL_MARGIN` 保护）。

### 循迹控制架构

```
Track_Control() [每1ms调用一次]
  |
  +-- 正中(00100) → 直走复位
  +-- 丢线(00000) → 三段式死区处理: 直行→偏转找线→掉头
  +-- 中心丢失窗口 + 最外侧 → 急转弯(兜底)
  +-- 十字路口(x111x) → 直行保持
  +-- 最外侧单边(1xx00/00xx1) → 急转弯(兜底)
  +-- 两侧同亮(10001) → 按上次方向
  +-- 内侧单触发(微修条件) → 微修差速
  +-- 微修中+外侧触发 → 升级急转弯
  +-- 其他所有情况 → PID正常循迹
```

**状态机状态**:
- `TRACK_FOLLOW` (0) — 正常循迹，PID控制
- `TRACK_CROSS_HOLD` (1) — 十字路口直行保持
- `TRACK_UTURN_STOP` (2) — 掉头前减速
- `TRACK_UTURN_RIGHT` (3) — 掉头旋转中
- `TRACK_SHARP_LEFT` (4) — 急转弯左(反转左轮+正转右轮)
- `TRACK_SHARP_RIGHT` (5) — 急转弯右(反转右轮+正转左轮)

## PID 调参指南

| 症状 | 调法 |
|------|------|
| 直道左右摇摆 | Kd↑(15→20~25) 或 Kp↓(5→3~4) |
| 小弯转不过来 | Kp↑(5→7~8) |
| 弯道切入不及时 | Kd↓(15→10~12) |
| 长期偏一侧不走正中 | 先确认Ki=2已启用，无效则检查积分限幅是否太小 |
| PID修正不够力但不想触发急转弯 | PID_OUTPUT_MAX↑(20→25~30) |
| 积分过度累积导致回正慢 | PID_INTEGRAL_MAX↓(20→10) 或 Ki↓(2→1) |

## 常见任务指导

### 1. 修改电机速度

编辑 `main.c` 中的参数:
```c
#define BASE_FAST      100  // 直行速度
#define BASE_NODE      100  // 十字路口速度
#define MAX_PWM        100  // 最大速度
#define MIN_RUN_PWM    70   // 最小正转速度
#define STALL_MARGIN   10   // PID路径防停转余量
#define UTURN_PWM      95   // 掉头速度
#define LOST_TURN_PWM  95   // 丢线转向速度
```

**注意**: 修改后需用Keil重新编译和下载到单片机。`STALL_MARGIN` 和 `MIN_RUN_PWM` 的关系见"防停转机制"。

### 2. 调整循迹平衡

修改 `LEFT_TRIM` 和 `RIGHT_TRIM` 值（范围-10~10），用于补偿电机速度差异：
- 车往**左**偏 → 右轮太快 → 减小RIGHT_TRIM 或 加大LEFT_TRIM
- 车往**右**偏 → 左轮太快 → 加大RIGHT_TRIM 或 减小LEFT_TRIM

当前值: `LEFT_TRIM=1, RIGHT_TRIM=3`

### 3. 调试PID参数

修改 `main.c` 中的PID宏定义：
```c
#define PID_KP   5     // 比例：增大=转弯更有力
#define PID_KI   2     // 积分：1~2=消除稳态偏置，0=禁用
#define PID_KD   15    // 微分：增大=抑制震荡
```

### 4. 更改OLED显示内容

编辑 `oled.*` 文件中的显示函数，通过I2C协议：
- `oled_ShowStr()` - 显示字符串
- `oled_ShowNum()` - 显示数字
- `oled_ShowChar()` - 显示单个字符

### 5. 添加传感器读取

在 `main.c` 的 `ReadTrackSensors()` 中查看5路循迹传感器读取逻辑。

### 6. 调试超声波测距

`Ultrasonic_GetDistance()` 函数在 `main.c` 中，返回距离(mm)。

### 7. 调整急转弯参数

编辑 `main.c` 中的急转弯宏定义：
```c
#define SHARP_FORWARD_PWM      95   // 正转轮速度（前进侧），越大转弯越猛
#define SHARP_REVERSE_PWM      100  // 反转轮速度（倒退侧），越大转弯越急
#define SHARP_MIN_PIVOT_TICKS  70   // 最短强转时间(ms)，越大转弯越久
```

- 直角转不过 → SHARP_FORWARD_PWM↑(95→100) 或增加 SHARP_MIN_PIVOT_TICKS
- 转弯过度/甩尾 → SHARP_FORWARD_PWM↓(95→85)、SHARP_MIN_PIVOT_TICKS↓(70→50)
- 提早退出急转弯 → SHARP_MIN_PIVOT_TICKS↓、退出条件放宽
- 急转弯误触发 → CENTER_SHARP_WINDOW_TICKS↓(20→15)

### 8. 调整微型修正参数

修改 `main.c` 中的微修宏定义：
```c
#define MICRO_OUTER_PWM         98   // 外侧轮速度
#define MICRO_INNER_PWM         92   // 内侧轮速度
#define MICRO_TO_SHARP_TICKS    20   // 微修→急转弯升级窗口
```

- 微修太弱压不住小弯 → MICRO_TO_SHARP_TICKS↓(20→15)，更快升级急转弯
- 微修太猛冲出去 → 增大内外侧差速（MICRO_OUTER_PWM↑/MICRO_INNER_PWM↓）
- 微修太频繁干扰PID → MICRO_TO_SHARP_TICKS↑(20→30)

> **核心原则**: 内侧轮不降速，仅靠差速转向。当前差速约 5~7 个百分点。

### 9. 防停转调试

电机低压或负载大时可能停转：
```c
#define MIN_RUN_PWM    70   // 最低正转速度
#define STALL_MARGIN   10   // PID路径额外余量 → 有效最低=80
```

- 电机嗡嗡响但不转 → MIN_RUN_PWM↑(70→75~80)
- PID路径突然慢下来 → STALL_MARGIN↑(10→15)
- 急转弯太猛冲出赛道 → MIN_RUN_PWM↓(70→65) 配合降低 SHARP_FORWARD_PWM

### 10. 死区/丢线调试

死区处理已改为三段式（直行→偏转→掉头），不再是简单保持上次PWM：

```c
#define LOST_STRAIGHT_TICKS    50   // 阶段1直行时间(ms)
#define LOST_CONFIRM_TICKS     150  // 阶段2偏转上限 / 阶段3掉头触发时间(ms)
```

- 转弯后死区冲不过（过早掉头） → LOST_CONFIRM_TICKS↑(150→200)
- 转弯后死区太长冲出去 → LOST_STRAIGHT_TICKS↓(50→30)，让阶段2更快开始偏转
- 阶段2偏转太弱找不到线 → 增大死区处理中 ±15 的偏转量
- 急转弯出口后偏转错误方向 → 已修复：急转弯退出清零 last_error=0
- 十字路口后误触发急转弯 → 已修复：CROSS_HOLD 入口/出口清除 center-sharp 窗口

**阶段2偏转量位于** `Track_HandleLostUturn` 函数中 `BASE_FAST ± 15`，可根据需要调整。

## 构建和烧录

**项目文件**: `PWM.uvproj` (Keil uVision项目文件)

1. 用Keil uVision打开 `PWM.uvproj`
2. 修改源文件
3. Build (F7) 编译
4. 用编程器烧录到STC32G单片机

## Git 分支

| 分支 | 内容 |
|------|------|
| `main` | 原始版本：纯状态机 + bang-bang 控制 |
| `PID+状态机` | **当前开发分支**：PID比例控制 + 状态机兜底 |

## 已知约定

- 所有引脚控制使用 `sbit` 定义
- I2C采用软件位移实现（P0_0/P0_1和P3_3/P5_4管脚）
- 循迹传感器活性为高电平(逻辑1)表示有线条
- PWM占空比设置后需调用 `UpdatePwm()` 才能生效
- 定时器中断处理应尽量简洁，避免长时间占用
- 状态切换时自动清零 pid_integral（StartSharpLeft/Right, ResetStraight, 过十字路口）

## 实用的调试技巧

1. **查看循迹状态** - 在OLED上显示当前状态和传感器读值
2. **电机测试** - 通过按键单独测试左右电机
3. **PID参数调优** - 先调Kp让弯道能过，再调Kd抑制直道震荡，最后开Ki消除稳态偏置
4. **串口调试** - 可以启用UART输出PID各分量用于分析

## 相关文件速查

| 任务 | 文件 | 位置 |
|------|------|------|
| 改速度 | `main.c` | BASE_FAST / BASE_NODE / MAX_PWM |
| 防停转 | `main.c` | MIN_RUN_PWM / STALL_MARGIN |
| 改平衡 | `main.c` | LEFT_TRIM / RIGHT_TRIM |
| 调PID | `main.c` | PID_KP / PID_KI / PID_KD / PID_OUTPUT_MAX |
| 调急转弯 | `main.c` | SHARP_FORWARD_PWM / SHARP_REVERSE_PWM / SHARP_MIN_PIVOT_TICKS |
| 调微修 | `main.c` | MICRO_OUTER_PWM / MICRO_INNER_PWM / MICRO_TO_SHARP_TICKS |
| PID核心算法 | `main.c` | Track_PIDControl() 函数 |
| 循迹主逻辑 | `main.c` | Track_NormalControl() 函数 |
| 急转弯入口 | `main.c` | Track_StartSharpLeft() / Track_StartSharpRight() |
| OLED显示 | `oled.c` | 所有 oled_Show* 函数 |
| 中断处理 | `STC32G_*_Isr.c` | 各中断处理函数 |
| PWM配置 | `main.c` | PWM_config() 函数 |
