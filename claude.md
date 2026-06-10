# STC32G小车项目 - AI助手指导

## 项目概述

这是一个**STC32G单片机自动循迹小车**项目，用于大学生电子设计竞赛。主要功能包括电机控制、循迹传感器、OLED显示和超声波测距。

## 架构和模块

### 核心功能模块

| 模块 | 文件 | 功能 |
|------|------|------|
| **循迹控制** | `main.c` | 主程序逻辑，循迹算法，电机控制 |
| **PWM驱动** | `STC32G_PWM.*` | 双电机速度控制（PWM5/6） |
| **定时器** | `STC32G_Timer.*` | 1ms周期中断，控制循环 |
| **GPIO** | `STC32G_GPIO.*` | 引脚模式配置 |
| **中断管理** | `STC32G_NVIC.*` | 中断优先级配置 |
| **I2C通信** | `iic.*` + `STC32G_Soft_I2C.*` | 软件I2C实现 |
| **OLED显示** | `oled.*` + `font.*` | 128×64 OLED显示 |
| **超声波** | `main.c` (Timer1) | 测距功能 |
| **延时** | `STC32G_Delay.*` | 毫秒延时 |

### 硬件配置

**主频**: 24MHz  
**编译器**: Keil µVision  

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
| Timer1 | 超声波测距计时 | 1µs | Priority_0 |
| Timer2 | 控制循环分频 | 100µs | Priority_0 |

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

### 关键参数(main.c)

```c
// 速度控制
BASE_FAST = 85          // 直行基础速度
MAX_PWM = 92            // PWM最大值
MIN_RUN_PWM = 60        // 最小运行速度
LOST_TURN_PWM = 80      // 丢线转向速度

// 平衡调整
LEFT_TRIM = 0           // 左电机补偿
RIGHT_TRIM = 6          // 右电机补偿

// 循迹状态
TRACK_FOLLOW            // 正常循迹
TRACK_CROSS_HOLD        // 十字路口
TRACK_UTURN_*           // U型掉头
TRACK_SHARP_LEFT/RIGHT  // 急转弯
```

## 常见任务指导

### 1. 修改电机速度

编辑 `main.c` 中的参数:
```c
#define BASE_FAST     85   // 直行速度
#define MAX_PWM       92   // 最大速度
#define MIN_RUN_PWM   60   // 最小速度
```

**注意**: 修改后需用Keil重新编译和下载到单片机。

### 2. 调整循迹平衡

修改 `LEFT_TRIM` 和 `RIGHT_TRIM` 值（范围-10~10），用于补偿电机速度差异：
```c
#define LEFT_TRIM     0
#define RIGHT_TRIM    6
```

### 3. 更改OLED显示内容

编辑 `oled.*` 文件中的显示函数，通过I2C协议：
- `oled_ShowStr()` - 显示字符串
- `oled_ShowNum()` - 显示数字
- `oled_ShowChar()` - 显示单个字符

### 4. 添加传感器读取

在 `main.c` 的 `ReadTrackSensors()` 中查看5路循迹传感器读取逻辑。

### 5. 调试超声波测距

`Ultrasonic_GetDistance()` 函数在 `main.c` 中，返回距离(mm)。

## 构建和烧录

**项目文件**: `PWM.uvproj` (Keil µVision项目文件)

1. 用Keil µVision打开 `PWM.uvproj`
2. 修改源文件
3. Build (F7) 编译
4. 用编程器烧录到STC32G单片机

## 已知约定

- 所有引脚控制使用 `sbit` 定义
- I2C采用软件位移实现（P0_0/P0_1和P3_3/P5_4管脚）
- 循迹传感器活性为高电平(逻辑1)表示有线条
- PWM占空比设置后需调用 `UpdatePwm()` 才能生效
- 定时器中断处理应尽量简洁，避免长时间占用

## 实用的调试技巧

1. **查看循迹状态** - 在OLED上显示当前状态和传感器读值
2. **电机测试** - 通过按键单独测试左右电机
3. **参数保存** - 考虑使用STC32G的EEPROM存储最优参数
4. **串口调试** - 可以启用UART输出调试信息

## 相关文件速查

| 任务 | 文件 | 位置 |
|------|------|------|
| 改速度 | `main.c` | 第50-90行 |
| 改平衡 | `main.c` | `LEFT_TRIM/RIGHT_TRIM` |
| OLED显示 | `oled.c` | 所有 oled_Show* 函数 |
| 中断处理 | `STC32G_*_Isr.c` | 各中断处理函数 |
| PWM配置 | `main.c` | `PWM_config()` 函数 |
| 循迹算法 | `main.c` | `Track_Control()` 函数 |
