/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- BBS: www.STCAIMCU.com  -----------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* 如需在程序中使用此代码,请在程序中注明使用STC的资料及引用            */
/*---------------------------------------------------------------------*/

#include	"config.h"
#include	"STC32G_PWM.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"
#include	"STC32G_Delay.h"
#include	"iic.h"
#include	"font.h"
#include	"oled.h"




/*************   功能说明   **************

高级PWM定时器 PWM5,PWM6,PWM7,PWM8 每个通道均可独立实现PWM输出.

4通道PWM输出都需要设置对应端口，各通道示例可观察输出信号.

PWM周期和占空比可自由设置，最高可达65535.

默认时钟, 选择时钟 24MHZ (用户可在"config.h"修改频率).

******************************************/

sbit AIN1=P4^5;
sbit AIN2=P2^7;
sbit BIN1=P2^5;
sbit BIN2=P2^6;
sbit zuo2=P0^3;
sbit zuo1=P0^4;
sbit zhong=P0^2;
sbit you1=P0^1;
sbit you2=P0^0;
sbit LED1=P4^1;
sbit LED2=P4^2;
sbit LED3=P4^4;
sbit TRIG = P0^6;
sbit ECHO = P1^4;
sbit KEY1 = P2^2;
sbit KEY2 = P2^3;
sbit KEY3 = P2^4;
sbit KEY4 = P1^3;
sbit KEY5 = P1^7;

volatile unsigned int time_us = 0;
bit measuring = 0;

PWMx_Duty PWMB_Duty;
u16 PWM_Period = 2000; // 2000
bit PWM5_Flag;
bit PWM6_Flag;

/* ===== 循环控制使用5路传感器，有效电平为1 =====
   优先级：响应速度 > 行走稳定 > 循环速度和准确性。
   调试建议：
   1. 响应速度：确保 CTRL_DIV_1MS 为10，不要调大固定转向保持时间。
   2. 低速抖/只抖不转：调整 MIN_RUN_PWM 和 MICRO_INNER_PWM。
   3. 小循出不去：调整 MICRO_OUTER_PWM 和 SHARP_FORWARD_PWM。
   4. 直行冲出去：调大 SHARP_FORWARD_PWM，不要增加固定转动时间。
*/
#define LINE_ON             1       /* 传感器有效电平：黑线=1 白底=0 */
#define CTRL_DIV_1MS        10      /* Timer2为100us中断，10次执行一次循环约1ms */

#define BASE_FAST           110     /* 00100直行速度 */
#define BASE_NODE           100     /* 十字路口中途丢线时匀速直行速度 */
#define MAX_PWM             130      /* PWM上限幅度 */
#define MIN_RUN_PWM         60      /* 最低不停转时的PWM */
#define LEFT_TRIM           3       /* 左电机平衡，调节范围-10~10 */
#define RIGHT_TRIM          6       /* 右电机平衡，调节范围-10~10 */
#define LOST_TURN_PWM       95      /* 丢线全黑后沿上次方向搜索的速度 */

#define HOLD_CURRENT_MS     110     /* 维持当前方向 */
#define CROSS_HOLD_MS       110      /* 十字路口直行保持时间 */
#define UTURN_STOP_MS       500     /* 调头前停车时间 */

#define MICRO_OUTER_PWM     105     /* 微调转弯外侧速度（≥BASE_FAST防顿挫） */
#define MICRO_INNER_PWM     115     /* 微调转弯内侧速度 */
#define MICRO_TO_SHARP_TICKS 20     /* 小循窗口触发的急转最短保持时间 */
#define CENTER_SHARP_WINDOW_TICKS 15 /* 中间和外传感器近期出现的时间窗口 */
#define POST_SHARP_GRACE_TICKS 20    /* 急转弯退出后宽限期，防止00100瞬间拉回 */
#define MICRO_DIR_NONE       0
#define MICRO_DIR_LEFT      -1
#define MICRO_DIR_RIGHT      1

#define COAST_TICKS          3       /* 惰行时长 (ms)，让齿轮脱离贴合面 */
#define KICK_TICKS           4       /* 过冲满扭时长 (ms)，暴力破静摩擦 */
#define BRAKE_PHASE_NONE     0
#define BRAKE_PHASE_COAST    1       /* 惰行中 */
#define BRAKE_PHASE_KICK     2       /* 过冲满扭中 */

#define SHARP_FORWARD_PWM   95      /* 急转弯外侧正转速度 */
#define SHARP_REVERSE_PWM    115    /* 急转弯内侧反转速度（加大快速旋转） */
#define SHARP_MIN_PIVOT_TICKS 30    /* 急转弯最短强制转时间 */
#define CENTER_SHARP_MIN_PIVOT_TICKS 35 /* 窗口急弯强制转时间 */

#define UTURN_PWM           90      /* 调头速度 */

#define LOST_CONFIRM_TICKS  70    /* 丢线确认掉头阈值（折线防误触） */
#define LOST_STRAIGHT_TICKS 30     /* 丢线全黑补线保持时间 */

#define MASK_ZUO2           0x10    /* 1号传感器左2 */
#define MASK_ZUO1           0x08    /* 2号传感器左1 */
#define MASK_ZHONG          0x04    /* 3号传感器中间 */
#define MASK_YOU1           0x02    /* 4号传感器右1 */
#define MASK_YOU2           0x01    /* 5号传感器右2 */
#define MASK_ALL            (MASK_ZUO2 | MASK_ZUO1 | MASK_ZHONG | MASK_YOU1 | MASK_YOU2)

#define TRACK_FOLLOW            0
#define TRACK_CROSS_HOLD        1
#define TRACK_UTURN_STOP        2
#define TRACK_UTURN_RIGHT       3
#define TRACK_SHARP_LEFT        4
#define TRACK_SHARP_RIGHT       5

signed char last_error = 0;
u16 lost_ticks = 0;
u8 ctrl_divider = 0;
int last_left_pwm = BASE_FAST;
int last_right_pwm = BASE_FAST;
u8 track_state = TRACK_FOLLOW;
u16 track_state_ticks = 0;
u8 cross_latched = 0;
signed char micro_dir = MICRO_DIR_NONE;
u8 micro_ticks = 0;
u8 center_seen_ticks = CENTER_SHARP_WINDOW_TICKS + 1;
u8 left_outer_seen_ticks = CENTER_SHARP_WINDOW_TICKS + 1;
u8 right_outer_seen_ticks = CENTER_SHARP_WINDOW_TICKS + 1;
u16 sharp_min_pivot_ticks = SHARP_MIN_PIVOT_TICKS;

/* 三阶段换向保护：惰行脱齿 → 过冲破摩擦 → 目标反转，防止齿轮冲击卡死 */
signed char left_dir = 1;           /* 左轮当前有效方向: 1=正转 -1=反转 0=停止 */
signed char right_dir = 1;          /* 右轮当前有效方向 */
u8 left_brake_phase = BRAKE_PHASE_NONE;   /* 左轮当前阶段 */
u8 right_brake_phase = BRAKE_PHASE_NONE;  /* 右轮当前阶段 */
u8 left_phase_ticks = 0;           /* 左轮当前阶段剩余 ms */
u8 right_phase_ticks = 0;          /* 右轮当前阶段剩余 ms */
int left_stored_target = 0;         /* 左轮刹车前保存的目标 PWM */
int right_stored_target = 0;        /* 右轮刹车前保存的目标 PWM */
u8 post_sharp_grace = 0;            /* 急转弯退出宽限期倒计时 */


/*************   函数声明   **************/

int LimitPwm(int pwm);
int LimitSignedPwm(int pwm);
int AbsPwm(int pwm);
void PWM_Run(int pwm1,int pwm2);
void Motor_RunSigned(int left_pwm, int right_pwm);
void Motor_RunSafe(int left_pwm, int right_pwm);
void Motor_HoldLast(void);
u8 ReadTrackSensors(void);
u8 CountBits(u8 mask);
u8 IsCrossMask(u8 mask);
u8 Track_IsImmediateSharpLeft(u8 mask);
u8 Track_IsImmediateSharpRight(u8 mask);
void Track_ClearMicroWindow(void);
void Track_RecordMicro(signed char dir);
void Track_TickMicroWindow(void);
void Track_ClearCenterSharpWindow(void);
void Track_TickCenterSharpWindow(u8 mask);
u8 Track_IsCenterSharpWindowActive(u8 ticks);
u8 Track_ShouldCenterSharpLeft(u8 mask);
u8 Track_ShouldCenterSharpRight(u8 mask);
u8 Track_ShouldMicroSharpLeft(u8 mask);
u8 Track_ShouldMicroSharpRight(u8 mask);
u8 Track_HandleLostUturn(u8 count);
void Track_EnterState(u8 state);
void Track_StartSharpLeft(u16 min_pivot_ticks);
void Track_StartSharpRight(u16 min_pivot_ticks);
void Track_ResetStraight(void);
void Track_NormalControl(u8 mask, u8 count);
void Track_Control(void);

/*************  外部函数和变量声明 *****************/



/************************ IO初始化 ****************************/
void	GPIO_config(void)
{

	
}

/************************ 定时器初始化 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;
	TIM_InitStructure.TIM_ClkOut    = DISABLE;
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));
	TIM_InitStructure.TIM_PS        = 0;
	TIM_InitStructure.TIM_Run       = ENABLE;
	Timer_Inilize(Timer0,&TIM_InitStructure);
	NVIC_Timer0_Init(ENABLE,Priority_0);
}

void Timer1_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;
    TIM_InitStructure.TIM_ClkOut    = DISABLE;
	TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 100000UL));
    TIM_InitStructure.TIM_PS        = 0;
    TIM_InitStructure.TIM_Run       = ENABLE;

    Timer_Inilize(Timer1, &TIM_InitStructure);
    NVIC_Timer1_Init(ENABLE, Priority_0);
}

void Timer2_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;
    TIM_InitStructure.TIM_ClkOut    = DISABLE;
	TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 10000UL));
    TIM_InitStructure.TIM_PS        = 0;
    TIM_InitStructure.TIM_Run       = ENABLE;

    Timer_Inilize(Timer2, &TIM_InitStructure);
    NVIC_Timer2_Init(ENABLE, Priority_0);
}

/***************  超声波测距 *****************/
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR
{
	if (measuring) time_us++;
}

unsigned int Ultrasonic_GetDistance(void) 
{
    unsigned int Distance_mm = 0;
    TRIG = 0;
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 1;
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 0;

    while (!ECHO);
    time_us = 0;
    measuring = 1;
    while (ECHO);

    measuring = 0;
	if(time_us/100<38)
	{
		Distance_mm=(time_us*346)/100;
	}
    return Distance_mm;
}

/***************  PWM初始化函数 *****************/
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;
	
//	PWMB_Duty.PWM5_Duty = 128;
//	PWMB_Duty.PWM6_Duty = 256;

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;
	PWM_Configuration(PWM5, &PWMx_InitStructure);

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;
	PWM_Configuration(PWM6, &PWMx_InitStructure);

	PWMx_InitStructure.PWM_Period   = PWM_Period;
	PWMx_InitStructure.PWM_DeadTime = 0;
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;
	PWM_Configuration(PWMB, &PWMx_InitStructure);

	PWM6_USE_P21();
	PWM5_USE_P20();
	NVIC_PWM_Init(PWMB,DISABLE,Priority_0);
}
int LimitPwm(int pwm)
{
	if(pwm < 0) return 0;
	if(pwm > MAX_PWM) return MAX_PWM;
	return pwm;
}

int LimitSignedPwm(int pwm)
{
	if(pwm > MAX_PWM) return MAX_PWM;
	if(pwm < -MAX_PWM) return -MAX_PWM;
	return pwm;
}

int AbsPwm(int pwm)
{
	if(pwm < 0) return -pwm;
	return pwm;
}

void Motor_LeftStop(void)
{
	AIN1 = 0;
	AIN2 = 0;
}

void Motor_LeftBrake(void)
{
	/* 电磁抱闸：H桥两路同时拉高 → 电机绕组短路 → 强力制动 */
	AIN1 = 1;
	AIN2 = 1;
}

void Motor_RightBrake(void)
{
	BIN1 = 1;
	BIN2 = 1;
}

void Motor_LeftForward(void)
{
	AIN1 = 0;
	AIN2 = 1;
}

void Motor_LeftReverse(void)
{
	AIN1 = 1;
	AIN2 = 0;
}

void Motor_RightStop(void)
{
	BIN1 = 0;
	BIN2 = 0;
}

void Motor_RightForward(void)
{
	BIN1 = 1;
	BIN2 = 0;
}

void Motor_RightReverse(void)
{
	BIN1 = 0;
	BIN2 = 1;
}

void PWM_Left(int pwm)     //以0~100表示
{
	
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);  //注意PWM_Period默认为100除以 2000
	
}

void PWM_Right(int pwm)     //以0~100表示
{
	
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);
	
}


void PWM_Run(int pwm1,int pwm2)  //更新PWM
{
	pwm1 = LimitPwm(pwm1);
	pwm2 = LimitPwm(pwm2);
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
}

void Motor_RunSigned(int left_pwm, int right_pwm)
{
	int left_abs;
	int right_abs;
	signed char target_left_dir;
	signed char target_right_dir;

	left_pwm = LimitSignedPwm(left_pwm);
	right_pwm = LimitSignedPwm(right_pwm);

	if(left_pwm != 0 || right_pwm != 0)
	{
		if(left_pwm != 0 && AbsPwm(left_pwm) < MIN_RUN_PWM)
		{
			left_pwm = (left_pwm < 0) ? -MIN_RUN_PWM : MIN_RUN_PWM;
		}

		if(right_pwm != 0 && AbsPwm(right_pwm) < MIN_RUN_PWM)
		{
			right_pwm = (right_pwm < 0) ? -MIN_RUN_PWM : MIN_RUN_PWM;
		}
	}

	/* ===== 三阶段换向：惰行脱齿 → 过冲破摩擦 → 目标反转 ===== */
	target_left_dir  = (left_pwm  > 0) ? 1 : (left_pwm  < 0) ? -1 : 0;
	target_right_dir = (right_pwm > 0) ? 1 : (right_pwm < 0) ? -1 : 0;

	/* ---- 左轮 ---- */
	switch(left_brake_phase)
	{
	case BRAKE_PHASE_COAST:
		/* 阶段1：惰行 — 齿轮脱开贴合面，无电磁阻力 */
		left_phase_ticks--;
		left_pwm = 0;
		target_left_dir = 0;
		if(left_phase_ticks == 0)
		{
			left_brake_phase = BRAKE_PHASE_KICK;
			left_phase_ticks = KICK_TICKS;
		}
		break;

	case BRAKE_PHASE_KICK:
		/* 阶段2：过冲满扭 — MAX_PWM 暴力破静摩擦 */
		left_phase_ticks--;
		left_pwm = (left_stored_target < 0) ? -MAX_PWM : MAX_PWM;
		target_left_dir = (left_pwm > 0) ? 1 : -1;
		if(left_phase_ticks == 0)
		{
			/* 过冲结束 → 回落到目标反转值 */
			left_brake_phase = BRAKE_PHASE_NONE;
			left_pwm = left_stored_target;
			target_left_dir = (left_pwm > 0) ? 1 : (left_pwm < 0) ? -1 : 0;
		}
		break;

	default: /* BRAKE_PHASE_NONE */
		if(left_dir != 0 && target_left_dir == -left_dir)
		{
			/* 正→反 或 反→正 → 启动惰行 */
			left_brake_phase = BRAKE_PHASE_COAST;
			left_phase_ticks = COAST_TICKS;
			left_stored_target = left_pwm;
			left_pwm = 0;
			target_left_dir = 0;
		}
		break;
	}

	/* ---- 右轮 ---- */
	switch(right_brake_phase)
	{
	case BRAKE_PHASE_COAST:
		right_phase_ticks--;
		right_pwm = 0;
		target_right_dir = 0;
		if(right_phase_ticks == 0)
		{
			right_brake_phase = BRAKE_PHASE_KICK;
			right_phase_ticks = KICK_TICKS;
		}
		break;

	case BRAKE_PHASE_KICK:
		right_phase_ticks--;
		right_pwm = (right_stored_target < 0) ? -MAX_PWM : MAX_PWM;
		target_right_dir = (right_pwm > 0) ? 1 : -1;
		if(right_phase_ticks == 0)
		{
			right_brake_phase = BRAKE_PHASE_NONE;
			right_pwm = right_stored_target;
			target_right_dir = (right_pwm > 0) ? 1 : (right_pwm < 0) ? -1 : 0;
		}
		break;

	default: /* BRAKE_PHASE_NONE */
		if(right_dir != 0 && target_right_dir == -right_dir)
		{
			right_brake_phase = BRAKE_PHASE_COAST;
			right_phase_ticks = COAST_TICKS;
			right_stored_target = right_pwm;
			right_pwm = 0;
			target_right_dir = 0;
		}
		break;
	}

	left_dir  = target_left_dir;
	right_dir = target_right_dir;

	left_abs = AbsPwm(left_pwm);
	right_abs = AbsPwm(right_pwm);

	/* 惰行阶段用滑行（无阻力），过冲/正常按方向控制 */
	if(left_brake_phase == BRAKE_PHASE_COAST && left_pwm == 0)
		Motor_LeftStop();
	else if(left_pwm > 0) Motor_LeftForward();
	else if(left_pwm < 0) Motor_LeftReverse();
	else Motor_LeftStop();

	if(right_brake_phase == BRAKE_PHASE_COAST && right_pwm == 0)
		Motor_RightStop();
	else if(right_pwm > 0) Motor_RightForward();
	else if(right_pwm < 0) Motor_RightReverse();
	else Motor_RightStop();

	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
	PWM_Run(left_abs, right_abs);
}

void Motor_RunSafe(int left_pwm, int right_pwm)
{
	/* 只在0,0时停车，其余循迹时保持定速不给反转PWM */
	if(left_pwm == 0 && right_pwm == 0)
	{
		Motor_RunSigned(0, 0);
		return;
	}

	left_pwm += LEFT_TRIM;
	right_pwm += RIGHT_TRIM;
	left_pwm = LimitPwm(left_pwm);
	right_pwm = LimitPwm(right_pwm);

	if(left_pwm < MIN_RUN_PWM) left_pwm = MIN_RUN_PWM;
	if(right_pwm < MIN_RUN_PWM) right_pwm = MIN_RUN_PWM;

	Motor_RunSigned(left_pwm, right_pwm);
}

void Motor_HoldLast(void)
{
	Motor_RunSigned(last_left_pwm, last_right_pwm);
}

void test01(void)
{
		Motor_RunSafe(100,80);
		delay_ms(1000);
		Motor_RunSafe(80,100);
		delay_ms(1000);
		Motor_RunSafe(MIN_RUN_PWM,100);
		delay_ms(1000);
		Motor_RunSafe(100,MIN_RUN_PWM);
		delay_ms(1000);
}

u8 ReadTrackSensors(void)
{
	/* 读取5路循迹传感器 左2/左1/中/右1/右2 低位bit对齐 */
	u8 mask;
	mask = 0;
	if(zuo2 == LINE_ON)  mask |= MASK_ZUO2;
	if(zuo1 == LINE_ON)  mask |= MASK_ZUO1;
	if(zhong == LINE_ON) mask |= MASK_ZHONG;
	if(you1 == LINE_ON)  mask |= MASK_YOU1;
	if(you2 == LINE_ON)  mask |= MASK_YOU2;
	return mask;
}

u8 CountBits(u8 mask)
{
	u8 count;
	count = 0;
	if(mask & MASK_ZUO2)  count++;
	if(mask & MASK_ZUO1)  count++;
	if(mask & MASK_ZHONG) count++;
	if(mask & MASK_YOU1)  count++;
	if(mask & MASK_YOU2)  count++;
	return count;
}

u8 IsCrossMask(u8 mask)
{
	u8 has_left, has_right;
	/* x111x 标准十字 */
	if((mask & (MASK_ZUO1 | MASK_ZHONG | MASK_YOU1)) == (MASK_ZUO1 | MASK_ZHONG | MASK_YOU1))
		return 1;
	/* 十字非对称接近：中间亮 + 左右两侧都有传感器（排除单侧大弯） */
	has_left  = mask & (MASK_ZUO2 | MASK_ZUO1);
	has_right = mask & (MASK_YOU1 | MASK_YOU2);
	if((mask & MASK_ZHONG) && has_left && has_right)
		return 1;
	return 0;
}

u8 Track_IsImmediateSharpLeft(u8 mask)
{
	return ((mask & MASK_ZUO2) && !(mask & (MASK_YOU1 | MASK_YOU2)));
}

u8 Track_IsImmediateSharpRight(u8 mask)
{
	return ((mask & MASK_YOU2) && !(mask & (MASK_ZUO2 | MASK_ZUO1)));
}

void Track_ClearMicroWindow(void)
{
	micro_dir = MICRO_DIR_NONE;
	micro_ticks = 0;
}

void Track_RecordMicro(signed char dir)
{
	micro_dir = dir;
	micro_ticks = 0;
}

void Track_TickMicroWindow(void)
{
	if(micro_dir != MICRO_DIR_NONE)
	{
		if(micro_ticks < MICRO_TO_SHARP_TICKS)
		{
			micro_ticks++;
		}
		else
		{
			Track_ClearMicroWindow();
		}
	}
}

void Track_ClearCenterSharpWindow(void)
{
	center_seen_ticks = CENTER_SHARP_WINDOW_TICKS + 1;
	left_outer_seen_ticks = CENTER_SHARP_WINDOW_TICKS + 1;
	right_outer_seen_ticks = CENTER_SHARP_WINDOW_TICKS + 1;
}

void Track_TickCenterSharpWindow(u8 mask)
{
	if(mask & MASK_ZHONG) center_seen_ticks = 0;
	else if(center_seen_ticks <= CENTER_SHARP_WINDOW_TICKS) center_seen_ticks++;

	if(mask & MASK_ZUO2) left_outer_seen_ticks = 0;
	else if(left_outer_seen_ticks <= CENTER_SHARP_WINDOW_TICKS) left_outer_seen_ticks++;

	if(mask & MASK_YOU2) right_outer_seen_ticks = 0;
	else if(right_outer_seen_ticks <= CENTER_SHARP_WINDOW_TICKS) right_outer_seen_ticks++;
}

u8 Track_IsCenterSharpWindowActive(u8 ticks)
{
	return (ticks <= CENTER_SHARP_WINDOW_TICKS);
}

u8 Track_ShouldMicroSharpLeft(u8 mask)
{
	return (micro_dir == MICRO_DIR_LEFT && (mask & MASK_ZUO2) && (mask & (MASK_ZUO1 | MASK_ZHONG)));
}

u8 Track_ShouldMicroSharpRight(u8 mask)
{
	return (micro_dir == MICRO_DIR_RIGHT && (mask & MASK_YOU2) && (mask & (MASK_YOU1 | MASK_ZHONG)));
}

u8 Track_ShouldCenterSharpLeft(u8 mask)
{
	u8 center_recent;
	u8 left_recent;
	u8 right_recent;

	center_recent = Track_IsCenterSharpWindowActive(center_seen_ticks);
	left_recent = Track_IsCenterSharpWindowActive(left_outer_seen_ticks);
	right_recent = Track_IsCenterSharpWindowActive(right_outer_seen_ticks);

	if(!center_recent || !left_recent) return 0;
	if(left_recent && right_recent)
	{
		return ((mask & MASK_ZUO2) && !(mask & MASK_YOU2));
	}
	return 1;
}

u8 Track_ShouldCenterSharpRight(u8 mask)
{
	u8 center_recent;
	u8 left_recent;
	u8 right_recent;

	center_recent = Track_IsCenterSharpWindowActive(center_seen_ticks);
	left_recent = Track_IsCenterSharpWindowActive(left_outer_seen_ticks);
	right_recent = Track_IsCenterSharpWindowActive(right_outer_seen_ticks);

	if(!center_recent || !right_recent) return 0;
	if(left_recent && right_recent)
	{
		return ((mask & MASK_YOU2) && !(mask & MASK_ZUO2));
	}
	return 1;
}

u8 Track_HandleLostUturn(u8 count)
{
	if(count != 0)
	{
		return 0;
	}

	if(lost_ticks < 60000) lost_ticks++;
	if(lost_ticks < LOST_CONFIRM_TICKS)
	{
		/* 保持阶段：沿丢线前方向主动回找，而非直冲 */
		if(lost_ticks < 10)
		{
			Motor_HoldLast();
		}
		else if(last_error < 0)
		{
			Motor_RunSafe(BASE_NODE, LOST_TURN_PWM);
		}
		else if(last_error > 0)
		{
			Motor_RunSafe(LOST_TURN_PWM, BASE_NODE);
		}
		else
		{
			/* 无方向记忆 → 沿维持方向微转 */
			Motor_HoldLast();
		}
	}
	else
	{
		/* 确认掉头：按丢线前方向决定掉头方向 */
		Track_ClearMicroWindow();
		if(last_error < 0)
		{
			Track_EnterState(TRACK_UTURN_RIGHT);
			Motor_RunSigned(-UTURN_PWM, UTURN_PWM);
		}
		else
		{
			Track_EnterState(TRACK_UTURN_RIGHT);
			Motor_RunSigned(UTURN_PWM, -UTURN_PWM);
		}
	}
	return 1;
}

void Track_EnterState(u8 state)
{
	track_state = state;
	track_state_ticks = 0;
}

void Track_StartSharpLeft(u16 min_pivot_ticks)
{
	Track_ClearMicroWindow();
	Track_ClearCenterSharpWindow();
	last_error = -4;
	sharp_min_pivot_ticks = min_pivot_ticks;
	Track_EnterState(TRACK_SHARP_LEFT);
	Motor_RunSigned(-SHARP_REVERSE_PWM, SHARP_FORWARD_PWM);
}

void Track_StartSharpRight(u16 min_pivot_ticks)
{
	Track_ClearMicroWindow();
	Track_ClearCenterSharpWindow();
	last_error = 4;
	sharp_min_pivot_ticks = min_pivot_ticks;
	Track_EnterState(TRACK_SHARP_RIGHT);
	Motor_RunSigned(SHARP_FORWARD_PWM, -SHARP_REVERSE_PWM);
}

void Track_ResetStraight(void)
{
	Track_ClearMicroWindow();
	Track_ClearCenterSharpWindow();
	last_error = 0;
	lost_ticks = 0;
	Track_EnterState(TRACK_FOLLOW);
	Motor_RunSafe(BASE_FAST, BASE_FAST);
}

void Track_NormalControl(u8 mask, u8 count)
{
	/* 00100纯中间：宽限期内抑制复位，让车通过死区缝隙 */
	if(mask == MASK_ZHONG && post_sharp_grace == 0)
	{
		Track_ResetStraight();
		return;
	}

	/* 00000全部丢线达到确认时间后原地找掉头 */
	if(Track_HandleLostUturn(count))
	{
		return;
	}

	/* 十字路口优先于窗口急弯，防止右侧分支被误判为急转弯 */
	if(IsCrossMask(mask))
	{
		Track_ClearMicroWindow();
		if(!cross_latched)
		{
			cross_latched = 1;
			Track_EnterState(TRACK_CROSS_HOLD);
			Motor_RunSigned(BASE_NODE, BASE_NODE);
			return;
		}
		last_error = 0;
		Motor_RunSigned(BASE_NODE, BASE_NODE);
		return;
	}

	/* 窗口急弯 */
	if(Track_ShouldCenterSharpLeft(mask))
	{
		Track_StartSharpLeft(CENTER_SHARP_MIN_PIVOT_TICKS);
		return;
	}
	if(Track_ShouldCenterSharpRight(mask))
	{
		Track_StartSharpRight(CENTER_SHARP_MIN_PIVOT_TICKS);
		return;
	}


	/* 1xx00/00xx1是急转弯，低于掉头和十字直行 */
	if(Track_IsImmediateSharpLeft(mask))
	{
		Track_StartSharpLeft(SHARP_MIN_PIVOT_TICKS);
		return;
	}
	if(Track_IsImmediateSharpRight(mask))
	{
		Track_StartSharpRight(SHARP_MIN_PIVOT_TICKS);
		return;
	}

	/* 小循后20ms内，同向外侧触发时触发为急转弯 */
	if(Track_ShouldMicroSharpLeft(mask))
	{
		Track_StartSharpLeft(SHARP_MIN_PIVOT_TICKS);
		return;
	}
	if(Track_ShouldMicroSharpRight(mask))
	{
		Track_StartSharpRight(SHARP_MIN_PIVOT_TICKS);
		return;
	}

	/* 大循窗口：11000/10000左循，00011/00001右循，10001沿上次方向 */
	if(mask == (MASK_ZUO2 | MASK_YOU2))
	{
		if(last_error < 0) Track_StartSharpLeft(SHARP_MIN_PIVOT_TICKS);
		else Track_StartSharpRight(SHARP_MIN_PIVOT_TICKS);
		return;
	}
	if(mask == MASK_ZUO2 || mask == (MASK_ZUO2 | MASK_ZUO1))
	{
		Track_StartSharpLeft(SHARP_MIN_PIVOT_TICKS);
		return;
	}
	if(mask == MASK_YOU2 || mask == (MASK_YOU1 | MASK_YOU2))
	{
		Track_StartSharpRight(SHARP_MIN_PIVOT_TICKS);
		return;
	}

	/* 微调窗口：只有内侧/中线偏离为微偏移 */
	if(mask == MASK_ZUO1 || mask == (MASK_ZUO1 | MASK_ZHONG))
	{
		Track_RecordMicro(MICRO_DIR_LEFT);
		last_error = -2;
		Motor_RunSafe(MICRO_INNER_PWM, MICRO_OUTER_PWM);
		return;
	}
	if(mask == MASK_YOU1 || mask == (MASK_ZHONG | MASK_YOU1))
	{
		Track_RecordMicro(MICRO_DIR_RIGHT);
		last_error = 2;
		Motor_RunSafe(MICRO_OUTER_PWM, MICRO_INNER_PWM);
		return;
	}

	if(mask & MASK_ZHONG)
	{
		last_error = 0;
		Motor_RunSigned(BASE_NODE, BASE_NODE);
		return;
	}

	Motor_RunSafe(BASE_FAST, BASE_FAST);
}

void Track_Control(void)
{
	u8 mask;
	u8 count;

	mask = ReadTrackSensors();
	count = CountBits(mask);
	if(count > 0) lost_ticks = 0;
	Track_TickMicroWindow();
	Track_TickCenterSharpWindow(mask);

	/* 急转弯退出宽限期倒计时 */
	if(post_sharp_grace > 0) post_sharp_grace--;

	if(mask == MASK_ZHONG && post_sharp_grace == 0)
	{
		Track_ResetStraight();
		return;
	}
	if(Track_HandleLostUturn(count))
	{
		return;
	}
	if(IsCrossMask(mask) && track_state != TRACK_CROSS_HOLD)
	{
		Track_ClearMicroWindow();
		cross_latched = 1;
		Track_EnterState(TRACK_CROSS_HOLD);
		Motor_RunSigned(BASE_NODE, BASE_NODE);
		return;
	}
	if(Track_ShouldCenterSharpLeft(mask) && track_state != TRACK_SHARP_LEFT)
	{
		Track_StartSharpLeft(CENTER_SHARP_MIN_PIVOT_TICKS);
		return;
	}
	if(Track_ShouldCenterSharpRight(mask) && track_state != TRACK_SHARP_RIGHT)
	{
		Track_StartSharpRight(CENTER_SHARP_MIN_PIVOT_TICKS);
		return;
	}
	if(Track_IsImmediateSharpLeft(mask) && track_state != TRACK_SHARP_LEFT)
	{
		Track_StartSharpLeft(SHARP_MIN_PIVOT_TICKS);
		return;
	}
	if(Track_IsImmediateSharpRight(mask) && track_state != TRACK_SHARP_RIGHT)
	{
		Track_StartSharpRight(SHARP_MIN_PIVOT_TICKS);
		return;
	}

	if(!IsCrossMask(mask))
	{
		cross_latched = 0;
	}

	switch(track_state)
	{
		case TRACK_CROSS_HOLD:
			if(Track_HandleLostUturn(count))
			{
				return;
			}
			Motor_RunSigned(BASE_NODE, BASE_NODE);
			if(track_state_ticks < 60000) track_state_ticks++;
			if(track_state_ticks >= CROSS_HOLD_MS)
			{
				Track_EnterState(TRACK_FOLLOW);
				Track_NormalControl(mask, count);
			}
			return;

		case TRACK_UTURN_STOP:
			Track_EnterState(TRACK_UTURN_RIGHT);
			Motor_RunSigned(UTURN_PWM, -UTURN_PWM);
			return;

		case TRACK_UTURN_RIGHT:
			if(count > 0)
			{
				Track_EnterState(TRACK_FOLLOW);
				Track_NormalControl(mask, count);
			}
			else
			{
				Motor_RunSigned(UTURN_PWM, -UTURN_PWM);
			}
			return;

		case TRACK_SHARP_LEFT:
			if(track_state_ticks < 60000) track_state_ticks++;
			if(track_state_ticks >= sharp_min_pivot_ticks && !Track_IsImmediateSharpLeft(mask) && (mask & (MASK_ZUO1 | MASK_ZHONG)))
			{
				post_sharp_grace = POST_SHARP_GRACE_TICKS;
				Track_EnterState(TRACK_FOLLOW);
				Track_NormalControl(mask, count);
			}
			else
			{
				Motor_RunSigned(-SHARP_REVERSE_PWM, SHARP_FORWARD_PWM);
			}
			return;

		case TRACK_SHARP_RIGHT:
			if(track_state_ticks < 60000) track_state_ticks++;
			if(track_state_ticks >= sharp_min_pivot_ticks && !Track_IsImmediateSharpRight(mask) && (mask & (MASK_YOU1 | MASK_ZHONG)))
			{
				post_sharp_grace = POST_SHARP_GRACE_TICKS;
				Track_EnterState(TRACK_FOLLOW);
				Track_NormalControl(mask, count);
			}
			else
			{
				Motor_RunSigned(SHARP_FORWARD_PWM, -SHARP_REVERSE_PWM);
			}
			return;

		case TRACK_FOLLOW:
			break;

		default:
			Track_EnterState(TRACK_FOLLOW);
			break;
	}

	Track_NormalControl(mask, count);
}

void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR		//中断时钟已经清除标志
{
	/* Timer2每100us一次，控制频率约1ms执行控制，改大保证响应速度 */
	ctrl_divider++;
	if(ctrl_divider >= CTRL_DIV_1MS)
	{
		ctrl_divider = 0;
		Track_Control();
	}
}
/******************** 主函数 **************************/
void main(void)
{
//	unsigned char i;
	WTST = 0;		//设置程序指令等待时间为0，将CPU执行指令的速度提高为最快
	EAXSFR();		//扩展SFR(XFR)访问使能
	CKCON = 0;      //提高访问XRAM速度

	P2M1=0x00;
	P2M0=0xFF;
	P2M0 &= ~(1 << 2);  // M0=0		p2.2
	P2M0 &= ~(1 << 3);  // M0=0		p2.3
	P2M0 &= ~(1 << 4);  // M0=0		p2.4	
	P0M1=0x00;
	P0M0=0x00;
	P1M1=0x00;
	P1M0=0x00;
	P4M1=0x00;
	P4M0=0xFF;	
	P1M1 |= (1 << 4);   // M1=1
	P1M0 &= ~(1 << 4);  // M0=0	
	
	Timer_config();
	Timer1_config();
	Timer2_config();
	PWM_config();
	oled_Init();
	
	AIN1=0;
	AIN2=1;
	BIN1=1;
	BIN2=0;

	TRIG = 0;
	ECHO = 0;
	time_us = 0;
	EA = 1;
	while (1)
	{
//			unsigned int d = Ultrasonic_GetDistance();
//			oled_clear();
//			oled_ShowNum(56,2,d,5,16);
//			oled_ShowNum(10,2,1,2,16);		
//			delay_ms(300);	

//		if(KEY4 == 0)
//		{
//			LED1=0;
//			delay_ms(1000);
//			LED1=1;
//			delay_ms(1000);
//		}
//		if(KEY5 == 0)
//		{
//			LED2=0;
//			delay_ms(1000);
//			LED2=1;
//			delay_ms(1000);
//		}
//		if(KEY3 == 0)
//		{
//			LED3=0;
//			delay_ms(1000);
//			LED3=1;
//			delay_ms(1000);
//		}
//		for(i=10;i>0;i--)
//		{
//			oled_clear();
//			oled_ShowNum(10,2,i,2,16);		
//			delay_ms(1000);
//		}



//		if(T0_1ms)
//		{
//			
//			PWMB_Duty.PWM5_Duty = 2047;
//			PWMB_Duty.PWM6_Duty = 1;
//			T0_1ms = 0;
			
//			if(!PWM5_Flag)
//			{
//				PWMB_Duty.PWM5_Duty++;
//				if(PWMB_Duty.PWM5_Duty >= 2047) PWM5_Flag = 1;
//			}
//			else
//			{
//				PWMB_Duty.PWM5_Duty--;
//				if(PWMB_Duty.PWM5_Duty <= 0) PWM5_Flag = 0;
//			}
//			if(!PWM6_Flag)
//			{
//				PWMB_Duty.PWM6_Duty++;
//				if(PWMB_Duty.PWM6_Duty >= 2047) PWM6_Flag = 1;
//			}
//			else
//			{
//				PWMB_Duty.PWM6_Duty--;
//				if(PWMB_Duty.PWM6_Duty <= 0) PWM6_Flag = 0;
//			}
			
			
		}
	}




