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
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序            */
/*---------------------------------------------------------------------*/

#include	"config.h"
#include	"STC32G_PWM.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"
#include	"STC32G_Delay.h"
#include  "iic.h"
#include  "font.h"
#include  "oled.h"




/*************	功能说明	**************

高级PWM定时器 PWM5,PWM6,PWM7,PWM8 每个通道都可独立实现PWM输出.

4个通道PWM根据需要设置对应输出口，可通过示波器观察输出的信号.

PWM周期和占空比可以自定义设置，最高可达65535.

下载时, 选择时钟 24MHZ (用户可在"config.h"修改频率).

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

/* ===== 循迹调参区：5路传感器，黑线输出为1 =====
   优先级：马达响应及时 > 运行稳定 > 循迹速度和准确性。
   调参建议：
   1. 响应不及时：确认 CTRL_DIV_1MS 为10，不要加入固定转向保持时间。
   2. 电机卡死/只响不转：提高 MIN_RUN_PWM 或 CURVE_INNER_PWM。
   3. 小弯冲出去：降低 BASE_CURVE 或 BASE_SHARP。
   4. 直角转不过：提高 BASE_SHARP，不要增加固定保持时间。
*/
#define LINE_ON             1       /* 黑线有效电平：黑线=1，白底=0 */
#define CTRL_DIV_1MS        5       /* Timer2为100us中断，5次执行一次循迹，约0.5ms */

#define BASE_FAST           75      /* 00100直行速度。太低会卡，太高会抖 */
//#define BASE_CURVE          75      /* 小弯外侧轮速度。小弯冲出就降低 */
#define BASE_SHARP          80      /* 大弯/直角外侧轮速度。转不过再加到90~92 */
#define BASE_NODE           62      /* 11111/路口调试直行速度，调试阶段不停车 */
#define MAX_PWM             100      /* PWM最大限幅，保护电机并减少高速失控 */
#define MIN_RUN_PWM         55      /* 非停车时左右轮最低速度，必须高于电机起转阈值 */
//#define CURVE_INNER_PWM     70      /* 小弯内侧轮速度，不能太低，否则电机可能卡死 */
#define SHARP_REVERSE_PWM   70      /* 大弯内侧轮反转速度。太猛就降低，转不过就提高 */
#define STRAIGHT_SOFT_SPEED 60      /* 2/4号内侧传感器触发时的基础慢速 */
#define STRAIGHT_SOFT_DELTA 15      /* 直线附近轻微偏线修正量，越大修正越明显 */
#define TURN_RECOVER_SPEED  62      /* 大弯后恢复阶段基础速度 */
#define TURN_RECOVER_DELTA  5       /* 大弯后恢复阶段轻微修正量 */
#define CROSS_HOLD_TICKS   100     /* 十字路口11111触发后强制直行时间，约100ms */
#define SHARP_HOLD_TICKS   180     /* 1/5号外侧传感器触发后强制大弯时间，约90ms */
#define PRE_TURN_SPEED     60      /* 大弯前预减速直行速度 */
#define PRE_TURN_TICKS     60      /* 大弯前预减速时间，约30ms */
#define LEFT_TRIM           0       /* 左轮配平。车偏左/左轮偏慢就加左轮，常用范围-10~10 */
#define RIGHT_TRIM          5.5       /* 右轮配平。车偏右/右轮偏慢就加右轮，常用范围-10~10 */
#define LOST_TURN_PWM       95      /* 丢线找线外侧轮速度。找线慢就增大 */

#define LOST_CONFIRM_TICKS  240     /* 全白确认时间：240*0.5ms=120ms，持续全白才找线 */
#define LOST_STRAIGHT_TICKS 75      /* 短暂全白先保持上一输出：60*0.5ms=30ms，避免瞬时抖动 */
#define START_PWM           88      /* 上电在起点黑方块上时，用这个速度冲出起点 */

#define MASK_ZUO2           0x10    /* 最左传感器 */
#define MASK_ZUO1           0x08    /* 左内传感器 */
#define MASK_ZHONG          0x04    /* 中间传感器 */
#define MASK_YOU1           0x02    /* 右内传感器 */
#define MASK_YOU2           0x01    /* 最右传感器 */

signed char last_error = 0;
u16 lost_ticks = 0;
u8 start_square_mode = 1;
u8 ctrl_divider = 0;
u8 turn_recover_mode = 0;
u8 cross_hold_ticks = 0;
u8 sharp_hold_ticks = 0;
signed char sharp_hold_dir = 0;
u8 pre_turn_ticks = 0;
signed char pre_turn_dir = 0;
int last_left_pwm = BASE_FAST;
int last_right_pwm = BASE_FAST;


/*************	本地函数声明	**************/

int LimitPwm(int pwm);
void PWM_Run(int pwm1,int pwm2);
void Motor_LeftForward(void);
void Motor_LeftReverse(void);
void Motor_RightForward(void);
void Motor_RightReverse(void);
void Motor_RunSigned(int left_pwm, int right_pwm);
void Motor_RunSafe(int left_pwm, int right_pwm);
void Motor_HoldLast(void);


/*************  外部函数和变量声明 *****************/



/************************ IO口配置 ****************************/
void	GPIO_config(void)
{

	
}

/************************ 定时器配置 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		//中断频率, 1000次/秒
	TIM_InitStructure.TIM_PS        = 0;					//8位预分频器(n+1), 0~255
	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

void Timer1_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16位自动重装
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // 使用1T时钟（最快）
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // 不输出高频脉冲
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 100000UL)); // 10us定时
    TIM_InitStructure.TIM_PS        = 0;                                // 不预分频
    TIM_InitStructure.TIM_Run       = ENABLE;                           // 初始化后立即启动

    Timer_Inilize(Timer1, &TIM_InitStructure);                          // 初始化定时器1
    NVIC_Timer1_Init(ENABLE, Priority_0);                               // 开启中断，设置优先级为0
}

void Timer2_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16位自动重装
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // 使用1T时钟（最快）
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // 不输出高频脉冲
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 10000UL)); // 100us定时
    TIM_InitStructure.TIM_PS        = 0;                                // 不预分频
    TIM_InitStructure.TIM_Run       = ENABLE;                           // 初始化后立即启动

    Timer_Inilize(Timer2, &TIM_InitStructure);                          // 初始化定时器2
    NVIC_Timer2_Init(ENABLE, Priority_0);                               // 开启中断，设置优先级为0
}

/***************  超声波函数 *****************/
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR		//进中断时已经清除标志
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
    _nop_(); _nop_(); _nop_(); _nop_();    // 等价大约10us
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 0;

    // 等待 ECHO 高电平开始
    while (!ECHO);
    
    // 开始计时
    time_us = 0;
    measuring = 1;

    // 等待 ECHO 拉低（结束）
    while (ECHO);

    measuring = 0;
		if(time_us/100<38)									//判断是否小于38毫秒，大于38毫秒的就是超时，直接调到下面返回0
		{
			Distance_mm=(time_us*346)/100;						//计算距离，25°C空气中的音速为346m/s   因为上面的time_end的单位是10微秒，所以要得出单位为毫米的距离结果，还得除以100
		}
    return Distance_mm;
}

/***************  PWM初始化函数 *****************/
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;
	
//	PWMB_Duty.PWM5_Duty = 128;
//	PWMB_Duty.PWM6_Duty = 256;


	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;	//PWM占空比时间, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;					//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM5, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;	//PWM占空比时间, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;					//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM6, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB


	PWMx_InitStructure.PWM_Period   = PWM_Period; //2000							//周期时间,   0~65535
	PWMx_InitStructure.PWM_DeadTime = 0;								//死区发生器设置, 0~255
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;				//主输出使能, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;				//使能计数器, ENABLE,DISABLE
	PWM_Configuration(PWMB, &PWMx_InitStructure);				//初始化PWM通用寄存器,  PWMA,PWMB


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

void Motor_RunSigned(int left_pwm, int right_pwm)
{
	int left_abs;
	int right_abs;

	if(left_pwm >= 0)
	{
		Motor_LeftForward();
		left_abs = left_pwm;
	}
	else
	{
		Motor_LeftReverse();
		left_abs = -left_pwm;
	}

	if(right_pwm >= 0)
	{
		Motor_RightForward();
		right_abs = right_pwm;
	}
	else
	{
		Motor_RightReverse();
		right_abs = -right_pwm;
	}

	left_abs = LimitPwm(left_abs);
	right_abs = LimitPwm(right_abs);

	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
	PWM_Run(left_abs, right_abs);
}

void Motor_RunSafe(int left_pwm, int right_pwm)
{
	Motor_LeftForward();
	Motor_RightForward();

	/* 只有明确传入0,0时才允许真正停车；循迹/转弯时不让任一轮低于起转PWM */
	if(left_pwm == 0 && right_pwm == 0)
	{
		last_left_pwm = 0;
		last_right_pwm = 0;
		PWM_Run(0, 0);
		return;
	}

	left_pwm += LEFT_TRIM;
	right_pwm += RIGHT_TRIM;
	left_pwm = LimitPwm(left_pwm);
	right_pwm = LimitPwm(right_pwm);

	if(left_pwm < MIN_RUN_PWM) left_pwm = MIN_RUN_PWM;
	if(right_pwm < MIN_RUN_PWM) right_pwm = MIN_RUN_PWM;

	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
	PWM_Run(left_pwm, right_pwm);
}

void Motor_HoldLast(void)
{
	/* 短暂丢线时保持上一周期输出，避免立刻乱转 */
	if(last_left_pwm == 0 && last_right_pwm == 0)
	{
		Motor_RunSafe(BASE_FAST, BASE_FAST);
	}
	else
	{
		Motor_RunSigned(last_left_pwm, last_right_pwm);
	}
}
void PWM_Left(int pwm)     //用0~100表示
{
	
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);  //注意PWM_Period此时为100倍数 2000
	
}

void PWM_Right(int pwm)     //用0~100表示
{
	
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);
	
}


void PWM_Run(int pwm1,int pwm2)  //左右PWM
{
	pwm1 = LimitPwm(pwm1);
	pwm2 = LimitPwm(pwm2);
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
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
	/* 读取5路循迹，按 左外/左内/中/右内/右外 打包成bit掩码 */
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

void Track_NormalControl(u8 mask, u8 count)
{
	u8 has_left;
	u8 has_right;

	has_left = mask & (MASK_ZUO2 | MASK_ZUO1);
	has_right = mask & (MASK_YOU1 | MASK_YOU2);

	/* 00000：短暂全白保持上一输出，持续全白后按上次方向找线，不进入长时间掉头锁定 */
	if(count == 0)
	{
		if(lost_ticks < 60000) lost_ticks++;
		if(lost_ticks <= LOST_STRAIGHT_TICKS)
		{
			Motor_HoldLast();
		}
		else if(last_error >= 0)
		{
			Motor_RunSafe(LOST_TURN_PWM, MIN_RUN_PWM);
		}
		else
		{
			Motor_RunSafe(MIN_RUN_PWM, LOST_TURN_PWM);
		}
		return;
	}
	lost_ticks = 0;
	if(cross_hold_ticks > 0)
	{
		cross_hold_ticks--;
		last_error = 0;
		turn_recover_mode = 0;
		sharp_hold_ticks = 0;
		sharp_hold_dir = 0;
		pre_turn_ticks = 0;
		pre_turn_dir = 0;
		Motor_RunSafe(BASE_NODE, BASE_NODE);
		return;
	}


	/* 11111：调试阶段按路口直行处理，不触发永久停车 */
	if(mask == (MASK_ZUO2 | MASK_ZUO1 | MASK_ZHONG | MASK_YOU1 | MASK_YOU2))
	{
		last_error = 0;
		turn_recover_mode = 0;
		sharp_hold_ticks = 0;
		sharp_hold_dir = 0;
		pre_turn_ticks = 0;
		pre_turn_dir = 0;
		cross_hold_ticks = CROSS_HOLD_TICKS;
		Motor_RunSafe(BASE_NODE, BASE_NODE);
		return;
	}
	if(pre_turn_ticks > 0)
	{
		if(mask == MASK_ZHONG)
		{
			pre_turn_ticks = 0;
			pre_turn_dir = 0;
			sharp_hold_ticks = 0;
			sharp_hold_dir = 0;
			turn_recover_mode = 0;
			last_error = 0;
			Motor_RunSafe(BASE_FAST, BASE_FAST);
			return;
		}

		pre_turn_ticks--;
		turn_recover_mode = 0;
		sharp_hold_ticks = 0;
		sharp_hold_dir = 0;
		if(pre_turn_ticks > 0)
		{
			Motor_RunSafe(PRE_TURN_SPEED, PRE_TURN_SPEED);
			return;
		}

		if(pre_turn_dir > 0)
		{
			last_error = 4;
			sharp_hold_ticks = SHARP_HOLD_TICKS;
			sharp_hold_dir = 1;
			pre_turn_dir = 0;
			Motor_RunSigned(BASE_SHARP, -SHARP_REVERSE_PWM);
			turn_recover_mode = 1;
			return;
		}
		else if(pre_turn_dir < 0)
		{
			last_error = -4;
			sharp_hold_ticks = SHARP_HOLD_TICKS;
			sharp_hold_dir = -1;
			pre_turn_dir = 0;
			Motor_RunSigned(-SHARP_REVERSE_PWM, BASE_SHARP);
			turn_recover_mode = 1;
			return;
		}
		else
		{
			pre_turn_dir = 0;
		}
	}

	if(sharp_hold_ticks > 0)
	{
		if(mask == MASK_ZHONG)
		{
			sharp_hold_ticks = 0;
			sharp_hold_dir = 0;
			turn_recover_mode = 0;
			last_error = 0;
			Motor_RunSafe(BASE_FAST, BASE_FAST);
			return;
		}

		sharp_hold_ticks--;
		turn_recover_mode = 0;
		if(sharp_hold_dir > 0)
		{
			last_error = 4;
			Motor_RunSigned(BASE_SHARP, -SHARP_REVERSE_PWM);
		}
		else if(sharp_hold_dir < 0)
		{
			last_error = -4;
			Motor_RunSigned(-SHARP_REVERSE_PWM, BASE_SHARP);
		}
		else
		{
			sharp_hold_ticks = 0;
		}
		if(sharp_hold_ticks == 0)
		{
			turn_recover_mode = 1;
			sharp_hold_dir = 0;
		}
		return;
	}
	if(turn_recover_mode)
	{
		if(mask == MASK_ZHONG)
		{
			turn_recover_mode = 0;
			last_error = 0;
			Motor_RunSafe(BASE_FAST, BASE_FAST);
			return;
		}
		else if(mask & (MASK_ZUO2 | MASK_YOU2))
		{
			turn_recover_mode = 0;
		}
		else if(mask == MASK_YOU1 || mask == (MASK_ZHONG | MASK_YOU1))
		{
			last_error = 2;
			Motor_RunSafe(TURN_RECOVER_SPEED, TURN_RECOVER_SPEED - TURN_RECOVER_DELTA);
			return;
		}
		else if(mask == MASK_ZUO1 || mask == (MASK_ZUO1 | MASK_ZHONG))
		{
			last_error = -2;
			Motor_RunSafe(TURN_RECOVER_SPEED - TURN_RECOVER_DELTA, TURN_RECOVER_SPEED);
			return;
		}
		else
		{
			turn_recover_mode = 0;
		}
	}


	/* 00100：标准直走，下一周期立即响应 */
	if(mask == MASK_ZHONG)
	{
		last_error = 0;
		turn_recover_mode = 0;
		sharp_hold_ticks = 0;
		sharp_hold_dir = 0;
		pre_turn_ticks = 0;
		pre_turn_dir = 0;
		Motor_RunSafe(BASE_FAST, BASE_FAST);
		return;
	}

	/* 右偏：00010/00110 为小弯，00001/00011/00101/00111 为大弯 */
	if(mask == MASK_YOU1 || mask == (MASK_ZHONG | MASK_YOU1))
	{
		last_error = 2;
		Motor_RunSafe(STRAIGHT_SOFT_SPEED, STRAIGHT_SOFT_SPEED - STRAIGHT_SOFT_DELTA);
		return;
	}

	if(mask == MASK_YOU2 || mask == (MASK_YOU1 | MASK_YOU2) ||
	   mask == (MASK_ZHONG | MASK_YOU2) || mask == (MASK_ZHONG | MASK_YOU1 | MASK_YOU2))
	{
		last_error = 4;
		pre_turn_ticks = PRE_TURN_TICKS;
		pre_turn_dir = 1;
		turn_recover_mode = 0;
		Motor_RunSafe(PRE_TURN_SPEED, PRE_TURN_SPEED);
		return;
	}

	/* 左偏：01000/01100 为小弯，10000/11000/10100/11100 为大弯 */
	if(mask == MASK_ZUO1 || mask == (MASK_ZUO1 | MASK_ZHONG))
	{
		last_error = -2;
		Motor_RunSafe(STRAIGHT_SOFT_SPEED - STRAIGHT_SOFT_DELTA, STRAIGHT_SOFT_SPEED);
		return;
	}

	if(mask == MASK_ZUO2 || mask == (MASK_ZUO2 | MASK_ZUO1) ||
	   mask == (MASK_ZUO2 | MASK_ZHONG) || mask == (MASK_ZUO2 | MASK_ZUO1 | MASK_ZHONG))
	{
		last_error = -4;
		pre_turn_ticks = PRE_TURN_TICKS;
		pre_turn_dir = -1;
		turn_recover_mode = 0;
		Motor_RunSafe(PRE_TURN_SPEED, PRE_TURN_SPEED);
		return;
	}

	/* 其他混合状态：左右都有线且中间在线，按路口直行；否则按偏向侧修正 */
	if((mask & MASK_ZHONG) && has_left && has_right)
	{
		last_error = 0;
		turn_recover_mode = 0;
		Motor_RunSafe(BASE_NODE, BASE_NODE);
	}
	else if(has_right && !has_left)
	{
		last_error = 3;
		pre_turn_ticks = PRE_TURN_TICKS;
		pre_turn_dir = 1;
		turn_recover_mode = 0;
		Motor_RunSafe(PRE_TURN_SPEED, PRE_TURN_SPEED);
	}
	else if(has_left && !has_right)
	{
		last_error = -3;
		pre_turn_ticks = PRE_TURN_TICKS;
		pre_turn_dir = -1;
		turn_recover_mode = 0;
		Motor_RunSafe(PRE_TURN_SPEED, PRE_TURN_SPEED);
	}
	else
	{
		Motor_RunSafe(BASE_FAST, BASE_FAST);
	}
}

void Track_Control(void)
{
	u8 mask;
	u8 count;

	mask = ReadTrackSensors();
	count = CountBits(mask);

	/* 调试阶段禁用终点永久停车；上电若压在起点黑方块上，先直行冲出 */
	if(start_square_mode)
	{
		if(count == 5)
		{
			Motor_RunSafe(START_PWM, START_PWM);
			return;
		}
		start_square_mode = 0;
	}

	Track_NormalControl(mask, count);
}

void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR		//进中断时已经清除标志
{
	/* Timer2每100us进一次，这里分频到约0.5ms执行控制，优先保证响应及时 */
	ctrl_divider++;
	if(ctrl_divider >= CTRL_DIV_1MS)
	{
		ctrl_divider = 0;
		Track_Control();
	}
}
/******************** 主函数**************************/
void main(void)
{
//	unsigned char i;
	WTST = 0;		//设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
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
	
	EA = 1;
	
	AIN1=0;
	AIN2=1;
	BIN1=1;
	BIN2=0;

	TRIG = 0;
	ECHO = 0;
	time_us = 0;
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




