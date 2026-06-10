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
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ����STC�����ϼ�����            */
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




/*************	����˵��	**************

�߼�PWM��ʱ�� PWM5,PWM6,PWM7,PWM8 ÿ��ͨ�����ɶ���ʵ��PWM���.

4��ͨ��PWM������Ҫ���ö�Ӧ����ڣ���ͨ��ʾ�����۲�������ź�.

PWM���ں�ռ�ձȿ����Զ������ã���߿ɴ�65535.

����ʱ, ѡ��ʱ�� 24MHZ (�û�����"config.h"�޸�Ƶ��).

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

/* ===== ѭ����������5·���������������Ϊ1 =====
   ���ȼ���������Ӧ��ʱ > �����ȶ� > ѭ���ٶȺ�׼ȷ�ԡ�
   ���ν��飺
   1. ��Ӧ����ʱ��ȷ�� CTRL_DIV_1MS Ϊ10����Ҫ����̶�ת�򱣳�ʱ�䡣
   2. �������/ֻ�첻ת����� MIN_RUN_PWM �� MICRO_INNER_PWM��
   3. С����ȥ������ MICRO_OUTER_PWM �� SHARP_FORWARD_PWM��
   4. ֱ��ת��������� SHARP_FORWARD_PWM����Ҫ���ӹ̶�����ʱ�䡣
*/
#define LINE_ON             1       /* ������Ч��ƽ������=1���׵�=0 */
#define CTRL_DIV_1MS        10      /* Timer2Ϊ100us�жϣ�10��ִ��һ��ѭ����Լ1ms */

#define BASE_FAST           89      /* 00100ֱ���ٶ� */
#define BASE_NODE           78      /* ʮ���ж�����ѹ������ʱ��ֱ���ٶ� */
#define MAX_PWM             92      /* PWM����޷� */
#define MIN_RUN_PWM         60      /* ��ͣ����תʱ���PWM */
#define LEFT_TRIM           0       /* ������ƽ�����÷�Χ-10~10 */
#define RIGHT_TRIM          6       /* ������ƽ�����÷�Χ-10~10 */
#define LOST_TURN_PWM       80      /* ����ȫ�׺��ϴη��������ٶ� */

#define HOLD_CURRENT_MS     110     /* ά�ֵ�ǰ������ */
#define CROSS_HOLD_MS       90      /* ʮ��·��ֱ�б���ʱ�� */
#define UTURN_STOP_MS       500     /* �ҵ�ͷǰͣ��ʱ�� */

#define MICRO_OUTER_PWM     85      /* ΢������������ٶ� */
#define MICRO_INNER_PWM     80      /* ΢�������ڲ����ٶ� */
#define MICRO_TO_SHARP_TICKS 20     /* С�޺�����������ռ��ʱ�� */
#define CENTER_SHARP_WINDOW_TICKS 15 /* ���ߺ���ഫ������ϴ��������ʱ�䴰�� */
#define MICRO_DIR_NONE       0
#define MICRO_DIR_LEFT      -1
#define MICRO_DIR_RIGHT      1

#define SHARP_FORWARD_PWM   75      /* ����ת����ת���ٶ� */
#define SHARP_REVERSE_PWM   90      /* ����ת�䷴ת���ٶ� */
#define SHARP_MIN_PIVOT_TICKS 50    /* �����������ǿתʱ�� */

#define UTURN_PWM           80      /* �ҵ�ͷ�����ֵ��ٷ����ٶ� */

#define LOST_CONFIRM_TICKS  60     /* Ԥ����120*1ms=120ms */
#define LOST_STRAIGHT_TICKS 30      /* ����ȫ�ױ�����һ���ʱ�� */

#define MASK_ZUO2           0x10    /* 1�Ŵ����������� */
#define MASK_ZUO1           0x08    /* 2�Ŵ����������� */
#define MASK_ZHONG          0x04    /* 3�Ŵ��������м� */
#define MASK_YOU1           0x02    /* 4�Ŵ����������� */
#define MASK_YOU2           0x01    /* 5�Ŵ����������� */
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


/*************	���غ�������	**************/

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
void Track_StartSharpLeft(void);
void Track_StartSharpRight(void);
void Track_ResetStraight(void);
void Track_NormalControl(u8 mask, u8 count);
void Track_Control(void);

/*************  �ⲿ�����ͱ������� *****************/



/************************ IO������ ****************************/
void	GPIO_config(void)
{

	
}

/************************ ��ʱ������ ****************************/
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

/***************  ���������� *****************/
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

/***************  PWM��ʼ������ *****************/
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

void PWM_Left(int pwm)     //��0~100��ʾ
{
	
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);  //ע��PWM_Period��ʱΪ100���� 2000
	
}

void PWM_Right(int pwm)     //��0~100��ʾ
{
	
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);
	
}


void PWM_Run(int pwm1,int pwm2)  //����PWM
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

	left_pwm = LimitSignedPwm(left_pwm);
	right_pwm = LimitSignedPwm(right_pwm);

	if(left_pwm != 0 || right_pwm != 0)
	{
		if(left_pwm == 0)
		{
			left_pwm = (right_pwm < 0) ? -MIN_RUN_PWM : MIN_RUN_PWM;
		}
		else if(AbsPwm(left_pwm) < MIN_RUN_PWM)
		{
			left_pwm = (left_pwm < 0) ? -MIN_RUN_PWM : MIN_RUN_PWM;
		}

		if(right_pwm == 0)
		{
			right_pwm = (left_pwm < 0) ? -MIN_RUN_PWM : MIN_RUN_PWM;
		}
		else if(AbsPwm(right_pwm) < MIN_RUN_PWM)
		{
			right_pwm = (right_pwm < 0) ? -MIN_RUN_PWM : MIN_RUN_PWM;
		}
	}

	left_abs = AbsPwm(left_pwm);
	right_abs = AbsPwm(right_pwm);

	if(left_pwm > 0) Motor_LeftForward();
	else if(left_pwm < 0) Motor_LeftReverse();
	else Motor_LeftStop();

	if(right_pwm > 0) Motor_RightForward();
	else if(right_pwm < 0) Motor_RightReverse();
	else Motor_RightStop();

	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
	PWM_Run(left_abs, right_abs);
}

void Motor_RunSafe(int left_pwm, int right_pwm)
{
	/* ֻ��0,0����ͣ������ͨѭ��ʱ�����ֱ��ָ�����תPWM */
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
	/* ��ȡ5·ѭ������ ����/����/��/����/���� �����bit���� */
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
	return ((mask & (MASK_ZUO1 | MASK_ZHONG | MASK_YOU1)) == (MASK_ZUO1 | MASK_ZHONG | MASK_YOU1));
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
		Motor_HoldLast();
	}
	else
	{
		Track_ClearMicroWindow();
		Track_EnterState(TRACK_UTURN_RIGHT);
		Motor_RunSigned(UTURN_PWM, -UTURN_PWM);
	}
	return 1;
}

void Track_EnterState(u8 state)
{
	track_state = state;
	track_state_ticks = 0;
}

void Track_StartSharpLeft(void)
{
	Track_ClearMicroWindow();
	Track_ClearCenterSharpWindow();
	last_error = -4;
	Track_EnterState(TRACK_SHARP_LEFT);
	Motor_RunSigned(-SHARP_REVERSE_PWM, SHARP_FORWARD_PWM);
}

void Track_StartSharpRight(void)
{
	Track_ClearMicroWindow();
	Track_ClearCenterSharpWindow();
	last_error = 4;
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
	/* 00100��������ȼ����������������С�޴��� */
	if(mask == MASK_ZHONG)
	{
		Track_ResetStraight();
		return;
	}

	/* 00000�������ﵽȷ��ʱ�������ԭ���ҵ�ͷ */
	if(Track_HandleLostUturn(count))
	{
		return;
	}

	/* 30ms������+ͬ��������ʱ��ռΪ������� */
	if(Track_ShouldCenterSharpLeft(mask))
	{
		Track_StartSharpLeft();
		return;
	}
	if(Track_ShouldCenterSharpRight(mask))
	{
		Track_StartSharpRight();
		return;
	}

	/* x111x��ʮ��·��ֱ�б��֣����ȼ����ڴ��� */
	if(IsCrossMask(mask))
	{
		Track_ClearMicroWindow();
		if(!cross_latched)
		{
			cross_latched = 1;
			Track_EnterState(TRACK_CROSS_HOLD);
			Motor_RunSafe(BASE_NODE, BASE_NODE);
			return;
		}
		last_error = 0;
		Motor_RunSafe(BASE_NODE, BASE_NODE);
		return;
	}

	/* 1xx00/00xx1���������䣬�����ڵ�ͷ��ʮ��ֱ�� */
	if(Track_IsImmediateSharpLeft(mask))
	{
		Track_StartSharpLeft();
		return;
	}
	if(Track_IsImmediateSharpRight(mask))
	{
		Track_StartSharpRight();
		return;
	}

	/* С�޺�20ms�ڣ�ͬ������ഥ��ʱ��ռΪ������� */
	if(Track_ShouldMicroSharpLeft(mask))
	{
		Track_StartSharpLeft();
		return;
	}
	if(Track_ShouldMicroSharpRight(mask))
	{
		Track_StartSharpRight();
		return;
	}

	/* ����������11000/10000���ޣ�00011/00001���ޣ�10001���ϴη��� */
	if(mask == (MASK_ZUO2 | MASK_YOU2))
	{
		if(last_error < 0) Track_StartSharpLeft();
		else Track_StartSharpRight();
		return;
	}
	if(mask == MASK_ZUO2 || mask == (MASK_ZUO2 | MASK_ZUO1))
	{
		Track_StartSharpLeft();
		return;
	}
	if(mask == MASK_YOU2 || mask == (MASK_YOU1 | MASK_YOU2))
	{
		Track_StartSharpRight();
		return;
	}

	/* ΢��������ֻ�����ڲ�/���߸�������΢ƫ�� */
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
		Motor_RunSafe(BASE_NODE, BASE_NODE);
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

	if(mask == MASK_ZHONG)
	{
		Track_ResetStraight();
		return;
	}
	if(Track_HandleLostUturn(count))
	{
		return;
	}
	if(Track_ShouldCenterSharpLeft(mask) && track_state != TRACK_SHARP_LEFT)
	{
		Track_StartSharpLeft();
		return;
	}
	if(Track_ShouldCenterSharpRight(mask) && track_state != TRACK_SHARP_RIGHT)
	{
		Track_StartSharpRight();
		return;
	}
	if(IsCrossMask(mask) && track_state != TRACK_CROSS_HOLD)
	{
		Track_ClearMicroWindow();
		cross_latched = 1;
		Track_EnterState(TRACK_CROSS_HOLD);
		Motor_RunSafe(BASE_NODE, BASE_NODE);
		return;
	}
	if(Track_IsImmediateSharpLeft(mask) && track_state != TRACK_SHARP_LEFT)
	{
		Track_StartSharpLeft();
		return;
	}
	if(Track_IsImmediateSharpRight(mask) && track_state != TRACK_SHARP_RIGHT)
	{
		Track_StartSharpRight();
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
			Motor_RunSafe(BASE_NODE, BASE_NODE);
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
			if(track_state_ticks >= SHARP_MIN_PIVOT_TICKS && !Track_IsImmediateSharpLeft(mask) && (mask & (MASK_ZUO1 | MASK_ZHONG)))
			{
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
			if(track_state_ticks >= SHARP_MIN_PIVOT_TICKS && !Track_IsImmediateSharpRight(mask) && (mask & (MASK_YOU1 | MASK_ZHONG)))
			{
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

void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR		//���ж�ʱ�Ѿ������־
{
	/* Timer2ÿ100us��һ�Σ������Ƶ��Լ1msִ�п��ƣ����ȱ�֤��Ӧ��ʱ */
	ctrl_divider++;
	if(ctrl_divider >= CTRL_DIV_1MS)
	{
		ctrl_divider = 0;
		Track_Control();
	}
}
/******************** ������**************************/
void main(void)
{
//	unsigned char i;
	WTST = 0;		//���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���
	EAXSFR();		//��չSFR(XFR)����ʹ�� 
	CKCON = 0;      //��߷���XRAM�ٶ�

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




