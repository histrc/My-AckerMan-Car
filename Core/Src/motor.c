#include "motor.h"
#include "stm32f10x.h"                  // Device header

// * ----------------------左右电机--------------------------
//左右轮电机PWM变量
int motorLeft     = 0;
int motorRight    = 0;         

//乘以1000之后的速度实时值
int leftSpeedNow  = 0; 
int rightSpeedNow = 0; 

//乘以1000之后的速度设定值
int leftSpeedSet  = 0; 
int rightSpeedSet = 0; 

// * ----------------------前轮舵机--------------------------
//舵机PWM变量
int motorFrontSteer = 4500; // (60000 / 20ms) * 1.5ms = 4500 90度

//舵机控制角度的设定值
int frontAngleSet = 0;

// 物理结构限幅
#define MAX_FRONT_ANGLE_SET (50)

/**************************************************************************
函数功能：舵机角度控制处理（MG996R）角度线性变化
入口参数：ros端设定转向角度，需要给舵机的pwm，此处可根据不同的舵机自行更改
分辨率：1度 --> ((2.5ms - 0.5ms) / 180度) * (60000 / 20ms) = 33.3
0.5ms ------------------ 0度
1.5ms ------------------ 90度
2.5ms ------------------ 180度
中值 4700 ----90度
最小 1700 -----0 70度
max 7700 -----180 110度
返回  值：无
Angle_Servo=17.54*pow(AngleR,3)-0.1624*pow(AngleR,2)-5.166*AngleR+1.626;
**************************************************************************/
void Steer_Ctrl(int frontAngleSet,int *motorFrontSteer)
{
	// 物理结构限幅
	if(frontAngleSet > MAX_FRONT_ANGLE_SET)
	{
		frontAngleSet = MAX_FRONT_ANGLE_SET;
	}
	if(frontAngleSet < -MAX_FRONT_ANGLE_SET)
	{
		frontAngleSet = -MAX_FRONT_ANGLE_SET;
	}
	// 正常计算角度
	if(frontAngleSet == 0) //默认初始角度 90度
	{
		*motorFrontSteer = 4500; // (60000 / 20ms) * 1.5ms = 4500 
	}
	else if(frontAngleSet > 0) // left
	{
		*motorFrontSteer = 4500 - (int)(myabs(frontAngleSet) * 33.3 + 0.5);
	}
	else                       //right
	{
		*motorFrontSteer = 4500 + (int)(myabs(frontAngleSet) * 33.3 + 0.5);
	}
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM、舵机PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motorLeft,int motorRight,int motorFrontSteer)
{
	// * ------------左右电机部分--------------
	if(motorRight > 0){
		PWMA1=myabs(motorRight);
		PWMA2=myabs(0);
	}
	else
	{
		PWMA1=myabs(0);
		PWMA2=myabs(motorRight);
	}

	if(motorLeft > 0)
	{
		PWMB1=myabs(motorLeft);	
		PWMB2=myabs(0);
	}
	else
	{
		PWMB1=myabs(0);	
		PWMB2=myabs(motorLeft);	
	}

	// * ------------舵机控制部分--------------
	PWMC=motorFrontSteer;
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
uint8_t Turn_Off(int voltage)
{
	uint8_t temp =0;
	if(voltage<1110)//电池电压低于11.1V关闭电机
	{	                                             
		temp=1;
		motorLeft=0;
		motorRight=0;
		frontAngleSet=0;
		Steer_Ctrl(frontAngleSet,&motorFrontSteer);
		Set_Pwm(motorLeft,motorRight,motorFrontSteer);
	}
	else
	{
		temp=0;
	}
	return temp;			
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  
	  temp=-a;  
	else 
	  temp=a;
	return temp;
}

