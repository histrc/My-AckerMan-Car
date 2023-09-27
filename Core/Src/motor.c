#include "motor.h"
#include "stm32f10x.h"                  // Device header

// * ----------------------���ҵ��--------------------------
//�����ֵ��PWM����
int motorLeft     = 0;
int motorRight    = 0;         

//����1000֮����ٶ�ʵʱֵ
int leftSpeedNow  = 0; 
int rightSpeedNow = 0; 

//����1000֮����ٶ��趨ֵ
int leftSpeedSet  = 0; 
int rightSpeedSet = 0; 

// * ----------------------ǰ�ֶ��--------------------------
//���PWM����
int motorFrontSteer = 4500; // (60000 / 20ms) * 1.5ms = 4500 90��

//������ƽǶȵ��趨ֵ
int frontAngleSet = 0;

// ����ṹ�޷�
#define MAX_FRONT_ANGLE_SET (50)

/**************************************************************************
�������ܣ�����Ƕȿ��ƴ���MG996R���Ƕ����Ա仯
��ڲ�����ros���趨ת��Ƕȣ���Ҫ�������pwm���˴��ɸ��ݲ�ͬ�Ķ�����и���
�ֱ��ʣ�1�� --> ((2.5ms - 0.5ms) / 180��) * (60000 / 20ms) = 33.3
0.5ms ------------------ 0��
1.5ms ------------------ 90��
2.5ms ------------------ 180��
��ֵ 4700 ----90��
��С 1700 -----0 70��
max 7700 -----180 110��
����  ֵ����
Angle_Servo=17.54*pow(AngleR,3)-0.1624*pow(AngleR,2)-5.166*AngleR+1.626;
**************************************************************************/
void Steer_Ctrl(int frontAngleSet,int *motorFrontSteer)
{
	// ����ṹ�޷�
	if(frontAngleSet > MAX_FRONT_ANGLE_SET)
	{
		frontAngleSet = MAX_FRONT_ANGLE_SET;
	}
	if(frontAngleSet < -MAX_FRONT_ANGLE_SET)
	{
		frontAngleSet = -MAX_FRONT_ANGLE_SET;
	}
	// ��������Ƕ�
	if(frontAngleSet == 0) //Ĭ�ϳ�ʼ�Ƕ� 90��
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
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motorLeft,int motorRight,int motorFrontSteer)
{
	// * ------------���ҵ������--------------
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

	// * ------------������Ʋ���--------------
	PWMC=motorFrontSteer;
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
uint8_t Turn_Off(int voltage)
{
	uint8_t temp =0;
	if(voltage<1110)//��ص�ѹ����11.1V�رյ��
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
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
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

