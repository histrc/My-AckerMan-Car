#ifndef __MOTOR_H
#define __MOTOR_H

	 
//#include "pid.h"
//#include "control.h"

//���PWM����
extern int    motorLeft,motorRight;                      
//�ٶ�
extern int    leftSpeedNow; 
extern int    rightSpeedNow; 
//����1000֮����ٶ��趨ֵ
extern int    leftSpeedSet; 
extern int    rightSpeedSet; 

//���PWM����
extern int motorFrontSteer;    
//������ƽǶȵ��趨ֵ
extern int frontAngleSet;

// * -------------�����ֵ��---------------
#define PWMB1   TIM2->CCR4    //PA3
#define PWMB2   TIM2->CCR2    //PA1
#define PWMA1   TIM2->CCR3    //PA2
#define PWMA2   TIM2->CCR1    //PA0 

// * -------------���---------------------
#define PWMC    TIM1->CCR2    //PB0

void Steer_Ctrl(int frontAngleSet,int *motorFrontSteer);
void Set_Pwm(int motorLeft,int motorRight,int motorFrontSteer);

//u8 Turn_Off(int voltage);
int myabs(int a);


#endif

