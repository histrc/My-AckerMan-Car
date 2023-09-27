#ifndef __PID_H__
#define __PID_H__
#include "main.h"

struct pid_uint
{
	int32_t U_kk;    	    //上一次的输出量
	int32_t ekk;		 	//上一次的输入偏差
	int32_t ekkk;			//前一次的输入偏差
	int32_t Ur;				//限幅输出值,需初始化
	int32_t Kp;				//比例
	int32_t Ki;				//积分
	int32_t Kd;				//微分
	
	uint8_t  En;             //开关
	int16_t Adjust;         //调节量
	int16_t speedSet;       //速度设置
	int16_t speedNow;       //当前速度
};
/****************************外接函数***************************/

extern struct pid_uint pid_Task_Letf;
extern struct pid_uint pid_Task_Right;

void PID_Init(void);
void reset_Uk(struct pid_uint *p);
int32_t  PID_common(int set,int jiance,struct pid_uint *p);
void Pid_Ctrl(int *leftMotor,int  *rightMotor);

#endif //__PID_H__
