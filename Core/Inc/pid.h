#ifndef __PID_H__
#define __PID_H__
#include "main.h"

struct pid_uint
{
	int32_t U_kk;    	    //��һ�ε������
	int32_t ekk;		 	//��һ�ε�����ƫ��
	int32_t ekkk;			//ǰһ�ε�����ƫ��
	int32_t Ur;				//�޷����ֵ,���ʼ��
	int32_t Kp;				//����
	int32_t Ki;				//����
	int32_t Kd;				//΢��
	
	uint8_t  En;             //����
	int16_t Adjust;         //������
	int16_t speedSet;       //�ٶ�����
	int16_t speedNow;       //��ǰ�ٶ�
};
/****************************��Ӻ���***************************/

extern struct pid_uint pid_Task_Letf;
extern struct pid_uint pid_Task_Right;

void PID_Init(void);
void reset_Uk(struct pid_uint *p);
int32_t  PID_common(int set,int jiance,struct pid_uint *p);
void Pid_Ctrl(int *leftMotor,int  *rightMotor);

#endif //__PID_H__
