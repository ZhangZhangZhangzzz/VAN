/*
 * pid.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Admin
 */


#include "pid.h"

pid_struct_t gimbal_yaw_speed_pid;
pid_struct_t gimbal_yaw_angle_pid;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)//PID��ʼ������
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

void LIMIT_MIN_MAX(float out, float fu, float zheng)
{
	if(out > zheng || out < fu)
	{
		out = 0;
	}
}

float pid_calc(pid_struct_t *pid, float ref, float fdb)//PID���㺯��
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;

  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);


  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

  pid->output = pid->p_out + pid->i_out + pid->d_out;

  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}

void gimbal_PID_init()//�ǶȻ����ٶȻ���PID��ʼ��,ֻ�ǳ�����������ݣ����廹��Ҫ����
{
	pid_init(&gimbal_yaw_speed_pid, 35, 18, 0.5, 0, 0);//P=30,I=0,D=0
	pid_init(&gimbal_yaw_angle_pid, 100, 0, 0, 0, 0);//P=400,I=0,D=0
}


double msp(double x, double in_min, double in_max, double out_min, double out_max)//ӳ�亯��������������ֵ��0~8191��ת��Ϊ�����ƵĽǶ�ֵ��-pi~pi��
{
	return (x-in_min)/(in_max-in_min)*360.0f;
}


