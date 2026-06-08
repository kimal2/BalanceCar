#include "pid.h"
#include <math.h>

void PID_Clear(PID_t *p)
{
	p->error = 0;
	p->integral = 0;
	p->derivative = 0;
	p->output = 0;
	p->last_error = 0;
	p->actual = 0;
	p->actual_last=0;
	p->target = 0;
}


void PID_Calc(PID_t *p)
{
	p->last_error = p->error;
	p->error = p->target - p->actual;

	//计算积分项
	if(p->Ki!=0)
	{
		if(p->int_threshold==0)
		{
			p->integral += p->error*p->dt;
		}
		else if (fabs(p->error) < p->int_threshold)
		{
			p->integral += p->error*p->dt;
		}
		else
		{
			p->integral = 0;
		}
	}
	else
	{
		p->integral = 0;
	}

	if(fabs(p->integral)>p->integ_limit)//积分限制幅度
	{
		p->integral = p->integ_limit*(p->integral/fabs(p->integral));
	}

	//计算微分项
	if(p->Kd!=0)
	{
		p->derivative = (p->error - p->last_error)/p->dt;
	}

	//p->output = p->Kp*p->error + p->Ki*p->integral + p->Kd*p->derivative;
	p->output = p->Kp*p->error + p->Ki*p->integral - p->Kd*(p->actual-p->actual_last);
	p->actual_last=p->actual;

	//死区处理
	if(p->output > -p->Out_Offset && p->output < p->Out_Offset)
	{
		p->output = p->Out_Offset*(p->output/fabs(p->output));
	}

	//限幅后输出
	if(p->output>p->Out_Max) p->output = p->Out_Max;
	if(p->output<p->Out_Min) p->output = p->Out_Min;
}
