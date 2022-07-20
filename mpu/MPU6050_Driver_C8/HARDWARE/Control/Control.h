#ifndef __CONTROL_H
#define __CONTROL_H

int Position_PID_X (float value,float Target);
int Position_PID_Y (float value,float Target);
u32 myabs(long int a);
void Xianfu_Pwm_Max(int amplitude);
void Set_Pwm(int Motor_X,int Motor_Y);
void Get_RC(void);

#endif
