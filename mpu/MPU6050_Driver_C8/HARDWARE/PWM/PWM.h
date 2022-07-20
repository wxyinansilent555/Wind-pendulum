#ifndef __MOTOR_H
#define __MOTOR_H

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);

#define PWMA1   TIM3->CCR1 
#define PWMB1   TIM3->CCR2 
#define PWMC1   TIM3->CCR3
#define PWMD1   TIM3->CCR4

#endif
