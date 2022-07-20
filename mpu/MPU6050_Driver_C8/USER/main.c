#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"   
#include "key.h"   
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "oled.h"
#include "stm32f10x.h"
#include "PWM.h"
#include "Timer.h"
#include "Control.h"

#define Pi 3.14

float Position_KP = 200,Position_KI = 10,Position_KD =100;  //位置控制PID参数200/100
float Position_Kp = 500,Position_Ki = 0,Position_Kd =80;  //位置控制PID参数8/1600
int Motor_A,Motor_B,Motor_C,Motor_D,Motor_X,Motor_Y; //电机PWM变量
float  Target_X,Target_Y;     //电机目标值                    
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据 
u8 delay_50,delay_flag; //延时相关变量
float Data_Amplitude,Data_Period,Data_Phase,Data_Gama;
float Basic_Amplitude = 300 ,Amplitude_x = 0,Amplitude_y = 0,Phase = Pi/2 ,Alpha;    //振幅X，振幅Y，时间变量
float Radius,Height = 655,Measure_X,Measure_Y;          //半径，转点高度
float ZHONGZHI_A,ZHONGZHI_B;							//MPU6050传感器平衡位置时对应的初始值
float Xishu=1.4142;    //系统设定振幅与圆周运动时半径R相差√2倍
int pwm;
u32 TimeCnt;

uint8_t mode = 0;

int main(void)
{		   
 	Stm32_Clock_Init(9);		//系统时钟设置
	delay_init(72);	   	 		//延时初始化 
	PWM_Init();                 //pwm初始化
	MPU_Init();					//初始化MPU6050
	Timer_Init();               //定时中断初始化

	OLED_Init();								//OLED初始化
	delay_ms(1000);	
	mpu_dmp_init();
	//while();			           //dmp初始化 	MPU_IIC_Init();初始化IIC总线
    OLED_Clear();
	
 	while(1)
    {
	    
    }	
}




