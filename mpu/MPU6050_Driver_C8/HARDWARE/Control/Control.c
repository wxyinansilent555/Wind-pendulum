#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "mpu6050.h"
#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "oled.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "math.h"
#include "key.h"
#include "Control.h"
#include "PWM.h"


#define PI 3.14159265359
#define Period 1150

//声明全局变量
extern float Position_KP,Position_KI,Position_KD;  //位置控制PID参数8/1600
extern float Position_Kp,Position_Ki,Position_Kd;  //位置控制PID参数8/1600
extern int Motor_A,Motor_B,Motor_C,Motor_D,Motor_X,Motor_Y; //电机PWM变量
extern float  Target_X,Target_Y;     //电机目标值                    
extern float pitch,roll,yaw; 		//欧拉角
extern short aacx,aacy,aacz;		//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据 
extern u8 delay_50,delay_flag; 
extern float Data_Amplitude,Data_Period,Data_Phase,Data_Gama;
extern float Basic_Amplitude,Amplitude_x,Amplitude_y,Phase,Alpha;    //振幅X，振幅Y，时间变量
extern float Radius,Height,Measure_X,Measure_Y;          //半径，转点高度
extern float ZHONGZHI_A,ZHONGZHI_B;    //MPU6050传感器平衡位置时对应的初始值
extern uint8_t mode;
extern u32 TimeCnt;
extern int pwm;
extern float Xishu;    //系统设定振幅与圆周运动时半径R相差√2倍
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{			
		TimeCnt += 5;              //运行时间定时器
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)		//获取数据
		{   
			Get_RC();
				
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据	
			OLED_ShowSignedNum(1,1,pitch,4);				
			OLED_ShowSignedNum(1,6,roll,4);
			OLED_ShowSignedNum(1,11,yaw,4);
		
			Alpha=(float)TimeCnt/Period*2*PI;      //float不可省略，单摆周期处理成三角函数2π周期
			Target_X=Amplitude_x*sin(Alpha);       //X方向目标值函数
			Target_Y=Amplitude_y*sin(Alpha+Phase); //Y方向目标值函数
			
			Measure_X=(float)tan((roll-ZHONGZHI_B)/180*2*PI)*Height;
			Measure_Y=(float)tan((pitch-ZHONGZHI_A)/180*2*PI)*Height;
			Motor_X=Position_PID_X(Measure_X,Target_X);
			Motor_Y=Position_PID_X(Measure_Y,Target_Y);                            
				
			OLED_ShowNum(3,1,mode,1);
			Xianfu_Pwm_Max(5800);//限幅5800，满占空比5800，避免出现异常	
			}
		    Set_Pwm(Motor_X,0);     //===如无异常，赋值给PWM寄存器
		    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	}
}

/**************************************************************************
函数功能：通过按键进行遥控
入口参数：mode
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	if(mode == 0)
	{
		//摆动模式
		Data_Amplitude=Basic_Amplitude;//初始振幅

	}
	else if(mode == 1)
	{
		//停止模式
		Data_Amplitude=0;  //振幅为0==维持稳定
		Data_Gama=PI/4;    //振幅比例系数(tan(PI/4)==1)
		Phase=0;           //相位差为0
	}
	else if(mode == 2)
	{
		//圆周模式
		Data_Amplitude=500;//初始振幅
		Data_Gama=PI/4;    //振幅比例系数(tan(PI/4)==1)
		Data_Phase=PI/2;   //相位差
	}
	else if(mode == 3)
	{
		//自由模式
		Data_Amplitude=0;  //振幅为0==维持稳定
		Data_Gama=PI/4;    //振幅比例系数(tan(PI/4)==1)
		Phase=0;           //相位差为0
	}
	/********根据模式对应的数据，计算得到所需要的参数：振幅、振幅比例系数、相位差**********/
	Amplitude_x=Data_Amplitude*sin(Data_Gama)*Xishu;//振幅X
	Amplitude_y=Data_Amplitude*cos(Data_Gama)*Xishu;//振幅Y
	Phase=Data_Phase;  //相位差
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID_X (float value,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=value-Target;                                    //计算偏差
	 Integral_bias+=Bias;	                               //求出偏差的积分
	 Pwm=Position_KP*Bias+                                 //PID控制器比例项
	     Position_KI*Integral_bias+                        //PID控制器积分项
	     Position_KD*(Bias-Last_Bias);                     //PID控制器微分项 
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}

int Position_PID_Y (float value,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=value-Target;                                    //计算偏差
	 Integral_bias+=Bias;	                               //求出偏差的积分
	 Pwm=Position_Kp*Bias+                                 //PID控制器比例项
	     Position_Ki*Integral_bias+                        //PID控制器积分项
	     Position_Kd*(Bias-Last_Bias);                     //PID控制器微分项 
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Xianfu_Pwm_Max(int amplitude)
{	
    if(Motor_X<-amplitude) Motor_X=-amplitude;	
	if(Motor_X>amplitude)  Motor_X=amplitude;	
	if(Motor_Y<-amplitude) Motor_Y=-amplitude;	
    if(Motor_Y>amplitude)  Motor_Y=amplitude;		
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int Motor_X,int Motor_Y)
{
	if(mode == 1)
	{
		if(Motor_X>0)			PWMA1=0,
			                    PWMC1=Motor_X;
			else 	            PWMA1=-Motor_X,
		                        PWMC1=0;
		
	  	if(Motor_Y>0)			PWMB1=Motor_Y,
			                    PWMD1=0;
			else 	            PWMB1=0,
		                        PWMD1=-Motor_Y;
	}
	else
	{
	  	if(Motor_X>0)			PWMA1=Motor_X,
			                    PWMC1=0;
			else 	            PWMA1=0,
		                        PWMC1=-Motor_X;
		
	  	if(Motor_Y>0)			PWMB1=0,
			                    PWMD1=Motor_Y;
			else 	            PWMB1=-Motor_Y,
		                        PWMD1=0;
	}
}






