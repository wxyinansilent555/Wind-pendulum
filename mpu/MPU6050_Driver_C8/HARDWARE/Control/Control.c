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

//����ȫ�ֱ���
extern float Position_KP,Position_KI,Position_KD;  //λ�ÿ���PID����8/1600
extern float Position_Kp,Position_Ki,Position_Kd;  //λ�ÿ���PID����8/1600
extern int Motor_A,Motor_B,Motor_C,Motor_D,Motor_X,Motor_Y; //���PWM����
extern float  Target_X,Target_Y;     //���Ŀ��ֵ                    
extern float pitch,roll,yaw; 		//ŷ����
extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
extern short gyrox,gyroy,gyroz;	//������ԭʼ���� 
extern u8 delay_50,delay_flag; 
extern float Data_Amplitude,Data_Period,Data_Phase,Data_Gama;
extern float Basic_Amplitude,Amplitude_x,Amplitude_y,Phase,Alpha;    //���X�����Y��ʱ�����
extern float Radius,Height,Measure_X,Measure_Y;          //�뾶��ת��߶�
extern float ZHONGZHI_A,ZHONGZHI_B;    //MPU6050������ƽ��λ��ʱ��Ӧ�ĳ�ʼֵ
extern uint8_t mode;
extern u32 TimeCnt;
extern int pwm;
extern float Xishu;    //ϵͳ�趨�����Բ���˶�ʱ�뾶R����2��
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{			
		TimeCnt += 5;              //����ʱ�䶨ʱ��
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)		//��ȡ����
		{   
			Get_RC();
				
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������	
			OLED_ShowSignedNum(1,1,pitch,4);				
			OLED_ShowSignedNum(1,6,roll,4);
			OLED_ShowSignedNum(1,11,yaw,4);
		
			Alpha=(float)TimeCnt/Period*2*PI;      //float����ʡ�ԣ��������ڴ�������Ǻ���2������
			Target_X=Amplitude_x*sin(Alpha);       //X����Ŀ��ֵ����
			Target_Y=Amplitude_y*sin(Alpha+Phase); //Y����Ŀ��ֵ����
			
			Measure_X=(float)tan((roll-ZHONGZHI_B)/180*2*PI)*Height;
			Measure_Y=(float)tan((pitch-ZHONGZHI_A)/180*2*PI)*Height;
			Motor_X=Position_PID_X(Measure_X,Target_X);
			Motor_Y=Position_PID_X(Measure_Y,Target_Y);                            
				
			OLED_ShowNum(3,1,mode,1);
			Xianfu_Pwm_Max(5800);//�޷�5800����ռ�ձ�5800����������쳣	
			}
		    Set_Pwm(Motor_X,0);     //===�����쳣����ֵ��PWM�Ĵ���
		    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	}
}

/**************************************************************************
�������ܣ�ͨ����������ң��
��ڲ�����mode
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	if(mode == 0)
	{
		//�ڶ�ģʽ
		Data_Amplitude=Basic_Amplitude;//��ʼ���

	}
	else if(mode == 1)
	{
		//ֹͣģʽ
		Data_Amplitude=0;  //���Ϊ0==ά���ȶ�
		Data_Gama=PI/4;    //�������ϵ��(tan(PI/4)==1)
		Phase=0;           //��λ��Ϊ0
	}
	else if(mode == 2)
	{
		//Բ��ģʽ
		Data_Amplitude=500;//��ʼ���
		Data_Gama=PI/4;    //�������ϵ��(tan(PI/4)==1)
		Data_Phase=PI/2;   //��λ��
	}
	else if(mode == 3)
	{
		//����ģʽ
		Data_Amplitude=0;  //���Ϊ0==ά���ȶ�
		Data_Gama=PI/4;    //�������ϵ��(tan(PI/4)==1)
		Phase=0;           //��λ��Ϊ0
	}
	/********����ģʽ��Ӧ�����ݣ�����õ�����Ҫ�Ĳ�����������������ϵ������λ��**********/
	Amplitude_x=Data_Amplitude*sin(Data_Gama)*Xishu;//���X
	Amplitude_y=Data_Amplitude*cos(Data_Gama)*Xishu;//���Y
	Phase=Data_Phase;  //��λ��
}

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID_X (float value,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=value-Target;                                    //����ƫ��
	 Integral_bias+=Bias;	                               //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+                                 //PID������������
	     Position_KI*Integral_bias+                        //PID������������
	     Position_KD*(Bias-Last_Bias);                     //PID������΢���� 
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}

int Position_PID_Y (float value,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=value-Target;                                    //����ƫ��
	 Integral_bias+=Bias;	                               //���ƫ��Ļ���
	 Pwm=Position_Kp*Bias+                                 //PID������������
	     Position_Ki*Integral_bias+                        //PID������������
	     Position_Kd*(Bias-Last_Bias);                     //PID������΢���� 
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm_Max(int amplitude)
{	
    if(Motor_X<-amplitude) Motor_X=-amplitude;	
	if(Motor_X>amplitude)  Motor_X=amplitude;	
	if(Motor_Y<-amplitude) Motor_Y=-amplitude;	
    if(Motor_Y>amplitude)  Motor_Y=amplitude;		
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
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






