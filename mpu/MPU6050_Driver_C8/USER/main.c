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

float Position_KP = 200,Position_KI = 10,Position_KD =100;  //λ�ÿ���PID����200/100
float Position_Kp = 500,Position_Ki = 0,Position_Kd =80;  //λ�ÿ���PID����8/1600
int Motor_A,Motor_B,Motor_C,Motor_D,Motor_X,Motor_Y; //���PWM����
float  Target_X,Target_Y;     //���Ŀ��ֵ                    
float pitch,roll,yaw; 		//ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ���� 
u8 delay_50,delay_flag; //��ʱ��ر���
float Data_Amplitude,Data_Period,Data_Phase,Data_Gama;
float Basic_Amplitude = 300 ,Amplitude_x = 0,Amplitude_y = 0,Phase = Pi/2 ,Alpha;    //���X�����Y��ʱ�����
float Radius,Height = 655,Measure_X,Measure_Y;          //�뾶��ת��߶�
float ZHONGZHI_A,ZHONGZHI_B;							//MPU6050������ƽ��λ��ʱ��Ӧ�ĳ�ʼֵ
float Xishu=1.4142;    //ϵͳ�趨�����Բ���˶�ʱ�뾶R����2��
int pwm;
u32 TimeCnt;

uint8_t mode = 0;

int main(void)
{		   
 	Stm32_Clock_Init(9);		//ϵͳʱ������
	delay_init(72);	   	 		//��ʱ��ʼ�� 
	PWM_Init();                 //pwm��ʼ��
	MPU_Init();					//��ʼ��MPU6050
	Timer_Init();               //��ʱ�жϳ�ʼ��

	OLED_Init();								//OLED��ʼ��
	delay_ms(1000);	
	mpu_dmp_init();
	//while();			           //dmp��ʼ�� 	MPU_IIC_Init();��ʼ��IIC����
    OLED_Clear();
	
 	while(1)
    {
	    
    }	
}




