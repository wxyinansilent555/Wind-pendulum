#include "sys.h"   
#include "led.h"
void LED_Init(void)
{
  	 
	RCC->APB2ENR|=1<<4;   			 	//ʹ��PORTCʱ��	   	 
	GPIOC->CRH &= 0XFF0FFFFF; 		//�������
	GPIOC->CRH |= 0X00300000;			//PC13 �������   	 
	GPIOC->ODR |= 0<<13;					//PC13 �͵�ƽ����
	}






