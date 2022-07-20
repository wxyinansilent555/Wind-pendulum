#include "sys.h"   
#include "led.h"
void LED_Init(void)
{
  	 
	RCC->APB2ENR|=1<<4;   			 	//使能PORTC时钟	   	 
	GPIOC->CRH &= 0XFF0FFFFF; 		//清除数据
	GPIOC->CRH |= 0X00300000;			//PC13 推挽输出   	 
	GPIOC->ODR |= 0<<13;					//PC13 低电平点亮
	}






