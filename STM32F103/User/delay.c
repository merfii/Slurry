#include "stm32f10x.h"
#include "delay.h"

static volatile u32 us_count;

void Delay_init(void)
{
    SysTick_Config(SystemCoreClock / 1000000);
// 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
	us_count = 0;
}								    

 
 
//systick中断服务函数,使用OS时用到
void SysTick_Handler(void)
{
    if(us_count)
    {
        us_count --;
    }
}

void Delay_us(u32 us)
{
    SysTick->VAL=0x00; 
    us_count = us;
    while(us_count)
        __NOP();
    /*
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//时间加载	  		 
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //关闭计数器
	SysTick->VAL =0X00;       				//清空计数器 
    */
}


void Delay_ms(u32 ms)
{	 	 
    while(ms--)
    {
        Delay_us(1000);
    }
} 
