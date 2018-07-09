#include "Comm.h"
#include "Trigger.h"
#include "Flicker.h"
#include "Delay.h"
#include "usart.h"
#include "led.h"
#include "Laser.h"
#include "Key.h"
#include "Watchdog.h"


/*
新定时器使用表
TIM2        RobotSync
TIM1 TIM8 Flicker
TIM5        LED
TIM3        Meter
*/

static void RCC_Configuration(void);
    
int main(void)
{ 
   // RCC_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置重入优先级0-3
	Delay_init();  //初始化延时函数
	LED_init();	    //初始化LED端口
    Laser_init();
    Trigger_init();
    Flicker_init();
    Comm_init();
    Key_init();
  //  Watchdog_init();
    
    __enable_irq();
    Laser_off();
    LED_warning();
    
	while(1)
	{
		Watchdog_feed();
        Delay_ms(50);
       
//        Trigger1_on();
//        Trigger2_on();
//       Tribot_on();
//        Laser_on();
//        LED_on();

//        Delay_ms(10);
//        Trigger1_off();
//        Trigger2_off();
//       Tribot_off();
//        Laser_off();
//        LED_off();
	}
}

void RCC_Configuration(void)
{
    RCC_DeInit();  
    RCC_HSEConfig(RCC_HSE_ON);
    while(SUCCESS != RCC_WaitForHSEStartUp()){} 
    FLASH_SetLatency(FLASH_Latency_2);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /* Fcpu = (PLL_src * PLL_MUL) = (8 Mhz / 1) * (9) = 72Mhz   */ 
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);            
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);  
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
    /* Set system clock dividers */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);   
    /* Embedded Flash Configuration */
    FLASH_SetLatency(FLASH_Latency_2);                           
    FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Disable);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /*SYSCLK configuration*/
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

/*
数据格式 
0xC9 头标示
cmd1 cmd2 命令 
cmd2 cmd1 命令重复 
cmd1 ^ cmd2 校验

CMD_FLICKER_RUN 后跟的是频率freq 单位Hz 不能大于一万 
*/
#define CMD_CLOSE_ALL 0x00
#define CMD_LED2_ON 0x10
#define CMD_LED2_OFF 0x11
#define CMD_LED3_ON 0x12
#define CMD_LED3_OFF 0x13
#define CMD_LASER_ON 0x20
#define CMD_LASER_OFF 0x21
#define CMD_TRIGGER1_ON 0x30
#define CMD_TRIGGER1_OFF 0x31
#define CMD_TRIGGER2_ON 0x32
#define CMD_TRIGGER2_OFF 0x33
#define CMD_TRIGBOT_ON 0x34
#define CMD_TRIGBOT_OFF 0x35
#define CMD_FLICKER_RUN 0x40
#define CMD_FLICKER_STOP 0x41
#define CMD_METER_RUN 0x50
#define CMD_METER_STOP 0x51
#define CMD_METER_STATUS 0x52

void cmd_protocol()
{
    switch(RX_BUF[0])
    {
        case CMD_CLOSE_ALL:
            LED_off();
            Laser_off();
            Trigger1_off();
            Trigger2_off();
        break;
        
        case CMD_LED2_ON:
            LED_on();
            break;
        
        case CMD_LED2_OFF:
            LED_off();
            break;

        case CMD_LED3_ON:
            LED_on();
            break;
        
        case CMD_LED3_OFF:
            LED_off();
            break;
        
        case CMD_LASER_ON:
            Laser_on();
            break;

        case CMD_LASER_OFF:
            Laser_off();
            break;
        
        case CMD_TRIGGER1_ON:  //Open Trigger1
            Trigger1_on();
            break;

        case CMD_TRIGGER1_OFF:
            Trigger1_off();
            break;
        
        case CMD_TRIGGER2_ON:  //Open Trigger2
            Trigger2_on();
            break;
        
        case CMD_TRIGGER2_OFF:
            Trigger2_off();
            break;

        case CMD_TRIGBOT_ON:  //Open Trigger for robot
            Tribot_on();
            break;
        
        case CMD_TRIGBOT_OFF:
            Tribot_off();
            break;
        
        case CMD_METER_RUN:
            //Meter_run();
            break;
        
        case CMD_METER_STOP:
            //Meter_stop();
            break;
        
        case CMD_METER_STATUS:
            //Meter_status();
            break;
                
        case CMD_FLICKER_RUN:
            Flicker_run(RX_BUF[1], RX_BUF[2]); 
            break;

        case CMD_FLICKER_STOP:
            Flicker_stop();
            Laser_off();
            break;
        
        default:
            LED_warning();
            break;
    }
    
}
