#include "stm32f10x_conf.h"
#include "Led.h"
#include "Comm.h"
#include "Watchdog.h"

#include "Key.h"

static void IO_init(void);

void Key_init()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	IO_init(); //按键对应的IO口初始化
 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);//PE0 连接到中断线0
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);//PE1 连接到中断线1

    /* 配置EXTI_Line4 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;//LINE0
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE4
    EXTI_Init(&EXTI_InitStructure);//配置

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//外部中断0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;//子优先3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置

    /* 配置EXTI_Line5 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);//配置

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
}    

void IO_init()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//PIOE时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5 ; //KEY0 KEY1对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//输入上拉
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}


void EXTI4_IRQHandler(void)
{
    EXTI_ClearFlag(EXTI_Line4);
    LED_warning();
}

void EXTI9_5_IRQHandler(void)
{
    EXTI_ClearFlag(EXTI_Line5);
    //Reboot();
    Comm_send(0x20);
}



