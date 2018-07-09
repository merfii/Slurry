#include "stm32f10x_conf.h"
#include "Laser.h"
#include "Led.h"
#include "Trigger.h"
#include "Flicker.h"

//TIM10作为循环定时器  TIM11作为触发延迟器

static TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
static u8 flicker_T8_status;
    
void Flicker_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  ///使能TIM1时钟 用于曝光循环
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  ///使能TIM8时钟 用于激光打开和关闭

    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200-1;  //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2; 

    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn; //定时器8中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; //定时1中断
    NVIC_Init(&NVIC_InitStructure);	

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  ///使能TIM2时钟 用于机械臂同步
    NVIC_InitStructure.NVIC_IRQChannel =  TIM2_IRQn ; //定时器2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //抢占优先级0
    NVIC_Init(&NVIC_InitStructure);	
}
 


void Flicker_run(u8 freq, u8 inteval)
{
    TIM_TimeBaseInitStructure.TIM_Period = 10000/freq;  // 曝光循环 单位0.1ms   
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);//初始化
    TIM_SetCounter(TIM1, 0);
    
    TIM_TimeBaseInitStructure.TIM_Period = inteval*2; 	//从激光打开到关闭的延时 单位0.2ms
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);//初始化
    TIM_SetCounter(TIM8, 0);

    TIM_TimeBaseInitStructure.TIM_Period = 360; 	//机械臂同步信号宽度 36ms
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化
    TIM_SetCounter(TIM2, 0);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);   //允许更新中断

    flicker_T8_status = 0;
    Trigger1_off();
    Trigger2_off();
    Laser_off();
    Tribot_off();
    
    TIM_Cmd(TIM2,DISABLE);
    TIM_Cmd(TIM1,ENABLE);
    TIM_Cmd(TIM8,DISABLE);
}

void Flicker_stop()
{
    TIM_Cmd(TIM2,DISABLE); 
    TIM_Cmd(TIM1,DISABLE);
    TIM_Cmd(TIM8,DISABLE); 
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    TIM_ITConfig(TIM8, TIM_IT_Update, DISABLE);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
    Trigger1_off();Trigger2_off();
    Tribot_off();
  //Laser_off();
}

void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM1, TIM_FLAG_Update);
        //TIM_SetCounter(TIM11, 0);
        
        //下一次轮曝光
        Laser_off();
        Trigger1_on();
        Trigger2_on();
        //debug
            LED_tog();
        //debug
        flicker_T8_status = 0;
        TIM_Cmd(TIM8, ENABLE);
        TIM_Cmd(TIM2, ENABLE);
        Tribot_on();
    }
}

void TIM8_UP_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM8, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM8, TIM_FLAG_Update);
        if(flicker_T8_status == 0)
        {
            Trigger1_off();
            Trigger2_off();
            flicker_T8_status = 1;
        }else if(flicker_T8_status == 1)
        {
            Laser_on(); 
            Trigger1_on();
            Trigger2_on();
            flicker_T8_status = 2;
        }else
        {
            Trigger1_off();         
            Trigger2_off();         
            TIM_Cmd(TIM8, DISABLE);
            TIM_SetCounter(TIM8, 0);
            flicker_T8_status = 0;            
        }        
    }
}


void TIM2_IRQHandler(void)
{

   if(TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
        Tribot_off();  
        TIM_Cmd(TIM2, DISABLE);
        TIM_SetCounter(TIM2, 0);
    }
}
