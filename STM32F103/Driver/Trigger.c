#include "Trigger.h"

void Trigger_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  //初始化

    GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_8);    //关
}
    
void Trigger1_on(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_8);    //开   
}    

void Trigger1_off(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);    //关
}    

void Trigger2_on(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_6);    //开
}

void Trigger2_off(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);    //关
}

void Tribot_off(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_9);    //关
}

void Tribot_on(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);    //开
}
