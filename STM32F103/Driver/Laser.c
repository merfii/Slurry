#include "Laser.h"


void Laser_init()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能时钟

    //GPIOF9,F10初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

    GPIO_ResetBits(GPIOB, GPIO_Pin_5);//关闭
}

void Laser_off()
{
	GPIO_SetBits(GPIOB, GPIO_Pin_5);    
}

void Laser_on()
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

void Laser_tog()
{
    if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_5))
    {
      	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
    }else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_5);
    }
}
