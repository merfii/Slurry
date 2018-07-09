#include "Led.h" 

//初始化PF9和PF10为输出口.并使能这两个口的时钟		    

static void warning_init(void);

void LED_init(void)
{    	 
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能时钟

    //GPIOB11初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
    
    LED_off(); //设置灯灭
    warning_init();
}

void LED_on()
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_11);
}

void LED_off()
{
	GPIO_SetBits(GPIOB, GPIO_Pin_11);
}

void LED_tog()
{
    if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_11))
    {
      	GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    }else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_11);
    }
}

static const u16 warning_table[] = {100,200,100,600,100,200,100};    // 单位 0.5ms
static volatile u8 warning_step;

void warning_init(void)    //基本定时器 用于延时闪灯
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM5时钟
   
    TIM_TimeBaseInitStructure.TIM_Period = 0; 	//自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36000-1;  //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =  TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化

    TIM_SetCounter(TIM5, 0);
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; //子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
}

void LED_warning()
{
    LED_on();
    warning_step = 1;
    TIM5->ARR = warning_table[0]; 
    TIM_Cmd(TIM5,ENABLE); //使能定时器5
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    TIM_SetCounter(TIM5, 0);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); //允许定时器中断
}

void TIM5_IRQHandler(void)
{
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    TIM_Cmd(TIM5, DISABLE);
    LED_tog();
     
    if(warning_step <= 6)
    {
        TIM5->ARR = warning_table[warning_step++];
        TIM_Cmd(TIM5, ENABLE);
    }

}

