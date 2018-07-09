#include "Led.h"
#include "Comm.h"
#include "Meter.h"
 
void Meter_timer_init(void);
void Meter_init(){
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟

    //GPIO端口设置 使能串口1对应引脚复用映射
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2复用为USART2
//   GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3复用为USART2

    //USART2端口配置 此处有问题需要重写
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD ;//开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;//输入上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA10
    

    //Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;   //串口2中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;     //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化 NVIC

    //USART2 初始化
    USART_InitStructure.USART_BaudRate = METER_RATE;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;   //无校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口2
    USART_Cmd(USART2, ENABLE);  //使能串口2
    USART_OverSampling8Cmd (USART2, DISABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开接收中断

    Meter_timer_init();
}

void Meter_timer_init()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  ///使能TIM3时钟

    TIM_TimeBaseInitStructure.TIM_Period = 3000; 	//发送测量信号的间隔
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200-1;  //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2; 

    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //定时器11中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03; //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
    
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void TIM3_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
    USART_SendData(USART2, 'D');
    LED_tog();
}

void USART2_IRQHandler(void) 
{

    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET && 
        USART_GetFlagStatus(USART2, USART_FLAG_NE | USART_FLAG_FE |USART_FLAG_PE) == RESET)
    {
        u16 byte16 = USART_ReceiveData(USART2);   //(USART2->DR) 清除错误标志位
		u8 byte = *((u8*)&byte16);     //little endian
        Comm_send(byte);
    }else
    {
           u16 byte16 = USART_ReceiveData(USART2);   //(USART2->DR) 清除错误标志位
    }
}

void Meter_stop()
{
    USART_SendData(USART2, 'C');
    TIM_Cmd(TIM3,DISABLE);    
}

void Meter_run()
{
    USART_SendData(USART2, 'O');
    TIM_Cmd(TIM3,ENABLE);
}

void Meter_status()
{
    TIM_Cmd(TIM3,DISABLE);
    USART_SendData(USART2, 'S');
}
