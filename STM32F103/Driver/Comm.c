#include "Led.h"
#include "Laser.h"
#include "Trigger.h"
#include "Comm.h"
 
	
u8 RX_BUF[COMM_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
volatile u8 debug_byte;
volatile u8 debug_step;
void cmd_protocol(void);

void Comm_init(){
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟

    //GPIO引脚复用映射
 // GPIO_PinRemapConfig  (GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9复用为USART1
 // GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10复用为USART1

    //USART1端口配置
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD ;//开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;//输入上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA10
    
    //USART1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   //串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;     //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化 NVIC

    //USART1 初始化
    USART_InitStructure.USART_BaudRate = COMM_RATE;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_Even;   //偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_Cmd(USART1, ENABLE);  //使能串口1 
    USART_OverSampling8Cmd (USART1, DISABLE);
    //USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开接收中断	
}


void USART1_IRQHandler(void) 
{
    u8 byte;
    static int step = 0;
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET && 
        USART_GetFlagStatus(USART1, USART_FLAG_NE | USART_FLAG_FE |USART_FLAG_PE) == RESET)
    {
        u16 byte16 = USART_ReceiveData(USART1);   //(USART1->DR) 清除错误标志位
		byte = *((u8*)&byte16);     //little endian
        debug_byte = byte;
        switch(step)
        {
            case 0:
                if(byte == COMM_HEAD_MAGIC)
                {
                    step = 1;
                }
            break;
            
            case 1:
                RX_BUF[0] = byte;
                step ++;
            break;
            
            case 2:
                RX_BUF[1] = byte;
                step ++;
            break;
            

            case 3:
                RX_BUF[2] = byte;
                    step++;
/*                if(byte == RX_BUF[1])
                {
                    RX_BUF[2] = byte;
                    step++;
                }else
                {
                    step = 0;
                }*/
            break;
            
            case 4:
               if(byte == RX_BUF[0])
                {
                    RX_BUF[3] = byte;
                    step++;
                }else
                {
                    step = 0;
                }
            break;            
             
            case 5:
               if(byte == (RX_BUF[0] ^ RX_BUF[1]))
                {
                    //Good Data Received
                    cmd_protocol();    
                }
                step = 0;
                break;
                
            default:
                if(byte == COMM_HEAD_MAGIC)
                {
                    step = 1;
                }else
                {
                    step = 0;
                }
        }    
        debug_step = step;
    }else
    {
           u16 byte16 = USART_ReceiveData(USART1);   //(USART1->DR) 清除错误标志位
    }
}

void Comm_send(u8 dat)
{
    USART_SendData(USART1, dat);
}
