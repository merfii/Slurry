#ifndef __COMM_H
#define __COMM_H

#include "stm32f10x_conf.h"

#define COMM_RATE 115200    //波特率
#define COMM_REC_LEN  			4  	//定义接收字节数
extern u8 RX_BUF[COMM_REC_LEN];

#define COMM_HEAD_MAGIC 0xC9

void Comm_init(void);
void Comm_send(u8 dat);

#endif


