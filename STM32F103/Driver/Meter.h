#ifndef __METER_H
#define __METER_H

#include "stm32f10x_conf.h"

/*
数据格式 
O 开激光
C 光激光
D 测距
S 查看状态
*/

#define METER_RATE 19200    //波特率

void Meter_init(void);
void Meter_run(void);
void Meter_stop(void);
void Meter_status(void);

#endif


