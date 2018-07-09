#ifndef _FLICKER_H
#define _FLICKER_H

#include "stm32f10x_conf.h"

void Flicker_init(void);
void Flicker_run(u8 freq, u8 inteval);
void Flicker_stop(void);    
    

#endif

