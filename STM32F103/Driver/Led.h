#ifndef __LED_H
#define __LED_H

#include "stm32f10x_conf.h"

void LED_init(void);
void LED_on(void);
void LED_off(void);
void LED_tog(void);

void LED_warning(void);


#endif
