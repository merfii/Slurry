#include "stm32f10x_conf.h"

#include "Watchdog.h"


void Watchdog_init()
{    	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE); //使能窗口看门狗时钟

    WWDG_SetWindowValue(0x7f);
    WWDG_SetPrescaler(WWDG_Prescaler_8);
    WWDG_Enable(0x7e); 
}

void Watchdog_feed()
{
    WWDG_SetCounter(0x7e);
}

void Reboot()
{
    NVIC_SystemReset();
}
