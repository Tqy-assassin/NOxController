/*
 * clock.c

 *
 *  Created on: 2021年3月30日
 *      Author: sunkaixiang
 */
#include "clock.h"
#include "rtc.h"

static uint32_t Clock = 0;
void RTC_Task(void);
void Clock_init(void)
{
	RTC_ConfigType  RTC_Config = {0};	//计数器配置类
    /* configure RTC to 1000Hz interrupt frequency */
    RTC_Config.u16ModuloValue = 239;
    RTC_Config.bInterruptEn   = RTC_INTERRUPT_ENABLE;     /*!< enable interrupt */
    RTC_Config.bClockSource   = RTC_CLKSRC_BUS;
    RTC_Config.bClockPrescaler = RTC_CLK_PRESCALER_100;    /*!< prescaler is 100 */

    RTC_SetCallback(RTC_Task);//1ms执行一次
    RTC_Init(&RTC_Config);
}

void RTC_Task(void)
{
	Clock++;
}

uint32_t Gets_Clock_value(void)
{
	return Clock;
}

