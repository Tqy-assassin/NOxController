/*
 * power_manage.c
 *
 *  Created on: 2021年4月1日
 *      Author: sunkaixiang
 */
#include "power_manage.h"
#include "config.h"
#include "adc.h"
#include <string.h>
#include "clock.h"

float WorkVoltageValue = 0;
uint32_t Period60STimer = 0;
float get_WorkVoltage(void){
	return WorkVoltageValue;
}

void Judge_WorkVoltage(void)
{
#ifdef VOLTAGE_COMPATIBILITY
	ADC_ConfigType ADC_Config;
	uint16_t Voltage_BeforeTransf;
	float Volage_AfterTransf;

    memset(&ADC_Config, 0, sizeof(ADC_ConfigType));
    ADC_Config.u8ClockDiv = ADC_ADIV_DIVIDE_4;
    ADC_Config.u8ClockSource = CLOCK_SOURCE_BUS_CLOCK;
    ADC_Config.u8Mode = ADC_MODE_12BIT;
    ADC_Config.sSetting.bContinuousEn=1;
    ADC_Config.u16PinControl= 0x1<<3; /* Disable I/O control on ADC channel 3*/
    ADC_Init(ADC, &ADC_Config);

    Voltage_BeforeTransf = ADC_PollRead(ADC,ADC_CHANNEL_AD3);
    ADC_DeInit(ADC);
    Volage_AfterTransf = (((float)Voltage_BeforeTransf * 5.0 / 4096) - 0.1) / 0.1485;
//    Volage_AfterTransf = ((float)Voltage_BeforeTransf * 5.0 / 4096);

    WorkVoltageValue = Volage_AfterTransf;
#else
#if WORKVOLAGE12Vor24V
    WorkVoltageValue = 12;
#else
    WorkVoltageValue = 24;
#endif
#endif
#ifdef DEBUG
//	printf("WorkVoltage = %dV\n", (int)WorkVoltageValue);
#endif
}

void Judge_WorkVoltage_loop(void){

	if(clock_time_exceed(Period60STimer,60 * 1000)){			//周期性事件 T=60s
		Period60STimer = Gets_Clock_value();
		Judge_WorkVoltage();
	}
}
