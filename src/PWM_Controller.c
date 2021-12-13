/*
 * PWM_Controller.c
 *
 *  Created on: 2021��3��31��
 *      Author: sunkaixiang
 */

#include "PWM_Controller.h"
#include "ftm.h"
#include "sensor_control.h"

void FTM2_Task(void);			//FTM2�ж�
#define PWM_MOD			59999

void PWM_init(void)			//PWM��ʼ��
{
	FTM_ConfigType FTM2_Config={0};
	FTM_ChParamsType FTM2CH5_Config={0};

	FTM2_Config.modulo=PWM_MOD;							 /*10ms The Frequency equal to CLK/DIV/(PWM_MOD+1) */
	FTM2_Config.clk_source=FTM_CLOCK_SYSTEMCLOCK;//48M
	FTM2_Config.prescaler=FTM_CLOCK_PS_DIV8;
	FTM2_Config.mode=1;
	FTM2_Config.toie=1;

	SIM_PINSEL1 |= SIM_PINSEL1_FTM2PS5(0);

	FTM2CH5_Config.ctrl.bits.bMode=FTM_PWMMODE_EDGEALLIGNED;
	FTM2CH5_Config.ctrl.bits.bPWMPol=FTM_PWM_HIGHTRUEPULSE;
	FTM2CH5_Config.ctrl.bits.bCHIE=0;
	FTM2CH5_Config.u16CnV= 0;

	//FTM_SetCallback(FTM2, FTM2_Task);	//TODO �Ƶ�����������ģ����
	FTM_ChannelInit(FTM2,5,FTM2CH5_Config);

//	ENABLE_PWM;
//	NVIC_SetPriority(FTM1_IRQn,0);
	FTM_Init(FTM2,&FTM2_Config);
#ifdef DEBUG
	printf("\r\nPWM initialization successful.\r\n");
#endif
}

void set_pwm_dutycycle(uint8_t dc)		//����PWMcnVֵ
{
	FTM_SetChannelValue(FTM2, FTM_CHANNEL_CHANNEL5, (PWM_MOD+1) * dc /100);
}
