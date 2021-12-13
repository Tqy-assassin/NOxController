/*
 * device_state_manger.c
 *
 *  Created on: 2021��3��30��
 *      Author: tianqingyuan
 */
#include "device_state_manger.h"
#include "common.h"
#include "ASIC_Controller.h"
#include "clock.h"
#include "heating_controller.h"
#include "can_manage.h"
#include "power_manage.h"
#include "sensor_control.h"

uint32_t StartTimer = 0;
uint32_t Start_Timer = 0;

stage device_working_stage = STAGE_IDLE;            //��ǰ�Ĺ���״̬
stage normal_working_stage = STAGE_ENVIRONMENT;		//�¶��ȶ�������״̬
stage pre_working_stage    = STAGE_IDLE;			//��һ��״̬

extern uint8_t HEXorDEC;
extern float PWM_Duty_Cycle;
stage get_working_stage()
{
	return device_working_stage;
}

stage get_pre_working_stage()
{
	return pre_working_stage;
}


void init_working_stage()
{
	float Volage = get_WorkVoltage();
	device_working_stage = STAGE_IDLE;
	PWM_Duty_Cycle = 120 / Volage;//СԤ����
	set_PWM_Duty_Cycle(PWM_Duty_Cycle);
	init_Ip_Value();
#ifdef DEBUG
	printf("\r\nEntry Idle stage                  \r\n");
#endif
	ASIC_InitSensor(SPI0);
	ASIC_InitSensor(SPI0);
}




void set_working_stage(stage new_stage)
{
	float Voltage = get_WorkVoltage();
	if(device_working_stage != new_stage){			//Ҫ���õĹ���״̬�뵱ǰ״̬��һ�£���ʱ��ת��
		pre_working_stage = device_working_stage;
		if(new_stage == normal_working_stage){
			device_working_stage = normal_working_stage;
		}else{
			device_working_stage = new_stage;
		}
		switch(device_working_stage){
		case STAGE_IDLE:
			//set_PWM_Duty_Cycle(0);
			set_PWM_Duty_Cycle(120 / Voltage);//24V-->5  12V-->10
			init_Ip_Value();
#ifdef DEBUG
			printf("\r\nEntry Idle stage                  \r\n");
#endif
			ASIC_InitSensor(SPI0);
			ASIC_InitSensor(SPI0);
			break;
		case STAGE_PREHEATING:
			//set_PWM_Duty_Cycle(360 / (Voltage * Voltage));
			set_PWM_Duty_Cycle(240 / Voltage);//24V-->10  12V-->20
			Heating_Start();
			init_Ip_Value();
#ifdef DEBUG
			printf("\r\nEntry preheating stage            \r\n");
#endif
			ASIC_InitSensor(SPI0);
			ASIC_InitSensor(SPI0);
			break;
		case STAGE_HEATING:
			init_Ip_Value();
#ifdef DEBUG
			printf("\r\nEntry Heating stage               \r\n");
#endif
			ASIC_InitSensor(SPI0);
			ASIC_InitSensor(SPI0);
			break;
		case STAGE_CRITICALPOINT:
			init_Ip_Value();
#ifdef DEBUG
			printf("\r\nEntry Critical point stage        \r\n");
#endif
			HEXorDEC = 1;
			break;
		case STAGE_ENVIRONMENT:
			init_Ip_Value();
#ifdef DEBUG
			printf("\r\nEntry enironment stage            \r\n");
#endif
			if(StartTimer && !clock_time_exceed(StartTimer,3*60*1000))
			{
				StartTimer = 0;
				start_NOValueOffsetTimer(Gets_Clock_value());
				start_O2CValueOffsetTimer(Gets_Clock_value());
			}
			HEXorDEC = 0;
			break;
		case STAGE_TESTSTAGE:
#ifdef DEBUG
			printf("\r\nEntry test stage\r\n");
#endif
			HEXorDEC = 0;		//10�������ݿ���
			break;
		case STAGE_TESTSTAGE2:
#ifdef DEBUG
			printf("\r\nEntry test2 stage\r\n");
#endif
			HEXorDEC = 1;		//16���ƿ���
			break;
		case STAGE_InspectError:
#ifdef DEBUG
			printf("\r\nEntry inspect error stage\r\n");
#endif
			break;
		default:
#ifdef DEBUG
			printf("\r\nError stage\r\n");
#endif
			break;
		}
	}
}
