/*
 * heating_controller.c
 *
 *  Created on: 2021年3月31日
 *      Author: sunkaixiang
 */
#include "heating_controller.h"
#include "clock.h"
#include "device_state_manger.h"
#include "PWM_Controller.h"
#include "gpio.h"
#include "Survey_Control.h"
#include "common.h"
#include "flash_manage.h"
#include "vendor.h"
#include "power_manage.h"
#include "sensor_control.h"
#include"Filter.h"
static uint32_t Period100MSTimer = 0;
#define PreHeat_Time	(10)	//预加热时间  单位：s
#define Max_preHeatTemp	(0.0800)//预加热最大温度，及若当温度大于该值则跳过预加热

void Temperature_Controller(void);//温度控制
//#define Min_Temp		(CLT_pointer()->temper - 0.01)	//最小工作温度
#define Min_Temp		(CLT_pointer()->temper - 0.01)	//最小工作温度
#define Max_Temp		(CLT_pointer()->temper + 0.01)	//最大工作温度

#define PWM_DC_STEP		0.5			//最大步进	防止升温过快
#define PWM_MaxValue	85
#define PWM_MinValue	0
#define TempStabilize_Time	(5)	//温度稳定时间  单位：s

float PWM_Duty_Cycle = 0;			//当前PWM占空比
uint8_t HeatingStart = 0;		//重新开始加热 1:true 0:false
uint16_t HeatingTimer = 0;
#ifdef TEMPERATURE_CONTROLLER
SurveyVariate_Type Temper= {0};				//PWM -> temp
#endif

float TemperKp[2];
static uint32_t idle_time = 0;
uint8_t idle_stop_heat = 0;

float Pre_PWM_Duty[5];

void init_heating_module()
{
	float Volage = get_WorkVoltage();
	PWM_init();
	CONFIG_PIN_AS_GPIO(PTH,PTH0,OUTPUT);	//配置gpio模式
	set_PWM_Duty_Cycle(0);
	//设置温控pid系数
	Temper.Kp = 360 / Volage;
	TemperKp[0] = 360 / Volage;
	//TemperKp[1] = 3*(360 / Volage);
	TemperKp[1] = 360 / Volage;
	Temper.Ki = 120 / Volage;
	Temper.Kd = 240 / Volage;

	//if(Volage < 16){//12V
	//	Pre_PWM_Duty[0] = 10;
	//	Pre_PWM_Duty[1] = 20;
	//	Pre_PWM_Duty[2] = 30;
	//	Pre_PWM_Duty[3] = 45;
	//	Pre_PWM_Duty[4] = 50;
	//}else{//24V
	//	Pre_PWM_Duty[0] = 8;
	//	Pre_PWM_Duty[1] = 10;
	//	Pre_PWM_Duty[2] = 15;
	//	Pre_PWM_Duty[3] = 18;
	//	Pre_PWM_Duty[4] = 18;
	//}
	//if(Volage < 16){//12V
		Pre_PWM_Duty[0] = 720 / (Volage * Volage);
		Pre_PWM_Duty[1] = 1440 / (Volage * Volage);
		Pre_PWM_Duty[2] = 2160 / (Volage * Volage);
		Pre_PWM_Duty[3] = 2880 / (Volage * Volage);
		Pre_PWM_Duty[4] = 2880 / (Volage * Volage);
	//}else{//24V
	//	Pre_PWM_Duty[0] = 360 / (WorkVolageValue * WorkVolageValue);
	//	Pre_PWM_Duty[1] = 720 / (WorkVolageValue * WorkVolageValue);
	//	Pre_PWM_Duty[2] = 1080 / (WorkVolageValue * WorkVolageValue);
	//	Pre_PWM_Duty[3] = 1440 / (WorkVolageValue * WorkVolageValue);
	//	Pre_PWM_Duty[4] = 1440 / (WorkVolageValue * WorkVolageValue);
	//}
}

void enable_pwm(void){
	OUTPUT_SET(PTH,PTH0);
}

void disable_pwm(void){
	OUTPUT_CLEAR(PTH,PTH0);
}

void set_PWM_Duty_Cycle(float value){
	if(value > 0){
		enable_pwm();
	}else{
		disable_pwm();
	}
	PWM_Duty_Cycle = value;
	set_pwm_dutycycle(PWM_Duty_Cycle);
}

void Heating_Start(void){
	HeatingStart = 1;
}

float get_PWM_Duty_Cycle(void){
	return PWM_Duty_Cycle;
}


void heating_control(void){
	ADtfValue * ADtf_Value = get_ADtf_Value();
	measureCoeffi_Typedef* PCLT = CLT_pointer();
	float Volage = get_WorkVoltage();
	if(clock_time_exceed(Period100MSTimer,100)){			//周期性事件 T=100ms
		Period100MSTimer = Gets_Clock_value();
		switch(get_working_stage()){				//周期性发生
			case STAGE_IDLE:
			{
				//加热到指定温度后停止加热5秒
				Status_HeaterOff();

				if(!idle_stop_heat){
					idle_time = Gets_Clock_value();
					PWM_Duty_Cycle = 72 / Volage;//小预加热
				}else{
					PWM_Duty_Cycle = 0;
					if(clock_time_exceed(idle_time,5000)){
						idle_stop_heat = 0;
					}
				}

				if(ADtf_Value->VTempValue > 0.075){
					idle_stop_heat = 1;
				}

				set_PWM_Duty_Cycle(PWM_Duty_Cycle);
			}
			break;
			case STAGE_PREHEATING:
			{
//				static uint8_t perheatcount = 0;
//				perheatcount ++;
//				Status_PreHeater();
//				if(perheatcount < PreHeat_Time * 2){
//					PWM_Duty_Cycle = 360 / (Volage * Volage);
//				}else if(perheatcount < PreHeat_Time * 4){
//					PWM_Duty_Cycle = 720 / (Volage * Volage);
//				}else if(perheatcount < PreHeat_Time * 7){
//					PWM_Duty_Cycle = 1080 / (Volage * Volage);
//				}else if((perheatcount < PreHeat_Time * 10/*) || (VTempValue > Max_preHeatTemp*/)){
//					PWM_Duty_Cycle = 1440 / (Volage * Volage);
//				}else{
//					perheatcount = 0;
//					set_working_stage(STAGE_HEATING);
//				}
//				set_pwm_dutycycle(PWM_Duty_Cycle);

				static uint8_t perheatcount = 0;
				perheatcount ++;
				Status_PreHeater();

				if(perheatcount < PreHeat_Time * 4){
					PWM_Duty_Cycle = Pre_PWM_Duty[0];
				}else if(perheatcount < PreHeat_Time * 8){
					PWM_Duty_Cycle = Pre_PWM_Duty[1];
				}else if(perheatcount < PreHeat_Time * 14){
					PWM_Duty_Cycle = Pre_PWM_Duty[2];
				}else{
					PWM_Duty_Cycle = Pre_PWM_Duty[3];

				}

				if(ADtf_Value->VTempValue > 0.7*PCLT->temper){//到设定温度的70%后开始PID计算
					perheatcount = 0;
					PWM_Duty_Cycle = Pre_PWM_Duty[4];
					set_working_stage(STAGE_HEATING);
				}

				set_pwm_dutycycle(PWM_Duty_Cycle);
			}
			break;
			case STAGE_HEATING:
			{
				Status_Heating();
				Temperature_Controller();			//adjust pwm dutycycle
				if((ADtf_Value->VTempValue > Min_Temp) && (ADtf_Value->VTempValue < Max_Temp))
				{
					set_working_stage(STAGE_CRITICALPOINT);
				}
			}
			break;
			case STAGE_CRITICALPOINT:
			{
				Status_HeatCriticalPoint();
				Temperature_Controller();			//adjust pwm dutycycle
			}
			break;
			case STAGE_ENVIRONMENT:
			{
				Status_AutomaticHeat();
				Temperature_Controller();			//adjust pwm dutycycle
			}
			case STAGE_TESTSTAGE:
			{
				Temperature_Controller();			//adjust pwm dutycycle
			}
			break;
			case STAGE_TESTSTAGE2:
			{
				Temperature_Controller();			//adjust pwm dutycycle
			}
			break;
			case STAGE_InspectError:
			{
				static int ErrorTimer;
				ErrorTimer++;
				if(get_pre_working_stage() > STAGE_PREHEATING)
					Temperature_Controller();		//adjust pwm dutycycle
				if(ErrorTimer>60){
					ErrorTimer = 0;
					set_working_stage(STAGE_IDLE);
				}
			}
			break;
			default:
			{}
			break;
		}
	}
}


void Temperature_Controller(void)		//温度控制
{
#ifdef TEMPERATURE_CONTROLLER
#if 1
	/*********************function 1*********************************/
	float step;
	float TCR;
	static uint8_t Temper_stabilize = 0;//heatinit = 1;
	ADtfValue * ADtf_Value = get_ADtf_Value();
	//KalmanADtfValue * KalmanADtf_Value = get_KalmanADtf_Value();
	measureCoeffi_Typedef* PCLT = CLT_pointer();
	if(HeatingStart){
		HeatingStart = 0;
		if(ADtf_Value->VTempValue > 0.99 * PCLT->temper){
			Temper.Kp = TemperKp[0];
			HeatingTimer = 600;
		}else if(ADtf_Value->VTempValue > 0.85 * PCLT->temper){
			Temper.Kp = TemperKp[0];
			HeatingTimer = 300;
		}else if(ADtf_Value->VTempValue > 0.7 * PCLT->temper){
			Temper.Kp = TemperKp[1];
			HeatingTimer = 150;
		}else{
			Temper.Kp = TemperKp[1];
			HeatingTimer = 0;
		}
	}

	if(HeatingTimer < 150){
		HeatingTimer++;
		TCR = 0.4 + 3.0*HeatingTimer/1500;
	}else if(HeatingTimer < 300){
		HeatingTimer++;
		TCR = 0.55 + 3.0*HeatingTimer/3000;
	}else if(HeatingTimer < 600){
		HeatingTimer++;
		TCR = 0.7 + 3.0*HeatingTimer/6000;
	}else{
		TCR = 1;
	}

	if(ADtf_Value->VTempValue < 0.85 * PCLT->temper){
		Temper.Kp = TemperKp[1];
	}else{
		Temper.Kp = TemperKp[0];
	}

//	KalmanADtf_Value->KalmanVTempValue = ADtf_Value->VTempValue;
//	if(heatinit){
//		heatinit = 0;
//		Heat_KalmanFilter(KalmanADtf_Value->KalmanVTempValue,0);
//	}else{
//		KalmanADtf_Value->KalmanVTempValue = Heat_KalmanFilter(KalmanADtf_Value->KalmanVTempValue,1);
//		//printstring("\r\n Kalman :");printfloat(ADtf_Value.VTempValue);
//	}

	Temper.E_2 = Temper.E_1;
	Temper.E_1 = Temper.E;
	Temper.E = PCLT->temper*TCR - ADtf_Value->VTempValue;//设定值-测量值
	//Temper.E = PCLT->temper*TCR - KalmanADtf_Value->KalmanVTempValue;

	if(myabs(Temper.E) > 0.0002){
		step = (Temper.Kp+Temper.Ki+Temper.Kd)*Temper.E - (Temper.Kp+2*Temper.Kd)*Temper.E_1 + Temper.Kd*Temper.E_2;
		//step = (TemperKp+Temper.Ki+Temper.Kd)*Temper.E - (TemperKp+2*Temper.Kd)*Temper.E_1 + Temper.Kd*Temper.E_2;
		PWM_Duty_Cycle += range(step,PWM_DC_STEP);
		PWM_Duty_Cycle = PWM_Duty_Cycle < PWM_MaxValue ? (PWM_Duty_Cycle > PWM_MinValue ? PWM_Duty_Cycle : PWM_MinValue) : PWM_MaxValue;
		set_pwm_dutycycle(PWM_Duty_Cycle);
	}

#else
	/**********************function 2*******************************/
	static uint8_t Heating_stage = 0;
	static uint8_t Temper_stabilize = 0;
	float step;

	Temper.E_2 = Temper.E_1;
	Temper.E_1 = Temper.E;
	Temper.E = PCLT->temper - VTempValue;

	if(HeatingTimer < 150){
		HeatingTimer++;
		if(Heating_stage != 1){
			Heating_stage = 1;
			PWM_Duty_Cycle = Heater_1;
			set_pwm_dutycycle(PWM_Duty_Cycle);
		}
	}else if(HeatingTimer < 300){
		HeatingTimer++;
		if(Heating_stage != 2){
			Heating_stage = 2;
			PWM_Duty_Cycle = Heater_2;
			set_pwm_dutycycle(PWM_Duty_Cycle);
		}
	}else if(HeatingTimer < 600){
		HeatingTimer++;
		if(Heating_stage != 3){
			Heating_stage = 3;
			PWM_Duty_Cycle = Heater_3;
			set_pwm_dutycycle(PWM_Duty_Cycle);
		}
	}else{
		if(Heating_stage != 4){
			Heating_stage = 4;
		}
		if(myabs(Temper.E) > 0.0002)
		{
			step = (Temper.Kp+Temper.Ki+Temper.Kd)*Temper.E - (Temper.Kp+2*Temper.Kd)*Temper.E_1 + Temper.Kd*Temper.E_2;
			PWM_Duty_Cycle += range(step,PWM_DC_STEP);
			PWM_Duty_Cycle = PWM_Duty_Cycle < PWM_MaxValue ? (PWM_Duty_Cycle > PWM_MinValue ? PWM_Duty_Cycle : PWM_MinValue) : PWM_MaxValue;
			set_pwm_dutycycle(PWM_Duty_Cycle);
		}
	}
#endif
	if((ADtf_Value->VTempValue > Min_Temp) && (ADtf_Value->VTempValue < Max_Temp)){		//当温度系数一定时间内保持在范围之内，则
		if(Temper_stabilize > TempStabilize_Time * 10){
			Status_AtTemperature();
			set_working_stage(STAGE_ENVIRONMENT);
		}else{
			Temper_stabilize ++;
		}
	}else{
		if(Temper_stabilize > 0){
			Temper_stabilize --;
		}else{
			Status_nAtTemperature();
			set_working_stage(STAGE_HEATING);
		}
	}
#else
	float step,pwm_DC;
	if(PWM_Duty_Cycle != PWM_Duty_Cycle_Target)
	{
		pwm_DC = (float)((int)(PWM_Duty_Cycle*100))/100;
		step = ((PWM_Duty_Cycle_Target-pwm_DC) > 0)?(PWM_Duty_Cycle_Target-pwm_DC)/50:(pwm_DC-PWM_Duty_Cycle_Target)/50;

		if(PWM_Duty_Cycle < PWM_Duty_Cycle_Target)
			PWM_Duty_Cycle += step;
		else
			PWM_Duty_Cycle -= step;
		set_pwm_dutycycle(PWM_Duty_Cycle);
	}
#endif
}
