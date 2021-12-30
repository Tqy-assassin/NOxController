/*
 * sensor_control.c
 *
 *  Created on: 2021��4��1��
 *      Author: sunkaixiang
 */
#include "sensor_control.h"
#include "can_manage.h"
#include "device_state_manger.h"
#include "ASIC_controller.h"
#include "ftm.h"
#include "pit.h"
#include "Survey_Control.h"
#include "flash_manage.h"
#include "config.h"
#include "Filter.h"
#include <stdlib.h>
#include <string.h>
#include "printf.h"
#include "common.h"
#include "vendor.h"
void Survey_Ready(void);		//PID׼��
void Survey_Controller(void);	//PID����
void Value_Dispose(void);		//���ݽ���
void PIT0_init(void);			//�ж϶�ʱ��
void PIT1_init(void);			//�ж϶�ʱ��
void PIT_Task0(void);			//��ʱ��1�ж�
void PIT_Task1(void);
void ASIC_DACAndInspectHandler(void);//��ɭ��DAC������Ӽ����ƾ��
void ASIC_ReadMeasurementResultHandler(void);//��ɭ����ȡ����������ƾ��
void get_ip012(uint32_t* ip0,uint16_t* ip1,uint16_t* ip2);
void Value_Dispose(void);


#define IP1_EN_SHIFT			(14)
#define IP1_EN__MASK			0x4000
#define IP1_AC_1_0_SHIFT		(12)
#define IP1_AC_1_0_MASK			0x3000
#define IP1_AR_2_0_SHIFT		(9)
#define IP1_AR_2_0_MASK			0x0E00
#define IP1_BC_1_SHIFT			(8)
#define IP1_BC_1_MASK			0x0100
#define IP1_BC_0_SHIFT			(6)
#define IP1_BC_0_MASK			0x0040
#define IP1_BR_1_0_SHIFT		(4)
#define IP1_BR_1_0_MASK			0x0030
#define IP1_IP1_S_SHIFT			(0)
#define IP1_IP1_S_MASK			0x000F
#define IP1_EN_MASK				0x2000
#define IP1_MAXVALUE			0x1FFF
#define IP1_MINVALUE			0x23FF  //10,0011,1 111,1111

#define IP0_EN_SHIFT			(18)
#define IP0_EN__MASK			0x00040000
#define IP0_AC_2_1_SHIFT		(16)
#define IP0_AC_2_1_MASK			0x00030000
#define IP0_AC_0_SHIFT			(14)
#define IP0_AC_0_MASK			0x00004000
#define IP0_AR_3_0_SHIFT		(10)
#define IP0_AR_3_0_MASK			0x00003C00
#define IP0_BC_1_0_SHIFT		(8)
#define IP0_BC_1_0_MASK			0x00000300
#define IP0_BR_2_0_SHIFT		(4)
#define IP0_BR_2_0_MASK			0x00000070
#define IP0_IP0_S_SHIFT			(0)
#define IP0_IP0_S_MASK			0x0000000F
#define IP0_EN_MASK				0x00010000
#define IP0_MAXVALUE			0x0000FFFF
#define IP0_MINVALUE			0x0001FFFF

#define IP2_INS_D2_6_0_SHIFT	(0)
#define IP2_INS_D2_6_0_MASK		0x007F
#define IP2_INS_D3_5_0_SHIFT	(8)
#define IP2_INS_D3_5_0_MASK		0x3F00
#define IP2_INS_D3_6_SHIFT		(13)
#define IP2_INS_D3_6_MASK		0x4000
#define IP2_EN_MASK				0x2000
#define IP2_MINVALUE			0x3FFF
#define IP2_MAXVALUE			0x1FFF

#define IP1_ADC_COEF			(0.5)		//transfer coefficient of Ip1 ADC value
#define IP0_ADC_COEF			(2)			//transfer coefficient of Ip0 ADC value
#define IP2_ADC_COEF			(0.5)		//transfer coefficient of Ip2 ADC value
#define COMM_ADC_COEF			(-2)		//transfer coefficient of COMM ADC value
#define TEMP_ADC_COEF			(0.5)		//transfer coefficient of temperature ADC value
#define TEST_ADC_COEF			(2)			//transfer coefficient of test sign ADC value


P_GeneralTable IPTableV;



#if VREF0_METHOD == 1
SurveyVariate_extern_Type IP1	= {0};		//Vref0 -> Ip1
#elif VREF0_METHOD == 0
SurveyVariate_Type IP1 = {0};
#elif VREF0_METHOD == 2
SurveyVariate_Type IP1tab = {0};
#endif
#define PIT_LOAD_VALUE	239999		//per 20ms
extern LoopArray Ip0Totle;
extern LoopArray Ip2Totle;
extern LoopArray Vref0Totle;
extern LoopArray Vref1Totle;
extern uint32_t Start_Timer;
SurveyVariate_Type Vref0 = {0};				//Ip0 -> Vref0
SurveyVariate_Type Vref1 = {0};				//Ip1 -> Vref1
SurveyVariate_Type Vref2 = {0};				//Ip2 -> Vref2

uint8_t ASIC_Flag = 1;
uint8_t HEXorDEC = 0;			//0��10���ƣ�1��16����

uint8_t DBFS = 2;

uint8_t NoPIDflag = 0;


uint32_t DBFSTimemr = 0;
const float Vrefbound[4] = {0.01, 0.05, 0.15, 0.3};//V
float Kp[7][5] =	   {
							{4500,5400,6480,7776,9331},				//0: I0--->V0(1)//4500
							{1000, 1000, 1000, 1000, 1000},			//1: I1--->V1
							{50,   50,   50,   50,   50},			//2: I2--->V2(1)
							{0.0005,0.0005,0.0005,0.0005,0.0005},	//3: I1--->V0
							{0.005,0.005,0.005,0.005,0.005},		//4: I1+���--->V0
							{7776,7776,7776,9331,11197},			//5: I0--->V0(2)//7776
							{50,   50,   50,   50,   50}			//6: I2--->V2(2)
																		};

float Ki[7][5] = 	   {
							{2000, 2000, 2000, 2000, 2000},				//0: I0--->V0(1)
							{900,  900,  900,  900,  900},				//1: I1--->V1
							{5,  5,  5,  5,  5},						//2: I2--->V2(1)
							{0.00001,0.00001,0.00001,0.00001,0.00001},	//3: I1--->V0
							{0.0002,0.0002,0.0002,0.0002,0.0002},		//4: I1+���--->V0
							{2000, 2000, 2000, 2000, 2000},				//5: I0--->V0(2)
							{5,  5,  5,  5,  5}							//6: I2--->V2(2)
																		};

float Kd[7][5] = 	   {
							{0,0,0,0,0},							//0: I0--->V0(1)
							{0,0,0,0,0},							//1: I1--->V1
							{0,0,0,0,0},							//2: I2--->V2(1)
							{0.0001,0.0001,0.0001,0.0001,0.0001},	//3: I1--->V0
							{0.0001,0.0001,0.0001,0.0001,0.0001},	//4: I1+���--->V0
							{0,0,0,0,0},							//5: I0--->V0(2)
							{0,0,0,0,0}								//2: I2--->V2(2)
																	};
/*
float Kd[6][5] = 	   {
							{2000,2000,2000,2000,2000},				//0: I0--->V0(1)
							{100, 100, 100, 100, 100},				//1: I1--->V1
							{1,  1,  1,  1,  1},					//2: I2--->V2
							{0.0001,0.0001,0.0001,0.0001,0.0001},	//3: I1--->V0
							{0.0001,0.0001,0.0001,0.0001,0.0001},	//4: I1+���--->V0
							{2000,2000,2000,2000,2000}				//5: I0--->V0(2)
																	};
*/
#define Vref0_MaxValue	(0.42)//(CLT.Vref0)
#define Vref0_MinValue	(0.34)
#define Vref1_MaxValue	(0.4)//(CLT.Vref1)
#define Vref1_MinValue	(0.4)
#define Ip0_MinValue (-6474.9)
#define Ip0_MaxValue (6474.9)
#define Ip1_MinValue (-20.1)
#define Ip1_MaxValue (160.6)
#define Ip2_MinValue (-1)
#define Ip2_MaxValue (6)
#define IP1controlVref0_MaxValue	(0.025)
#define IP1controlVref0_MinValue	(-0.025)

float Vref = 0;
//float Ip0_Value = 0,Ip1_Value = 0,Ip2_Value = 0;

IpValue Ip_Value =
	{
		.Ip0_Value = 0,
		.Ip1_Value = 0,
		.Ip2_Value = 0
	};

float Vref0_V = 0.35;					//Vref0Ŀ��ֵ
float Vref1_V = 0.405;					//Vref1Ŀ��ֵ
int16_t adc_value[6];

ADtfValue ADtf_Value = {0,0,0,0,0,0};
KalmanADtfValue KalmanADtf_Value = {0,0,0,0,0,0};


KalmanADtfValue * get_KalmanADtf_Value(void){
	return &KalmanADtf_Value;
}

ADtfValue * get_ADtf_Value(void){
	return &ADtf_Value;
}

IpValue * get_Ip_Value(void){
	return &Ip_Value;
}

void init_Ip_Value(void){
	Ip_Value.Ip0_Value=0;
	Ip_Value.Ip1_Value=0;
	Ip_Value.Ip2_Value=0;
}


void ASIC_DACAndInspectHandler(void){					//FTM2����ж�		����IP0 IP1 IP2��ֵ����
	extern uint8_t Sensor_Status;
//	__IO ASIC_StatusTypeDef Status;
	uint32_t ip0;
	uint16_t ip1,ip2;
	static uint8_t count = 0;
//	ASIC_DACAndInspectFlag = 0;
	switch(get_working_stage()){
	case STAGE_IDLE:
	{
		if(0 == count){
			ASIC_InitSensor(SPI0);
			ASIC_InitSensor(SPI0);
			ASIC_SetIp012(SPI0,0,0,0);
			ASIC_ClearOBDreg(SPI0);
#if ONSEMI_SCHEME
			DelayUS(500);
#else
			DelayUS(1000);
#endif
			ASIC_SensorInspect(SPI0);
#if ONSEMI_SCHEME
			DelayUS(800);
#else
			DelayUS(1000);
#endif
			ASIC_FliterAndADCEnable(SPI0);
			DelayUS(1500);
			ASIC_ReadMeasurementResult(SPI0,adc_value);
			Value_Dispose();
		}
		count = (count+1)%10;
	}
	break;
	case STAGE_PREHEATING:
	if(0 == count){
		ASIC_InitSensor(SPI0);
		ASIC_InitSensor(SPI0);
		ASIC_SetIp012(SPI0,0,0,0);
		ASIC_ClearOBDreg(SPI0);
#if ONSEMI_SCHEME
		DelayUS(500);
#else
		DelayUS(1000);
#endif
		ASIC_SensorInspect(SPI0);
	}
	count = (count+1)%10;
	break;
	case STAGE_HEATING:
	{
		ASIC_InitSensor(SPI0);
		ASIC_InitSensor(SPI0);
		ASIC_SetIp012(SPI0,0,0,0);
		ASIC_ClearOBDreg(SPI0);
#if ONSEMI_SCHEME
		DelayUS(500);
#else
		DelayUS(1000);
#endif
		ASIC_SensorInspect(SPI0);
	}
	break;
	case STAGE_CRITICALPOINT:
	{
		Survey_Ready();
		get_ip012(&ip0,&ip1,&ip2);

		ASIC_InitRef(SPI0);
		if(Sensor_Status == Sensor_RESET){
			ASIC_SensorEnable(SPI0);
			DelayUS(100);
		}
		ASIC_SetIp012(SPI0,ip0,ip1,ip2);
		ASIC_ClearOBDreg(SPI0);
#if ONSEMI_SCHEME
		DelayUS(500);
#else
		DelayUS(1000);
#endif
		ASIC_SensorInspect(SPI0);
#if DBFLAG
		DBFSTimemr = Gets_Clock_value();
		DBFS = 1;
#endif
//		ADC_Value_Updata++;
	}
	break;
	case STAGE_ENVIRONMENT:
	{
/*
#if DBFLAG
		if(DBFS == 1){
			ASIC_OutREF(SPI0);
			if(clock_time_exceed(DBFSTimemr,7500)){
				DBFS = 0;
				DBFSTimemr = 0;
			}
		}else if(DBFS == 0){
			ASIC_InREF(SPI0);
			DBFS = 2;
		}
#endif
*/

#if DBFLAG
		if(DBFS == 1){
			ASIC_RefEnable(SPI0);
			DelayUS(250);
			ASIC_InREF(SPI0);
			//ASIC_OutREF(SPI0);
			//DelayUS(250);
			ASIC_InitRef(SPI0);
			ASIC_InitRef(SPI0);
			if(clock_time_exceed(DBFSTimemr,15000)){
				DBFSTimemr = 0;
				DBFS = 2;
				ASIC_InREF(SPI0);
			}
		}
#endif

		if(DBFS == 2){
			if(clock_time_exceed(Start_Timer, 7*1000)){
				NoPIDflag = 0;
			}
			Survey_Controller();

			if(Sensor_Status == Sensor_RESET){
				ASIC_SensorEnable(SPI0);
				DelayUS(100);
			}
			get_ip012(&ip0,&ip1,&ip2);
			//ASIC_ResetOBDreg(SPI0);
			ASIC_SetIp012(SPI0,ip0,ip1,ip2);

			ASIC_ClearOBDreg(SPI0);
#if ONSEMI_SCHEME
			DelayUS(500);
#else
			DelayUS(1350);
#endif
			ASIC_SensorInspect(SPI0);
			DelayUS(1475);
			//ADC_Value_Updata ++;
		}
	}
	break;
	case STAGE_TESTSTAGE:{
		get_ip012(&ip0,&ip1,&ip2);

		if(Sensor_Status == Sensor_RESET){
			ASIC_SensorEnable(SPI0);
			DelayUS(100);
		}
//		ASIC_ResetOBDreg(SPI0);
		ASIC_SetIp012(SPI0,ip0,ip1,ip2);
		ASIC_ClearOBDreg(SPI0);
#if ONSEMI_SCHEME
		DelayUS(500);
#else
		DelayUS(1000);
#endif
		ASIC_SensorInspect(SPI0);

//		ADC_Value_Updata ++;
	}
	break;
	case STAGE_TESTSTAGE2:{
		get_ip012(&ip0,&ip1,&ip2);
//		ASIC_ResetOBDreg(SPI0);
		ASIC_SetIp012(SPI0,ip0,ip1,ip2);
		ASIC_ClearOBDreg(SPI0);
#if ONSEMI_SCHEME
		DelayUS(500);
#else
		DelayUS(1000);
#endif
		ASIC_SensorInspect(SPI0);

//		ADC_Value_Updata ++;
	}
	break;
	case STAGE_InspectError:{
		ASIC_ClearOBDreg(SPI0);
#if ONSEMI_SCHEME
		DelayUS(500);
#else
		DelayUS(1000);
#endif
		ASIC_SensorInspect(SPI0);
	}
	break;
	default:
	{}
	break;
	}
}






void ASIC_ReadMeasurementResultHandler(void){					//���Ͱ�ɭ�����ݲɼ�����
	static uint8_t count = 0;//,heatinit = 1;
//	ASIC_ReadMeasurementResultFlag = 0;
	switch(get_working_stage()){
	case STAGE_IDLE:
	break;
	case STAGE_PREHEATING:
		RTC_STAGE1:
	if(0 == count){
		ASIC_FliterAndADCEnable(SPI0);
		DelayUS(1000);
		ASIC_ReadMeasurementResult(SPI0,adc_value);
		Value_Dispose();
	}
	count = (count+1)%10;
	break;
	case STAGE_HEATING:
	{
		RTC_STAGE2:
		ASIC_FliterAndADCEnable(SPI0);
		DelayUS(1000);
		ASIC_ReadMeasurementResult(SPI0,adc_value);
		Value_Dispose();
	}
	break;
	case STAGE_CRITICALPOINT:
	{
		RTC_STAGE3:
		ASIC_FliterAndADCEnable(SPI0);
#if ONSEMI_SCHEME
		DelayUS(500);
		ASIC_RefEnable(SPI0);
		DelayUS(250);
#else
		DelayUS(1000);
		ASIC_RefEnable(SPI0);
		DelayUS(500);
#endif
		ASIC_ReadMeasurementResult(SPI0,adc_value);
		Value_Dispose();
#if ONSEMI_SCHEME
		DelayUS(250);
#else
		DelayUS(500);
#endif
		ASIC_InitRef(SPI0);
		ASIC_InitRef(SPI0);
	}
	break;
	case STAGE_ENVIRONMENT:
	{
		RTC_STAGE4:
		ASIC_FliterAndADCEnable(SPI0);
#if ONSEMI_SCHEME
		DelayUS(500);
#else
		DelayUS(1620);
#endif
		ASIC_RefEnable(SPI0);
//		ASIC_ResetOBDreg(SPI0);
#if ONSEMI_SCHEME
		DelayUS(250);
#else
		DelayUS(2260);
#endif
		ASIC_ReadMeasurementResult(SPI0,adc_value);
		Value_Dispose();
		//printstring("\r\n measure :");printfloat(ADtf_Value.VTempValue);
		/*
		if(heatinit){
			heatinit = 0;
			Heat_KalmanFilter(ADtf_Value.VTempValue,0);
		}else{
			ADtf_Value.VTempValue = Heat_KalmanFilter(ADtf_Value.VTempValue,1);
			//printstring("\r\n Kalman :");printfloat(ADtf_Value.VTempValue);
		}
		*/
		//printstring("\r\nadc_value[3] :"); printf("0x%04X",adc_value[3]);
		//printf("ADtf_Value.VCommValue : %f\n", ADtf_Value.VCommValue);
		//printstring("\r\nADtf_Value.VCommValue :"); printfloat(ADtf_Value.VCommValue);
#if ONSEMI_SCHEME
		DelayUS(250);
#else
		DelayUS(300);
#endif
		ASIC_InitRef(SPI0);
		ASIC_InitRef(SPI0);
	}
	break;
	case STAGE_TESTSTAGE:{
		RTC_STAGE5:
		ASIC_FliterAndADCEnable(SPI0);
		DelayUS(500);
		ASIC_RefEnable(SPI0);
//		ASIC_ResetOBDreg(SPI0);
		DelayUS(250);
		ASIC_ReadMeasurementResult(SPI0,adc_value);
		Value_Dispose();
		DelayUS(250);
		ASIC_InitRef(SPI0);
		ASIC_InitRef(SPI0);
	}
	break;
	case STAGE_TESTSTAGE2:{
		RTC_STAGE6:
		ASIC_FliterAndADCEnable(SPI0);
		DelayUS(500);
		ASIC_RefEnable(SPI0);
//		ASIC_ResetOBDreg(SPI0);
		DelayUS(250);
		ASIC_ReadMeasurementResult(SPI0,adc_value);
		Value_Dispose();
		DelayUS(250);
		ASIC_InitRef(SPI0);
		ASIC_InitRef(SPI0);
	}
	break;
	case STAGE_InspectError:{
		switch(get_pre_working_stage()){
		case STAGE_PREHEATING:
			goto RTC_STAGE1;
		case STAGE_HEATING:
			goto RTC_STAGE2;
		case STAGE_CRITICALPOINT:
			goto RTC_STAGE3;
		case STAGE_ENVIRONMENT:
			goto RTC_STAGE4;
		case STAGE_TESTSTAGE:
			goto RTC_STAGE5;
		case STAGE_TESTSTAGE2:
			goto RTC_STAGE6;
		case STAGE_IDLE:
		case STAGE_InspectError:
		default:
			break;
		}
	}
	default:
	{}
	break;
	}
}

void FTM2_Task(void){
	if(FTM_GetOverFlowFlag(FTM2) && ASIC_Flag){
		FTM_ClrOverFlowFlag(FTM2);
		static uint8_t flag = 1;
		if(flag){
			PIT0_init();
			flag = 0;
		}

#if ONSEMI_SCHEME
		ASIC_DACAndInspectHandler();
#endif
	}
}
void PIT_Task0(void)
{
    Disable_Interrupt(PIT_CH0_IRQn);
    PIT_SetLoadVal(0,0);
    PIT_ChannelDisable(0);
    PIT_ChannelDisableInt(0);
    PIT_ChannelDisableChain(0);
    PIT_ChannelClrFlags(0);
    PIT1_init();
}

void PIT0_init(void){
    PIT_ConfigType  PIT_Config0 ={0};

    /* configure PIT module in chain mode */
    /* PIT clock source is bus clock,24MHz */
    /* PIT channel 0 load value = (2400000-1), channel 1 load value = (10-1) */

    /* configure PIT channel 0, only enable timer */
#if ONSEMI_SCHEME
    PIT_Config0.u32LoadValue      = 215999;  //9ms
#else
    PIT_Config0.u32LoadValue      = 119999;
#endif
    PIT_Config0.bFreeze           = FALSE;
    PIT_Config0.bModuleDis        = FALSE;    /*!< enable PIT module */
    PIT_Config0.bInterruptEn      = TRUE;
    PIT_Config0.bChainMode        = FALSE;
    PIT_Config0.bTimerEn          = TRUE;

    PIT_SetCallback(PIT_CHANNEL0, PIT_Task0);
    PIT_Init(PIT_CHANNEL0, &PIT_Config0);
}

void PIT_Task1(void)
{
	ASIC_Flag = 0;
#if ONSEMI_SCHEME == 0
	ASIC_DACAndInspectHandler();
#endif
	ASIC_ReadMeasurementResultHandler();
	ASIC_Flag = 1;
}

void PIT1_init(void){
    PIT_ConfigType  PIT_Config1 ={0};

    /* configure PIT module in chain mode */
    /* PIT clock source is bus clock,24MHz */
    /* PIT channel 0 load value = (2400000-1), channel 1 load value = (10-1) */

    /* configure PIT channel 1 in chain mode, enable interrupt and timer */
    PIT_Config1.u32LoadValue      = 239999;//10ms
    PIT_Config1.bFreeze           = FALSE;
    PIT_Config1.bModuleDis        = FALSE;    /*!< enable PIT module */
    PIT_Config1.bInterruptEn      = TRUE;
    PIT_Config1.bChainMode        = FALSE;
    PIT_Config1.bTimerEn          = TRUE;

    PIT_SetCallback(PIT_CHANNEL1, PIT_Task1);
    PIT_Init(PIT_CHANNEL1, &PIT_Config1);
}



void MCLK_init(void)
{
	FTM_ConfigType FTM0_Config={0};
	FTM_ChParamsType FTM0CH1_Config={0};

	FTM0_Config.modulo=23;							 /* The Frequency equal to CLK/4 */
	FTM0_Config.clk_source=FTM_CLOCK_SYSTEMCLOCK;
	FTM0_Config.prescaler=FTM_CLOCK_PS_DIV1;//1MHZ
	FTM0_Config.toie=0;

	SIM_PINSEL0 |= SIM_PINSEL_FTM0PS1(1); /* Select Pins corresponds to the PTB3 for output */

	FTM0CH1_Config.ctrl.bits.bMode=FTM_PWMMODE_EDGEALLIGNED;
	FTM0CH1_Config.ctrl.bits.bPWMPol=FTM_PWM_HIGHTRUEPULSE;
	FTM0CH1_Config.u16CnV=12;//ռ�ձ�0.5

//	FTM_SetCallback(FTM0, FTM_Task);
	FTM_ChannelInit(FTM0,1,FTM0CH1_Config);
	FTM_Init(FTM0,&FTM0_Config);
#ifdef DEBUG
	printf("\r\nASIC Extern MCLK initialization successful.\r\n");
#endif
}


void get_ip012(uint32_t* ip0,uint16_t* ip1,uint16_t* ip2){
	if(!HEXorDEC){
		if(Ip_Value.Ip0_Value < 0){
			*ip0 = -Ip_Value.Ip0_Value / 0.0988;
			*ip0 |=	IP0_EN_MASK;
			*ip0 = *ip0 > IP0_MINVALUE ? IP0_MINVALUE : *ip0;
		}else{
			*ip0 = Ip_Value.Ip0_Value / 0.0988;
			*ip0 = *ip0 > IP0_MAXVALUE ? IP0_MAXVALUE : *ip0;
		}
		if(Ip_Value.Ip1_Value < 0){
			*ip1 = -Ip_Value.Ip1_Value / 0.0196;
			*ip1 |=	IP1_EN_MASK;
			*ip1 = *ip1 > IP1_MINVALUE ? IP1_MINVALUE : *ip1;
		}else{
			*ip1 = Ip_Value.Ip1_Value / 0.0196;
			*ip1 = *ip1 > IP1_MAXVALUE ? IP1_MAXVALUE : *ip1;
		}
		if(Ip_Value.Ip2_Value < 0){
			*ip2 = -Ip_Value.Ip2_Value / 0.00142;
			*ip2 |= IP2_EN_MASK;
			*ip2 = *ip2 > IP2_MINVALUE ? IP2_MINVALUE : *ip2;
		}else{
			*ip2 = Ip_Value.Ip2_Value / 0.00142;
			*ip2 = *ip2 > IP2_MAXVALUE ? IP2_MAXVALUE : *ip2;
		}
	}else{
		*ip0 = Ip_Value.Ip0_Value;
		*ip1 = Ip_Value.Ip1_Value;
		*ip2 = Ip_Value.Ip2_Value;
	}
	*ip0 = (*ip0 & (IP0_BR_2_0_MASK | IP0_IP0_S_MASK)) | ((*ip0<<(IP0_BC_1_0_SHIFT-7)) & (IP0_AC_0_MASK | IP0_AR_3_0_MASK | IP0_BC_1_0_MASK)) | ((*ip0<<(IP0_AC_2_1_SHIFT-14)) & (IP0_EN__MASK | IP0_AC_2_1_MASK));

	*ip1 = (*ip1 & (IP1_BC_0_MASK | IP1_BR_1_0_MASK | IP1_IP1_S_MASK)) | ((*ip1<<(IP1_BC_1_SHIFT-7)) & (IP1_EN__MASK | IP1_AC_1_0_MASK | IP1_AR_2_0_MASK | IP1_BC_1_MASK));

	*ip2 = ((*ip2<<1)&IP2_INS_D3_6_MASK) | ((*ip2<<1)&IP2_INS_D3_5_0_MASK) | ((*ip2&IP2_INS_D2_6_0_MASK));
}

void spi_start(void)
{
	int16_t Rx_data[6];
//	__IO ASIC_StatusTypeDef Status;

	ASIC_InitSensor(SPI0);
	ASIC_InitSensor(SPI0);
	ASIC_SetIp012(SPI0,0,0,0);
	DelayUS(1000);
	ASIC_ReadMeasurementResult(SPI0,Rx_data);
	DelayUS(1000);
	ASIC_SensorInspect(SPI0);

	ASIC_InitSensor(SPI0);
	ASIC_InitSensor(SPI0);
	ASIC_SetIp012(SPI0,0,0,0);
	DelayUS(3000);

	ASIC_InitSensor(SPI0);
	ASIC_InitSensor(SPI0);
	ASIC_SetIp012(SPI0,0,0,0);
	DelayUS(1000);
	ASIC_InitSensor(SPI0);
	ASIC_InitSensor(SPI0);
	ASIC_SetIp012(SPI0,0,0,0);

	ASIC_FliterAndADCEnable(SPI0);
	DelayUS(2000);
	ASIC_ReadMeasurementResult(SPI0,Rx_data);
	DelayUS(2000);
	ASIC_SensorInspect(SPI0);

	ASIC_ResetReg(SPI0);
	ASIC_ConfigCLK(SPI0,I_OSC1);
//	PIT_Enable();
}


void Value_Dispose(void)		//���ݽ���������AD��ȡֵ
{
	ADtf_Value.Vref0Value = (float)adc_value[0]*IP0_ADC_COEF/8192;
	ADtf_Value.Vref1Value = (float)adc_value[1]*IP1_ADC_COEF/8192;
	ADtf_Value.Vref2Value = (float)adc_value[2]*IP2_ADC_COEF/8192;
	ADtf_Value.VCommValue = (float)adc_value[3]*COMM_ADC_COEF/8192;
	ADtf_Value.VTempValue = (float)adc_value[4]*TEMP_ADC_COEF/8192;
	ADtf_Value.VTestValue = (float)adc_value[5]*TEST_ADC_COEF/8192;
	Vref = ADtf_Value.VTestValue * 21 / 5;
}

void Survey_Ready(void)
{
	measureCoeffi_Typedef* PCLT = CLT_pointer();
	Vref0.E_2 = Vref0.E_1;
	Vref0.E_1 = Vref0.E;
	Vref0.E = Vref0_V - ADtf_Value.Vref0Value;

	Vref1.E_2 = Vref1.E_1;
	Vref1.E_1 = Vref1.E;
	Vref1.E = PCLT->Vref1 - ADtf_Value.Vref1Value;

	Vref2.E_2 = Vref2.E_1;
	Vref2.E_1 = Vref2.E;
	Vref2.E = PCLT->Vref2 - ADtf_Value.Vref2Value;

#if VREF0_METHOD == 2
	IP1tab.E_2 = IP1tab.E_1;
	IP1tab.E_1 = IP1tab.E;
	IP1tab.E = PCLT->Ip1 - Ip_Value.Ip1_Value;
#else
	IP1.E_2 = IP1.E_1;
	IP1.E_1 = IP1.E;
	IP1.E = PCLT->Ip1 - Ip_Value.Ip1_Value;

#endif
}


void PID_init(void)
{
#if VREF0_METHOD == 1
	IP1.Kp = 0.001;
	IP1.Ki = 0.00002;
	IP1.Kd = 0.0002;
	IP1.Ke = IP0EMax/3;
	IP1.Kde = IP0dEmax/3;
	IP1.Ku_p = IP0dKpMax/3;
	IP1.Ku_i = IP0dKiMax/3;
	IP1.Ku_d = IP0dKdMax/3;
#endif
//	IP1.Kp = 0.5;
//	IP1.Ki = 0.02;
//	IP1.Kd = 0.1;

//	Vref0.Kp = 100;
//	Vref0.Ki = 12;
//	Vref0.Kd = 100;
//
//	Vref1.Kp = 250;
//	Vref1.Ki = 30;
//	Vref1.Kd = 100;
//
//	Vref2.Kp = 3;
//	Vref2.Ki = 1;
//	Vref2.Kd = 5;
}
float PID_func(float Target, float Current, SurveyVariate_Type* Vref){
	float absE,inc;
	int section,id;
	if(Vref == &Vref0){
		if(Hysteresis_Comparator(Ip_Value.Ip0_Value,950,1050)){
			id = 5;//7776
		}else{
			id = 0;//4500
		}

	}else if(Vref == &Vref1){
		id = 1;
	}else if(Vref == &Vref2){
		if(Hysteresis_Comparator(Ip_Value.Ip2_Value,950,1050)){
			id = 6;
		}else{
			id = 2;
		}
	}
#if VREF0_METHOD == 0
	else if(Vref == &IP1){
		id = 3;
	}
#endif
	else{
		id = 4;
	}

	Vref->E_2 = Vref->E_1;
	Vref->E_1 = Vref->E;
	Vref->E = Target - Current;

	absE = myabs(Vref->E);
	if(absE > Vrefbound[0]){
		if(absE > Vrefbound[1]){
			if(absE > Vrefbound[2]){
				if(absE > Vrefbound[3]){
					section = 4;
				}else{
					section = 3;
				}
			}else{
				section = 2;
			}
		}else{
			section = 1;
		}
	}else{
		section = 0;
	}
	if(Current >= 0.4999 && (id == 1 || id == 2)){
		section = 3;
	}

	Vref->Kp = Kp[id][section];
	Vref->Ki = Ki[id][section];
	Vref->Kd = Kd[id][section];

	if(id == 0 || id == 5){
		Vref->Ki = Ip_Value.Ip0_Value;
		Vref->Ki = Vref->Ki < 3500 ? (Vref->Ki > 500 ? Vref->Ki : 500) : 3500;
	}

	inc = (Vref->Kp+Vref->Ki+Vref->Kd)*Vref->E - (Vref->Kp+2*Vref->Kd)*Vref->E_1 + Vref->Kd*Vref->E_2;
	return inc;
}

void Survey_Controller(void){
//===================================================================================================================
#if VREF0_METHOD == 1
		IP1.E_2 = IP1.E_1;
		IP1.E_1 = IP1.E;
		IP1.E = CLT_pointer()->Ip1 - Ip_Value.Ip1_Value;
		IP1.dE = IP1.E - IP1.E_1;
		fuzzy_Controller(&IP1);			//ģ������Ӧ�������Զ�����IP1.Ki��ֵ

	//	if(myabs(IP1.E) > 0.1){
		float Vref0_inc;
		Vref0_inc = (IP1.Kp+IP1.Ki+IP1.Kd)*IP1.E - (IP1.Kp+2*IP1.Kd)*IP1.E_1 + IP1.Kd*IP1.E_2;
		if(myabs(Vref0.E) > 0.1){			//Vref0δ����Ŀ��ֵʱ  �Ż���Vref0Ŀ��ֵ�ı仯
			Vref0_inc /= 2.0;
		}else if(myabs(IP1.dE) < 1){		//Ip1������Сʱ����Ip1�����ȶ���������PID�������ӿ�Ip1�ջ���7uA���ٶ�
			if(IP1.E*Vref0_inc > 0){
				Vref0_inc *= 3;
			}
			if(ADtf_Value.Vref0Value > CLT_pointer()->Vref0Limit){
				Vref0_V = CLT_pointer()->Vref0Limit;
			}
		}
		Vref0_V -= Vref0_inc;
		Vref0_V = Vref0_V < Vref0_MaxValue ? (Vref0_V > Vref0_MinValue ? Vref0_V : Vref0_MinValue) : Vref0_MaxValue;
	//	}


#elif VREF0_METHOD == 0
	static float detal_Vref0 = 0;
	float Vref0_inc;
	Vref0_inc = PID_func(IP1.E, IP1.E_1, &IP1);
	detal_Vref0 -= Vref0_inc;
//	Vref0_V = VCommValue * 1.2293 + 0.45 + detal_Vref0;
	Vref0_V = CLT_pointer()->Vref0Limit;
	Vref0_V = Vref0_V < Vref0_MaxValue ? (Vref0_V > Vref0_MinValue ? Vref0_V : Vref0_MinValue) : Vref0_MaxValue;



#elif VREF0_METHOD == 2

	float Vref0_inc1;
	Vref0_inc1 = PID_func(CLT_pointer()->Ip1, Ip_Value.Ip1_Value, &IP1tab);
	Vref0_inc1 = Vref0_inc1 < IP1controlVref0_MaxValue ? (Vref0_inc1 > IP1controlVref0_MinValue ? Vref0_inc1 : IP1controlVref0_MinValue) : IP1controlVref0_MaxValue;
	//Vref0_V = ((float)Lookup_V0table(Ip_Value.Ip0_Value,Ip_Value.Ip1_Value))/1000 + Vref0_inc1;
	Vref0_V = ((float)Lookup_table(Ip_Value.Ip0_Value,Ip_Value.Ip1_Value,get_IPTable()))/1000 + Vref0_inc1;

	Vref0_V = Vref0_V < Vref0_MaxValue ? (Vref0_V > Vref0_MinValue ? Vref0_V : Vref0_MinValue) : Vref0_MaxValue;
	push(&Vref0Totle,Vref0_V);
	Vref0_V = getavg(&Vref0Totle);
#endif


//===================================================================================================================
	float Ip0_inc;
	if(NoPIDflag){
		Ip0_inc = PID_func(0.6, ADtf_Value.Vref0Value, &Vref0);
	}else{
		Ip0_inc = PID_func(Vref0_V, ADtf_Value.Vref0Value, &Vref0);
	}
//	Ip0_inc = Ip0_inc > 0.5 ? 0.5 : (Ip0_inc < -0.5 ? -0.5 : Ip0_inc);
	Ip_Value.Ip0_Value += Ip0_inc;
	Ip_Value.Ip0_Value = Ip_Value.Ip0_Value < Ip0_MaxValue ? (Ip_Value.Ip0_Value > Ip0_MinValue ? Ip_Value.Ip0_Value : Ip0_MinValue) : Ip0_MaxValue;
//===================================================================================================================
#if VREF1_METHOD == 0

	float Ip1_inc;
	if(NoPIDflag){
		Ip1_inc = PID_func(0.6, ADtf_Value.Vref1Value, &Vref1);
	}else{
		Ip1_inc = PID_func(CLT_pointer()->Vref1, ADtf_Value.Vref1Value, &Vref1);
	}
//	Ip1_inc = Ip1_inc > 0.05 ? 0.05 : (Ip1_inc < -0.05 ? -0.05 : Ip1_inc);
	Ip_Value.Ip1_Value += Ip1_inc;
	Ip_Value.Ip1_Value = Ip_Value.Ip1_Value < Ip1_MaxValue ? (Ip_Value.Ip1_Value > Ip1_MinValue ? Ip_Value.Ip1_Value : Ip1_MinValue) : Ip1_MaxValue;
#elif VREF1_METHOD == 1
	float Ip1_inc;
	Vref1_V = ((float)Lookup_V1table(Vref0_V*1000,Ip_Value.Ip1_Value))/1000;
	push(&Vref1Totle,Vref1_V);
	Vref1_V = getavg(&Vref1Totle);
	Ip1_inc = PID_func(Vref1_V, ADtf_Value.Vref1Value, &Vref1);
	Ip_Value.Ip1_Value += Ip1_inc;
	Ip_Value.Ip1_Value = Ip_Value.Ip1_Value < Ip1_MaxValue ? (Ip_Value.Ip1_Value > Ip1_MinValue ? Ip_Value.Ip1_Value : Ip1_MinValue) : Ip1_MaxValue;
#endif

//===================================================================================================================
	float Ip2_inc;

	if(NoPIDflag){
		Ip2_inc = PID_func(0.6, ADtf_Value.Vref2Value, &Vref2);
	}else{
		Ip2_inc = PID_func(CLT_pointer()->Vref2, ADtf_Value.Vref2Value, &Vref2);
	}
	if(Ip_Value.Ip2_Value > 0){
		Ip2_inc = Ip2_inc > 0.03 ? (Ip2_inc - 0.03) / 10 + 0.03 : Ip2_inc;
	}
//	Ip2_inc = Ip2_inc > 0.01 ? 0.01 : (Ip2_inc < -0.01 ? -0.01 : Ip2_inc);
	Ip_Value.Ip2_Value += Ip2_inc;
	Ip_Value.Ip2_Value = Ip_Value.Ip2_Value < Ip2_MaxValue ? (Ip_Value.Ip2_Value > Ip2_MinValue ? Ip_Value.Ip2_Value : Ip2_MinValue) : Ip2_MaxValue;

//===================================================================================================================
//	if(Vref0Value > 0.35){				//�ж�Vref0����350mV������ʱ�������������350mV
//		count1++;
//		if(count1>100 * 30){
//			Vref0_V = 0.35;
//			count1 = 0;
//		}
//	}else{
//		count1 = 0;
//	}


#ifdef CAN_CONTROL
#if Filter_type == AvgFilter
	static uint8_t Vref2_Stable = 0;

	push(&Ip0Totle,Ip_Value.Ip0_Value);

	if(ADtf_Value.Vref2Value < 0.5 && ADtf_Value.Vref2Value > 0.35){
		if(Vref2_Stable < 10){
			Vref2_Stable++;
		}
	}else{
		if(Vref2_Stable > 0){
			Vref2_Stable--;
		}
	}

	if(Vref2_Stable > 5){
		push(&Ip2Totle,Ip_Value.Ip2_Value);
	}


#elif Filter_type == LowFilter
	filter_input(&Ip0Filter, Ip_Value.Ip0_Value);
	filter_input(&Ip2Filter, Ip_Value.Ip2_Value);
#elif Filter_type == DF2Filter
	filter_input(&Ip0Filter, Ip_Value.Ip0_Value);
	filter_input(&Ip2Filter, Ip_Value.Ip2_Value);
#endif
#endif
}

void sensor_init(void){
	MCLK_init();//��ʼ��MCLK
	ASIC_SPI_Init(SPI0);
	spi_start();//����ONSEMEоƬ
	FTM_SetCallback(FTM2, FTM2_Task);
	PID_init();//PID������ʼ��
}


