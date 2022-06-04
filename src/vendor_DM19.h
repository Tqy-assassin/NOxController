/*
 * vendor_DM19.h
 *
 *  Created on: 2022年6月4日
 *      Author: Administrator
 */

#ifndef VENDOR_NTK_H_
#define VENDOR_NTK_H_

#include "config.h"
#if VENDOR_ID == DM19_KIND
#include "common.h"
#include "can_api.h"

#define CANTx_INTERVAL	50	//(ms)
#define CAN_BAUDRATE_250kbps
//===========SourceAddr====================================

enum{
	SourceAddrATI1 = 0x51,
	SourceAddrATO1 = 0x52,
	SourceAddrATI2 = 0x56,
	SourceAddrATO2 = 0x57
}J1939_Addr;
#define DefaultSourceAddr	SourceAddrATO1

//=========================================================
typedef union{
	struct{
		uint8_t SupplyinRange :2;
		uint8_t TempHeaterElement :2;
		uint8_t NOxSignal :2;
		uint8_t O2Signal :2;
	}status;
	uint8_t byte;
}StatusByte_t;
typedef struct{
	union{
		struct{
			uint16_t NOx;
			uint16_t O2;
			struct{
				uint8_t SupplyinRange :2;//�ֽڵ�
				uint8_t TempHeaterElement :2;
				uint8_t NOxSignal :2;
				uint8_t O2Signal :2;//�ֽڸ�
			}StatusByte;
			uint8_t ErrorHeater : 5;
			uint8_t StatusHeaterMode : 2;
			uint8_t Noused_1 :1;
			uint8_t ErrorNOx :5;
			uint8_t Diagnosisfeedback  :3;		//����
			uint8_t ErrorO2  :5;
			uint8_t Noused_3 :3;
		}TxFrame;
		uint8_t TxData[8];
	};
}TramsmitFrame_TypeDef;

typedef struct{
	union{
		struct{
			uint8_t CorrectPressureLambda;
			uint8_t CorrectPressureNOx;
			uint8_t NO2Correction;
			uint8_t NH3Correction;
			uint8_t SelfDiagnosisResultValue;
			uint8_t Noused_1;
			uint8_t Noused_2;
			uint8_t Noused_3;
		}TxFrame;
		uint8_t TxData[8];
	};
}CorrectO2NOxTFrame_TypeDef;

typedef struct{
	union{
		struct{
			uint8_t HeaterRatio_L;//0byte
			uint8_t HeaterRatio_H;//1byte
			uint8_t NOx_corr_gain_L;
			uint8_t NOx_corr_gain_H;
			uint8_t NOx_corr_off;
			uint8_t OperationHoursCounter_L;
			uint8_t OperationHoursCounter_H;
			uint8_t Noused;
		}TxFrame;
		uint8_t TxData[8];
	};
}HeaterRatioTFrame_TypeDef;

typedef struct{
	union{
		struct{
			uint8_t Torque;
			uint8_t Speed_L;
			uint8_t Speed_H;
			uint8_t SpeedAdjusting;
			uint8_t TorqueLosses;
			uint8_t AfterTreatment1ExhaustGasMassFlow_L;
			uint8_t AfterTreatment1ExhaustGasMassFlow_H;
			struct{
				uint8_t AfterTreatment1IntakeGas :2;
				uint8_t AfterTreatment1OutletGas :2;
				uint8_t AfterTreatment2IntakeGas :2;
				uint8_t AfterTreatment2OutletGas :2;
			}startCode;
		}RxFrame;
		uint8_t RxData[8];
	};
}CANRxFrameDataType;

enum{
	DewPointNoReached = 0,
	DewPointReached = 1,
}startCode;

//=====================Status-Byte==================================
#define SupplynInRange	0
#define SupplyInRange	1
#define SensornAtTemper	0
#define SensorAtTemper	1
#define NOxsignalnValid	0
#define NOxsignalValid	1
#define O2signalnValid	0
#define O2signalValid	1
#define Notused			2
#define Notavaliable	3

//=====================StatusHeaterMode==============================
#define AutomaticMode	0
#define HeatupSlope3or4	1
#define HeatupSlope1or2	2
#define HeaterOfforPre	3

//========================Error=======================================
#define NoError			0x1F
#define OpenWire		0x05
#define ShortCircuit	0x03
typedef struct{
	uint16_t IfSupplyInRange :1;
	uint16_t IfAtTemperature :1;
	uint16_t IfNOxValid		:1;
	uint16_t IfO2Valid		:1;
	uint16_t Res			:4;
	uint16_t HeaterMode		:2;
	uint16_t HeaterError	:2;
	uint16_t NOxError		:2;
	uint16_t O2Error		:2;
}J1939_SensorStatus;

//=========================================================
enum{
	DataAllAddr = 0xEAFF,

	TransmitDataATI1  = 0xF00E,
	TransmitDataATO1  = 0xF00F,
	TransmitDataATI2  = 0xF010,
	TransmitDataATO2  = 0xF011,

	CorrectO2NOxATI1R = 0xEA51,
	CorrectO2NOxATO1R = 0xEA52,
	CorrectO2NOxATI2R = 0xEA56,
	CorrectO2NOxATO2R = 0xEA57,

	CorrectO2NOxATI1T = 0xFD10,
	CorrectO2NOxATO1T = 0xFD0E,
	CorrectO2NOxATI2T = 0xFD0C,
	CorrectO2NOxATO2T = 0xFD0A,

	HeaterRatioATI1T = 0xFD11,
	HeaterRatioATO1T = 0xFD0F,
	HeaterRatioATI2T = 0xFD0D,
	HeaterRatioATO2T = 0xFD0B,

	TriggerforSelfDiagnosis = 0xFCCF,

	ReceiveSignal     = 65247,//FEDF
	TransmitMessage   = 61680,	//

	PrivateCmd_Intake = 0x5A99,
	PrivateCmd_Outlet = 0x99A5,
}J1939_PGN;

#define DewPoint_Invalid 	0
#define DewPoint_Start		1
#define DewPoint_Stop   	2
//=========================================================
extern void CAN_RxTask(uint32_t PGN, CANRxFrameDataType* RxFrame);
extern void InspectResultAnalysis(uint8_t* InspectResult);
extern void JudgeDeviceType(void);
extern void CAN_TxTask();
extern float getTorque(CANRxFrameDataType* RxFrame);
extern float getSpeed(CANRxFrameDataType* RxFrame);
extern float getSpeedAdjust(CANRxFrameDataType* RxFrame);
extern float getTorqueLosses(CANRxFrameDataType* RxFrame);
extern float getGasMassFlow(CANRxFrameDataType* RxFrame);
extern void Status_HeaterOff(void);
extern void Status_PreHeater(void);
extern void Status_Heating(void);
extern void Status_HeatCriticalPoint(void);
extern void Status_AutomaticHeat(void);
extern void Status_NOxValid(void);
extern void Status_NOxnValid(void);
extern void Status_O2Valid(void);
extern void Status_O2nValid(void);
extern void Status_AtTemperature(void);
extern void Status_nAtTemperature(void);
extern uint8_t Status_Get(void);
extern void CAN_Heaterratio_deviation_Transmit(uint16_t cmdid);
extern void CAN_TxDATA2(void);
extern void CAN_RxManyPostback(void);
extern void CAN_DeclareAddress(void);

enum{
	InspectBit0 = 1,
	InspectBit1 = 2,
	InspectBit2 = 4,
	InspectBit3 = 8,
	InspectBit4 = 16,
	InspectBit5 = 32,
	InspectBit6 = 64,
	InspectBit7 = 128
}InspectTypeDef;
#endif /* VENDOR_ID == NTK_KIND*/

#endif /* VENDOR_NTK_H_ */
