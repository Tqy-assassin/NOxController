/*
 * vendor.h
 *
 *  Created on: 2020��12��16��
 *      Author: tianqingyuan
 */

#ifndef VENDOR_H_
#define VENDOR_H_

//=========================================================
#define PDU_P_MASK		0x1C000000
#define PDU_P_SHIFT		26
#define PDU_P(x)		(((uint32_t)(((uint32_t)(x))<<PDU_P_SHIFT))&PDU_P_MASK)
#define PDU_R_MASK		0x02000000
#define PDU_R_SHIFT		25
#define PDU_R(x)		(((uint32_t)(((uint32_t)(x))<<PDU_R_SHIFT))&PDU_R_MASK)
#define PDU_DP_MASK		0x01000000
#define PDU_DP_SHIFT	24
#define PDU_DP(x)		(((uint32_t)(((uint32_t)(x))<<PDU_DP_SHIFT))&PDU_DP_MASK)
#define PDU_PGN_MASK	0x00FFFF00
#define PDU_PGN_SHIFT	8
#define PDU_PGN(x)		(((uint32_t)(((uint32_t)(x))<<PDU_PGN_SHIFT))&PDU_PGN_MASK)
#define PDU_SA_MASK		0x000000FF
#define PDU_SA_SHIFT	0
#define PDU_SA(x)		(((uint32_t)(x))&PDU_SA_MASK)

#if VENDOR_ID == GWM_KIND
#include "gwm_vendor.h"
#else
#include "config.h"
#include "common.h"
#include "can_api.h"


#if VENDOR_ID == NTK_KIND
#define CANTx_INTERVAL	50	//(ms)
#define CAN_BAUDRATE_250kbps
#elif VENDOR_ID == BENZ_KIND
#define CANTx_INTERVAL	10	//(ms)
#define CAN_BAUDRATE_500kbps
#endif
//===========SourceAddr====================================
#if VENDOR_ID == NTK_KIND
enum{
	SourceAddrATI1 = 0x51,
	SourceAddrATO1 = 0x52,
	SourceAddrATI2 = 0x56,
	SourceAddrATO2 = 0x57
}J1939_Addr;
#define DefaultSourceAddr	SourceAddrATO1
#elif VENDOR_ID == BENZ_KIND
#define DefaultSourceAddr	0x00
#endif
//=========================================================
#if VENDOR_ID == NTK_KIND || VENDOR_ID == OTHER_KIND
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
			uint8_t byte0Data;
			uint8_t byte1Data;
			uint8_t byte2Data;
			uint8_t byte3Data;
			uint8_t byte4Data;
			uint8_t byte5Data;
			uint8_t byte6Data;
			uint8_t byte7Data;
		}TxFrame;
		uint8_t TxData[8];
	};
}GeneralTFrame_TypeDef;

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
#elif VENDOR_ID == BENZ_KIND
enum{
	DewPointNoReached = 0,
	DewPointReached = 2,
}startCode;

typedef struct{
	union{
		struct{
			struct{
				uint8_t AfterTreatment2OutletGas :2;
				uint8_t AfterTreatment2IntakeGas :2;
				uint8_t AfterTreatment1OutletGas :2;
				uint8_t AfterTreatment1IntakeGas :2;
			}startCode;
			uint8_t Dat1;
			uint8_t Dat2;
			uint8_t Dat3;
			uint8_t Dat4;
			uint8_t Dat5;
			uint8_t Dat6;
			uint8_t Dat7;
		}RxFrame;
		uint8_t RxData[8];
	};
}CANRxFrameDataType;
typedef struct{
	union{
		struct{
			uint8_t NOx[2];
			uint8_t O2[2];
			uint8_t UNKnow[2];
			uint8_t Status;
			uint8_t res;
		}TxFrame;
		uint8_t TxData[8];
	};
}TramsmitFrame_TypeDef;
#endif

//=========================================================
enum{
#if VENDOR_ID == NTK_KIND
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
#elif VENDOR_ID == BENZ_KIND
	TransmitDataATI1  = 0x200,

	ReceiveSignal     = 0x120,
	TransmitMessage   = 0x350,	//
#else
	TransmitDataATI1  = 0xF00E,
	TransmitDataATO1  = 0xF00F,
	TransmitDataATI2  = 0xF010,
	TransmitDataATO2  = 0xF011,

	ReceiveSignal     = 65247,
	TransmitMessage   = 61680,	//
#endif

	PrivateCmd_Intake = 36690,//0x8F52
	PrivateCmd_Outlet = 36689,//0x8F51
}J1939_PGN;

#define DewPoint_Invalid 	0
#define DewPoint_Start		1
#define DewPoint_Stop   	2
//=========================================================
void CAN_RxTask(uint32_t PGN, CANRxFrameDataType* RxFrame);
void InspectResultAnalysis(uint8_t* InspectResult);
void JudgeDeviceType(void);
void CAN_TxTask();
#if VENDOR_ID == NTK_KIND
float getTorque(CANRxFrameDataType* RxFrame);
float getSpeed(CANRxFrameDataType* RxFrame);
float getSpeedAdjust(CANRxFrameDataType* RxFrame);
float getTorqueLosses(CANRxFrameDataType* RxFrame);
float getGasMassFlow(CANRxFrameDataType* RxFrame);
void Status_HeaterOff(void);
void Status_PreHeater(void);
void Status_Heating(void);
void Status_HeatCriticalPoint(void);
void Status_AutomaticHeat(void);
void Status_NOxValid(void);
void Status_NOxnValid(void);
void Status_O2Valid(void);
void Status_O2nValid(void);
void Status_AtTemperature(void);
void Status_nAtTemperature(void);
uint8_t Status_Get(void);
void CAN_Heaterratio_deviation_Transmit(uint16_t cmdid);
void CAN_TxDATA2(void);
void CAN_RxManyPostback(void);
void CAN_DeclareAddress(void);
#else
#define getTorque(x)				(0)
#define getSpeed(x)					(0)
#define getSpeedAdjust(x)			(0)
#define getTorqueLosses(x)			(0)
#define getGasMassFlow(x)			(0)
#define Status_HeaterOff()
#define Status_PreHeater()
#define Status_Heating()
#define Status_HeatCriticalPoint()
#define Status_AutomaticHeat()
#define Status_NOxValid()
#define Status_NOxnValid()
#define Status_O2Valid()
#define Status_O2nValid()
#define Status_AtTemperature()
#define Status_nAtTemperature()
#define Status_Get()				(0)
#endif

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
#endif /* VENDOR_ID == GWM_KIND*/
#endif /* VENDOR_H_ */
