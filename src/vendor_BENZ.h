/*
 * vendor_BENZ.h
 *
 *  Created on: 2022Äê1ÔÂ29ÈÕ
 *      Author: LiquidSource
 */

#ifndef VENDOR_BENZ_H_
#define VENDOR_BENZ_H_

#include "config.h"
#if VENDOR_ID == BENZ_KIND

#include "common.h"
#include "can_api.h"

#define CANTx_INTERVAL	10	//(ms)
#define CAN_BAUDRATE_500kbps

#define DefaultSourceAddr	0x00

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

enum{
	TransmitDataATI1  = 0x200,

	ReceiveSignal     = 0x120,
	TransmitMessage   = 0x350,	//

	PrivateCmd_Intake = 0x5A99,
	PrivateCmd_Outlet = 0x99A5,
}J1939_PGN;

#define DewPoint_Invalid 	0
#define DewPoint_Start		1
#define DewPoint_Stop   	2
//=========================================================
void CAN_RxTask(uint32_t PGN, CANRxFrameDataType* RxFrame);
void InspectResultAnalysis(uint8_t* InspectResult);
void JudgeDeviceType(void);
void CAN_TxTask();
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

#define CAN_TxDATA2()
#define CAN_RxManyPostback()
#define CAN_DeclareAddress()

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

#endif /* VENDOR_ID == BENZ_KIND */
#endif /* VENDOR_BENZ_H_ */
