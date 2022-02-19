/*
 * YUC_Y24.h
 *
 *  Created on: 2022年1月19日
 *      Author: Administrator
 */

#ifndef YUC_Y24_H_
#define YUC_Y24_H_

#include "config.h"
#if VENDOR_ID == YUC_Y24
#include "common.h"
#include "can_api.h"

#define CANTx_INTERVAL	10	//(ms)
#define CAN_BAUDRATE_500kbps

enum{
    DeviceType_Outlet = 0,
    DeviceType_Intake = 1
}DeviceType;
#define DefaultSourceAddr DeviceType_Outlet

enum{
	OutletReceiveSignal                  = 0x1F1,
    OutletTransmitData                   = 0x1FA,
    OutletTransmitParameterCorrection    = 0x1FB, 

	IntakeReceiveSignal                  = 0x1F2,
	IntakeTransmitData                   = 0x1FC,
    IntakeTransmitParameterCorrection    = 0x1FD,

	PrivateCmd_Intake = 0x5A99,
	PrivateCmd_Outlet = 0x99A5,
}CAN_PGN;

typedef union{
	uint8_t byte;
	struct{
		uint8_t HeaterOC :1;
		uint8_t NOxOC :1;
		uint8_t O2OC :1;
		uint8_t Vp0OC :1;
		uint8_t HeaterSC :1;
		uint8_t NOxSC :1;
		uint8_t O2SC :1;
		uint8_t Vp0SC :1;
	}bits;
}Error_t;

enum DewPointReachedIdentification{
	NormalMode = 0,
	WatingForDewPoint = 1,
	TransmitInitData = 2,
	DataPart1AfterSignal = 4,
	DataPart2AfterSignal = 5,
	DataPart1BeforeSignal = 6,
	DataPart2BeforeSignal = 7,
};

typedef union{
	uint8_t byte;
	struct{
		uint8_t SensorSupply :1;
		uint8_t HeaterTemperature :1;
		uint8_t LamdaBinarySignal :1;
		uint8_t LamdaLinearSignal :1;
		uint8_t NOxSignal :1;
		uint8_t ExternMode :3;
	}bits;
}Status_t;

typedef union{
    struct{
        int16_t NOx;
        int16_t O2;
        int16_t Vp0;
        Status_t Status;
        Error_t Error;
    }Frame;
    uint8_t TxData[8];
}CANTxDateFrame;

typedef union{
    struct{
        uint16_t NOxGain;
        uint16_t HeaterRadio;
        int8_t NOxOffsetRaw;
        uint8_t PressureLambdal_lRaw;
        uint8_t Rst;
        uint8_t PressureNOxRaw;
    }Frame;
    uint8_t TxData[8];
}CANTxParameterCorrection1Frame;

typedef union{
    struct{
        uint8_t Rst0;
        uint8_t Rst1;
        uint8_t Rst2;
        uint8_t Rst3;
        uint8_t NO2Raw;
        uint8_t NH3Raw;
        uint8_t Rst6;
        uint8_t Rst7;
    }Frame;
    uint8_t TxData[8];
}CANTxParameterCorrection2Frame;

typedef union{
    struct{
        struct 
        {
            uint8_t bit0 :1;
            uint8_t Request1 :1;
            uint8_t bit2 :1;
            uint8_t Request2 :1;
            uint8_t bit4 :1;
            uint8_t bit5 :1;
            uint8_t bit6 :1;
            uint8_t DewPoint :1;
        }code;
        uint8_t Rst1;
        uint8_t Rst2;
        uint8_t Rst3;
        uint8_t Rst4;
        uint8_t Rst5;
        uint8_t Rst6;
        uint8_t Rst7;
    }Frame;
    uint8_t RxData[8];
}CANRxFrameDataType;

void CAN_RxTask(uint32_t CAN_ID, CANRxFrameDataType* RxFrame);
void InspectResultAnalysis(uint8_t* InspectResult);
void JudgeDeviceType(void);
void CAN_TxTask();
float getTorque(CANRxFrameDataType* RxFrame);
float getSpeed(CANRxFrameDataType* RxFrame);
float getSpeedAdjust(CANRxFrameDataType* RxFrame);
float getTorqueLosses(CANRxFrameDataType* RxFrame);
float getGasMassFlow(CANRxFrameDataType* RxFrame);
void StatusPowerHasProblem(uint8_t err);
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
#define CAN_TxDATA2()
#define CAN_RxManyPostback()
#define CAN_DeclareAddress()

#endif /* #if VENDOR_ID == YUC_Y24 */
#endif /* YUC_Y24_H_ */
