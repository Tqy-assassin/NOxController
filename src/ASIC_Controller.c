/*
 * ASIC_Controller.c
 *
 *  Created on: 2019年7月11日
 *      Author: tianqingyuan
 */

#include "ASIC_controller.h"
#include "vendor.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "gpio.h"
#include "vendor.h"

uint8_t Rx_Buf[32];
extern void DelayUS(uint32_t u32TimeUS);
uint8_t REF_Status = 0;
uint8_t Sensor_Status = 0;
uint8_t InspectResult[5] = {0};

void ASIC_SPI_Init(SPI_Type *hspi)
{
	SPI_ConfigType sSPIConfig = {{0}};

	SIM_PINSEL0|=SIM_PINSEL_SPI0PS(1);  // SPI on PTE

	/* initialize SPI0 as master    */
	sSPIConfig.u32BitRate = 100000;
	sSPIConfig.u32BusClkHz = BUS_CLK_HZ;
	sSPIConfig.sSettings.bModuleEn             = 1;
	sSPIConfig.sSettings.bMasterMode           = 1;
	sSPIConfig.sSettings.bClkPhase1            = 1;//相位
	sSPIConfig.sSettings.bClkPolarityLow       = 0;//极性
	sSPIConfig.sSettings.bMasterAutoDriveSS    = 1;
	//CPOL=0，CPHA=1： Mode1

	SPI_Init(SPI0, &sSPIConfig);

//	CONFIG_PIN_AS_GPIO(SPI_CS_PORT, SPI_CS_PIN, OUTPUT);
//	OUTPUT_SET(SPI_CS_PORT, SPI_CS_PIN);

	DelayUS(1000000);			//delay 1s
	ASIC_ConfigCLK(hspi,I_OSC1);

	sSPIConfig.u32BitRate = SPI_BIT_RATE;
	SPI_Init(SPI0, &sSPIConfig);

	ASIC_InitSensor(hspi);
	ASIC_InitSensor(hspi);
	ASIC_SetIp012(hspi,0,0,0);
	DelayUS(10000);			//delay 10ms
}

ASIC_StatusTypeDef ASIC_TransmitReceive(SPI_Type *pSPI, uint8_t *pTxData, uint8_t *pRxData, const uint16_t Size)
{
	ASIC_StatusTypeDef Status = 0;
	int i;

	for(i = 0;(i<Size) && (Status == 0);i++){
		Status = SPI_TransferWait(pSPI,pRxData+i,pTxData+i,1);
		if(Status)
			Status = SPI_TransferWait(pSPI,pRxData+i,pTxData+i,1);
	}
	return Status;
}

ASIC_StatusTypeDef ASIC_SensorInspect(SPI_Type *pSPI)
{
//	static ASIC_Stage ASIC_FLAG_temp;
	ASIC_StatusTypeDef Status = 0;
	const uint8_t Length = 5;
	uint8_t Tx_Buf[]={0xe8,0xe9,0xea,0xeb,0xeb};
	const uint8_t NormalValue[] = {0x08,0x00,0x00,0x00,0x80};
//	uint8_t i;
	memset(Rx_Buf,0,32);

	if(ASIC_TransmitReceive(pSPI,Tx_Buf,Rx_Buf,Length) == ASIC_OK){
		int i;
		memcpy(InspectResult,Rx_Buf,5);

		for(i = 1;i<5;i++){
			if(Rx_Buf[i] == NormalValue[i]){
				Status &= ~(1<<i);
			}else{
				Status |= 1<<i;
			}
		}
	}
	InspectResultAnalysis(Rx_Buf);

	return Status;
}
ASIC_StatusTypeDef ASIC_FliterAndADCEnable(SPI_Type *hspi)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 2;
	uint8_t Tx_Buf[]={0xA7,0xA8};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);

	return Status;
}
inline int16_t AnalyseDatas(const uint8_t H,const uint8_t L)
{
	int16_t data;
	data = ((H << 8) | L) & 0x3FFF ;
	if(data & 0x2000){
		data = ((int)data << 18)>>18;
	}else{
		data = data & 0x1FFF;
	}

	return data;
}

ASIC_StatusTypeDef ASIC_ReadMeasurementResult(SPI_Type *pSPI,int16_t *pResult)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 13;
	uint8_t Tx_Buf[]={0xa9,0xa8,0xb1,0xb0,0xb9,0xb8,0xc9,0xc8,0xd1,0xd0,0xc1,0xc0,0xc1};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(pSPI,Tx_Buf,Rx_Buf,Length);

	memset(pResult,0,6);
	if(Status == ASIC_OK)
	{
		int i;
		for(i = 0;i < 6;i++){
			*(pResult+i) = AnalyseDatas(Rx_Buf[2*i+1],Rx_Buf[2*i+2]);
		}
	}
	return Status;
}

ASIC_StatusTypeDef ASIC_ReadIpMeasurementResult(SPI_Type *pSPI,int16_t *pResult)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 13;
	uint8_t Tx_Buf[]={0xa9,0xa8,0xb1,0xb0,0xb9,0xb8,0xb9};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(pSPI,Tx_Buf,Rx_Buf,Length);

	memset(pResult,0,3);
	if(Status == ASIC_OK)
	{
		int i;
		for(i = 0;i < 3;i++){
			*(pResult+i) = AnalyseDatas(Rx_Buf[2*i+1],Rx_Buf[2*i+2]);
		}
	}
	return Status;
}

ASIC_StatusTypeDef ASIC_ReadMeasurementResultExclIp(SPI_Type *pSPI,int16_t *pResult)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 13;
	uint8_t Tx_Buf[]={0xc9,0xc8,0xd1,0xd0,0xc1,0xc0,0xc1};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(pSPI,Tx_Buf,Rx_Buf,Length);

	memset(pResult,0,3);
	if(Status == ASIC_OK)
	{
		int i;
		for(i = 0;i < 3;i++){
			*(pResult+i) = AnalyseDatas(Rx_Buf[2*i+1],Rx_Buf[2*i+2]);
		}
	}
	return Status;
}


ASIC_StatusTypeDef ASIC_SetIp012(SPI_Type *hspi,uint32_t ip0,uint16_t ip1,uint16_t ip2)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 9;
	uint8_t Tx_Buf[]={0x88,0x00,0x00,0x90,0x00,0x00,0x98,0x00,0x00};
	Tx_Buf[0] += ip0 >> 16;
	Tx_Buf[1] = (ip0 >> 8) & 0xFF;
	Tx_Buf[2] = ip0 & 0xFF;
	Tx_Buf[4] = ip1 >> 8;
	Tx_Buf[5] = ip1 & 0xFF;
	Tx_Buf[7] = ip2 >> 8;
	Tx_Buf[8] = ip2 & 0xFF;

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);

	return Status;
}

ASIC_StatusTypeDef ASIC_InitSensor(SPI_Type *hspi)			//0xFA
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[] = {0xFA};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	Sensor_Status = Sensor_RESET;
	return Status;
}

ASIC_StatusTypeDef ASIC_SensorEnable(SPI_Type *hspi)			//0xFB
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[] = {0xFB};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	Sensor_Status = Sensor_ENABLE;
	return Status;
}

ASIC_StatusTypeDef ASIC_InitRef(SPI_Type *hspi)					//0xFC
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[] = {0xFC};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	REF_Status = REF_DISABLE;
	return Status;
}


ASIC_StatusTypeDef ASIC_OutREF(SPI_Type *hspi)					//0xDB
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[] = {0xDB};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	return Status;
}

ASIC_StatusTypeDef ASIC_InREF(SPI_Type *hspi)					//0xDA
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[] = {0xDA};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	return Status;
}

ASIC_StatusTypeDef ASIC_RefEnable(SPI_Type *hspi)				//0xFD*4
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 4;
	uint8_t Tx_Buf[] = {0xFD,0xFD,0xFD,0xFD};

	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	REF_Status = REF_ENABLE;
	return Status;
}

ASIC_StatusTypeDef ASIC_ConfigCLK(SPI_Type *hspi,uint8_t CLKType)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[Length];
	Tx_Buf[0] = CLKType;
	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	return Status;
}
ASIC_StatusTypeDef ASIC_ResetReg(SPI_Type *hspi)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[] = {0xF9};
	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	return Status;
}

ASIC_StatusTypeDef ASIC_ClearOBDreg(SPI_Type *hspi)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[] = {0xD8};
	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	return Status;
}

ASIC_StatusTypeDef ASIC_ResetOBDreg(SPI_Type *hspi)
{
	ASIC_StatusTypeDef Status;
	const uint8_t Length = 1;
	uint8_t Tx_Buf[] = {0xD9};
	memset(Rx_Buf,0,32);
	Status = ASIC_TransmitReceive(hspi,Tx_Buf,Rx_Buf,Length);
	return Status;
}


void ASIC_Status_ERROR(char* error)
{
//	printstring(error);
//	printstring("	");
//	while(1);
}

