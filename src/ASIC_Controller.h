/*
 * ASIC_Controller.h
 *
 *  Created on: 2019年7月11日
 *      Author: tianqingyuan
 */

#ifndef ASIC_CONTROLLER_H_
#define ASIC_CONTROLLER_H_

#include "spi.h"
#include "common.h"

#define ASIC_OK						0x00
#define ASIC_ERROR					0x01
#define ASIC_SENSOR_ANOMALY 		0x02
typedef uint32_t ASIC_StatusTypeDef;

#define I_OSC1		0xF0	//Internal oscillator 7.48MHz
#define I_OSC2		0xF1	//Internal oscillator 7.46MHz
#define I_OSC3		0xF2	//Internal oscillator 7.45MHz
#define E_CLK		0xF3	//external clock

#define ASIC_SET		1
#define ASIC_RESET	0

#define SPI_BIT_RATE          1000000     /* ~1Mbps */
#define SPI_CS_PORT						GPIOB
#define SPI_CS_PIN						PTE3

#define REF_DISABLE	0
#define REF_ENABLE	1

#define Sensor_RESET	0
#define Sensor_ENABLE	1
void ASIC_SPI_Init(SPI_Type *hspi);			//On-Semi 控制芯片初始化
ASIC_StatusTypeDef ASIC_SensorInspect(SPI_Type *hspi);			//传感器检测
ASIC_StatusTypeDef ASIC_FliterAndADCEnable(SPI_Type *hspi);		//Fliter使能，开始ADC转换
ASIC_StatusTypeDef ASIC_ReadMeasurementResult(SPI_Type *hspi,int16_t *pResult);	//读取测量结果即ADC转换结果
ASIC_StatusTypeDef ASIC_ReadIpMeasurementResult(SPI_Type *pSPI,int16_t *pResult); //读取Ip0-2的测量结果
ASIC_StatusTypeDef ASIC_ReadMeasurementResultExclIp(SPI_Type *pSPI,int16_t *pResult); //读取除Ip0-2的测量结果
ASIC_StatusTypeDef ASIC_SetIp012(SPI_Type *hspi,uint32_t ip1,uint16_t ip0,uint16_t ip2);	//设置Ip0/Ip1/Ip2
ASIC_StatusTypeDef ASIC_InitSensor(SPI_Type *hspi);		//初始化传感器（使Ip0,Ip1,Ip2的控制电路短接
ASIC_StatusTypeDef ASIC_SensorEnable(SPI_Type *hspi);	//使能传感器（使能Ip0/1/2的控制
ASIC_StatusTypeDef ASIC_InitRef(SPI_Type *hspi);		//初始化ref驱动（开启ref驱动电路短接
ASIC_StatusTypeDef ASIC_RefEnable(SPI_Type *hspi);		//使能ref驱动
ASIC_StatusTypeDef ASIC_ConfigCLK(SPI_Type *hspi,uint8_t CLKType);	//设置On-Semi芯片主频
ASIC_StatusTypeDef ASIC_ResetReg(SPI_Type *hspi);		//复位所有寄存器
ASIC_StatusTypeDef ASIC_ClearOBDreg(SPI_Type *hspi);	//清空OBD寄存器
ASIC_StatusTypeDef ASIC_ResetOBDreg(SPI_Type *hspi);	//重置OBD寄存器
void ASIC_Status_ERROR(char* error);					//传感器检测错误后调用，未完成
ASIC_StatusTypeDef ASIC_TransmitReceive(SPI_Type *pSPI, uint8_t *pTxData, uint8_t *pRxData, const uint16_t Size);
int16_t AnalyseDatas(const uint8_t H,const uint8_t L);
ASIC_StatusTypeDef ASIC_InREF(SPI_Type *hspi);
ASIC_StatusTypeDef ASIC_OutREF(SPI_Type *hspi);

#endif /* ASIC_CONTROLLER_H_ */
