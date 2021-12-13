/*
 * flash_manage.h
 *
 *  Created on: 2021年3月31日
 *      Author: sunkaixiang
 */

#ifndef FLASH_MANAGE_H_
#define FLASH_MANAGE_H_
#include "stdint.h"

#define COEFFI_SECTOR			(0x1F400)		//250
#define CLT_SECTOR				(0x1F600)		//251


enum{
	QuadraticEquation = 0xF0,//ip2一元二次方程 O2C一元二次方程
	LinearEquation = 0xF1,//ip2线性 O2C线性
	Quadratic_LinearEquation = 0xF2,//ip2一元二次方程 O2C线性
	PressureCorrection = 0xF3,//压力标定
};

typedef struct{
	float Ip1;
	float Vref0Limit;
	float Vref1;
	float Vref2;
	float temper;
	uint8_t Save_flag;
}measureCoeffi_Typedef;

typedef struct{
	uint8_t Save_flag;
	float O2_k;
	float O2_d;
	float NOx_a1;
	float NOx_b1;
	float NOx_c1;
	float NOx_a2;
	float NOx_b2;
	float NOx_c2;
	float NOx_a3;
	float NOx_b3;
	float NOx_c3;

	float NOx_sb1;
	float NOx_sc1;
	float NOx_sb2;
	float NOx_sc2;
	float NOx_sb3;
	float NOx_sc3;

	float NOx_A1;
	float NOx_B1;
	float NOx_A2;
	float NOx_B2;
	float NOx_A3;
	float NOx_B3;
	float NOx_A4;
	float NOx_B4;

	float O2_Pressure;
	float NOx_Pressure;
}CalculateCoeffi_Typedef,*PCalculateCoeffi_Typedef;



CalculateCoeffi_Typedef* Coeffi_pointer(void);
measureCoeffi_Typedef* CLT_pointer(void);

void flash_manage_loop(void);
void ready_store_Coeffi(void);
void ready_store_CLT(void);

uint32_t get_run_time(void);
uint32_t get_new_run_time(void);
void flash_manage_init(void);


#endif /* FLASH_MANAGE_H_ */
