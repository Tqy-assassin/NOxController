/*
 * flash_manage.c
 *
 *  Created on: 2021年3月31日
 *      Author: sunkaixiang
 */

#include "flash_manage.h"
#include "flash.h"
#include <string.h>
#include "common.h"
#include "config.h"
#include "flash.h"
#define Default_Vref1 0.410
#define Default_Vref2 0.440
#define Default_Temper 0.160
#define Default_Vref0L 0.350
#define Default_Ip1	7.0

/* 校准值默认参数 */
#define Default_O2_k	0
#define Default_O2_d	0
#define Default_NOx_A1	0
#define Default_NOx_B1	0
#define Default_NOx_A2	0
#define Default_NOx_B2	0

#define Default_O2_Pressure 0.36
#define Default_NOx_Pressure 0.11


#define TIME_SECTOR				(0x1F800)
#define TIME_SECTOR_1			(0x1F800)		//252
#define TIME_SECTOR_2			(0x1FA00)		//253
#define TIME_SECTOR_3			(0x1FC00)		//254
#define TIME_SECTOR_4			(0x1FE00)		//255
#define TIME_SECTOR_SIZE		(4 * 512)

void Coeffi_init();
void Coeffi_store(CalculateCoeffi_Typedef* Coeffi);
void Coeffi_retrieve(CalculateCoeffi_Typedef* Coeffi);
void CLT_init();		//closed loop target initial
void CLT_store(measureCoeffi_Typedef * CLT);	//closed loop target store
void CLT_retrieve(measureCoeffi_Typedef* CLT);
void save_time(uint32_t run_time);
void load_time();

CalculateCoeffi_Typedef Coeffi;	//O2和NOx的计算系数以及压力修正系数
measureCoeffi_Typedef CLT;		//控制参数 ：Vref0/Vref1目标值  和温度系数目标值

uint32_t CLT_Store_timer = 0;
uint32_t Coeffi_Store_timer = 0;
uint32_t run_time;
uint32_t new_run_time;

uint32_t Period30STimer = 0;



uint32_t Coeffi_addr = 0;
uint32_t CLT_addr = 0;

uint32_t get_run_time(void){
	return run_time;//S
}

uint32_t get_new_run_time(void){
	return new_run_time;//S
}

CalculateCoeffi_Typedef* Coeffi_pointer(void){
	return &Coeffi;
}

measureCoeffi_Typedef* CLT_pointer(void){
	return &CLT;
}

void Update_time(void)
{

#ifdef DEBUG
	uint32_t min_sec;
#endif
	new_run_time = run_time + (Gets_Clock_value() / 1000);//S
	save_time(new_run_time);
#ifdef DEBUG
	min_sec = new_run_time % 3600;
	printf("%lu:%02lu:%02lu\n", new_run_time/3600, min_sec/60, min_sec%60);
#endif
}


void erase_data_falsh(uint32_t fLASH_erase_addr){
	//__disable_irq();
	FLASH_EraseSector(fLASH_erase_addr);//擦除flash扇区，每个flash扇区长度为512字节。
	//FLASH_EraseVerifySection(fLASH_erase_addr,128);//擦除验证flash部分。
	//__enable_irq();
}


void Coeffi_erase(CalculateCoeffi_Typedef* Coeffi){
	Coeffi_addr = COEFFI_SECTOR;
	FLASH_EraseSector(COEFFI_SECTOR);
	Coeffi_store(Coeffi);
}

void Coeffi_store(CalculateCoeffi_Typedef* Coeffi)
{
	if(Coeffi_addr >= (COEFFI_SECTOR + FLASH_SECTOR_SIZE - sizeof(CalculateCoeffi_Typedef)) || Coeffi_addr < COEFFI_SECTOR)
	{
		Coeffi_erase(Coeffi);
		return;
	}
	CalculateCoeffi_Typedef Coeffi_save = {0};
	memcpy(&Coeffi_save,Coeffi,sizeof(CalculateCoeffi_Typedef));
	while(FLASH_Program(Coeffi_addr,(uint8_t *)&Coeffi_save,sizeof(CalculateCoeffi_Typedef)));
	Coeffi_addr += sizeof(CalculateCoeffi_Typedef);
	return;
}
void Coeffi_retrieve(CalculateCoeffi_Typedef* Coeffi)
{
	int i = 0;
	for(i = 0;i < FLASH_SECTOR_SIZE;i += sizeof(CalculateCoeffi_Typedef)){
		Coeffi_addr = COEFFI_SECTOR + i;
		CalculateCoeffi_Typedef Coeffi_r;
		memcpy((uint8_t*)&Coeffi_r,(uint8_t*)(Coeffi_addr),sizeof(CalculateCoeffi_Typedef));
		if(Coeffi_r.Save_flag == QuadraticEquation || Coeffi_r.Save_flag == LinearEquation || Coeffi_r.Save_flag == PressureCorrection || Coeffi_r.Save_flag == Quadratic_LinearEquation)
		{
			memcpy(Coeffi,&Coeffi_r,sizeof(CalculateCoeffi_Typedef));
		}else{
			break;
		}
	}
}
void Coeffi_init()
{
	Coeffi_retrieve(&Coeffi);//查找之前是否保存数据
	if(!(Coeffi.Save_flag == QuadraticEquation || Coeffi.Save_flag == LinearEquation || Coeffi.Save_flag == Quadratic_LinearEquation || Coeffi.Save_flag == PressureCorrection)){
		erase_data_falsh(COEFFI_SECTOR);
		Coeffi.Save_flag = LinearEquation;
		Coeffi.NOx_A1 = Default_NOx_A1;
		Coeffi.NOx_B1 = Default_NOx_A1;
		Coeffi.NOx_A2 = Default_NOx_A1;
		Coeffi.NOx_B2 = Default_NOx_A1;
		Coeffi.NOx_A3 = Default_NOx_A1;
		Coeffi.NOx_B3 = Default_NOx_A1;
		Coeffi.NOx_A4 = Default_NOx_A1;
		Coeffi.NOx_B4 = Default_NOx_A1;
		Coeffi.NOx_a1 = Default_NOx_A1;
		Coeffi.NOx_b1 = Default_NOx_A1;
		Coeffi.NOx_c1 = Default_NOx_A1;
		Coeffi.NOx_a2 = Default_NOx_A1;
		Coeffi.NOx_b2 = Default_NOx_A1;
		Coeffi.NOx_c2 = Default_NOx_A1;
		Coeffi.NOx_a3 = Default_NOx_A1;
		Coeffi.NOx_b3 = Default_NOx_A1;
		Coeffi.NOx_c3 = Default_NOx_A1;
		Coeffi.NOx_sb1 = Default_NOx_A1;
		Coeffi.NOx_sc1 = Default_NOx_A1;
		Coeffi.NOx_sb2 = Default_NOx_A1;
		Coeffi.NOx_sc2 = Default_NOx_A1;
		Coeffi.NOx_sb3 = Default_NOx_A1;
		Coeffi.NOx_sc3 = Default_NOx_A1;
		Coeffi.O2_d = Default_O2_d;
		Coeffi.O2_k = Default_O2_k;
		Coeffi.NOx_Pressure = Default_NOx_Pressure;
		Coeffi.O2_Pressure = Default_O2_Pressure;
		Coeffi_store(&Coeffi);
	}
}

void CLT_init()
{
	CLT_retrieve(&CLT);
	if(CLT.Save_flag != 0xFA){
		erase_data_falsh(CLT_SECTOR);
		CLT.Save_flag = 0xFA;
		CLT.Vref1 = Default_Vref1;
		CLT.Vref2 = Default_Vref2;
		CLT.Vref0Limit = Default_Vref0L;
		CLT.Ip1 = Default_Ip1;
		CLT.temper = Default_Temper;
		CLT_store(&CLT);
	}
}

void CLT_erase(measureCoeffi_Typedef* CLT){
	CLT_addr = CLT_SECTOR;
	FLASH_EraseSector(CLT_SECTOR);
	CLT_store(CLT);
}
void CLT_store(measureCoeffi_Typedef* CLT)
{
	if(CLT_addr >= (CLT_SECTOR + FLASH_SECTOR_SIZE - sizeof(measureCoeffi_Typedef)) || CLT_addr < CLT_SECTOR){
		CLT_erase(CLT);
		return;
	}
	measureCoeffi_Typedef CLT_save = {0};
	memcpy(&CLT_save,CLT,sizeof(measureCoeffi_Typedef));
	while(FLASH_Program(CLT_addr,(uint8_t *)&CLT_save,sizeof(measureCoeffi_Typedef)));
	CLT_addr += sizeof(measureCoeffi_Typedef);
	return;
}
void CLT_retrieve(measureCoeffi_Typedef* CLT)
{
	int i;
	for(i = 0;i < FLASH_SECTOR_SIZE;i += sizeof(measureCoeffi_Typedef)){
		CLT_addr = CLT_SECTOR + i;
		measureCoeffi_Typedef CLT_retrieve;
		memcpy((uint8_t*)&CLT_retrieve,(uint8_t*)(CLT_addr),sizeof(measureCoeffi_Typedef));
		if(CLT_retrieve.Save_flag == 0xFA)
		{
			memcpy(CLT,&CLT_retrieve,sizeof(measureCoeffi_Typedef));
		}else{
			break;
		}
	}
}

uint32_t time_save_addr = 0;

void time_sector_erase(uint32_t run_time){
	time_save_addr = TIME_SECTOR;
	FLASH_EraseSector(TIME_SECTOR_1);
	FLASH_EraseSector(TIME_SECTOR_2);
	FLASH_EraseSector(TIME_SECTOR_3);
	FLASH_EraseSector(TIME_SECTOR_4);
	save_time(run_time);
}

void save_time(uint32_t run_time)
{
	if(time_save_addr >= (TIME_SECTOR + TIME_SECTOR_SIZE - sizeof(uint32_t)) || time_save_addr < TIME_SECTOR)
	{
		time_sector_erase(run_time);
		return;
	}
	uint32_t time;
	time = run_time;
	while(FLASH_Program(time_save_addr,(uint8_t *)&time,sizeof(uint32_t)));
	time_save_addr += sizeof(uint32_t);
	return;
}

void load_time()
{
	int i = 0;
	for(i = 0;i < TIME_SECTOR_SIZE;i += sizeof(uint32_t)){
		time_save_addr = TIME_SECTOR + i;
		uint32_t time;
		memcpy((uint8_t*)&time,(uint8_t*)(time_save_addr),sizeof(uint32_t));
		if(time != 0xFFFFFFFF)
		{
			run_time = time;
		}else{

			return;
		}
	}
	run_time = 0;
}

void flash_manage_loop(void){
	if(clock_time_exceed(Period30STimer,60 * 1000)){			//周期性事件 T=60s
		Period30STimer = Gets_Clock_value();
		Update_time();

	}
	if(CLT_Store_timer && clock_time_exceed(CLT_Store_timer,4 * 1000)){			//周期性事件 T=4s
		CLT_Store_timer = 0;
		CLT_store(&CLT);
	}
	if(Coeffi_Store_timer && clock_time_exceed(Coeffi_Store_timer,4 * 1000)){			//周期性事件 T=4s
		Coeffi_Store_timer = 0;
		Coeffi_store(&Coeffi);
	}
}

void ready_store_CLT(){
	CLT_Store_timer = Gets_Clock_value();
}
void ready_store_Coeffi(){
	Coeffi_Store_timer = Gets_Clock_value();
}

void flash_manage_init(){
	FLASH_Init(BUS_CLK_HZ);
	Coeffi_init();
	CLT_init();
	load_time();
}
