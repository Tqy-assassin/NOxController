/*
 * can_manage.h
 *
 *  Created on: 2021��4��1��
 *      Author: sunkaixiang
 */

#ifndef CAN_MANAGE_H_
#define CAN_MANAGE_H_
#include <stdint.h>

#define CAN_CONTROL		//CAN��ʾ+����

void stop_NOValueOffsetTimer();
void start_NOValueOffsetTimer(uint32_t value);
void start_O2CValueOffsetTimer(uint32_t value);
float get_O2C(void);
float get_NOxC(void);
void CAN_control(void);
void Can_Init(void);
void CAN_PC_Init(void);
#endif /* CAN_MANAGE_H_ */
