/*
 * power_manage.h
 *
 *  Created on: 2021��4��1��
 *      Author: sunkaixiang
 */

#ifndef POWER_MANAGE_H_
#define POWER_MANAGE_H_

void Judge_WorkVoltage(void);	//����ADC����⹤����ѹ
float get_WorkVoltage(void);
void Judge_WorkVoltage_loop(void);

#endif /* POWER_MANAGE_H_ */
