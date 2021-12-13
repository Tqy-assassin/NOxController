/*
 * power_manage.h
 *
 *  Created on: 2021年4月1日
 *      Author: sunkaixiang
 */

#ifndef POWER_MANAGE_H_
#define POWER_MANAGE_H_

void Judge_WorkVoltage(void);	//配置ADC，检测工作电压
float get_WorkVoltage(void);
void Judge_WorkVoltage_loop(void);

#endif /* POWER_MANAGE_H_ */
