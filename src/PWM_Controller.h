/*
 * PWM_Controller.h
 *
 *  Created on: 2021��3��31��
 *      Author: sunkaixiang
 */

#ifndef PWM_CONTROLLER_H_
#define PWM_CONTROLLER_H_
#include <stdint.h>

void PWM_init(void);
void set_pwm_dutycycle(uint8_t dc);

#endif /* PWM_CONTROLLER_H_ */
