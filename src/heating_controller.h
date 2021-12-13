/*
 * heating_controller.h
 *
 *  Created on: 2021Äê3ÔÂ31ÈÕ
 *      Author: sunkaixiang
 */

#ifndef HEATING_CONTROLLER_H_
#define HEATING_CONTROLLER_H_
#define TEMPERATURE_CONTROLLER	//controller temperature by PID algorithm

void init_heating_module();
void heating_control(void);
void set_PWM_Duty_Cycle(float value);
float get_PWM_Duty_Cycle(void);
void enable_pwm(void);
void disable_pwm(void);
void Heating_Start(void);
void perheat_PWM_Duty_Cycle_Init(void);

#endif /* HEATING_CONTROLLER_H_ */
