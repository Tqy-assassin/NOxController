/*
 * clock.h
 *
 *  Created on: 2021Äê3ÔÂ30ÈÕ
 *      Author: sunkaixiang
 */

#ifndef CLOCK_H_
#define CLOCK_H_
#include "stdint.h"
#define clock_time_exceed(ref,ms)	((uint32_t)(Gets_Clock_value() - ref) > ms)

void Clock_init(void);
uint32_t Gets_Clock_value(void);



#endif /* CLOCK_H_ */
