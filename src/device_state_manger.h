/*
 * device_state_manger.h
 *
 *  Created on: 2021年3月30日
 *      Author: tianqingyuan
 */

#ifndef DEVICE_STATE_MANGER_H_
#define DEVICE_STATE_MANGER_H_

typedef enum{
	STAGE_IDLE 			= 0,
	STAGE_PREHEATING 	= 1,
	STAGE_HEATING 		= 2,
	STAGE_CRITICALPOINT = 3,
	STAGE_ENVIRONMENT 	= 4,
	STAGE_TESTSTAGE 	= 5,
	STAGE_TESTSTAGE2 	= 6,
	STAGE_InspectError 	= 7
}stage;


stage get_working_stage();//获取当前工作状态
void set_working_stage(stage new_stage);
void init_working_stage();
stage get_pre_working_stage();

#endif /* DEVICE_STATE_MANGER_H_ */
