
#include "derivative.h" /* include peripheral declarations SSKEAZ128M4 */
#include "uart.h"
#include "vendor.h"
#include "device_state_manger.h"
#include "heating_controller.h"
#include "common.h"
#include "can_manage.h"
#include "sensor_control.h"
#include "power_manage.h"
#include "flash_manage.h"
#include "clock.h"
#include "ASIC_Controller.h"
#include <string.h>
#include "Filter.h"
#include "common.h"
#include "gpio.h"
#ifdef DEBUG
#include "printf.h"
#endif


int main(void)
{
//	CONFIG_PIN_AS_GPIO(PTB,PTB2,OUTPUT);
//	OUTPUT_SET(PTB,PTB2);
//	OUTPUT_CLEAR(PTB,PTB2);

	sysinit();
	Judge_WorkVoltage();
	JudgeDeviceType();
	flash_manage_init();
	Clock_init();
	Can_Init();

	Filter_init();
	sensor_init();
	init_heating_module();
	init_working_stage();

	while(1)
	{
		heating_control();
		CAN_control();//对上位机发送接收
		Judge_WorkVoltage_loop();
		flash_manage_loop();
	}
	return 0;
}




