#include "stm32f4xx_hal.h"
#include "DisplayTask.h"
#include "da04_11gwa.h"



void ThreadDisplayTask(void const * argument)
{




	for(;;)
	{

		dig1_set_0();
		dig2_set_0();
		dig3_set_0();
		dig4_set_0();
		HAL_Delay(1000);
		dig1_set_1();
		dig2_set_1();
		dig3_set_1();
		dig4_set_1();
		HAL_Delay(1000);
		dig1_set_2();
		dig2_set_2();
		dig3_set_2();
		dig4_set_2();
		HAL_Delay(1000);
		dig1_set_3();
		dig2_set_3();
		dig3_set_3();
		dig4_set_3();
		HAL_Delay(1000);
		dig1_set_4();
		dig2_set_4();
		dig3_set_4();
		dig4_set_4();
		HAL_Delay(1000);
		dig1_set_5();
		dig2_set_5();
		dig3_set_5();
		dig4_set_5();
		HAL_Delay(1000);
		dig1_set_6();
		dig2_set_6();
		dig3_set_6();
		dig4_set_6();
		HAL_Delay(1000);
		dig1_set_7();
		dig2_set_7();
		dig3_set_7();
		dig4_set_7();
		HAL_Delay(1000);
		dig1_set_8();
		dig2_set_8();
		dig3_set_8();
		dig4_set_8();
		HAL_Delay(1000);
		dig1_set_9();
		dig2_set_9();
		dig3_set_9();
		dig4_set_9();
		HAL_Delay(1000);

		osDelay(100); // ждем 100 милисекунд
	}
}

