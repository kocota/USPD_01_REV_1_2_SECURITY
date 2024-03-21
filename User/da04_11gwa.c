#include "stm32f4xx_hal.h"

//---- Первая цифра---------------------------------------
void LED1_A2_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void LED1_A2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

void LED1_B2_ON(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
}

void LED1_B2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
}

void LED1_C2_ON(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
}

void LED1_C2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
}

void LED1_D2_ON(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
}

void LED1_D2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
}

void LED1_E2_ON(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
}

void LED1_E2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
}

void LED1_F2_ON(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
}

void LED1_F2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
}

void LED1_G2_ON(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
}

void LED1_G2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}
//---------------------------------------------------------


//---- Вторая цифра---------------------------------------
void LED1_A1_ON(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}

void LED1_A1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}

void LED1_B1_ON(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
}

void LED1_B1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
}

void LED1_C1_ON(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}

void LED1_C1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}

void LED1_D1_ON(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void LED1_D1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

void LED1_E1_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void LED1_E1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void LED1_F1_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
}

void LED1_F1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

void LED1_G1_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
}

void LED1_G1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}
//---------------------------------------------------------

//---- третья цифра---------------------------------------
void LED2_A2_ON(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}

void LED2_A2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}

void LED2_B2_ON(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
}

void LED2_B2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
}

void LED2_C2_ON(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
}

void LED2_C2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
}

void LED2_D2_ON(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
}

void LED2_D2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
}

void LED2_E2_ON(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}

void LED2_E2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}

void LED2_F2_ON(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
}

void LED2_F2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
}

void LED2_G2_ON(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
}

void LED2_G2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
}
//---------------------------------------------------------

//---- четвертая цифра---------------------------------------
void LED2_A1_ON(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}

void LED2_A1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
}

void LED2_B1_ON(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
}

void LED2_B1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}

void LED2_C1_ON(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
}

void LED2_C1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED2_D1_ON(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
}

void LED2_D1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
}

void LED2_E1_ON(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
}

void LED2_E1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
}

void LED2_F1_ON(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
}

void LED2_F1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
}

void LED2_G1_ON(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
}

void LED2_G1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
}
//---------------------------------------------------------


//----Управление первой цифрой-----------------------------
void dig1_set_0(void)
{
	LED1_A2_ON();
	LED1_B2_ON();
	LED1_C2_ON();
	LED1_D2_ON();
	LED1_E2_ON();
	LED1_F2_ON();
	LED1_G2_OFF();
}

void dig1_set_1(void)
{
	LED1_A2_OFF();
	LED1_B2_OFF();
	LED1_C2_OFF();
	LED1_D2_OFF();
	LED1_E2_ON();
	LED1_F2_ON();
	LED1_G2_OFF();
}

void dig1_set_2(void)
{
	LED1_A2_ON();
	LED1_B2_ON();
	LED1_C2_OFF();
	LED1_D2_ON();
	LED1_E2_ON();
	LED1_F2_OFF();
	LED1_G2_ON();
}

void dig1_set_3(void)
{
	LED1_A2_ON();
	LED1_B2_OFF();
	LED1_C2_OFF();
	LED1_D2_ON();
	LED1_E2_ON();
	LED1_F2_ON();
	LED1_G2_ON();
}

void dig1_set_4(void)
{
	LED1_A2_OFF();
	LED1_B2_OFF();
	LED1_C2_ON();
	LED1_D2_OFF();
	LED1_E2_ON();
	LED1_F2_ON();
	LED1_G2_ON();
}

void dig1_set_5(void)
{
	LED1_A2_ON();
	LED1_B2_OFF();
	LED1_C2_ON();
	LED1_D2_ON();
	LED1_E2_OFF();
	LED1_F2_ON();
	LED1_G2_ON();
}

void dig1_set_6(void)
{
	LED1_A2_ON();
	LED1_B2_ON();
	LED1_C2_ON();
	LED1_D2_ON();
	LED1_E2_OFF();
	LED1_F2_ON();
	LED1_G2_ON();
}

void dig1_set_7(void)
{
	LED1_A2_OFF();
	LED1_B2_OFF();
	LED1_C2_OFF();
	LED1_D2_ON();
	LED1_E2_ON();
	LED1_F2_ON();
	LED1_G2_OFF();
}

void dig1_set_8(void)
{
	LED1_A2_ON();
	LED1_B2_ON();
	LED1_C2_ON();
	LED1_D2_ON();
	LED1_E2_ON();
	LED1_F2_ON();
	LED1_G2_ON();
}

void dig1_set_9(void)
{
	LED1_A2_ON();
	LED1_B2_OFF();
	LED1_C2_ON();
	LED1_D2_ON();
	LED1_E2_ON();
	LED1_F2_ON();
	LED1_G2_ON();
}
//-----------------------------------------------------------


//----Управление второй цифрой-----------------------------
void dig2_set_0(void)
{
	LED1_A1_ON();
	LED1_B1_ON();
	LED1_C1_ON();
	LED1_D1_ON();
	LED1_E1_ON();
	LED1_F1_ON();
	LED1_G1_OFF();
}

void dig2_set_1(void)
{
	LED1_A1_OFF();
	LED1_B1_OFF();
	LED1_C1_OFF();
	LED1_D1_OFF();
	LED1_E1_ON();
	LED1_F1_ON();
	LED1_G1_OFF();
}

void dig2_set_2(void)
{
	LED1_A1_ON();
	LED1_B1_ON();
	LED1_C1_OFF();
	LED1_D1_ON();
	LED1_E1_ON();
	LED1_F1_OFF();
	LED1_G1_ON();
}

void dig2_set_3(void)
{
	LED1_A1_ON();
	LED1_B1_OFF();
	LED1_C1_OFF();
	LED1_D1_ON();
	LED1_E1_ON();
	LED1_F1_ON();
	LED1_G1_ON();
}

void dig2_set_4(void)
{
	LED1_A1_OFF();
	LED1_B1_OFF();
	LED1_C1_ON();
	LED1_D1_OFF();
	LED1_E1_ON();
	LED1_F1_ON();
	LED1_G1_ON();
}

void dig2_set_5(void)
{
	LED1_A1_ON();
	LED1_B1_OFF();
	LED1_C1_ON();
	LED1_D1_ON();
	LED1_E1_OFF();
	LED1_F1_ON();
	LED1_G1_ON();
}

void dig2_set_6(void)
{
	LED1_A1_ON();
	LED1_B1_ON();
	LED1_C1_ON();
	LED1_D1_ON();
	LED1_E1_OFF();
	LED1_F1_ON();
	LED1_G1_ON();
}

void dig2_set_7(void)
{
	LED1_A1_OFF();
	LED1_B1_OFF();
	LED1_C1_OFF();
	LED1_D1_ON();
	LED1_E1_ON();
	LED1_F1_ON();
	LED1_G1_OFF();
}

void dig2_set_8(void)
{
	LED1_A1_ON();
	LED1_B1_ON();
	LED1_C1_ON();
	LED1_D1_ON();
	LED1_E1_ON();
	LED1_F1_ON();
	LED1_G1_ON();
}

void dig2_set_9(void)
{
	LED1_A1_ON();
	LED1_B1_OFF();
	LED1_C1_ON();
	LED1_D1_ON();
	LED1_E1_ON();
	LED1_F1_ON();
	LED1_G1_ON();
}
//-----------------------------------------------------------

//----Управление третьей цифрой-----------------------------
void dig3_set_0(void)
{
	LED2_A2_ON();
	LED2_B2_ON();
	LED2_C2_ON();
	LED2_D2_ON();
	LED2_E2_ON();
	LED2_F2_ON();
	LED2_G2_OFF();
}

void dig3_set_1(void)
{
	LED2_A2_OFF();
	LED2_B2_OFF();
	LED2_C2_OFF();
	LED2_D2_OFF();
	LED2_E2_ON();
	LED2_F2_ON();
	LED2_G2_OFF();
}

void dig3_set_2(void)
{
	LED2_A2_ON();
	LED2_B2_ON();
	LED2_C2_OFF();
	LED2_D2_ON();
	LED2_E2_ON();
	LED2_F2_OFF();
	LED2_G2_ON();
}

void dig3_set_3(void)
{
	LED2_A2_ON();
	LED2_B2_OFF();
	LED2_C2_OFF();
	LED2_D2_ON();
	LED2_E2_ON();
	LED2_F2_ON();
	LED2_G2_ON();
}

void dig3_set_4(void)
{
	LED2_A2_OFF();
	LED2_B2_OFF();
	LED2_C2_ON();
	LED2_D2_OFF();
	LED2_E2_ON();
	LED2_F2_ON();
	LED2_G2_ON();
}

void dig3_set_5(void)
{
	LED2_A2_ON();
	LED2_B2_OFF();
	LED2_C2_ON();
	LED2_D2_ON();
	LED2_E2_OFF();
	LED2_F2_ON();
	LED2_G2_ON();
}

void dig3_set_6(void)
{
	LED2_A2_ON();
	LED2_B2_ON();
	LED2_C2_ON();
	LED2_D2_ON();
	LED2_E2_OFF();
	LED2_F2_ON();
	LED2_G2_ON();
}

void dig3_set_7(void)
{
	LED2_A2_OFF();
	LED2_B2_OFF();
	LED2_C2_OFF();
	LED2_D2_ON();
	LED2_E2_ON();
	LED2_F2_ON();
	LED2_G2_OFF();
}

void dig3_set_8(void)
{
	LED2_A2_ON();
	LED2_B2_ON();
	LED2_C2_ON();
	LED2_D2_ON();
	LED2_E2_ON();
	LED2_F2_ON();
	LED2_G2_ON();
}

void dig3_set_9(void)
{
	LED2_A2_ON();
	LED2_B2_OFF();
	LED2_C2_ON();
	LED2_D2_ON();
	LED2_E2_ON();
	LED2_F2_ON();
	LED2_G2_ON();
}
//-----------------------------------------------------------

//----Управление четвертой цифрой-----------------------------
void dig4_set_0(void)
{
	LED2_A1_ON();
	LED2_B1_ON();
	LED2_C1_ON();
	LED2_D1_ON();
	LED2_E1_ON();
	LED2_F1_ON();
	LED2_G1_OFF();
}

void dig4_set_1(void)
{
	LED2_A1_OFF();
	LED2_B1_OFF();
	LED2_C1_OFF();
	LED2_D1_OFF();
	LED2_E1_ON();
	LED2_F1_ON();
	LED2_G1_OFF();
}

void dig4_set_2(void)
{
	LED2_A1_ON();
	LED2_B1_ON();
	LED2_C1_OFF();
	LED2_D1_ON();
	LED2_E1_ON();
	LED2_F1_OFF();
	LED2_G1_ON();
}

void dig4_set_3(void)
{
	LED2_A1_ON();
	LED2_B1_OFF();
	LED2_C1_OFF();
	LED2_D1_ON();
	LED2_E1_ON();
	LED2_F1_ON();
	LED2_G1_ON();
}

void dig4_set_4(void)
{
	LED2_A1_OFF();
	LED2_B1_OFF();
	LED2_C1_ON();
	LED2_D1_OFF();
	LED2_E1_ON();
	LED2_F1_ON();
	LED2_G1_ON();
}

void dig4_set_5(void)
{
	LED2_A1_ON();
	LED2_B1_OFF();
	LED2_C1_ON();
	LED2_D1_ON();
	LED2_E1_OFF();
	LED2_F1_ON();
	LED2_G1_ON();
}

void dig4_set_6(void)
{
	LED2_A1_ON();
	LED2_B1_ON();
	LED2_C1_ON();
	LED2_D1_ON();
	LED2_E1_OFF();
	LED2_F1_ON();
	LED2_G1_ON();
}

void dig4_set_7(void)
{
	LED2_A1_OFF();
	LED2_B1_OFF();
	LED2_C1_OFF();
	LED2_D1_ON();
	LED2_E1_ON();
	LED2_F1_ON();
	LED2_G1_OFF();
}

void dig4_set_8(void)
{
	LED2_A1_ON();
	LED2_B1_ON();
	LED2_C1_ON();
	LED2_D1_ON();
	LED2_E1_ON();
	LED2_F1_ON();
	LED2_G1_ON();
}

void dig4_set_9(void)
{
	LED2_A1_ON();
	LED2_B1_OFF();
	LED2_C1_ON();
	LED2_D1_ON();
	LED2_E1_ON();
	LED2_F1_ON();
	LED2_G1_ON();
}
//-----------------------------------------------------------
