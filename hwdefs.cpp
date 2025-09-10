/*****************************************************************
hwdefs.cpp

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

#include "hwdefs.h"

////////////////////////////
// RODOS HAL Interfaces //
////////////////////////////

HAL_GPIO TelecommandLED(LED_RED);
HAL_GPIO TelemetryLED(LED_BLUE);

HAL_UART TeleBT(BLUETOOTH_UART);

HAL_PWM  MotorPWM1(PWM_IDX02);		  // TIM1 CH3 - PE13
HAL_PWM  MotorPWM2(PWM_IDX01);		  // TIM1 CH2 - PE11










