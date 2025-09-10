/*****************************************************************
hwdefs.h

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

#ifndef __hwdefs_h__
#define __hwdefs_h__

/* Includes --------------------------------------------------------*/
#include "rodos.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"

/* BTM182 BlueTooth ------------------------------------------------*/
#define BLUETOOTH_UART     UART_IDX2				// UART2 - RTS:PD4,  CTS:PD3,  RX:PD6, TX:PD5
#define BLUETOOTH_RESET    GPIO_047 			    // PC15: Active-Low (Reset is disabled, when GPIO is floating)

/* FT232RL USB to Serial / WF121-A WiFi ----------------------------*/
#define USB_WIFI_UART 	   UART_IDX3                // UART3 - RTS:PB14, CTS:PD11, RX:PD9, TX:PD8
#define WIFI_POWER_EN      GPIO_046  			    // PC14: Active-High
#define WIFI_SPI 	       SPI_IDX2                 // SPI2 - SCK:PB13, MISO:PC2, MOSI:PB15
#define WIFI_CS 	       GPIO_028                 // PB12: Wifi Chip Select pin

/* LED -------------------------------------------------------------*/
#define LED_GREEN          GPIO_060                 // PD12
#define LED_ORANGE         GPIO_061					// PD13
#define LED_RED            GPIO_062					// PD14
#define LED_BLUE           GPIO_063                 // PD15

/* OV9655 CAMERA / INA219A CURRENT MONITOR ---------------------------*/
#define CAMERA_CURRENT_I2C I2C_IDX1           		// I2C1 - SCL:PB8, SDA:PB9
#define CAMERA_DCMI		   DCMI_IDX1           		// DCMI1 - SOI_DO:PB9, SOI_Clk:PB8, HREF:PA4, VSYNC:PB7, ExtClock:PA8, PCLK:PA6, D0:PC6, D1:PC7, D2:PE0, D3:PE1, D4:PE4, D5:PB6, D6:PE5, D7:PE6
#define CAMERA_PWDN        GPIO_033 				// PC01: Active-High
#define CAMERA_RESET       GPIO_010					// PA10: Active-Low (Reset is disabled, when GPIO is floating)

#define CURRENT_TOTAL      0x40                     // Current Sensor 7-bit I2C Address of the Board
#define CURRENT_HBRIDGE_A  0x41                     // Current Sensor 7-bit I2C Address of H-Bridge A
#define CURRENT_HBRIDGE_B  0x42                     // Current Sensor 7-bit I2C Address of H-Bridge B
#define CURRENT_HBRIDGE_C  0x43                     // Current Sensor 7-bit I2C Address of H-Bridge C
#define CURRENT_HBRIDGE_D  0x44                     // Current Sensor 7-bit I2C Address of H-Bridge D
#define CURRENT_SERVO_1    0x45                     // Current Sensor 7-bit I2C Address of Servo 1
#define CURRENT_SERVO_2    0x46                     // Current Sensor 7-bit I2C Address of Servo 2
#define CURRENT_SERVO_3    0x47                     // Current Sensor 7-bit I2C Address of Servo 3
#define CURRENT_SERVO_4    0x48                     // Current Sensor 7-bit I2C Address of Servo 4

/* LSM9DS0 IMU -----------------------------------------------------*/
#define IMU_I2C			   I2C_IDX2	                // I2C2 - SCL:PB10, SDA:PB11
#define IMU_SPI	           SPI_IDX1                 // SPI1 - SCK:PB3, MISO:PB4, MOSI:PB5
#define IMU_EN             GPIO_055 			    // PD7: Active-High
#define IMU_G_CS           GPIO_018 			    // PB2: Gyro Chip Select pin
#define IMU_XM_CS          GPIO_032 			    // PC0: Accelerometer + Magnetometer Chip Select pin

#define LSM9DS0_XM         0x1D                     // Accelerometer + Magnetometer 7-bit I2C Address (Would be 0x1E if SDO_XM is LOW)
#define LSM9DS0_G          0x6B                     // Gyro 7-bit I2C Address (Would be 0x6A if SDO_G is LOW)

/* VNH3SP30 H-Bridge -----------------------------------------------*/
#define HBRIDGE_POWER_EN   GPIO_066 				// PE2: Active-High
#define HBRIDGE_POWER_ON   GPIO_067 				// PE3: High indicate Power On

#define HBRIDGE_A_PWM      PWM_IDX12 	            // TIM4 CH1 - PD12
#define HBRIDGE_B_PWM      PWM_IDX13			    // TIM4 CH2 - PD13
#define HBRIDGE_C_PWM      PWM_IDX14				// TIM4 CH3 - PD14
#define HBRIDGE_D_PWM      PWM_IDX15				// TIM4 CH4 - PD15

#define HBRIDGE_A_INA      GPIO_036					// PC4
#define HBRIDGE_A_INB      GPIO_017				    // PB1
#define HBRIDGE_A_EN       GPIO_060 				// PD12

#define HBRIDGE_B_INA      GPIO_016 				// PB0
#define HBRIDGE_B_INB      GPIO_071 				// PE7
#define HBRIDGE_B_EN       GPIO_061 			    // PD13

#define HBRIDGE_C_INA      GPIO_072 				// PE8
#define HBRIDGE_C_INB      GPIO_074 				// PE10
#define HBRIDGE_C_EN       GPIO_062 				// PD14

#define HBRIDGE_D_INA      GPIO_076					// PE12
#define HBRIDGE_D_INB      GPIO_079 				// PE15
#define HBRIDGE_D_EN       GPIO_063 				// PD15

/* Servo -----------------------------------------------------------*/
#define SERVO_1_PWM    	   PWM_IDX00 				// TIM1 CH1 - PE9
#define SERVO_2_PWM    	   PWM_IDX01				// TIM1 CH2 - PE11
#define SERVO_3_PWM   	   PWM_IDX02				// TIM1 CH3 - PE13
#define SERVO_4_PWM    	   PWM_IDX03				// TIM1 CH4 - PE14

/* ADC -------------------------------------------------------------*/
#define ADC1               ADC_IDX1       			// ADC1
#define CurrentADC         ADC_CH_001     			// PA1
//#define SolarCurrentADC    ADC_CH_002     			// PA2
//#define ADC_CHANNEL_3      ADC_CH_003     		// PA3
//#define ADC_CHANNEL_5      ADC_CH_005    			// PA5

#define ADCRes              4095                     // ADC full scale resolution of 12 bits = 2^12-1 = 4095
#define ADCRef              3000.0                   // ADC adc reference voltage  = 3V = 3000mV
#define CurrentVoltageRatio 500                      //Current-sense feedback voltage of approximately 500 mV per amp


/* CAN -------------------------------------------------------------*/
#define CAN                CAN_IDX1       			// CAN1 - RXD:PD0, TXD:PD1

#endif

