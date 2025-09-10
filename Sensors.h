/*****************************************************************
Sensors.h

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

#ifndef Sensors_H_
#define Sensors_H_

/* Includes ------------------------------------------------------------------*/
#include "LSM9DS1.h"
#include "math.h"
#include "stdint.h"
#include "topics.h"
#include "stm32f4xx_conf.h"


/* Exported types ------------------------------------------------------------*/
enum AHRS_Mode
{
	Gyro_Cal = 1,
	Accel_Cal = 2,
	Mag_Cal = 3,
	Gyro_Update = 4,
	Gyro_Quaternion_Update = 5,
	Acc_Mag_Tilted_Compass_Update = 6,
	Madgwick_Quaternion_Update = 7,
	Mahony_Quaternion_Update = 8,
	Kalman_Quaternion_Update = 9,
};

/* Exported functions ------------------------------------------------------- */

void AHRSUpdate(AHRS_Mode mode);

void GyroUpdate(float gx, float gy, float gz);

void GyroQuaternionUpdate(float gx, float gy, float gz);

void AccMagUpdate(float ax, float ay, float az, float mx, float my, float mz);

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void KalmanQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void Quaternion2Euler();

void MagCal(float* magMax, float* magMin, float magCalTime);

void GyroCal(float* gbias);

void AccelCal(float* abias);

void MotorSpeedUpdate();

void ADCUpdate();

void EncoderInit();

#endif /* Sensors_H_ */
