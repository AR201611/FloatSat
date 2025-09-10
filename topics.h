/*****************************************************************
topics.h

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

#ifndef __topics_h__
#define __topics_h_

/* Includes ------------------------------------------------------------------*/
#include "matlib.h"

/* Exported types ------------------------------------------------------------*/
struct sSensorData{
	 int16_t RawDataGx, RawDataGy, RawDataGz; // x, y, and z axis raw data of the gyroscope
	 int16_t RawDataAx, RawDataAy, RawDataAz; // x, y, and z axis raw data of the accelerometer
	 int16_t RawDataMx, RawDataMy, RawDataMz; // x, y, and z axis raw data of the magnetometer
     float gx, gy, gz; // x, y, and z axis readings of the gyroscope in deg/s
     float ax, ay, az; // x, y, and z axis readings of the accelerometer in g
     float mx, my, mz; // x, y, and z axis readings of the magnetometer in Gs
     float temperature; // on-board temperature reading in degree
     float pitch, yaw, roll;
     float q[4];
     float motorSpeed;  // motor speed in RPM
     float motorCurrent;  // motor current in mA
     double deltaTime;
   };

struct sTelecommandData
{
	bool Telemetry;
	uint8_t SystemMode;
	uint8_t AHRSMode;
	uint8_t Controller;
	uint8_t ControllerParameter;
	float ControllerParameterGain;
	float Velocity;
	float Position;
	int32_t MotorSpeed;
	uint8_t TelecommandFlag;
};

extern Topic<sSensorData> 		SensorDataTopic;
extern Topic<sTelecommandData>  TelecommandDataTopic;

#endif

