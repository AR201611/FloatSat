/*****************************************************************
Modes.h

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

#ifndef Modes_H_
#define Modes_H_

/* Includes ------------------------------------------------------------------*/
#include "rodos.h"
#include "hal.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "hwdefs.h"
#include "Sensors.h"

/* Exported types ------------------------------------------------------------*/
enum Operation_Mode
{
	Standby_Mode = 0,
	Velocity_Mode = 1,
	Position_Mode = 2,
	MotorSpeed_Mode = 3,
	Mission_Mode = 4,
};

enum Controller
{
	Velocity_Controller = 1,
	Position_Controller = 2,
	MotorSpeed_Controller = 3,
};

enum ControllerParameter
{
	Kp = 1,
	Ki = 2,
	Kd = 3,
};

struct sPID_Data
{
	 float Kp, Ki, Kd;

	 float P, I, D;

	 float e, e_1, Upid, Upid_1, Usat, Usat_1;

	 float Umax, Umin, T;

	 bool AntiWindup;
};

/* Exported functions ------------------------------------------------------- */
void PID(sPID_Data* PID, float error);

void MotorSet(float Value);

void StandbyMode();

void VelocityMode();

void PositionMode();

void MotorSpeedMode();

void MissionMode();

#endif /* Modes_H_ */
