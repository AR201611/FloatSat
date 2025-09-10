/*****************************************************************
Modes.cpp

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

#include "Modes.h"

static Application module01("Atheel", 2001);

uint32_t ModesPeriod = 20; // Modes period in ms

CommBuffer<sSensorData> ModesSensorDataBuffer;
Subscriber ModesSensorSubscriber(SensorDataTopic, ModesSensorDataBuffer);

CommBuffer<sTelecommandData> ModesTelecommandDataBuffer;
Subscriber ModesTelecommandDataSubscriber(TelecommandDataTopic, ModesTelecommandDataBuffer);

//extern HAL_UART TeleUART;
//extern HAL_GPIO PowerOn;
extern HAL_GPIO MotorINA;
extern HAL_GPIO MotorINB;
extern HAL_PWM  MotorPWM1;
extern HAL_PWM  MotorPWM2;



namespace RODOS {
extern HAL_UART uart_stdout;
}
#define TeleUART uart_stdout



sSensorData SensorDataReceiver;

sTelecommandData TelecommandDataReceiver;


sPID_Data PID_Velocity  = {0, 0, 0,                                  // float Kp, Ki, Kd;
					      0, 0, 0, 									 // float P, I, D;
					      0, 0, 0, 0, 0, 0,                          // float e, e_1, Upid, Upid_1, Usat, Usat_1;
					      4000, -4000, (float)ModesPeriod/1000,      // float Umax, Umin, T;
                          1};                                        // bool AntiWindup;

sPID_Data PID_Position = {0, 0, 0,
					      0, 0, 0,
					      0, 0, 0, 0, 0, 0,
					      4000, -4000, (float)ModesPeriod/1000,
						  1};

sPID_Data PID_MotorSpeed = {0, 0, 0,
					       0, 0, 0,
					       0, 0, 0, 0, 0, 0,
					       1000, -1000, (float)ModesPeriod/1000,
						   1};


float VelocityError, PositionError, MotorSpeedError;
                                                                                 ;

void PID(sPID_Data* PID, float error)
{

}

void MotorSet(float Value)
{

}


void StandbyMode()
{

}

void VelocityMode()
{

}

void PositionMode()
{

}

void MotorSpeedMode()
{

}

void MissionMode()
{

}


class Modes: public Thread {
	uint64_t periode;

public:

	Modes(const char* name, uint64_t periode) : Thread(name) {
		this->periode = periode;
	}

	void init() {
		MotorPWM1.init(4000,1000);
		MotorPWM2.init(4000,1000);
	}

	void run(){

        init();

		TIME_LOOP(0, periode){

			ModesSensorDataBuffer.get(SensorDataReceiver);

			switch (TelecommandDataReceiver.SystemMode)
			{
			case Standby_Mode:
				StandbyMode();
				break;
			case Velocity_Mode:
				VelocityMode();
				break;
			case Position_Mode:
				PositionMode();
				break;
			case MotorSpeed_Mode:
				MotorSpeedMode();
				break;
			case Mission_Mode:
				MissionMode();
				break;
			default:
				break;
			}
		}
	}
};



Modes Modes("Modes", ModesPeriod * MILLISECONDS);




