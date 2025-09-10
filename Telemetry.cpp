/*****************************************************************
Telemetry.cpp

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

#include "Telemetry.h"

static Application module01("Atheel", 2001);

uint32_t TelemetryPeriod = 1000; // Telemetry period in ms

CommBuffer<sSensorData> SensorDataBuffer;
Subscriber SensorDataSubscriber(SensorDataTopic, SensorDataBuffer);

CommBuffer<sTelecommandData> TelecommandDataBuffer;
Subscriber TelecommandDataSubscriber(TelecommandDataTopic, TelecommandDataBuffer);

extern HAL_UART TeleBT;
extern HAL_GPIO TelemetryLED;


namespace RODOS {
extern HAL_UART uart_stdout;
}
#define TeleUART uart_stdout


char string[100];

class Telemetry: public Thread {
	uint64_t periode;
	sSensorData SensorDataReceiver;
	sTelecommandData TelecommandDataReceiver;
public:

	Telemetry(const char* name, uint64_t periode) : Thread(name) {
		this->periode = periode;
	}

	void init() {
		TelemetryLED.init(true, 1, 1);
	}

	void write(const char *buf, int size){
	    int sentBytes=0;
	    int retVal;
	    while(sentBytes < size){
	        retVal = TeleUART.write(&buf[sentBytes],size-sentBytes);
	        if (retVal < 0){
	            PRINTF("UART sent error\n");
	        }else{
	            sentBytes+=retVal;
	        }
	    }
	}

	void run(){

		init();


		TIME_LOOP(0, periode){


			TelemetryLED.setPins(~TelemetryLED.readPins());

			SensorDataBuffer.get(SensorDataReceiver);

			TelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);

			if (TelecommandDataReceiver.Telemetry)

			{

			sprintf(string,"ax = %f, ay = %f, az = %f mg \r", SensorDataReceiver.ax, SensorDataReceiver.ay, SensorDataReceiver.az);
		    write(string,strlen(string));

		    sprintf(string,"gx = %f, gy = %f, gz = %f deg/s \r",SensorDataReceiver.gx,SensorDataReceiver.gy,SensorDataReceiver.gz);
			write(string,strlen(string));

		    sprintf(string,"mx = %f, my = %f, mz = %f mG \r",SensorDataReceiver.mx,SensorDataReceiver.my, SensorDataReceiver.mz);
			write(string,strlen(string));

			sprintf(string,"temperature = %f deg \r",SensorDataReceiver.temperature);
			write(string,strlen(string));

			sprintf(string,"Motor Speed = %f rpm \r", SensorDataReceiver.motorSpeed);
			write(string,strlen(string));

			sprintf(string,"Motor Current = %f mA \r", SensorDataReceiver.motorCurrent);
			write(string,strlen(string));

		    sprintf(string,"Yaw = %f, Pitch = %f, Roll = %f \r",SensorDataReceiver.yaw,SensorDataReceiver.pitch,SensorDataReceiver.roll);
			write(string,strlen(string));

			}
		}
	}
};

Telemetry Telemetry("Telemetry", TelemetryPeriod * MILLISECONDS);

class TelemetryBT: public Thread {
	uint64_t periode;
	sSensorData SensorDataReceiver;
	sTelecommandData TelecommandDataReceiver;
public:

	TelemetryBT(const char* name, uint64_t periode) : Thread(name) {
		this->periode = periode;
	}

	void init() {
		TelemetryLED.init(true, 1, 1);
	}

	void write(const char *buf, int size){
	    int sentBytes=0;
	    int retVal;
	    while(sentBytes < size){
	        retVal = TeleBT.write(&buf[sentBytes],size-sentBytes);
	        if (retVal < 0){
	            PRINTF("UART sent error\n");
	        }else{
	            sentBytes+=retVal;
	        }
	    }
	}

	void run(){

		init();


		TIME_LOOP(0, periode){


			TelemetryLED.setPins(~TelemetryLED.readPins());

			SensorDataBuffer.get(SensorDataReceiver);

			TelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);

			if (TelecommandDataReceiver.Telemetry)

			{

			sprintf(string,"ax = %f, ay = %f, az = %f mg \r", SensorDataReceiver.ax, SensorDataReceiver.ay, SensorDataReceiver.az);
		    write(string,strlen(string));

		    sprintf(string,"gx = %f, gy = %f, gz = %f deg/s \r",SensorDataReceiver.gx,SensorDataReceiver.gy,SensorDataReceiver.gz);
			write(string,strlen(string));

		    sprintf(string,"mx = %f, my = %f, mz = %f mG \r",SensorDataReceiver.mx,SensorDataReceiver.my, SensorDataReceiver.mz);
			write(string,strlen(string));

			sprintf(string,"temperature = %f deg \r",SensorDataReceiver.temperature);
			write(string,strlen(string));

			sprintf(string,"Motor Speed = %f rpm \r", SensorDataReceiver.motorSpeed);
			write(string,strlen(string));

			sprintf(string,"Motor Current = %f mA \r", SensorDataReceiver.motorCurrent);
			write(string,strlen(string));

		    sprintf(string,"Yaw = %f, Pitch = %f, Roll = %f \r",SensorDataReceiver.yaw,SensorDataReceiver.pitch,SensorDataReceiver.roll);
			write(string,strlen(string));

			}
		}
	}
};

TelemetryBT TelemetryBT("TelemetryBT", TelemetryPeriod * MILLISECONDS);






