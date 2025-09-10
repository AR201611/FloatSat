/*****************************************************************
Telecommand.cpp

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "Telecommand.h"

static Application module01("Atheel2", 2001);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t ReceiveState = 0;
uint8_t SignFlag = 0;
uint8_t	DotFlag = 0;
uint8_t DataIndex = 0;
char TelecommandID;
char ReceiveData[MaxLength];
/* Private function prototypes -----------------------------------------------*/
/* PTRU Components 1570694 Litze LiY 1 x 0.14mm² rotrivate functions ---------------------------------------------------------*/

extern HAL_UART TeleBT;
extern HAL_GPIO TelecommandLED;




namespace RODOS {
extern HAL_UART uart_stdout;
}
#define TeleUART uart_stdout


sTelecommandData TelecommandData;

uint8_t Decode(uint8_t RxBuffer)
{
	uint8_t success=0;

	switch (ReceiveState){

	case 0:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		break;

	case 1:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else {
			TelecommandID = RxBuffer;
			ReceiveState = 2;
		}
		break;

	case 2:
		if (RxBuffer=='+' || RxBuffer=='-')
		{
			if (SignFlag==0 && DataIndex==0)
				{
				SignFlag=1;
				ReceiveData[DataIndex]=RxBuffer;
				DataIndex++;
				ReceiveState = 2;
				}
			else {ReceiveState = 0;}
		}
		else if (RxBuffer=='.')
		{
			if (DotFlag==0)
				{
				DotFlag=1;
				ReceiveData[DataIndex]=RxBuffer;
				DataIndex++;
				ReceiveState = 2;
				}
			else {ReceiveState = 0;}
		}
		else if (RxBuffer>='0' && RxBuffer<='9')
		{
		    ReceiveData[DataIndex]=RxBuffer;
		    DataIndex++;
			if (DataIndex > MaxLength) {ReceiveState = 0;}
			else {ReceiveState = 2;}
		}
		else if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else if (RxBuffer==TelecommandStop)
		{
			ReceiveData[DataIndex]= 0x00;
			success=Command(TelecommandID);
			ReceiveState=0;
		}
		else { ReceiveState=0;}
		break;
	default:
		ReceiveState=0;
		break;
	}
	return success;
}

uint8_t Decode2(uint8_t RxBuffer)
{
	uint8_t success=0;

	switch (ReceiveState){

	case 0:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		break;

	case 1:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else {
			TelecommandID = RxBuffer;
			ReceiveState = 2;
		}
		break;

	case 2:
		if (RxBuffer=='+' || RxBuffer=='-')
		{
			if (SignFlag==0 && DataIndex==0)
				{
				SignFlag=1;
				ReceiveData[DataIndex]=RxBuffer;
				DataIndex++;
				ReceiveState = 2;
				}
			else {ReceiveState = 0;}
		}
		else if (RxBuffer=='.')
		{
			if (DotFlag==0)
				{
				DotFlag=1;
				ReceiveData[DataIndex]=RxBuffer;
				DataIndex++;
				ReceiveState = 2;
				}
			else {ReceiveState = 0;}
		}
		else if (RxBuffer>='0' && RxBuffer<='9')
		{
		    ReceiveData[DataIndex]=RxBuffer;
		    DataIndex++;
			if (DataIndex > MaxLength) {ReceiveState = 0;}
			else {ReceiveState = 2;}
		}
		else if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else if (RxBuffer==TelecommandStop)
		{
			ReceiveData[DataIndex]= 0x00;
			success=Command(TelecommandID);
			ReceiveState=0;
		}
		else { ReceiveState=0;}
		break;
	default:
		ReceiveState=0;
		break;
	}
	return success;
}

uint8_t Command(uint8_t TelecommandID)
{
	char string[40];

	switch (TelecommandID){

	case TelemetryID:
		TelecommandData.Telemetry = (bool)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		return 1;

	case SystemModeID:
		TelecommandData.SystemMode = (uint8_t)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
	    return 1;

	case AHRSModeID:
		TelecommandData.AHRSMode = (uint8_t)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		TelecommandData.TelecommandFlag=1;
		TelecommandDataTopic.publish(TelecommandData);
		TelecommandData.TelecommandFlag=0;
	    return 1;

	case VelocityID:
		TelecommandData.Velocity = (float)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
	    return 1;

	case PositionID:
		TelecommandData.Position = (float)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
	    return 1;

	case MotorSpeedID:
		TelecommandData.MotorSpeed = (int32_t)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		return 1;

	case ControllerID:
		TelecommandData.Controller = (uint8_t)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		return 1;

	case ControllerParameterID:
		TelecommandData.ControllerParameter = (uint8_t)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		return 1;

	case ControllerParameterGainID:
		TelecommandData.ControllerParameterGain = (float)(atof(ReceiveData));
		//TeleUART.write((char*)TelecommandAck, 1);
		TelecommandDataTopic.publish(TelecommandData);
		TelecommandData.Controller = 0;
		TelecommandData.ControllerParameter = 0;
		TelecommandData.ControllerParameterGain = 0;
		return 1;

	default:
		return 0;
	}
}

class Telecommand: public Thread {

public:

	Telecommand(const char* name) : Thread(name) {
	}

	void init() {
		TelecommandLED.init(true, 1, 0);
	}



	void run(){

		char RxBuffer;

		while (1)
		{
			TeleUART.suspendUntilDataReady();

            TelecommandLED.setPins(~TelecommandLED.readPins());

            TeleUART.read(&RxBuffer,1);

            Decode(RxBuffer);
		}
	}
};

Telecommand Telecommand("Telecommand");

class TelecommandBT: public Thread {

public:

	TelecommandBT(const char* name) : Thread(name) {
	}

	void init() {
		TelecommandLED.init(true, 1, 0);
		TeleBT.init(115200);
		TeleBT.config(UART_PARAMETER_ENABLE_DMA, 1);
	}



	void run(){

		char RxBuffer;

		while (1)
		{
			TeleBT.suspendUntilDataReady();

            TelecommandLED.setPins(~TelecommandLED.readPins());

            TeleBT.read(&RxBuffer,1);

            Decode2(RxBuffer);
		}
	}
};

TelecommandBT TelecommandBT("TelecommandBT");
