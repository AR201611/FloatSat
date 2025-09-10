/*****************************************************************
topics.cpp

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/

#include "rodos.h"
#include "topics.h"

Topic<sSensorData> SensorDataTopic(-1,"Sensor Data");
Topic<sTelecommandData> TelecommandDataTopic(-1,"Telecommand Data");


