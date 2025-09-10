/*****************************************************************
Sensors.cpp

Original Created by: Atheel Redah
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4
*****************************************************************/


#include "Sensors.h"
#include "LSM9DS1.h"

static Application module01("LSM9DS1 AHRS", 2001);

/* ADC -------------------------------------------------------------*/
#define ADC01               ADC_IDX1       			// ADC1
#define ADCChannel          ADC_CH_001     			// PA1
#define ADCRes              4095                    // ADC full scale resolution of 12 bits = 2^12-1 = 4095
#define ADCRef              3000.0             // ADC adc reference voltage  = 3V = 3000mV
#define CurrentVoltageRatio 500                     //Current-sense feedback voltage of approximately 500 mV per amp

HAL_ADC CurrentADC(ADC01);

CommBuffer<sTelecommandData> SensorsTelecommandDataBuffer;
Subscriber SensorsTelecommandDataSubscriber(TelecommandDataTopic, SensorsTelecommandDataBuffer);

sSensorData sensorData = {0, 0, 0,	   // int16_t RawDataGx, RawDataGy, RawDataGz
						  0, 0, 0,	   // int16_t RawDataAx, RawDataAy, RawDataAz
						  0, 0, 0,     // int16_t RawDataMx, RawDataMy, RawDataMz
						  0, 0, 0,     // float gx, gy, gz
						  0, 0, 0,     // float ax, ay, az
                          0, 0, 0,     // float mx, my, mz
                          0,           // float temperature
						  0, 0, 0,     // float pitch, yaw, roll;
						  {1, 0, 0, 0},// float q[4];
						  0, 		   // float motorSpeed;
						  0            // double deltaTime;
};

// Create an instance of the LSM9DS1 library called `imu`.
LSM9DS1 imu;

HAL_GPIO LSM9DS1_CSAG(GPIO_006); //PA6
HAL_GPIO LSM9DS1_CSM(GPIO_041);  //PC9
HAL_I2C  LSM9DS1_I2C(I2C_IDX1);
HAL_SPI  LSM9DS1_SPI(SPI_IDX1);

AHRS_Mode AHRSMode = Mahony_Quaternion_Update;
AHRS_Mode AHRSModeLast;

uint64_t SensorsPeriod = 5; // Sensors period in ms

uint64_t lastTime = 0;
uint64_t timeNow = 0;
uint64_t startTime = 0;

uint8_t MagCalState=0;

// global constants for 9DOF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError M_PI * (400.0f / 180.0f)     // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift M_PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 4.0f * 5.0f // This is Kp proportional feedback parameter in the Mahony filter and fusion scheme.
#define Ki 0.0f        // This is Ki integral feedback parameter in the Mahony filter and fusion scheme.
float eInt[3] = {0.0f, 0.0f, 0.0f};       // integral error vector for Mahony method

float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0}, magData[3] = {0, 0, 0};

float magMax[3] = {1000, 1000, 1000}, magMin[3] = {-1000, -1000, -1000};


void AHRSUpdate(AHRS_Mode mode)
{
	switch (mode)
	{

	case Gyro_Cal:
		GyroCal(gbias);
		AHRSMode = AHRSModeLast;
		break;

	case Accel_Cal:
		AccelCal(abias);
		AHRSMode = AHRSModeLast;
		break;

	case Mag_Cal:
		MagCal(magMax, magMin, 10);
		break;

	case Gyro_Update:

		imu.readGyro();    // Read raw gyro data
	    sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
		sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
		sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

		timeNow = NOW();
		sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
		lastTime = timeNow;

		GyroUpdate(sensorData.gx, sensorData.gy, sensorData.gz);
		break;

	case Gyro_Quaternion_Update:

		imu.readGyro();           // Read raw gyro data
		sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
		sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
		sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

		timeNow = NOW();
		sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
		lastTime = timeNow;

		GyroQuaternionUpdate(sensorData.gx, sensorData.gy, sensorData.gz);
		Quaternion2Euler();
		break;

	case Acc_Mag_Tilted_Compass_Update:

		imu.readAccel();         // Read raw accelerometer data
		sensorData.ax =imu.calcAccel(imu.ax -  abias[0]);   // Convert to g's, remove accelerometer biases
		sensorData.ay =imu.calcAccel(imu.ay -  abias[1]);
		sensorData.az =imu.calcAccel(imu.az -  abias[2]);

		imu.readMag();           // Read raw magnetometer data
		sensorData.mx =imu.calcMag(imu.mx);     // Convert to Gauss
		sensorData.my =imu.calcMag(imu.my);
		sensorData.mz =imu.calcMag(imu.mz);

		magData[0]=(float)(imu.mx-magMin[0])/(float)(magMax[0]-magMin[0])*2-1;
		magData[1]=(float)(imu.my-magMin[1])/(float)(magMax[1]-magMin[1])*2-1;
		magData[2]=(float)(imu.mz-magMin[2])/(float)(magMax[2]-magMin[2])*2-1;

		//The LSM9DS1's magnetometer x and y axes are opposite to the accelerometer, so my and mx are substituted for each other.
		AccMagUpdate((float)(imu.ax -  abias[0]), (float)(imu.ay -  abias[1]), (float)(imu.az -  abias[2]), -magData[1], -magData[0], magData[2]);
		break;

	case Madgwick_Quaternion_Update:

		imu.readGyro();           // Read raw gyro data
		sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
		sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
		sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

		imu.readAccel();         // Read raw accelerometer data
		sensorData.ax =imu.calcAccel(imu.ax -  abias[0]);   // Convert to g's, remove accelerometer biases
		sensorData.ay =imu.calcAccel(imu.ay -  abias[1]);
		sensorData.az =imu.calcAccel(imu.az -  abias[2]);

		imu.readMag();           // Read raw magnetometer data
		sensorData.mx =imu.calcMag(imu.mx);     // Convert to Gauss
		sensorData.my =imu.calcMag(imu.my);
		sensorData.mz =imu.calcMag(imu.mz);

		magData[0]=(float)(imu.mx-magMin[0])/(float)(magMax[0]-magMin[0])*2-1;
		magData[1]=(float)(imu.my-magMin[1])/(float)(magMax[1]-magMin[1])*2-1;
		magData[2]=(float)(imu.mz-magMin[2])/(float)(magMax[2]-magMin[2])*2-1;

		imu.readTemp();
		sensorData.temperature = (float)imu.temperature;

		timeNow = NOW();
		sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
		lastTime = timeNow;

		//The LSM9DS1's magnetometer x and y axes are opposite to the accelerometer, so my and mx are substituted for each other.
		MadgwickQuaternionUpdate(sensorData.ax, sensorData.ay, sensorData.az, sensorData.gx, sensorData.gy, sensorData.gz, -magData[1], -magData[0], magData[2]);  // Pass gyro rate as rad/s
		Quaternion2Euler();
		break;

	case Mahony_Quaternion_Update:

		imu.readGyro();           // Read raw gyro data
		sensorData.gx =round(imu.calcGyro(imu.gx - gbias[0]));   // Convert to degrees per seconds, remove gyro biases
		sensorData.gy =round(imu.calcGyro(imu.gy - gbias[1]));
		sensorData.gz =round(imu.calcGyro(imu.gz - gbias[2]));

		imu.readAccel();         // Read raw accelerometer data
		sensorData.ax =imu.calcAccel(imu.ax -  abias[0]);   // Convert to g's, remove accelerometer biases
		sensorData.ay =imu.calcAccel(imu.ay -  abias[1]);
		sensorData.az =imu.calcAccel(imu.az -  abias[2]);

		imu.readMag();           // Read raw magnetometer data
		sensorData.mx =imu.calcMag(imu.mx);     // Convert to Gauss
		sensorData.my =imu.calcMag(imu.my);
		sensorData.mz =imu.calcMag(imu.mz);

		magData[0]=(float)(imu.mx-magMin[0])/(float)(magMax[0]-magMin[0])*2-1;
		magData[1]=(float)(imu.my-magMin[1])/(float)(magMax[1]-magMin[1])*2-1;
		magData[2]=(float)(imu.mz-magMin[2])/(float)(magMax[2]-magMin[2])*2-1;

		imu.readTemp();          // Read raw temperature data
		sensorData.temperature = (float)imu.temperature;

		timeNow = NOW();
		sensorData.deltaTime = ((timeNow - lastTime) / (double)SECONDS);
		lastTime = timeNow;

		//The LSM9DS1's magnetometer x and y axes are opposite to the accelerometer, so my and mx are substituted for each other.
		MahonyQuaternionUpdate(sensorData.ax, sensorData.ay, sensorData.az, sensorData.gx, sensorData.gy, sensorData.gz, -magData[1], -magData[0], magData[2]);  // Pass gyro rate as rad/s
		Quaternion2Euler();
		break;
	}
}

// This function will calculate your LSM9DS0' orientation angles based on the gyroscope data.
void GyroUpdate(float gx, float gy, float gz)
{
	sensorData.pitch = sensorData.pitch + gx * sensorData.deltaTime;
	sensorData.roll  = sensorData.roll  + gy * sensorData.deltaTime;
	sensorData.yaw   = sensorData.yaw   + gz * sensorData.deltaTime;
}

// This function will calculate quaternion-based estimation of absolute device orientation using gyroscope data.
void GyroQuaternionUpdate(float gx, float gy, float gz)
{
	float q1 = sensorData.q[0], q2 = sensorData.q[1], q3 = sensorData.q[2], q4 = sensorData.q[3];   // short name local variable for readability
	float norm;
	float qDot1, qDot2, qDot3, qDot4;

	gx=gx*M_PI/180.0f;
	gy=gy*M_PI/180.0f;
	gz=gz*M_PI/180.0f;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

	// Integrate to yield quaternion
	q1 += qDot1 * sensorData.deltaTime;
	q2 += qDot2 * sensorData.deltaTime;
	q3 += qDot3 * sensorData.deltaTime;
	q4 += qDot4 * sensorData.deltaTime;

	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;

	sensorData.q[0] = q1 * norm;
	sensorData.q[1] = q2 * norm;
	sensorData.q[2] = q3 * norm;
	sensorData.q[3] = q4 * norm;
}

void Quaternion2Euler()
{
	sensorData.yaw   = atan2(2.0f * (sensorData.q[1] * sensorData.q[2] + sensorData.q[0] * sensorData.q[3]), sensorData.q[0] * sensorData.q[0] + sensorData.q[1] * sensorData.q[1] - sensorData.q[2] * sensorData.q[2] - sensorData.q[3] * sensorData.q[3]);
	sensorData.pitch = -asin(2.0f * (sensorData.q[1] * sensorData.q[3] - sensorData.q[0] * sensorData.q[2]));
	sensorData.roll  = atan2(2.0f * (sensorData.q[0] * sensorData.q[1] + sensorData.q[2] * sensorData.q[3]), sensorData.q[0] * sensorData.q[0] - sensorData.q[1] * sensorData.q[1] - sensorData.q[2] * sensorData.q[2] + sensorData.q[3] * sensorData.q[3]);
	sensorData.pitch*= 180.0f / M_PI;
	sensorData.roll *= 180.0f / M_PI;
	sensorData.yaw  *= 180.0f / M_PI;

	if (sensorData.yaw < 0)
	{
	sensorData.yaw=sensorData.yaw+360;
	}
}

// This function will calculate your IMU orientation based on accelerometer and magnetometer data.
void AccMagUpdate(float ax, float ay, float az, float mx, float my, float mz)
{
	float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch;
	float fTiltedX,fTiltedY;
	float fAcc[3];

	 //Rescale the accelerometer radings (in order to increase accuracy in the following norm computation)
	fAcc[0] = ax/100.0; fAcc[1] = ay/100.0; fAcc[2] = az/100.0;

	 //Compute the scaled acceleration vector norm
	fNormAcc = sqrt(pow(fAcc[0],2)+pow(fAcc[1],2)+pow(fAcc[2],2));

	 //Compute some useful parameters for the g vector rotation matrix
	fSinRoll=fAcc[1]/sqrt(pow(fAcc[1],2)+pow(fAcc[2],2));
	fCosRoll=sqrt(1.0-fSinRoll*fSinRoll);
	fSinPitch=-fAcc[0]/fNormAcc;
	fCosPitch=sqrt(1.0-fSinPitch*fSinPitch);

	 //Apply the rotation matrix to the magnetic field vector to obtain the X and Y components on the earth plane
	fTiltedX = mx * fCosPitch + mz * fSinPitch;
	fTiltedY = mx * fSinRoll * fSinPitch + my * fCosRoll - mz * fSinRoll * fCosPitch;


	 //return the roll, pitch and heading angles expressed in rad
	sensorData.yaw = -atan2(fTiltedY,fTiltedX);
	sensorData.roll  = atan2(fSinRoll,fCosRoll);
	sensorData.pitch = atan2(fSinPitch,fCosPitch);

	 //return the roll, pitch and heading angles expressed in degree
	sensorData.pitch*= 180.0f / M_PI;
	sensorData.roll *= 180.0f / M_PI;
	sensorData.yaw  *= 180.0f / M_PI;
	sensorData.yaw+= 2.22; // Declination at Würzburg, Germany is +2.22 degrees on 27.02.2015.
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	            float q1 = sensorData.q[0], q2 = sensorData.q[1], q3 = sensorData.q[2], q4 = sensorData.q[3];   // short name local variable for readability
	            float norm;
	            float hx, hy, _2bx, _2bz;
	            float s1, s2, s3, s4;
	            float qDot1, qDot2, qDot3, qDot4;

	            // Auxiliary variables to avoid repeated arithmetic
	            float _2q1mx;
	            float _2q1my;
	            float _2q1mz;
	            float _2q2mx;
	            float _4bx;
	            float _4bz;
	            float _2q1 = 2.0f * q1;
	            float _2q2 = 2.0f * q2;
	            float _2q3 = 2.0f * q3;
	            float _2q4 = 2.0f * q4;
	            float _2q1q3 = 2.0f * q1 * q3;
	            float _2q3q4 = 2.0f * q3 * q4;
	            float q1q1 = q1 * q1;
	            float q1q2 = q1 * q2;
	            float q1q3 = q1 * q3;
	            float q1q4 = q1 * q4;
	            float q2q2 = q2 * q2;
	            float q2q3 = q2 * q3;
	            float q2q4 = q2 * q4;
	            float q3q3 = q3 * q3;
	            float q3q4 = q3 * q4;
	            float q4q4 = q4 * q4;

	            gx=gx*M_PI/180.0f;
	            gy=gy*M_PI/180.0f;
	            gz=gz*M_PI/180.0f;

	            // Normalise accelerometer measurement
	            norm = sqrt(ax * ax + ay * ay + az * az);
	            if (norm == 0.0f) return; // handle NaN
	            norm = 1.0f/norm;
	            ax *= norm;
	            ay *= norm;
	            az *= norm;

	            // Normalise magnetometer measurement
	            norm = sqrt(mx * mx + my * my + mz * mz);
	            if (norm == 0.0f) return; // handle NaN
	            norm = 1.0f/norm;
	            mx *= norm;
	            my *= norm;
	            mz *= norm;

	            // Reference direction of Earth's magnetic field
	            _2q1mx = 2.0f * q1 * mx;
	            _2q1my = 2.0f * q1 * my;
	            _2q1mz = 2.0f * q1 * mz;
	            _2q2mx = 2.0f * q2 * mx;
	            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	            _2bx = sqrt(hx * hx + hy * hy);
	            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	            _4bx = 2.0f * _2bx;
	            _4bz = 2.0f * _2bz;

	            // Gradient decent algorithm corrective step
	            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	            norm = 1.0f/norm;
	            s1 *= norm;
	            s2 *= norm;
	            s3 *= norm;
	            s4 *= norm;

	            // Compute rate of change of quaternion
	            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	            // Integrate to yield quaternion
	            q1 += qDot1 * sensorData.deltaTime;
	            q2 += qDot2 * sensorData.deltaTime;
	            q3 += qDot3 * sensorData.deltaTime;
	            q4 += qDot4 * sensorData.deltaTime;
	            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	            norm = 1.0f/norm;
	            sensorData.q[0] = q1 * norm;
	            sensorData.q[1] = q2 * norm;
	            sensorData.q[2] = q3 * norm;
	            sensorData.q[3] = q4 * norm;
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	            float q1 = sensorData.q[0], q2 = sensorData.q[1], q3 = sensorData.q[2], q4 = sensorData.q[3];   // short name local variable for readability
	            float norm;
	            float hx, hy, bx, bz;
	            float vx, vy, vz, wx, wy, wz;
	            float ex, ey, ez;
	            float pa, pb, pc;

	            // Auxiliary variables to avoid repeated arithmetic
	            float q1q1 = q1 * q1;
	            float q1q2 = q1 * q2;
	            float q1q3 = q1 * q3;
	            float q1q4 = q1 * q4;
	            float q2q2 = q2 * q2;
	            float q2q3 = q2 * q3;
	            float q2q4 = q2 * q4;
	            float q3q3 = q3 * q3;
	            float q3q4 = q3 * q4;
	            float q4q4 = q4 * q4;

	            gx=gx*M_PI/180.0f;
	            gy=gy*M_PI/180.0f;
	            gz=gz*M_PI/180.0f;

	            // Normalise accelerometer measurement
	            norm = sqrt(ax * ax + ay * ay + az * az);
	            if (norm == 0.0f) return; // handle NaN
	            norm = 1.0f / norm;        // use reciprocal for division
	            ax *= norm;
	            ay *= norm;
	            az *= norm;

	            // Normalise magnetometer measurement
	            norm = sqrt(mx * mx + my * my + mz * mz);
	            if (norm == 0.0f) return; // handle NaN
	            norm = 1.0f / norm;        // use reciprocal for division
	            mx *= norm;
	            my *= norm;
	            mz *= norm;

	            // Reference direction of Earth's magnetic field
	            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	            bx = sqrt((hx * hx) + (hy * hy));
	            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	            // Estimated direction of gravity and magnetic field
	            vx = 2.0f * (q2q4 - q1q3);
	            vy = 2.0f * (q1q2 + q3q4);
	            vz = q1q1 - q2q2 - q3q3 + q4q4;
	            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	            // Error is cross product between estimated direction and measured direction of gravity
	            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	            if (Ki > 0.0f)
	            {
	                eInt[0] += ex;      // accumulate integral error
	                eInt[1] += ey;
	                eInt[2] += ez;
	            }
	            else
	            {
	                eInt[0] = 0.0f;     // prevent integral wind up
	                eInt[1] = 0.0f;
	                eInt[2] = 0.0f;
	            }

	            // Apply feedback terms
	            gx = gx + Kp * ex + Ki * eInt[0];
	            gy = gy + Kp * ey + Ki * eInt[1];
	            gz = gz + Kp * ez + Ki * eInt[2];

	            // Integrate rate of change of quaternion
	            pa = q2;
	            pb = q3;
	            pc = q4;
	            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * sensorData.deltaTime);
	            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * sensorData.deltaTime);
	            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * sensorData.deltaTime);
	            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * sensorData.deltaTime);

	            // Normalise quaternion
	            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	            norm = 1.0f / norm;
	            sensorData.q[0] = q1 * norm;
	            sensorData.q[1] = q2 * norm;
	            sensorData.q[2] = q3 * norm;
	            sensorData.q[3] = q4 * norm;
}

void MagCal(float* magMax, float* magMin, float magCalTime)
{
	switch (MagCalState){
	case 0:
		imu.readMag();
		magMax[0]=imu.mx; magMax[1]=imu.my; magMax[2]=imu.mz;
		magMin[0]=imu.mx; magMin[1]=imu.my; magMin[2]=imu.mz;
		PRINTF("Please rotate the magnetometer in all directions around the three axis within %f seconds. \r",magCalTime);
		startTime = NOW();
		MagCalState=1;
		break;
	case 1:
		imu.readMag();
		if (imu.mx > magMax[0]){magMax[0]=imu.mx;}
		else if (imu.mx < magMin[0]){magMin[0]=imu.mx;}

		if (imu.my > magMax[1]){magMax[1]=imu.my;}
		else if (imu.my < magMin[1]){magMin[1]=imu.my;}

		if (imu.mz > magMax[2]){magMax[2]=imu.mz;}
		else if (imu.mz < magMin[2]){magMin[2]=imu.mz;}

		if (((NOW() - startTime) / (double)SECONDS) > magCalTime)
		{
			PRINTF("mMax=[%f %f %f], mMin=[%f %f %f] \r",magMax[0],magMax[1],magMax[2],magMin[0],magMin[1],magMin[2]);
			MagCalState=0;
			AHRSMode=AHRSModeLast;
		}
		break;
	}
}

void GyroCal(float* gbias)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  float gyro_bias[3] = {0, 0, 0};
  float samples= 1000;

  PRINTF("Please make sure that the gyroscope is at standstill, the calibration will take about 5 seconds. \r\n");

  for(int i = 0; i < samples ; i++) {

	imu.readGyro();    // Read raw gyro data
    gyro_bias[0] += imu.gx;
    gyro_bias[1] += imu.gy;
    gyro_bias[2] += imu.gz;
    AT(NOW()+5*MILLISECONDS);
  }

  gbias[0] = round(gyro_bias[0] / samples); // average the data
  gbias[1] = round(gyro_bias[1] / samples);
  gbias[2] = round(gyro_bias[2] / samples);

  PRINTF("gxbias = %f, gybias = %f, gzbias = %f \r\n", gbias[0], gbias[1], gbias[2]);
}

void AccelCal(float* abias)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  float accel_bias[3] = {0, 0, 0};
  float samples= 1000;

  PRINTF("Please make sure that the accelerometer is standstill at x=0g, y=0g, z=1g position, the calibration will take about 5 seconds. \r\n");

  for(int i = 0; i < samples ; i++) {

	imu.readAccel();    // Read raw Accel data
	accel_bias[0] += imu.ax;
	accel_bias[1] += imu.ay;
	////accel_bias[2] += imu.az; // Assumes sensor facing up!;
    AT(NOW()+5*MILLISECONDS);
  }

  abias[0] = round(accel_bias[0] / samples); // average the data
  abias[1] = round(accel_bias[1] / samples);
  abias[2] = round(accel_bias[2] / samples);

  PRINTF("axbias = %f, aybias = %f, azbias = %f \r\n", abias[0], abias[1], abias[2]);
}

/* Private variables ---------------------------------------------------------*/
__IO uint32_t IC4ReadValue1 = 0, IC4ReadValue2 = 0, Capture = 0;
__IO uint8_t CaptureNumber = 0;
__IO uint32_t TIM2Freq = 0;
__IO uint8_t EncoderB;
__IO double CaptureTime;

void EncoderInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* TIM2 channel 4 pin (PA3) configuration for Encoder A (Yellow)*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect TIM pins to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

  /* Configure (PA5) pin as input floating for Encoder B (White)*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* -----------------------------------------------------------------------
     TIM2 Configuration:

     In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1):
     	 TIM2CLK = SystemCoreClock / 2 = 84000000 Hz

     To get TIM2 counter clock at X Hz, the prescaler is computed as follows:
     	 Prescaler = (TIM3CLK / TIM3 counter clock) - 1
     	 Prescaler = ((SystemCoreClock /2) / X Hz) - 1

     Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
    ----------------------------------------------------------------------- */
  //TIM_PrescalerConfig(TIM2, (uint16_t) (((SystemCoreClock/2) / X) - 1), TIM_PSCReloadMode_Immediate);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM2 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM2 CH4 pin (PA3)
     The Rising edge is used as active edge,
     The TIM2 CCR4 is used to compute the frequency value
  ------------------------------------------------------------ */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;		/*!< Capture performed once every 8 events. */
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
}

extern "C" {
/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET)
  {
    /* Clear TIM2 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    CaptureTime = NOW();
    if(CaptureNumber == 0)
    {
      /* Get the Input Capture value */
      IC4ReadValue1 = TIM_GetCapture4(TIM2);
      CaptureNumber = 1;
    }
    else if(CaptureNumber == 1)
    {
      EncoderB = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
      /* Get the Input Capture value */
      IC4ReadValue2 = TIM_GetCapture4(TIM2);

      /* Capture computation */
      if (IC4ReadValue2 > IC4ReadValue1)
      {
        Capture = (IC4ReadValue2 - IC4ReadValue1);
      }
      else if (IC4ReadValue2 < IC4ReadValue1)
      {
        Capture = ((0xFFFFFFFF - IC4ReadValue1) + IC4ReadValue2);
      }
      /* Frequency computation */
      TIM2Freq = (uint32_t) ((SystemCoreClock/2)) * 8 / Capture;
      CaptureNumber = 0;
    }
  }
}
}

void MotorSpeedUpdate()
{
	double SensorTime = ((NOW()-CaptureTime)/(double)MILLISECONDS);
	if (SensorTime>250) //minimum measured speed is 2 RPS(120 RPM). This can give us 250ms of minimum interval between interrupts (2 interrupts every one revolution).
	{
		TIM2Freq=0;
	}

	if (EncoderB)
	{
		sensorData.motorSpeed  = -1*((float)TIM2Freq / 16) * 60;  //CCW
	}
	else {sensorData.motorSpeed  = ((float)TIM2Freq / 16) * 60;}  //CW
}


void ADCUpdate()
{
	float ADCVaule = CurrentADC.read(ADCChannel);

	sensorData.motorCurrent = (ADCVaule / ADCRes) * (3000 / CurrentVoltageRatio) * 1000;

}


class Sensors: public Thread {

	uint64_t periode;
	sTelecommandData TelecommandDataReceiver;

public:

	Sensors(const char* name, uint64_t periode) : Thread(name) {
		this->periode = periode;
	}

	void init() {

	}

	void run(){

		LSM9DS1_I2C.init(400000);

		CurrentADC.init(ADCChannel);

		EncoderInit();

		if (imu.begin() == false) // note, we need to sent this our CS pins (defined above)
			      {
					 PRINTF("Failed to communicate with LSM9DS1.\r\n");
				  }

		GyroCal(gbias);


		TIME_LOOP(0,periode){

	  if(SensorsTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver))
	  {
		  if (TelecommandDataReceiver.TelecommandFlag)
		  {	  AHRSModeLast = AHRSMode;
			  AHRSMode = (AHRS_Mode) TelecommandDataReceiver.AHRSMode;
		  }
	  }

	    AHRSUpdate(AHRSMode);

	    MotorSpeedUpdate();

	    ADCUpdate();

        SensorDataTopic.publish(sensorData);

		}
	}
};

Sensors Sensors("Sensors", SensorsPeriod * MILLISECONDS);


