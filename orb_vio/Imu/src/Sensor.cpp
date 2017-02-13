#include "Sensor.h"
#include "SensorFusion.h"
#include <iostream>
//#include <android/log.h>
#define  LOG_TAG    "test"
//#define  LOGI(...)  //do {__android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__); }while(0)
//#define  LOGG(...)  do {__android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__); }while(0)
#define  LOGI(...)  do {} while(0)

static SensorFusion* g_sensorFusion = NULL;
using namespace std;
void InitSensorFusion(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ, float timeDelta)
{
	if (g_sensorFusion == NULL)
	{
		g_sensorFusion = new SensorFusion();
	}
    std::cout << "Sensor Pretreatment" << endl;
	g_sensorFusion->SensorPretreatment(gyroX, gyroY, gyroZ);
	cout << "Handle Message" << endl;
	cout << gyroX << " " << gyroY << " " << gyroZ << accX << " " << accY << " " << accZ << " " << timeDelta <<endl;
	g_sensorFusion->handlemessage(gyroX, gyroY, gyroZ, accX, accY, accZ, timeDelta);
}

void GetEulerAngles(float* angles)
{
	if (g_sensorFusion != NULL)
	{
		angles[0] = g_sensorFusion->Euler[0];
		angles[1] = g_sensorFusion->Euler[1];
		angles[2] = g_sensorFusion->Euler[2];
	}
}
void GetRotateMetrix(float* metrix)
{
    for(int i=0;i<9;i++)
    {
		metrix[i] = g_sensorFusion->RotateMetrix[i];
    }
}

void SensorFusionUpdate(float* Accel , float* Gyro, float deltatime){

	vector<float> accel_data(3,0);
	vector<float> gdata(3,0);
	for(int i=0;i<3;i++){
		accel_data[i] = Accel[i];
		gdata[i] = Gyro[i];
	}
	g_sensorFusion->handlemessage(accel_data , gdata , deltatime);
}
