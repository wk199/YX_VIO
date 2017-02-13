#ifndef _SENSOR_H_
#define _SENSOR_H_

void InitSensorFusion(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ, float timeDelta);
void SensorFusionUpdate(float* Accel , float* Gyro, float deltatime);
void GetEulerAngles(float* angles);
void GetRotateMetrix(float* rotatemetrix);
#endif
