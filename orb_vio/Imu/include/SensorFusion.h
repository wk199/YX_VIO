#ifndef SensorFusion_h
#define SensorFusion_h
#include <stdio.h>
#define Capacity 10
#include<vector>
#define sample_time 0.001
using namespace std;
class SensorFusion
{
public:
//	SensorFusion(FILE *);
	SensorFusion();
	void SensorPretreatment(vector <float>); //传感器数据预处理
	void SensorPretreatment(float gyroX, float gyroY, float gyroZ);
	void TitleCorrection(float); //加速度计校正
	void handlemessage(vector <float> , vector <float> , float);
	void handlemessage(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ, float timedelta);
	vector <float> QuaToEuler(vector <float>);
	vector<float> QuatoMetrix(vector <float>);
	// void YawCorrection(); //磁力计校正
	vector <float> CaculatePos(vector <float> , float ,vector <float>);
	vector <float> StateTransformOrientation;
	vector <float>  LastGyro;
	vector <float> RunningTotal;
	vector <float> AngularVelocity;
	vector <float> LinearAcceleration;
	vector <float> AccelOffset;
	vector <float> GyroAutoOffset;
	vector <float> Euler;
	vector <float> RotateMetrix;
	vector <float> StateTransformPos;
	vector<vector<float> > AccelCalibrationMatrix;
	vector<vector<float> > GyroCalibrationMatrix;
	
private:
	int DataSize;
	vector <vector<float> > GyroArray;
	vector <vector<float> > AccelArray;
	vector <vector<float> > GyroFilterArray;
	vector <float>  GyroRunningTotal;
	vector <float>  output;
	vector <float>  Q;
	vector <float>  QuatfMean;
	float RunningTotalLenthSq;
	vector <float> GyroRuningTotal;
	float GyroFilterDataSize;
	float accelRunningTotalLenthSq;
	vector <float> accelRunningTotal;
	int runningelment;
	int thiselment;
	bool caculat;
	struct AutoGyrolimit
	{
	  float abslimit;
	  float Mabslimit;
	  float noiselimit;
	  float Mnoiselimit;
	  FILE *pfile;
	  int num;
	};
	AutoGyrolimit autoGyrolimit;

	float Length(vector <float> A);
		
	//void Clear();
	void PushBack1(vector <float>);
	void PushBack2(vector <float>);
	void Update(vector <float>, float, vector <float>, int);
	//void clear();
	float Confidence(vector <float>);
	float Angle(vector <float>, vector <float>);
	vector <float> GetFilteredValue(vector <float>);
	vector <float> Rotate(vector <float> , vector <float>);
	vector <float> vectorAlignmentRotation(vector <float> , vector <float>);
	vector <float> Nlerp(vector <float>, float a, vector <float>);
	vector <float> mutiple(vector <float>, vector <float>);
	vector <float> Quatf(vector <float>, float);
	vector <float> Normalized(vector <float>);
	vector <float> Mean(vector <float>, int);
	vector <float> Inversed(vector <float>);
	float Lengthsq(vector <float> A);
//	void ReadCalibration(FILE *);
	void ReadCalibration();

};
#endif
