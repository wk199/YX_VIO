//#include <jni.h>
#include "SensorFusion.h"
#include <math.h>
//#include <android/log.h>
#include "OVR_Math.h"

#define  LOG_TAG    "test"
//#define  LOGI(...)  //do {__android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__); }while(0)
//#define  LOGG(...)  do {__android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__); }while(0)
#define  LOGG(...)  do {} while(0)

using namespace std;
using namespace OVR;

//SensorFusion::SensorFusion(FILE *calibrationFile)
SensorFusion::SensorFusion()
{
	for (int i = 0; i < 3; i++)
	{
		output.push_back(0.0);
		QuatfMean.push_back(0.0);
		GyroRunningTotal.push_back(0.0);
		accelRunningTotal.push_back(0.0);
		RunningTotal.push_back(0.0);
		LastGyro.push_back(0.0);
		Q.push_back(0.0);
		StateTransformOrientation.push_back(0.0);
		LinearAcceleration.push_back(0.0);
		StateTransformPos.push_back(0.0);
	}
	Q.push_back(1.0);
	StateTransformOrientation.push_back(1.0);
	/////////////////////////Add Calibration Program///////////////////
	SensorFusion::ReadCalibration();
	///////////////////////////////////////////////////////////////////
	DataSize = 0;
	RunningTotalLenthSq = 0.0;
	GyroFilterDataSize = 0.0;
	accelRunningTotalLenthSq = 0.0;
	runningelment=0;
	thiselment=0;
	SensorFusion::caculat=false;
	autoGyrolimit.num=0;
	autoGyrolimit.abslimit=0.0;
	autoGyrolimit.noiselimit=0.0;
	autoGyrolimit.Mabslimit = 1.25f * 0.1f;// 1.25f * 0.349066f;
	autoGyrolimit.Mnoiselimit = 0.0175f;//Terry 0.0175f;
	autoGyrolimit.pfile = fopen("autogyrolimit.txt", "r");

}


void SensorFusion::SensorPretreatment(float gyroX, float gyroY, float gyroZ)
{
	vector<float> v;
	v.push_back(gyroX);
	v.push_back(gyroY);
	v.push_back(gyroZ);
	this->SensorPretreatment(v);
}

//////////////����������Ԥ����/////////////////////
void SensorFusion::SensorPretreatment(vector <float> gyro)
{
	const float alpha = 0.4f;
    // 1.25f is a scaling factor related to conversion from per-axis comparison to length comparison
    //const float absLimit = 1.25f * 0.349066f;
//  const float noiseLimit = 1.25f * 0.03f;		// This was the original threshold based on reported device noise characteristics.
	//const float noiseLimit = 0.0175f;			// This is the updated threshold based on analyzing ten GearVR devices and determining a value that best
	//const float noiseLimit = 0.175f;											// discriminates between stationary on a desk and almost stationary when on the user's head.
	vector <float> avg;
	vector <float> diff;
	vector <float> mean;
	vector <float> totle;
    // do a moving average to reject short term noise
	//�Ƚ���������Gyro���ݵ�״̬
    if (SensorFusion::GyroFilterArray.size()==0)
		SensorFusion::GyroFilterDataSize=1;
	else
        SensorFusion::GyroFilterDataSize =SensorFusion::GyroFilterArray.size()+1;

	mean=SensorFusion::Mean(SensorFusion::GyroRunningTotal, SensorFusion::GyroFilterDataSize);
	for (int i=0;i<3;i++)
	{
		avg.push_back((SensorFusion::GyroFilterDataSize==1) ? gyro[i] : gyro[i] * alpha + SensorFusion::LastGyro[i] * (1 - alpha));//SensorFusion::compare(SensorFusion::LastGyro, gyro)
		diff.push_back(avg[i] - mean[i]);
	}
	LOGG("Gyrohll:%f,%f,%f",gyro[0],gyro[1],gyro[2]);
	SensorFusion::LastGyro.assign(gyro.begin(),gyro.end());
    // Make sure the absolute value is below what is likely motion
    // Make sure it is close enough to the current average that it is probably noise and not motion

    if(autoGyrolimit.pfile==NULL)
    {
    	    if(SensorFusion::autoGyrolimit.num<=Capacity)
            {
    	    	SensorFusion::autoGyrolimit.abslimit+=SensorFusion::Length(avg);
    	    	SensorFusion::autoGyrolimit.noiselimit+=SensorFusion::Length(diff);
    	    	SensorFusion::autoGyrolimit.num++;
    	    }
    	    else
    	    {
    	    	SensorFusion::autoGyrolimit.Mabslimit=SensorFusion::autoGyrolimit.abslimit/SensorFusion::autoGyrolimit.num;
    	    	SensorFusion::autoGyrolimit.Mnoiselimit=SensorFusion::autoGyrolimit.noiselimit/SensorFusion::autoGyrolimit.num;
				autoGyrolimit.pfile = fopen("autogyrolimit.txt", "w+");
				if (NULL != autoGyrolimit.pfile)
				fprintf(autoGyrolimit.pfile,"%f,%f\n",autoGyrolimit.Mabslimit,autoGyrolimit.Mnoiselimit);
    	    }
    }
    else
    {
    	    if (fscanf(autoGyrolimit.pfile,"%f,%f\n",&autoGyrolimit.Mabslimit,&autoGyrolimit.Mnoiselimit))
		printf("read file error");
    }
	if (SensorFusion::Length(avg) >= SensorFusion::autoGyrolimit.Mabslimit || SensorFusion::Length(diff) >= SensorFusion::autoGyrolimit.Mnoiselimit)
		SensorFusion::GyroFilterArray.clear();
    // if had a reasonable number of samples already use it for the current offset
	if (SensorFusion::GyroFilterArray.size() < Capacity)
    {
		SensorFusion::GyroFilterArray.push_back(avg);//GyroArray����50�Ĵ�С
		for (unsigned int i = 0; i < avg.size(); i++)
		{
			totle.push_back(SensorFusion::GyroRunningTotal[i] +avg[i]);
		}
		SensorFusion::GyroRunningTotal.assign(totle.begin(),totle.end());
    }
	else
	{
		//ʵʱ����GyroArray�е�ֵ���������µ�GyroAutoOffset
		vector <float> onfirst;
		vector <float> totle;
		for (int i=0;i<1;i++)
			for (int j=0;j<3;j++)
			{
				onfirst.push_back(SensorFusion::GyroFilterArray[i][j]);
			}

		SensorFusion::GyroFilterArray.erase(SensorFusion::GyroFilterArray.begin());
		SensorFusion::GyroFilterArray.push_back(avg);
		for(int i=0;i<3;i++)
		{
			totle.push_back(SensorFusion::GyroRunningTotal[i]+ avg[i]-onfirst[i]);
		}
		SensorFusion::GyroRunningTotal.assign(totle.begin(),totle.end());
		SensorFusion::GyroAutoOffset = SensorFusion::Mean(SensorFusion::GyroRunningTotal , Capacity);
	}
}
/////////////���ٶȼ�У��/////////////////////////
void SensorFusion::TitleCorrection(float deltaT)
{
	const float gain = 0.25;
	//const float snapThreshold = 0.1;
	vector <float> up;
	//up.push_back(0);up.push_back(1.0);up.push_back(0.0);
	up.push_back(1); up.push_back(0.0); up.push_back(0.0);//Terry
	vector <float> accelLocalFiltered;
	vector <float> GFValue;
	accelLocalFiltered = SensorFusion::GetFilteredValue(Q);

	vector <float> accelW = SensorFusion::Rotate(accelLocalFiltered, SensorFusion::StateTransformOrientation);
	vector <float> Quatferror = SensorFusion::vectorAlignmentRotation(accelW, up);
	vector <float> Quatfcorrection;

	//�˴���Ҫ�ڵ�һ�ν��м��ٶȼ�У��
	if (SensorFusion::runningelment==1)//(abs(Quatferror[3])< cos(snapThreshold / 2))&& SensorFusion::Confidence(accelLocalFiltered) < 0.75)
	{
        // full correction for start-up
        // or large error with high confidence
        Quatfcorrection.assign(Quatferror.begin(),Quatferror.end());
	}
    else if (SensorFusion::Confidence(accelW) > 0.5)
	{
        //correction = error.Nlerp(Quatf(), gain * deltaT);
		//////////////
		vector <float> Quatf(4,0);Quatf[3]=1;
		Quatfcorrection = SensorFusion::Nlerp(Quatf, gain * deltaT,Quatferror);
	}
    else
	{
        // accelerometer is unreliable due to movement
        return;
	}

    StateTransformOrientation = SensorFusion::mutiple(Quatfcorrection , StateTransformOrientation);
}

void SensorFusion::handlemessage(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ, float timedelta)
{
	vector<float> vg;
	vg.push_back(gyroX);
	vg.push_back(gyroY);
	vg.push_back(gyroZ);
	vector<float> vc;
	vc.push_back(accX);
	vc.push_back(accY);
	vc.push_back(accZ);
	this->handlemessage(vc, vg, timedelta);
}

////////////sensorfusion����������////////////////
void SensorFusion::handlemessage(vector <float> Accel, vector <float> Gyro, float TimeDelta)
{

	// Put the sensor readings into convenient local variables
	vector <float> gyro;
	vector <float> accel;
	float ggg=0.0,aaa=0.0;
	for(int j=0;j<3;j++)
	{
		aaa=0.0;
		ggg=0.0;
		for (int i=0;i<3;i++)
		{
			aaa+=SensorFusion::AccelCalibrationMatrix[j][i]*(Accel[i]-SensorFusion::AccelOffset[i]);
			ggg+=SensorFusion::GyroCalibrationMatrix[j][i]*(Gyro[i]-SensorFusion::GyroAutoOffset[i]);
			//gyro.push_back(Gyro[i]-SensorFusion::GyroAutoOffset[i]);
			//accel.push_back(Accel[i]-SensorFusion::AccelOffset[i]);
		}
		gyro.push_back(ggg);
	    accel.push_back(aaa);
	}

	float DeltaT = TimeDelta;
	vector <float> gra(3,0);
	gra[0]=9.8f;
	//if (SensorFusion::runningelment>=Capacity)
	     //SensorFusion::runningelment=Capacity;
	//else
	SensorFusion::runningelment++;
	// Insert current sensor data into filter history
	SensorFusion::PushBack1(gyro);
	SensorFusion::Update(accel, DeltaT, Quatf(gyro, SensorFusion::Length(gyro) * DeltaT),SensorFusion::thiselment);
	SensorFusion::thiselment++;
	if (SensorFusion::runningelment>=Capacity)
		SensorFusion::caculat=true;
	// Process raw inputs
	SensorFusion::AngularVelocity.assign(gyro.begin(),gyro.end());
	vector <float> rotatedaccel=SensorFusion::Rotate(accel, StateTransformOrientation);
	/*for (int i = 0; i<3; i++)
	{
		SensorFusion::LinearAcceleration.push_back(rotatedaccel[i] - gra[i]);
	}
	for (int i=0; i<3; i++)
	{
		SensorFusion::LinearAcceleration.erase(SensorFusion::LinearAcceleration.begin());
	}*/
	// Update headset orientation
	float angle = SensorFusion::Length(gyro) * DeltaT;
	if (angle > 0)
		SensorFusion::StateTransformOrientation = SensorFusion::mutiple(SensorFusion::StateTransformOrientation , Quatf(gyro, angle));

	// Tilt correction based on accelerometer
	//if (EnableGravity)
	SensorFusion::TitleCorrection(DeltaT);
	// Yaw correction based on magnetometer
	//if (EnableYawCorrection && HasMagCalibration())
		//applyMagYawCorrection(mag, magBias, gyro, DeltaT);
	// The quaternion magnitude may slowly drift due to numerical error,
	// so it is periodically normalized.
	SensorFusion::StateTransformOrientation = SensorFusion::Normalized(SensorFusion::StateTransformOrientation);
	//SensorFusion::StateTransformPos         = SensorFusion::CaculatePos(SensorFusion::StateTransformOrientation,0.001,accel);
	SensorFusion::Euler                     = QuaToEuler(SensorFusion::StateTransformOrientation);
	SensorFusion::RotateMetrix              = QuatoMetrix(SensorFusion::StateTransformOrientation);
}
/////////////���������ǵ��ۻ�rotation//////////////////////
void SensorFusion::PushBack1(vector <float> avg)
{
	for (unsigned int i = 0; i < avg.size(); i++)
	{
		SensorFusion::RunningTotal.push_back(SensorFusion::RunningTotal[i]+avg[i]);
		SensorFusion::RunningTotal.erase(SensorFusion::RunningTotal.begin());
	}
}
//////////�������ٶȼƵ��ۻ�rotation////////////////////////
void SensorFusion::PushBack2(vector <float> avg)
{

	if (SensorFusion::runningelment > Capacity||SensorFusion::caculat)
	{
		vector <float> onfirst;
		vector <float> totle;
		for (int i=0;i<1;i++)
			for (int j=0;j<3;j++)
			{
				onfirst.push_back(SensorFusion::AccelArray[i][j]);
			}
		SensorFusion::AccelArray.erase(SensorFusion::AccelArray.begin());
		SensorFusion::AccelArray.push_back(avg);
		for(int i=0;i<3;i++)
		{
			totle.push_back(SensorFusion::accelRunningTotal[i]+ avg[i]-onfirst[i]);
		}
		SensorFusion::accelRunningTotal.assign(totle.begin(),totle.end());
		SensorFusion::runningelment=Capacity;
		SensorFusion::accelRunningTotalLenthSq = SensorFusion::accelRunningTotalLenthSq+SensorFusion::Lengthsq(avg)-SensorFusion::Lengthsq(onfirst);

		/*vector <float> totle;
		vector <vector <float> > lasthalf;
		lasthalf.assign(SensorFusion::AccelArray.begin()+Capacity/2,SensorFusion::AccelArray.end());
		SensorFusion::AccelArray.clear();
		SensorFusion::AccelArray.assign(lasthalf.begin(),lasthalf.end());
		SensorFusion::AccelArray.push_back(avg);

		for (int i=0;i<3;i++)
		{
			SensorFusion::accelRunningTotal[i]=0;
            for (int j=0;j<Capacity/2+1;j++)
				SensorFusion::accelRunningTotal[i]+=SensorFusion::AccelArray[j][i];
		}
		SensorFusion::runningelment=Capacity/2+1;*/
	/*	SensorFusion::AccelArray.clear();
		SensorFusion::AccelArray.push_back(avg);

		for (int i=0;i<3;i++)
		{
			SensorFusion::accelRunningTotal[i]=0;
            for (int j=0;j<1;j++)
				SensorFusion::accelRunningTotal[i]+=SensorFusion::AccelArray[j][i];
		}
		SensorFusion::runningelment=1;//Capacity/4+1;*/

	}
	else
	{
		SensorFusion::AccelArray.push_back(avg);
		SensorFusion::accelRunningTotalLenthSq += SensorFusion::Lengthsq(avg);
		for (int i=0;i<3;i++)
		{
              SensorFusion::accelRunningTotal.push_back(SensorFusion::accelRunningTotal[i]+avg[i]);
		}
		for (int i=0;i<3;i++)
	    {
              SensorFusion::accelRunningTotal.erase(SensorFusion::accelRunningTotal.begin());
	    }
	}

}
vector<float> SensorFusion::CaculatePos(vector <float> QuatOri , float delTa,vector <float>accel)
{
	float C11 = QuatOri[3]*QuatOri[3]+QuatOri[0]*QuatOri[0]-QuatOri[1]*QuatOri[1]-QuatOri[2]*QuatOri[2];
	float C12 = 2*(QuatOri[0]*QuatOri[1]-QuatOri[3]*QuatOri[2]);
	float C13 = 2*(QuatOri[0]*QuatOri[2]+QuatOri[3]*QuatOri[1]);
	float C21 = 2*(QuatOri[0]*QuatOri[1]+QuatOri[3]*QuatOri[2]);
	float C22 = QuatOri[3]*QuatOri[3]-QuatOri[0]*QuatOri[0]+QuatOri[1]*QuatOri[1]-QuatOri[2]*QuatOri[2];
	float C23 = 2*(QuatOri[1]*QuatOri[2]-QuatOri[3]*QuatOri[0]);
	float C31 = 2*(QuatOri[0]*QuatOri[2]-QuatOri[3]*QuatOri[1]);
	float C32 = 2*(QuatOri[1]*QuatOri[2]+QuatOri[3]*QuatOri[0]);
	float C33 = QuatOri[3]*QuatOri[3]-QuatOri[0]*QuatOri[0]-QuatOri[1]*QuatOri[1]+QuatOri[2]*QuatOri[2];
	vector <float> Navoutput;
	Navoutput.push_back(C11*accel[0]+C12*accel[1]+C13*accel[2]);
	Navoutput.push_back(C21*accel[0]+C22*accel[1]+C23*accel[2]-9.81);
	Navoutput.push_back(C31*accel[0]+C32*accel[1]+C33*accel[2]);
	vector <float> Displacement;
	Displacement.push_back(Navoutput[0]);
	Displacement.push_back(Navoutput[1]);
	Displacement.push_back(Navoutput[2]);
	return Displacement;
 }
///////////�й���ѧ�������������κ������Ŀ⺯��////////////////////////////////////
void SensorFusion::Update(vector <float> accel, float deltaT,vector <float> deltaQ,int thiselment)
{
	//Quatf(gyro, gyro.Length() * DeltaT)
	if (thiselment==0)
	{
		output.assign(accel.begin(), accel.end());

	}
	else
	{
		// rotate by deltaQ
		output = SensorFusion::Rotate(output , SensorFusion::Inversed(deltaQ));
		// apply low-pass filter
		for (int i = 0; i < 3; i++)
		{
			output.push_back(output[i]+(accel[i] - output[i]) * 2.5*deltaT);
		}
		for (int i=0;i<3;i++)
		{
			output.erase(output.begin());
		}

	}
	// put the value into the fixed frame for the stddev computation
	vector <float> mut=SensorFusion::mutiple(SensorFusion::Q, deltaQ);
	SensorFusion::Q.assign(mut.begin(), mut.end());
	SensorFusion::PushBack2(SensorFusion::Rotate(output, SensorFusion::Q));
}
vector <float> SensorFusion::GetFilteredValue(vector <float> Q)
{

	vector <float> QuatMean=SensorFusion::Mean(SensorFusion::accelRunningTotal,SensorFusion::runningelment);
	return SensorFusion::Rotate(QuatMean, SensorFusion::Inversed(Q));
	 //return GFValue;

}
vector <float> SensorFusion::Rotate(vector <float> A , vector <float> B)
{
	 vector <float> AA;
	 vector <float> rotate;
	 vector <float> re;
	 AA.push_back(A[0]);AA.push_back(A[1]);AA.push_back(A[2]);AA.push_back(0.0);
	 vector <float> BInvered;
	 BInvered.push_back(-B[0]);BInvered.push_back(-B[1]);BInvered.push_back(-B[2]);BInvered.push_back(B[3]);
	 rotate = mutiple(mutiple(B, AA), BInvered);
	 for (int i=0;i<3;i++)
	 {
		 re.push_back(rotate[i]);
	 }
	 return re;
}
vector <float> SensorFusion::vectorAlignmentRotation(vector <float> accelW , vector <float> up)
{
	vector <float> axis;
	axis.push_back(accelW[1]*up[2] - accelW[2]*up[1]);
	axis.push_back(accelW[2]*up[0] - accelW[0]*up[2]);
	axis.push_back(accelW[0]*up[1] - accelW[1]*up[0]);
    //if (axis.LengthSq() == 0)
        // this handles both collinear and zero-length input cases
    //    return Quatf();
    float angle = SensorFusion::Angle(accelW , up);
	vector <float> qqqq;
	qqqq=SensorFusion::Quatf(axis, angle);
    return qqqq;
}
float SensorFusion::Confidence(vector <float> acc)
{
	float StdDev = sqrt(SensorFusion::accelRunningTotalLenthSq/ SensorFusion::runningelment - SensorFusion::Lengthsq(SensorFusion::Mean(SensorFusion::accelRunningTotal, SensorFusion::runningelment)));
	float conf=OVR::Clamp(0.48f - 0.1f * logf(StdDev), 0.0f, 1.0f) * SensorFusion::runningelment / Capacity;
	//float conf=abs(sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])-9.8);
	return conf;
}
vector <float> SensorFusion::Mean(vector <float> Total ,int DataSize)
{
	vector <float> mean;
	for (unsigned int i = 0; i < Total.size(); i++)
	{
		mean.push_back(Total[i] / DataSize);
	}
	return mean;

}
float SensorFusion::Length(vector <float> A)
{
	float tempp = 0.0f;

	if (A.size()==3)
	    tempp=sqrt(A[0]*A[0]+A[1]*A[1]+A[2]*A[2]);
	if (A.size()==4)
		tempp=sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2] + A[3] * A[3]);
	return tempp;
}
float SensorFusion::Lengthsq(vector <float> A)
{
	float tempp = 0.0f;

	if (A.size()==3)
	    tempp= A[0]*A[0]+A[1]*A[1]+A[2]*A[2];
	if (A.size()==4)
		tempp = A[0] * A[0] + A[1] * A[1] + A[2] * A[2] + A[3] * A[3];
	return tempp;
}
vector <float> SensorFusion::Nlerp(vector <float> other , float a , vector <float> error)
{
	vector <float> myerror;
	for (int i=0;i<4;i++)
	{
		myerror.push_back(a*error[i] + (1 - a)*other[i]);
	}
	myerror = SensorFusion::Normalized(myerror);
	return myerror;
}
vector <float> SensorFusion::mutiple(vector <float> A , vector <float> B)
{
	vector <float> C;
	C.push_back( A[3] * B[0] + A[0] * B[3] + A[1] * B[2] - A[2] * B[1]);
	C.push_back( A[3] * B[1] - A[0] * B[2] + A[1] * B[3] + A[2] * B[0]);
	C.push_back(A[3] * B[2] + A[0] * B[1] - A[1] * B[0] + A[2] * B[3]);
	C.push_back(A[3] * B[3] - A[0] * B[0] - A[1] * B[1] - A[2] * B[2]);
	return C;
}
float SensorFusion::Angle(vector <float> from , vector <float> to)
{
	    float div = SensorFusion::Lengthsq( from )*SensorFusion::Lengthsq( to );
            float rad=(from[0]*to[0]+from[1]*to[1]+from[2]*to[2])/sqrt(div);
            if (rad>1.0f)
              rad=1.0f;
            if (rad<-1.0f)
              rad=-1.0f;
	    float result = acos(rad);
	    return result;
}
vector <float> SensorFusion::Quatf(vector <float> axis , float angle)
{
	 vector <float> quatf;
	 if (SensorFusion::Length( axis ) == 0)
        {
            //Assert if the axis is zero, but the angle isn't
            quatf.push_back(0); quatf.push_back(0); quatf.push_back(0); quatf.push_back(1);
            return quatf;
        }

		vector <float> unitAxis = SensorFusion::Normalized(axis);
		float          sinHalfAngle = sin(angle * 0.5);

		quatf.push_back(unitAxis[0] * sinHalfAngle);
		quatf.push_back(unitAxis[1] * sinHalfAngle);
		quatf.push_back(unitAxis[2] * sinHalfAngle);
		quatf.push_back(cos(angle * 0.5));

	return quatf;
}
vector <float> SensorFusion::Normalized(vector <float> axis)
{
	float length;
	length  = SensorFusion::Length( axis );
	for (unsigned int i = 0; i < axis.size(); i++)
	{
		axis[i] = axis[i] / length;
	}
	return axis;
}
vector <float> SensorFusion::Inversed(vector <float> A)
{
	vector <float> Ainversed;
	Ainversed.push_back(-A[0]);
	Ainversed.push_back(-A[1]);
	Ainversed.push_back(-A[2]);
	Ainversed.push_back(A[3]);
	return Ainversed;
}
vector <float> SensorFusion::QuaToEuler(vector <float> Quat)
{
	//vector <float>inQuat;
	//inQuat.push_back(Quat[3]);
	//inQuat.push_back(Quat[0]);
	//inQuat.push_back(Quat[1]);
	//inQuat.push_back(Quat[2]);
    vector <float> Euler;
	//Euler.push_back(-atan2(-2*(Quat[3]*Quat[1]+Quat[0]*Quat[2]) , Quat[3]*Quat[3]+Quat[2]*Quat[2]-Quat[1]*Quat[1]-Quat[0]*Quat[0]));
	//Euler.push_back(asin(2*(Quat[3]*Quat[0]-Quat[1]*Quat[2])));
	//Euler.push_back(atan2(2*(Quat[3]*Quat[2]+Quat[1]*Quat[0]) , Quat[3]*Quat[3]+Quat[1]*Quat[1]-Quat[0]*Quat[0]-Quat[2]*Quat[2]));
	//Euler.push_back(asin(2*(inQuat[3]*inQuat[2]-inQuat[0]*inQuat[1])));//pitch
	//Euler.push_back(atan2(-2*(inQuat[3]*inQuat[0]+inQuat[1]*inQuat[2]) , inQuat[3]*inQuat[3]+inQuat[1]*inQuat[1]-inQuat[2]*inQuat[2]-inQuat[0]*inQuat[0]));//yaw
	//Euler.push_back(-atan2(-2*(inQuat[3]*inQuat[1]+inQuat[2]*inQuat[0]) , inQuat[2]*inQuat[2]+inQuat[1]*inQuat[1]-inQuat[0]*inQuat[0]-inQuat[3]*inQuat[3]));//roll

	float ww=Quat[3]*Quat[3];
	float Q11=Quat[1]*Quat[1];
	float Q22=Quat[0]*Quat[0];
	float Q33=Quat[2]*Quat[2];
	float psign=-1.0;
	float s2=psign*2.0*(psign*Quat[3]*Quat[0]+Quat[1]*Quat[2]);
	if(s2<float(-1.0)+0.0001f)
	{
		Euler.push_back(0.0);
		Euler.push_back(-0.5*3.1415926);
		Euler.push_back(atan2(2*(psign*Quat[1]*Quat[0]+Quat[3]*Quat[2]),ww+Q22-Q11-Q33));
		return Euler;
	}
	else if(s2>float(1.0)-0.0001f)
	{
		Euler.push_back(0.0);
		Euler.push_back(0.5*3.1415926);
		Euler.push_back(atan2(2*(psign*Quat[1]*Quat[0]+Quat[3]*Quat[2]),ww+Q22-Q11-Q33));
		return Euler;
	}
	else
	{
		Euler.push_back(-atan2(-2*(Quat[3]*Quat[1]+Quat[0]*Quat[2]), Quat[3]*Quat[3]+Quat[2]*Quat[2]-Quat[1]*Quat[1]-Quat[0]*Quat[0]));
		Euler.push_back(asin(s2));
	    Euler.push_back(atan2(2*(Quat[3]*Quat[2]+Quat[1]*Quat[0]), Quat[3]*Quat[3]+Quat[1]*Quat[1]-Quat[0]*Quat[0]-Quat[2]*Quat[2]));
		return Euler;
	}
}
vector<float> SensorFusion::QuatoMetrix(vector <float> QuatOri)
{
	vector<float> temp;
	temp.push_back(QuatOri[3]*QuatOri[3]+QuatOri[0]*QuatOri[0]-QuatOri[1]*QuatOri[1]-QuatOri[2]*QuatOri[2]);
	temp.push_back(2*(QuatOri[0]*QuatOri[1]-QuatOri[3]*QuatOri[2]));
	temp.push_back(2*(QuatOri[0]*QuatOri[2]+QuatOri[3]*QuatOri[1]));
	temp.push_back(2*(QuatOri[0]*QuatOri[1]+QuatOri[3]*QuatOri[2]));
	temp.push_back(QuatOri[3]*QuatOri[3]-QuatOri[0]*QuatOri[0]+QuatOri[1]*QuatOri[1]-QuatOri[2]*QuatOri[2]);
	temp.push_back(2*(QuatOri[1]*QuatOri[2]-QuatOri[3]*QuatOri[0]));
	temp.push_back(2*(QuatOri[0]*QuatOri[2]-QuatOri[3]*QuatOri[1]));
	temp.push_back(2*(QuatOri[1]*QuatOri[2]+QuatOri[3]*QuatOri[0]));
	temp.push_back(QuatOri[3]*QuatOri[3]-QuatOri[0]*QuatOri[0]-QuatOri[1]*QuatOri[1]+QuatOri[2]*QuatOri[2]);
	return temp;
 }
//void SensorFusion::ReadCalibration(FILE *calibrationFile)
void SensorFusion::ReadCalibration()
{
	//AccelOffset.push_back(0.054701);AccelOffset.push_back(-0.058760);AccelOffset.push_back(-0.013766);
	//GyroAutoOffset.push_back(0.023023);GyroAutoOffset.push_back(0.000342);GyroAutoOffset.push_back(0.005956);
	//const char* calibrationFilename = "/system/etc/sensors_offset";
	//const char* calibrationFilename = "self_calibration.txt";
	float ax,ay,az;
	//float mat[3][3];
	vector<float> mat1(3);
	vector<float> mat2(3);
	vector<float> mat3(3);
	/* read accelerometer, gyrometer and magnetometer calibration parameters */
	//FILE* calibrationFile = fopen(calibrationFilename, "r");

	// read accelerometer calibration
//	if((fscanf(calibrationFile, "%f %f %f\n", &ax, &ay, &az)==3 &&
//	     fscanf(calibrationFile, "%f %f %f\n", &mat1[0], &mat1[1], &mat1[2])==3 &&
//	     fscanf(calibrationFile, "%f %f %f\n", &mat2[0], &mat2[1], &mat2[2])==3 &&
//	     fscanf(calibrationFile, "%f %f %f\n", &mat3[0], &mat3[1], &mat3[2])==3))
	{
		ax = 0.1683f;
		ay = 0.3556f;
		az = -0.0741f;

		mat1[0] = 0.992162f;
		mat1[1] = 0.0f;
		mat1[2] = 0.0f;

		mat2[1] = 0.970591f;
		mat2[0] = 0.0f;
		mat2[2] = 0.0f;

		mat3[2] = 0.998203f;
		mat3[1] = 0.0f;
		mat3[0] = 0.0f;

		SensorFusion::AccelOffset.push_back(ax);
		SensorFusion::AccelOffset.push_back(ay);
		SensorFusion::AccelOffset.push_back(az);
		SensorFusion::AccelCalibrationMatrix.push_back(mat1);
		SensorFusion::AccelCalibrationMatrix.push_back(mat2);
		SensorFusion::AccelCalibrationMatrix.push_back(mat3);

	}
//	else
//	{
//		printf("Read Failed on Accelerometer Calibration Parameters!!\n");
//		fclose(calibrationFile);
//
//		SensorFusion::AccelOffset.clear();
//		SensorFusion::AccelCalibrationMatrix.clear();
//
//		return ;
//	}

	// read gyrometer calibration
//	if((fscanf(calibrationFile, "%f %f %f\n", &ax, &ay, &az)==3 &&
//	     fscanf(calibrationFile, "%f %f %f\n", &mat1[0], &mat1[1], &mat1[2])==3 &&
//	     fscanf(calibrationFile, "%f %f %f\n", &mat2[0], &mat2[1], &mat2[2])==3 &&
//	     fscanf(calibrationFile, "%f %f %f\n", &mat3[0], &mat3[1], &mat3[2])==3))
	{
		ax = 0.002814f;
		ay = -0.00362f;
		az = 0.005638f;

		mat1[0] = 1.0f;
		mat1[1] = 0.0f;
		mat1[2] = 0.0f;

		mat2[1] = 1.0f;
		mat2[0] = 0.0f;
		mat2[2] = 0.0f;

		mat3[2] = 1.0f;
		mat3[1] = 0.0f;
		mat3[0] = 0.0f;

		SensorFusion::GyroAutoOffset.push_back(ax);
		SensorFusion::GyroAutoOffset.push_back(ay);
		SensorFusion::GyroAutoOffset.push_back(az);
		SensorFusion::GyroCalibrationMatrix.push_back(mat1);
		SensorFusion::GyroCalibrationMatrix.push_back(mat2);
		SensorFusion::GyroCalibrationMatrix.push_back(mat3);

	}
//	else
//	{
//		printf("Read Failed on Gyrometer Calibration Parameters!!\n");
//		fclose(calibrationFile);
//
//		SensorFusion::GyroAutoOffset.clear();
//		SensorFusion::GyroCalibrationMatrix.clear();
//
//		return ;
//	}

	// read magnetometer calibration
	/*if(!(fscanf(calibrationFile, "%f %f %f\n", &MagCalibrationOffset.x, &MagCalibrationOffset.y, &MagCalibrationOffset.z)==3 &&
	     fscanf(calibrationFile, "%f %f %f\n", &MagCalibrationMatrix.M[0][0], &MagCalibrationMatrix.M[0][1], &MagCalibrationMatrix.M[0][2])==3 &&
	     fscanf(calibrationFile, "%f %f %f\n", &MagCalibrationMatrix.M[1][0], &MagCalibrationMatrix.M[1][1], &MagCalibrationMatrix.M[1][2])==3 &&
	     fscanf(calibrationFile, "%f %f %f\n", &MagCalibrationMatrix.M[2][0], &MagCalibrationMatrix.M[2][1], &MagCalibrationMatrix.M[2][2])==3))
	{
		printf("Read %s Failed on Magnetometer Calibration Parameters!!\n", calibrationFilename);
		fclose(calibrationFile);

		MagCalibrationOffset = Vector3f();
		MagCalibrationMatrix = Matrix4f();

		return ;
	}*/
//	fclose(calibrationFile);

//	return;
}
