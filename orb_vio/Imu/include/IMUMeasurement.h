#ifndef IMUFMEASUREMENT_H
#define IMUFMEASUREMENT_H
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <time.h>
#include <vector>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <boost/thread/mutex.hpp>
#include "MPU9250.h"

#include "SensorFusion.h"

using namespace cv;
using namespace std;
using namespace gtsam;

struct my_sensor_6axis_t {
	float f_acc_x;
	float f_acc_y;
	float f_acc_z;
	float f_gyr_x;
	float f_gyr_y;
	float f_gyr_z;
    float *Rot;
	double wMilliseconds; // actually, it is seconds
    
    public:
     my_sensor_6axis_t(){
         Rot = new float[9];
     }
};

class IMUMeasurementListParam{
  public:
    Matrix3 biasAccCovariance;    ///< continuous-time "Covariance" describing accelerometer bias random walk
    Matrix3 biasOmegaCovariance;  ///< continuous-time "Covariance" describing gyroscope bias random walk
    Matrix6 biasAccOmegaInt;     ///< covariance of bias used for pre-integration
    Matrix3 accelerometerCovariance; ///< continuous-time "Covariance" of accelerometer
    Matrix3 integrationCovariance; ///< continuous-time "Covariance" describing integration uncertainty
    Matrix3 gyroscopeCovariance;
    bool use2ndOrderCoriolis; ///< Whether to use second order Coriolis integration
    Vector3 n_gravity; ///< Gravity vector in nav frame
   
    Pose3 Pose_Body_Sensor; // IMU Pose in the Body Frame;
    
    public:
      IMUMeasurementListParam(){
        biasAccCovariance = Matrix3::Identity() * (3.0000e-3);
        biasOmegaCovariance = Matrix3::Identity() * (1.9393e-05);
        biasAccOmegaInt = Matrix6::Identity();
        biasAccOmegaInt(0,0) = 3.0000e-3; biasAccOmegaInt(1,1) = 3.0000e-3; biasAccOmegaInt(2,2) = 3.0000e-3;
        biasAccOmegaInt(3,3) = 1.9393e-05; biasAccOmegaInt(4,4) = 1.9393e-05; biasAccOmegaInt(5,5) = 1.9393e-05;
        accelerometerCovariance = Matrix3::Identity() * (2e-3);
        integrationCovariance = Matrix3::Identity() * (2e-3);
        gyroscopeCovariance = Matrix3::Identity() * (1.6968e-04);
        use2ndOrderCoriolis = false;
        n_gravity = Vector3(-9.81 ,0.0 , 0.0); // note n_gravity is the quantity in the navigation frame
        
        Eigen::Matrix4d matTcs;
        matTcs<<0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0;  
         Pose3 Pose_Cam_Sensor = Pose3(matTcs);
         Pose_Body_Sensor = Pose_Cam_Sensor.inverse();  
              
      };
      
    public:
      void setPoseBodySensor(const cv::Mat _calib){
          //the Pose is from the IMU sensor to the Body
          Eigen::Matrix4d matTcs;
          for(int i=0;i<4;i++){
              for(int j=0;j<4;j++){
                  matTcs(i,j) = _calib.at<float>(i,j);
              }
          }
          Pose_Body_Sensor = Pose3(matTcs);
      }
      void setAccNoiseDensity(const double& sigma_a){
          accelerometerCovariance = Matrix3::Identity() * sigma_a;
          integrationCovariance = accelerometerCovariance;
      }
      void setGyroNoiseDensity(const double& sigma_w){
          gyroscopeCovariance = Matrix3::Identity() * sigma_w;
      }
      void setAccRandomWalk(const double& sigma_a){
          biasAccCovariance = Matrix3::Identity() * sigma_a;
          biasAccOmegaInt(0,0) = sigma_a;
          biasAccOmegaInt(1,1) = sigma_a;
          biasAccOmegaInt(2,2) = sigma_a;          
      }
      void setGryoRandomWalk(const double& sigma_w){
          biasOmegaCovariance = Matrix3::Identity() * sigma_w;
          biasAccOmegaInt(3,3) = sigma_w;
          biasAccOmegaInt(4,4) = sigma_w;
          biasAccOmegaInt(5,5) = sigma_w;         
      }
};

class IMUMeasurement{
    public:
        IMUMeasurement(){};
        ~IMUMeasurement(){};
    
    public:
        void setIMUMeasurement(gtsam::PreintegratedCombinedMeasurements* _pim){
            pim = _pim;
        }    
        void setId(const int& _id){
            Id = _id;
        }
        void setRot(const cv::Mat& _rMat){
            gtsam::Rot3 _rt(_rMat.at<float>(0,0),_rMat.at<float>(0,1),_rMat.at<float>(0,2),
                            _rMat.at<float>(1,0),_rMat.at<float>(1,1),_rMat.at<float>(1,2),
                            _rMat.at<float>(2,0),_rMat.at<float>(2,1),_rMat.at<float>(2,2));
            _rot3 = _rt;
        }
        void setRot(const std::vector<float>& _rd){
            gtsam::Rot3 _rt(_rd[0],_rd[1],_rd[2],
                            _rd[3],_rd[4],_rd[5],
                            _rd[6],_rd[7],_rd[8]);
             _rot3 = _rt;
        }
        gtsam::PreintegratedCombinedMeasurements* getIMUMeasurement(){
            return pim;
        }
        int getId(){
            return Id;
        }
        gtsam::Rot3 getRot(){
            return _rot3;
        }
        
    private:
        gtsam::PreintegratedCombinedMeasurements* pim;
        int Id;
        gtsam::Rot3 _rot3;    
    
};

class IMUMeasurementList
{
public:
  IMUMeasurementList();
  IMUMeasurementList(const Mat & _calib);
  IMUMeasurementList(const Mat & _calib , char* imu_file);
  ~IMUMeasurementList();

public:
  void Run();
  void Addimufactor(double,int);
  void AddimufactorDebug(double , int);
  void Addframeimufactor(double,int);
  gtsam::PreintegratedCombinedMeasurements *GetInstantimufactorDebug2(double, double);
  gtsam::PreintegratedCombinedMeasurements *GetInstantKFAndFrameimufactor(double, double);
  void AddKFframeimufactor(double, int);
  void setFirstTimeStample(const double& _time){ last_keyframe_time = _time;}
  void setFirstTimeStampleandmnId(const double& _time,int mnId, cv::Mat& _rot3);
  void setFirstTimeStampleandmnIdDebug(const double& _time,int mnId, cv::Mat& _rot3);
  
  void output_data(char* imu_file);
  float* getRotation(const double& ti);
  cv::Mat GetCalibMatrix(){return cabMatrix;}
  gtsam::PreintegratedCombinedMeasurements *Getimufactor(int);
  gtsam::PreintegratedCombinedMeasurements *GetInstantimufactorDebug(double);
  gtsam::PreintegratedCombinedMeasurements *Getframeimufactor();
  IMUMeasurement getImuMeasurement(int);
  static bool sensor_config(MPU9250 *, sensor_cfg_t );  
  
  vector<int> imu_frame_mnId;
  PreintegratedCombinedMeasurements *frameimupre;
  PreintegratedCombinedMeasurements *KFframeimupre;
private:
  double last_keyframe_time;
  double last_frame_time;
  MPU9250 mySensor;
  gtsam::Pose3 Pose3_cali;
  sensor_6axis_t my_data[SENSOR_BUF_SIZE];
  vector<my_sensor_6axis_t> my_data_vec;
  //vector<>
  imuBias::ConstantBias currentBias;
  // = imuBias::ConstantBias( Vector3(0.0, 0.0, 0.0), Vector3(0.0,0.0,0.0));
  int debug_last_keyframe_index;
  int debug_instant_last_frame_index;
  int debug_instant_last_KFframe_index;
  float *Rot;
  
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> params;
  //PreintegratedCombinedMeasurements::Params* params;
  SensorFusion* g_sensorFusion;

  Mat cabMatrix;

 private:
   vector<gtsam::PreintegratedCombinedMeasurements*> VImuFactorPreintegratedMeasurements;
   //std::map<int , gtsam::PreintegratedCombinedMeasurements*> measureListMap;
   //std::map<int , IMUMeasurement> measureListMap;
};
#endif