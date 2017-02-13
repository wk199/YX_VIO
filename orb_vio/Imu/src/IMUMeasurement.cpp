#include"IMUMeasurement.h"
using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

bool IMUMeasurementList::sensor_config(MPU9250 *mySensor, sensor_cfg_t sensor_cfg)
{
  mySensor->setFullScaleGyroRange(sensor_cfg.gyr_range);
  mySensor->setFullScaleAccelRange(sensor_cfg.acc_range);
   mySensor->setAccelFchoice(sensor_cfg.acc_fb_mode);
   mySensor->setFchoice(sensor_cfg.gry_fb_mode);
   if (sensor_cfg.mag_enabled)
     mySensor->enable_mag();
   printf("send avs config frame success\n");
   return true;
}
IMUMeasurementList::IMUMeasurementList(const Mat & _calib)/* : cabMatrix(_calib)*/{
	_calib.copyTo(cabMatrix);
  if (!mySensor.initialize())
       {
         std::cout << " initPort fail! " << std::endl;
         mySensor.set_Exit();
       }
     else
       {
         std::cout << " initPort success :)" << std::endl;
         if (!mySensor.testConnection())
           {
             std::cout << "  I2C read failed!" << std::endl;
             mySensor.set_Exit();
           }
       }
     mySensor.sensor_cfg.only_test_cap = false; //true;
     if (!mySensor.is_Exit() || mySensor.sensor_cfg.only_test_cap)
       {
         mySensor.sensor_cfg.log_enabled = false;//true; //true will creat log files
         mySensor.sensor_cfg.acc_range = INV_FS_02G;
         mySensor.sensor_cfg.gyr_range = INV_FSR_500DPS;
         mySensor.sensor_cfg.mag_enabled = false; //true;
         mySensor.sensor_cfg.mag_range = INV_MAG_16BIT;
         mySensor.sensor_cfg.ax_offset = 0;
         mySensor.sensor_cfg.ay_offset = 0;
         mySensor.sensor_cfg.az_offset = 0;
         mySensor.sensor_cfg.gx_offset = 0;
         mySensor.sensor_cfg.gy_offset = 0;
         mySensor.sensor_cfg.gz_offset = 0;
         mySensor.sensor_cfg.acc_fb_mode = INV_ACC_F_B_1K;
         mySensor.sensor_cfg.gry_fb_mode = INV_GRY_F_B_3K;
         mySensor.sensor_cfg.lpf = INV_FILTER_2100HZ_NOLPF;
         mySensor.sensor_cfg.lp_acc_odr = ACC_ODR_500HZ;
         mySensor.sensor_cfg.interval_max_ms = 2000; //1000ms
         sensor_config(&mySensor, mySensor.sensor_cfg);
         mySensor.sensor_set_buf(&my_data); //sensor will put data to my_data
         mySensor.set_max_data_num(1000); //about 300ms
         if (!mySensor.OpenListenThread())
           {
             std::cout << "OpenListenThread fail!" << std::endl;
           }
         else
           {
             std::cout << "OpenListenThread success!" << std::endl;
             std::cout << " Reading data from sensors... ..." << std::endl;
           }
       }
       
      debug_last_keyframe_index = 0;    
      IMUMeasurementListParam imu_params;  
      cv::Mat _calibMat = GetCalibMatrix();
      imu_params.setPoseBodySensor(_calibMat);
      params = boost::make_shared<PreintegratedCombinedMeasurements::Params>(imu_params.n_gravity); 
      params->biasAccCovariance = imu_params.biasAccCovariance;
      params->biasOmegaCovariance = imu_params.biasOmegaCovariance;
      params->biasAccOmegaInt = imu_params.biasAccOmegaInt;     
      params->setAccelerometerCovariance(imu_params.accelerometerCovariance);
      params->setIntegrationCovariance(imu_params.integrationCovariance);
      params->setGyroscopeCovariance(imu_params.gyroscopeCovariance); 
      params->setBodyPSensor(imu_params.Pose_Body_Sensor);
      g_sensorFusion = new SensorFusion();
}
IMUMeasurementList::IMUMeasurementList()
{
  if (!mySensor.initialize())
       {
         std::cout << " initPort fail! " << std::endl;
         mySensor.set_Exit();
       }
     else
       {
         std::cout << " initPort success :)" << std::endl;
         if (!mySensor.testConnection())
           {
             std::cout << "  I2C read failed!" << std::endl;
             mySensor.set_Exit();
           }
       }
     mySensor.sensor_cfg.only_test_cap = false; //true;
     if (!mySensor.is_Exit() || mySensor.sensor_cfg.only_test_cap)
       {
         mySensor.sensor_cfg.log_enabled = false;//true; //true will creat log files
         mySensor.sensor_cfg.acc_range = INV_FS_02G;
         mySensor.sensor_cfg.gyr_range = INV_FSR_500DPS;
         mySensor.sensor_cfg.mag_enabled = false; //true;
         mySensor.sensor_cfg.mag_range = INV_MAG_16BIT;
         mySensor.sensor_cfg.ax_offset = 0;
         mySensor.sensor_cfg.ay_offset = 0;
         mySensor.sensor_cfg.az_offset = 0;
         mySensor.sensor_cfg.gx_offset = 0;
         mySensor.sensor_cfg.gy_offset = 0;
         mySensor.sensor_cfg.gz_offset = 0;
         mySensor.sensor_cfg.acc_fb_mode = INV_ACC_F_B_1K;
         mySensor.sensor_cfg.gry_fb_mode = INV_GRY_F_B_3K;
         mySensor.sensor_cfg.lpf = INV_FILTER_2100HZ_NOLPF;
         mySensor.sensor_cfg.lp_acc_odr = ACC_ODR_500HZ;
         mySensor.sensor_cfg.interval_max_ms = 2000; //1000ms
         sensor_config(&mySensor, mySensor.sensor_cfg);
         mySensor.sensor_set_buf(&my_data); //sensor will put data to my_data
         mySensor.set_max_data_num(2000); //about 300ms
         if (!mySensor.OpenListenThread())
           {
             std::cout << "OpenListenThread fail!" << std::endl;
           }
         else
           {
             std::cout << "OpenListenThread success!" << std::endl;
             std::cout << " Reading data from sensors... ..." << std::endl;
           }
       }
  }
 
IMUMeasurementList::IMUMeasurementList(const Mat & _calib , char* imu_file) :  cabMatrix(_calib){
  
  //vector<sensor_6axis_t>
  g_sensorFusion = new SensorFusion();
  FILE *fp = fopen(imu_file , "r");
  double acc_x, acc_y , acc_z , gyo_x, gyo_y , gyo_z;
  long double vv;
  double timestample;
  double previous_time = 0.0;
  
  while(    fscanf(fp, "%Lf %lf %lf %lf %lf %lf %lf",&vv, &gyo_x, &gyo_y , &gyo_z ,
   &acc_x, &acc_y,&acc_z )!=EOF){
     timestample = vv / (1000000000);
      my_sensor_6axis_t cur_data;
      cur_data.f_acc_x = acc_x;
      cur_data.f_acc_y = acc_y;
      cur_data.f_acc_z = acc_z;
      cur_data.f_gyr_x = gyo_x;
      cur_data.f_gyr_y = gyo_y;
      cur_data.f_gyr_z = gyo_z;
      cur_data.wMilliseconds = timestample;
      if(my_data_vec.size() <= 0){
        previous_time = timestample - 0.0025;
      }
      double delta_t = timestample - previous_time;
      g_sensorFusion->handlemessage(gyo_x , gyo_y , gyo_z , acc_x , acc_y , acc_z , delta_t);
      for(int i=0;i<9;i++){
        cur_data.Rot[i] = g_sensorFusion->RotateMetrix[i];     
      }
      my_data_vec.push_back(cur_data);
      previous_time = timestample;
    }  
   fclose(fp);
   cout << "finish loading the IMU data "<<endl;
   debug_last_keyframe_index = 0;
   debug_instant_last_frame_index = 0;
   debug_instant_last_KFframe_index = 0;
   IMUMeasurementListParam imu_params;  
   cv::Mat _calibMat = GetCalibMatrix();
   imu_params.setPoseBodySensor(_calibMat);
   params = boost::make_shared<PreintegratedCombinedMeasurements::Params>(imu_params.n_gravity); 
   params->biasAccCovariance = imu_params.biasAccCovariance;
   params->biasOmegaCovariance = imu_params.biasOmegaCovariance;
   params->biasAccOmegaInt = imu_params.biasAccOmegaInt;     
   params->setAccelerometerCovariance(imu_params.accelerometerCovariance);
   params->setIntegrationCovariance(imu_params.integrationCovariance);
   params->setGyroscopeCovariance(imu_params.gyroscopeCovariance); 
   params->setBodyPSensor(imu_params.Pose_Body_Sensor);
   cout << "the timestample number is : "<<my_data_vec.size()<<endl;  
}
  
IMUMeasurementList::~IMUMeasurementList()
{
}
void IMUMeasurementList::Run()
{
  while(1)
  {
      mySensor.sensor_cap_start();
      int tep = mySensor.get_data_num();
  }
 }
 
//IMUMeasurement IMUMeasurementList::getImuMeasurement(int id){
//  return measureListMap[id];
//} 
PreintegratedCombinedMeasurements *IMUMeasurementList::GetInstantimufactorDebug(double _time){
  
   //imu_frame_mnId.push_back(Id);
  cout <<setiosflags(ios::fixed); 
  //cout << "Search an Instant IMUFactor in the IMU Debug Thread"<<endl;
  int start=0,end=0;
  bool start_bool=true,end_bool=true;
  //double imu_time = _time;
  PreintegratedCombinedMeasurements *pim = new PreintegratedCombinedMeasurements(params , currentBias);
  
  for(int i=0;i<my_data_vec.size()-1;i++)
   {
      double imu_time=my_data_vec[i].wMilliseconds;//my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data_vec[i+1].wMilliseconds;//my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;

        if(last_keyframe_time >= imu_time && last_keyframe_time <= imu_time2 && start_bool)
        {
          start=i;
          //cout << "Find start"<<endl;
          start_bool=false;
          continue;
        }
        else if (_time >= imu_time && _time <= imu_time2&&end_bool)
        {
          end=i;
          end_bool=false;
          //cout << "Find End"<<endl;
          debug_last_keyframe_index = end;
          break;
        }
    }
    //cout << "start = " << start << ", end = " << end << endl;
  double deltaT=0.0,temp=0.0;
  for (int j = start; j <= end; j++)
    {
      Vector3 measuredAcc(my_data_vec[j].f_acc_x, my_data_vec[j].f_acc_y,my_data_vec[j].f_acc_z);
      Vector3 measuredOmega(my_data_vec[j].f_gyr_x, my_data_vec[j].f_gyr_y,my_data_vec[j].f_gyr_z);
      if (j == start)
        {
          deltaT = (start >= 1) ? (my_data_vec[start].wMilliseconds - my_data_vec[start-1].wMilliseconds) : 0.0025;
          
          temp = my_data_vec[j].wMilliseconds;
        }
      else
        {
          deltaT = my_data_vec[j].wMilliseconds - temp;
          temp = my_data_vec[j].wMilliseconds;
        }
      pim->integrateMeasurement(measuredAcc, measuredOmega,double(deltaT));
    }
  pim->Id=1;
  //last_keyframe_time=time;
  return pim;  
}

PreintegratedCombinedMeasurements *IMUMeasurementList::Getimufactor(int Id)
{
  for (vector<PreintegratedCombinedMeasurements*>::iterator lit = VImuFactorPreintegratedMeasurements.begin(), lend = VImuFactorPreintegratedMeasurements.end(); lit != lend; lit++)
    {
      if((*lit)->Id==Id)
        {
          return *lit;
        }
     }
  //IMUMeasurement _imu_measurement = measureListMap[Id];
  //return _imu_measurement.getIMUMeasurement();
  //return measureListMap[Id];
}

void IMUMeasurementList::AddimufactorDebug(double time,int Id){
  
  //mutex.lock();
  imu_frame_mnId.push_back(Id);
  cout <<setiosflags(ios::fixed); 
  //cout << "Add an IMUFactor in the IMU Debug Thread"<<endl;
  int start=0,end=0;
  bool start_bool=true,end_bool=true;
  PreintegratedCombinedMeasurements *pim = new PreintegratedCombinedMeasurements(params , currentBias);
  
  for(int i=0;i<my_data_vec.size()-1;i++)
   {
      double imu_time=my_data_vec[i].wMilliseconds;//my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data_vec[i+1].wMilliseconds;//my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;

        if(last_keyframe_time >= imu_time && last_keyframe_time <= imu_time2 && start_bool)
        {
          start=i;
          //cout << "Find start"<<endl;
          start_bool=false;
          continue;
        }
        else if (time >= imu_time && time <= imu_time2&&end_bool)
        {
          end=i;
          end_bool=false;
          //cout << "Find End"<<endl;
          debug_last_keyframe_index = end;
          break;
        }
    }
    //cout << "start = " << start << ", end = " << end << endl;
  double deltaT=0.0,temp=0.0;
  for (int j = start; j <= end; j++)
    {
      Vector3 measuredAcc(my_data_vec[j].f_acc_x, my_data_vec[j].f_acc_y,my_data_vec[j].f_acc_z);
      Vector3 measuredOmega(my_data_vec[j].f_gyr_x, my_data_vec[j].f_gyr_y,my_data_vec[j].f_gyr_z);
      if (j == start)
        {
          deltaT = (start >= 1) ? (my_data_vec[start].wMilliseconds - my_data_vec[start-1].wMilliseconds) : 0.0025;
          
          temp = my_data_vec[j].wMilliseconds;
        }
      else
        {
          deltaT = my_data_vec[j].wMilliseconds - temp;
          temp = my_data_vec[j].wMilliseconds;
        }
      pim->integrateMeasurement(measuredAcc, measuredOmega,double(deltaT));
    }
  pim->Id=Id;
  last_keyframe_time=time;
  //pim->print();
  VImuFactorPreintegratedMeasurements.push_back(pim);
  //IMUMeasurement _imu_measurement;
  //_imu_measurement.setIMUMeasurement(pim);
  //_imu_measurement.setId(Id);
  //_imu_measurement.setRot(g_sensorFusion->RotateMetrix);
  //measureListMap[Id] = _imu_measurement;
  //measureListMap[Id] = pim;
  //PreintegratedCombinedMeasurements *ppim = _imu_measurement.getIMUMeasurement();
  //ppim->print();
  //cout << "Finish Add an IMUFactor in the IMU Debug Thread"<<endl;
}

PreintegratedCombinedMeasurements *IMUMeasurementList::GetInstantKFAndFrameimufactor(double _time1, double _time2)
{
  cout <<setiosflags(ios::fixed); 
  //cout << "Search an Instant IMUFactor in the IMU Debug Thread"<<endl;
  int start=0,end=0;
  bool start_bool=true,end_bool=true;
  //double imu_time = _time;
  PreintegratedCombinedMeasurements *pim = new PreintegratedCombinedMeasurements(params , currentBias);
  
  for(int i=debug_instant_last_KFframe_index;i<my_data_vec.size()-1;i++)
   {
      double imu_time=my_data_vec[i].wMilliseconds;//my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data_vec[i+1].wMilliseconds;//my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;

        if(_time1 >= imu_time && _time1 <= imu_time2 && start_bool)
        {
          start=i;
          //cout << "Find start"<<endl;
          start_bool=false;
          continue;
        }
        else if (_time2 >= imu_time && _time2 <= imu_time2&&end_bool)
        {
          end=i;
          end_bool=false;
          //cout << "Find End"<<endl;
          //debug_last_keyframe_index = end;
          break;
        }
    }
    //cout << "start = " << start << ", end = " << end << endl;
  double deltaT=0.0,temp=0.0;
  for (int j = start; j <= end; j++)
    {
      Vector3 measuredAcc(my_data_vec[j].f_acc_x, my_data_vec[j].f_acc_y,my_data_vec[j].f_acc_z);
      Vector3 measuredOmega(my_data_vec[j].f_gyr_x, my_data_vec[j].f_gyr_y,my_data_vec[j].f_gyr_z);
      if (j == start)
        {
          deltaT = (start >= 1) ? (my_data_vec[start].wMilliseconds - my_data_vec[start-1].wMilliseconds) : 0.0025;
          
          temp = my_data_vec[j].wMilliseconds;
        }
      else
        {
          deltaT = my_data_vec[j].wMilliseconds - temp;
          temp = my_data_vec[j].wMilliseconds;
        }
      pim->integrateMeasurement(measuredAcc, measuredOmega,double(deltaT));
    }
  pim->Id=1;
  //last_keyframe_time=time;
  debug_instant_last_KFframe_index = end;
  return pim;  
}

PreintegratedCombinedMeasurements *IMUMeasurementList::GetInstantimufactorDebug2(double _time1, double _time2){
  
   //imu_frame_mnId.push_back(Id);
  cout <<setiosflags(ios::fixed); 
  //cout << "Search an Instant IMUFactor in the IMU Debug Thread"<<endl;
  int start=0,end=0;
  bool start_bool=true,end_bool=true;
  //double imu_time = _time;
  PreintegratedCombinedMeasurements *pim = new PreintegratedCombinedMeasurements(params , currentBias);
  
  for(int i=debug_instant_last_frame_index;i<my_data_vec.size()-1;i++)
   {
      double imu_time=my_data_vec[i].wMilliseconds;//my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data_vec[i+1].wMilliseconds;//my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;

        if(_time1 >= imu_time && _time1 <= imu_time2 && start_bool)
        {
          start=i;
          //cout << "Find start"<<endl;
          start_bool=false;
          continue;
        }
        else if (_time2 >= imu_time && _time2 <= imu_time2&&end_bool)
        {
          end=i;
          end_bool=false;
          //cout << "Find End"<<endl;
          //debug_last_keyframe_index = end;
          break;
        }
    }
    //cout << "start = " << start << ", end = " << end << endl;
  double deltaT=0.0,temp=0.0;
  for (int j = start; j <= end; j++)
    {
      Vector3 measuredAcc(my_data_vec[j].f_acc_x, my_data_vec[j].f_acc_y,my_data_vec[j].f_acc_z);
      Vector3 measuredOmega(my_data_vec[j].f_gyr_x, my_data_vec[j].f_gyr_y,my_data_vec[j].f_gyr_z);
      if (j == start)
        {
          deltaT = (start >= 1) ? (my_data_vec[start].wMilliseconds - my_data_vec[start-1].wMilliseconds) : 0.0025;
          
          temp = my_data_vec[j].wMilliseconds;
        }
      else
        {
          deltaT = my_data_vec[j].wMilliseconds - temp;
          temp = my_data_vec[j].wMilliseconds;
        }
      pim->integrateMeasurement(measuredAcc, measuredOmega,double(deltaT));
    }
  pim->Id=1;
  //last_keyframe_time=time;
  debug_instant_last_frame_index = end;
  return pim;  
}

void IMUMeasurementList::Addimufactor(double time,int Id){
  
  //mutex.lock();
  imu_frame_mnId.push_back(Id);
  cout <<setiosflags(ios::fixed); 
  cout << "Add an IMUFactor in the IMU Debug Thread"<<endl;
  int start=0,end=0;
  bool start_bool=true,end_bool=true;
  PreintegratedCombinedMeasurements *pim = new PreintegratedCombinedMeasurements(params , currentBias);

 for(int i=0;i<SENSOR_BUF_SIZE;i++)
 {
      double imu_time=my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;

      //cout.setf(ios::fixed);
      //cout<<setprecision(4)<<"last_keyframe_time:"<<last_keyframe_time<<"keyframe:"<<time<<"imu_time:"<<imu_time<<endl;
        if(last_keyframe_time-imu_time>0&&last_keyframe_time-imu_time2<0&&start_bool)
        {
          start=i;
          start_bool=false;
          continue;
        }
        else if (time-imu_time>0&&time-imu_time2<0&&end_bool)
        {
          end=i;
          end_bool=false;
          break;
        }
  }
  int deltaT=0.0,temp=0.0;
  for (int j = start; j <= end; j++)
  {
      Vector3 measuredAcc(my_data[j].f_acc_x, my_data[j].f_acc_y,my_data[j].f_acc_z);
      Vector3 measuredOmega(my_data[j].f_gyr_x, my_data[j].f_gyr_y,my_data[j].f_gyr_z);
      if (j == start)
        {
          deltaT = 0.0025;
          temp = my_data[j].wMilliseconds;
        }
      else
        {
          deltaT = (my_data[j].wMilliseconds - temp)/1000;
          temp = my_data[j].wMilliseconds;
        }
      pim->integrateMeasurement(measuredAcc, measuredOmega,double(deltaT));
   }
  pim->Id=Id;
  last_keyframe_time=time;
  VImuFactorPreintegratedMeasurements.push_back(pim);
  //IMUMeasurement _imu_measurement;
  //_imu_measurement.setIMUMeasurement(pim);
  //_imu_measurement.setId(Id);
  //_imu_measurement.setRot(g_sensorFusion->RotateMetrix);
  //measureListMap[Id] = _imu_measurement;
  //measureListMap[Id] = pim;

  cout << "Finish Add an IMUFactor in the IMU Debug Thread"<<endl;
}

gtsam::PreintegratedCombinedMeasurements *IMUMeasurementList::Getframeimufactor()
{
  return frameimupre;
}

void IMUMeasurementList::Addframeimufactor(double time,int Id)
{
  int start=0,end=0;
  bool start_bool=true,end_bool=true;
  PreintegratedCombinedMeasurements *pim = new PreintegratedCombinedMeasurements(params , currentBias);
  
  for(int i=0;i<my_data_vec.size()-1;i++)
   {
      double imu_time=my_data_vec[i].wMilliseconds;//my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data_vec[i+1].wMilliseconds;//my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;

        if(last_frame_time >= imu_time && last_frame_time <= imu_time2 && start_bool)
        {
          start=i;
          //cout << "Find start"<<endl;
          start_bool=false;
          continue;
        }
        else if (time >= imu_time && time <= imu_time2&&end_bool)
        {
          end=i;
          end_bool=false;
          //cout << "Find End"<<endl;
          debug_last_keyframe_index = end;
          break;
        }
    }
 
  double deltaT=0.0,temp=0.0;
  for (int j = start; j <= end; j++)
    {
      Vector3 measuredAcc(my_data_vec[j].f_acc_x, my_data_vec[j].f_acc_y,my_data_vec[j].f_acc_z);
      Vector3 measuredOmega(my_data_vec[j].f_gyr_x, my_data_vec[j].f_gyr_y,my_data_vec[j].f_gyr_z);
      if (j == start)
        {
          deltaT = (start >= 1) ? (my_data_vec[start].wMilliseconds - my_data_vec[start-1].wMilliseconds) : 0.0025;
          
          temp = my_data_vec[j].wMilliseconds;
        }
      else
        {
          deltaT = my_data_vec[j].wMilliseconds - temp;
          temp = my_data_vec[j].wMilliseconds;
        }
      pim->integrateMeasurement(measuredAcc, measuredOmega,double(deltaT));
    }
  pim->Id=Id;
  last_frame_time=time;
  frameimupre = pim;
}

void IMUMeasurementList::AddKFframeimufactor(double time,int Id)
{
  int start=0,end=0;
  bool start_bool=true,end_bool=true;
  PreintegratedCombinedMeasurements *pim = new PreintegratedCombinedMeasurements(params , currentBias);
  
  for(int i=0;i<my_data_vec.size()-1;i++)
   {
      double imu_time=my_data_vec[i].wMilliseconds;//my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data_vec[i+1].wMilliseconds;//my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;

        if(last_keyframe_time >= imu_time && last_keyframe_time <= imu_time2 && start_bool)
        {
          start=i;
          //cout << "Find start"<<endl;
          start_bool=false;
          continue;
        }
        else if (time >= imu_time && time <= imu_time2&&end_bool)
        {
          end=i;
          end_bool=false;
          //cout << "Find End"<<endl;
          debug_last_keyframe_index = end;
          break;
        }
    }
 
  double deltaT=0.0,temp=0.0;
  for (int j = start; j <= end; j++)
    {
      Vector3 measuredAcc(my_data_vec[j].f_acc_x, my_data_vec[j].f_acc_y,my_data_vec[j].f_acc_z);
      Vector3 measuredOmega(my_data_vec[j].f_gyr_x, my_data_vec[j].f_gyr_y,my_data_vec[j].f_gyr_z);
      if (j == start)
        {
          deltaT = (start >= 1) ? (my_data_vec[start].wMilliseconds - my_data_vec[start-1].wMilliseconds) : 0.0025;
          
          temp = my_data_vec[j].wMilliseconds;
        }
      else
        {
          deltaT = my_data_vec[j].wMilliseconds - temp;
          temp = my_data_vec[j].wMilliseconds;
        }
      pim->integrateMeasurement(measuredAcc, measuredOmega,double(deltaT));
    }
  pim->Id=Id;
  last_frame_time=time;
  KFframeimupre = pim;
}



void IMUMeasurementList::output_data(char* imu_file){
  FILE *fp = fopen(imu_file , "w+");
  //fprintf(fp, "%d\n" , my_data.size());
  for(int i=0;i<1000;i++){
    fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf\n" , my_data_vec[i].f_acc_x , my_data_vec[i].f_acc_y,
    my_data_vec[i].f_acc_z , my_data_vec[i].f_gyr_x , my_data_vec[i].f_gyr_y , my_data_vec[i].f_gyr_z,
    my_data_vec[i].wMilliseconds);
    for(int j=0;j<9;j++){
      fprintf(fp, "%lf " , my_data_vec[i].Rot[j]);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
}

  void IMUMeasurementList::setFirstTimeStampleandmnId(const double& _time,int mnId, cv::Mat& _rot3)
  { 
    last_keyframe_time = _time;
    imu_frame_mnId.push_back(mnId);
    int start = 0;
    for(int i=0;i<SENSOR_BUF_SIZE;i++)
    {
      double imu_time=my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;

      if(last_keyframe_time-imu_time>0&&last_keyframe_time-imu_time2<0)
      {
          start=i;
          //continue;
          break;
       }
     }  
     _rot3.convertTo(_rot3 , CV_32F); // IMU pose in the IMU frame, i.e., map a point from the IMU frame to the Nav frame     
     cv::Mat _rot_3(3,3,CV_32F , my_data[start].Rot);
    
     cv::Mat _calib_rot3(3,3,CV_32F); // pose from the IMU frame to the Camera frame
     _calib_rot3 = cabMatrix.rowRange(0,3).colRange(0,3);
     cv::Mat _cam_rot3 = _calib_rot3 * _rot_3.t(); // Pose Cam equals 
     
     _cam_rot3.copyTo(_rot3.rowRange(0,3).colRange(0,3));
     _rot3.at<float>(3,3) = 1.0;
     VImuFactorPreintegratedMeasurements.clear();
  }


  void IMUMeasurementList::setFirstTimeStampleandmnIdDebug(const double& _time,int mnId, cv::Mat& _rot3)
  { 
    last_keyframe_time = _time;
    imu_frame_mnId.push_back(mnId);
    int start = 0;
  for(int i=0;i<my_data_vec.size()-1;i++)
   {
      double imu_time=my_data_vec[i].wMilliseconds;//my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
      double imu_time2=my_data_vec[i+1].wMilliseconds;//my_data[i+1].wDay*24.0*3600.0+my_data[i+1].wHour*3600.0+my_data[i+1].wMinute*60.0+my_data[i+1].wSecond+double(my_data[i+1].wMilliseconds)/1000.00;
      //cout << "last_keyframe_time = " << last_keyframe_time << ", imu_time = " << imu_time << std::endl;
        if(last_keyframe_time >= imu_time && last_keyframe_time <= imu_time2)
        {
          start=i;
          //cout << "Find start"<<endl;
          break;
          //continue;
        }
    }
    
     _rot3.convertTo(_rot3 , CV_32F); // IMU pose in the IMU frame, i.e., map a point from the IMU frame to the Nav frame     
     cv::Mat _rot_3(3,3,CV_32F , my_data_vec[start].Rot);
    
     cv::Mat _calib_rot3(3,3,CV_32F); // pose from the IMU frame to the Camera frame
     _calib_rot3 = cabMatrix.rowRange(0,3).colRange(0,3);
     cv::Mat _cam_rot3 = _calib_rot3 * _rot_3.t(); // Pose Cam equals 
     
     _cam_rot3.copyTo(_rot3.rowRange(0,3).colRange(0,3));
     _rot3.at<float>(3,3) = 1.0;
     VImuFactorPreintegratedMeasurements.clear();
  }
