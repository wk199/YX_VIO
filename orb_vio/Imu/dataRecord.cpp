

#include <iostream>
#include <vector>
#include <MPU9250.h>
#define SENSOR_BUF_SIZE 2000


int main(){
    
    MPU9250 mySensor;
    sensor_6axis_t my_data[SENSOR_BUF_SIZE];
    
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
   
   while(1){
        ofstream fout;
        fout.open("orb_out.txt", ios::app);
        //fout << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << endl;
        
        mySensor.sensor_cap_start();
        int tep = mySensor.get_data_num();
 
        for(int i=0;i<tep ;i ++){
            double imu_time=my_data[i].wDay*24.0*3600.0+my_data[i].wHour*3600.0+my_data[i].wMinute*60.0+my_data[i].wSecond+double(my_data[i].wMilliseconds)/1000.00;
            fout <<my_data[i].f_acc_x<<" "<<my_data[i].f_acc_y <<" "<<my_data[i].f_acc_z<<" "
            <<my_data[i].f_gyr_x<<" "<<my_data[i].f_gyr_y <<" "<<my_data[i].f_gyr_z<<" "<<imu_time<<endl;
        }
        fout.flush();
        fout.close();
       
   }
    
    return 0;
    
    
}

