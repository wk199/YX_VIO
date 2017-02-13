//ui051_mpu9150.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include "MPU9250.h"

sensor_6axis_t my_data[SENSOR_BUF_SIZE];

using namespace std;

static bool sensor_config(MPU9250 *mySensor, sensor_cfg_t sensor_cfg)
{
	mySensor->setFullScaleGyroRange(sensor_cfg.gyr_range);
	mySensor->setFullScaleAccelRange(sensor_cfg.acc_range);
	//please do not use below setting untill make sure their real meaning
	//mySensor->setXAccelOffset(sensor_cfg.ax_offset);
	//mySensor->setYAccelOffset(sensor_cfg.ay_offset);
	//mySensor->setZAccelOffset(sensor_cfg.az_offset);
	//mySensor->setXGyroOffsetUser(sensor_cfg.gx_offset);
	//mySensor->setYGyroOffsetUser(sensor_cfg.gy_offset);
	//mySensor->setZGyroOffsetUser(sensor_cfg.gz_offset);
	//end
	mySensor->setAccelFchoice(sensor_cfg.acc_fb_mode);
	mySensor->setFchoice(sensor_cfg.gry_fb_mode);
	if (sensor_cfg.mag_enabled)
		mySensor->enable_mag();
	printf("send avs config frame success\n");
	return true;
}

int main(void)
{
	MPU9250 mySensor;
	int temp = 9;
	int err;

	if (!mySensor.initialize()) {
		std::cout << " initPort fail! " << std::endl;
		mySensor.set_Exit();
	} else {
		std::cout << " initPort success :)" << std::endl;
		if (!mySensor.testConnection()) {
			std::cout << "  I2C read failed!" << std::endl;
			mySensor.set_Exit();
		}
	}
	mySensor.sensor_cfg.only_test_cap = false;//true;

	if (!mySensor.is_Exit() || mySensor.sensor_cfg.only_test_cap) {
		mySensor.sensor_cfg.log_enabled = /*false;*/true; //true will creat log files
		mySensor.sensor_cfg.acc_range = INV_FS_02G;
		mySensor.sensor_cfg.gyr_range = INV_FSR_500DPS;
		mySensor.sensor_cfg.mag_enabled = false;//true;
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
		mySensor.sensor_cfg.interval_max_ms = 1000;//1000ms
		sensor_config(&mySensor, mySensor.sensor_cfg);
		mySensor.sensor_set_buf(&my_data);//sensor will put data to my_data
		mySensor.set_max_data_num(500);//about 300ms
		if (!mySensor.OpenListenThread()){
			std::cout << "OpenListenThread fail!" << std::endl;
		}
		else {
			std::cout << "OpenListenThread success!" << std::endl;
			std::cout << " Reading data from sensors... ..." << std::endl;
		}
	}
	std::cout << " " << std::endl;
	std::cout << " Input 0 to exit , other number to do a test " << std::endl;
//	std::cout << " Ctrl + c to exit  ..." << std::endl;
	while (0 != temp) {
		//below only for test
		if (!mySensor.is_Exit() || mySensor.sensor_cfg.only_test_cap) {
			mySensor.sensor_cap_start();
			printf(" data_num=%d\n",mySensor.get_data_num());
		}
		sleep(1);
		//test end
		err = scanf("%d",&temp);
		if (0>err)
			 printf("scanf wrong!!\n");
		//printf("get %d \n",temp);
	}
	mySensor.set_Exit();
	printf(" will exit \n");
	sleep(1);
	return 0;
}

