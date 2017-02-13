/* ============================================
This code is placed under the MIT license
Copyright (c) 2016 Clark.Lyv

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "Sensor.h"
#include <pthread.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <string.h>
#include <fstream>
#include <unistd.h>
#include <time.h>
#include "MPU9250.h"
#include "usb2io.h"


libusb_device_handle* i2c_fp = NULL;
int i2c_closed = -1;
bool s_bExit = false;
char tmp_d[256];


int i2c_open()
{
	i2c_fp = USB2IO_Open(1);//currently only support one UI051 one time
	if (NULL != i2c_fp &&  ((void*)-1 != i2c_fp))
			i2c_closed = USB2IO_EnableI2c(i2c_fp);
	
	return i2c_closed;
}
char i2c_close()
{
	if (NULL != i2c_fp && ((void*)-1 != i2c_fp))
		return USB2IO_Close(i2c_fp);

	return 0;
}

/** Read a single bit from an 8-bit device register.
* @param devAddr I2C slave device address
* @param regAddr Register regAddr to read from
* @param bitNum Bit position to read (0-7)
* @param data Container for single bit value
* @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
* @return Status of read operation (true = success)
*/
char i2c_readBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned char *data)
{
	if (i2c_closed)
		return -1;

	char count = USB2IO_I2cRead(i2c_fp, devAddr, regAddr, 1, 1, (char *)data);
	data[0] = data[0] & (1 << bitNum);
	return count;
}

/** Read multiple bits from an 8-bit device register.
* @param devAddr I2C slave device address
* @param regAddr Register regAddr to read from
* @param bitStart First bit position to read (0-7)
* @param length Number of bits to read (not more than 8)
* @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
* @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
* @return Status of read operation (true = success)
*/
char i2c_readBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	if (i2c_closed)
		return -1;

	char count = USB2IO_I2cRead(i2c_fp, devAddr, regAddr, 1, 1, (char *)data);
	if (0 <= count) {
		unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
		data[0] &= mask;
		data[0] >>= (bitStart - length + 1);
	}
	return count;
}

/** write a single bit in an 8-bit device register.
* @param devAddr I2C slave device address
* @param regAddr Register regAddr to write to
* @param bitNum Bit position to write (0-7)
* @param value New bit value to write
* @return Status of operation (true = success)
*/
bool i2c_writeBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned char data)
{
	if (i2c_closed)
		return false;

	if (0 >= USB2IO_I2cRead(i2c_fp, devAddr, regAddr, 1, 1, tmp_d))
		return false;

	tmp_d[0] = (data != 0) ? (tmp_d[0] | (1 << bitNum)) : (tmp_d[0] & ~(1 << bitNum));

	int count = USB2IO_I2cWrite(i2c_fp, devAddr, regAddr, 1, 1, tmp_d);
	if (0 <= count)
		return true;
	else
		return false;
}

/** Write multiple bits in an 8-bit device register.
* @param devAddr I2C slave device address
* @param regAddr Register regAddr to write to
* @param bitStart First bit position to write (0-7)
* @param length Number of bits to write (not more than 8)
* @param data Right-aligned value to write
* @return Status of operation (true = success)
*/
bool i2c_writeBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	if (i2c_closed)
		return false;

	if (0 > USB2IO_I2cRead(i2c_fp, devAddr, regAddr, 1, 1, tmp_d))
		return false;

	unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp_d[0] &= ~(mask); // zero all important bits in existing byte
	tmp_d[0] |= data; // combine data with existing byte

	int count = USB2IO_I2cWrite(i2c_fp, devAddr, regAddr, 1, 1, tmp_d);
	if (0 <= count)
		return true;
	else
		return false;
}

/** Notify sensor that camera will capture a frame.
*/
void MPU9250::sensor_cap_start()
{
	end_num = current_num;
}

void MPU9250::set_max_data_num(int max)
{
	max_data_num = max;
}

void MPU9250::check_buf_num()
{
	current_num++;
	if (SENSOR_BUF_SIZE == current_num)
	{	
		current_num = 0;
		current_full=true;
	}
}

int MPU9250::get_data_num()
{
	/*int tmp_num = end_num - begin_num;
	int i;
	int looped = 0;
	if (0 < tmp_num) {
		looped = 0;
		if ((tmp_num > max_data_num) || (data_interval > sensor_cfg.interval_max_ms)) {
			tmp_num = max_data_num;
			begin_num = end_num - tmp_num;
		}
	}
	else {
		tmp_num = (SENSOR_BUF_SIZE - begin_num) + end_num;//not include end_num
		if ((tmp_num > max_data_num) || (data_interval > sensor_cfg.interval_max_ms)) {
			tmp_num = max_data_num;
			if (end_num > max_data_num) {
				begin_num = end_num - tmp_num;
				looped = 0;
			}
			else {
				begin_num = SENSOR_BUF_SIZE - (max_data_num - end_num);
				looped = 1;
			}
		}
	}

	if ( 0 == looped) {
		for (i = 0; i < tmp_num; i++)
			(*data_deal_ptr)[i] = data_ptr[begin_num+i];//end_num not include
	} else {
		for (i = 0; i < (tmp_num - end_num); i++)
			(*data_deal_ptr)[i] = data_ptr[begin_num + i];
		if (0 != end_num) 
			for (int j = 0; j < end_num; j++)
				(*data_deal_ptr)[i+j] = data_ptr[j];
	}
	begin_num = end_num;
	data_interval = 0;
	return tmp_num;*/
	if(current_full!=true)
		for(int j=0;j<current_num;j++)
		{
			(*data_deal_ptr)[j] = data_ptr[j];
		}
	else
	{
		for(int j=0;j<SENSOR_BUF_SIZE;j++)
		{	if(j+current_num+1<SENSOR_BUF_SIZE-1)
			{
				(*data_deal_ptr)[j] = data_ptr[j+current_num+1];
			}
			else
			{
				(*data_deal_ptr)[j] = data_ptr[j+current_num+1-SENSOR_BUF_SIZE+1];
			}			
		}
		
	}
	data_interval=0;
	return current_num;
}

/** ninitial pointer to pin-pong buffer.
* @return Status of operation (true = data is ready)
*/
bool MPU9250::sensor_set_buf(sensor_6axis_t(*pBuf)[SENSOR_BUF_SIZE])
{
	if (NULL == pBuf)
		return false;
	data_deal_ptr = pBuf;
	return true;
}
/** Default constructor, uses default I2C address.
 * @see MPU9250_DEFAULT_ADDRESS
 */
MPU9250::MPU9250()
{
	s_bExit = false;
	n_port = 1;//currently only support one MPU9250 one time
	devAddr = MPU9250_DEFAULT_ADDRESS;
	begin_num = 0;
	end_num = 0;
	current_num = 0;
	data_deal_ptr = NULL;
	max_data_num = 10;//60ms
	data_interval = 0;
             current_full=false;
	pthread_mutex_init(&m_csCommunicationSync,NULL);
}

/** Specific address constructor.
 * @param address I2C address
 * @see MPU9250_DEFAULT_ADDRESS
 * @see MPU9250_ADDRESS_AD0_LOW
 * @see MPU9250_ADDRESS_AD0_HIGH
 */
MPU9250::MPU9250(int address) 
{
	devAddr = address;
	s_bExit = false;
	n_port = 1;//currently only support one MPU9250 one time
              current_full=false;
	pthread_mutex_init(&m_csCommunicationSync,NULL);
}

MPU9250::~MPU9250()
{
	CloseListenThread();
	pthread_mutex_destroy(&m_csCommunicationSync);
}


bool MPU9250::openPort(unsigned char portNo)
{
	pthread_mutex_lock(&m_csCommunicationSync);

	pthread_mutex_unlock(&m_csCommunicationSync);
	return true;
}

bool MPU9250::OpenListenThread()
{
	int err;
	int policy;
	s_bExit = false;
	struct sched_param param;

	err = pthread_create(&m_hListenThread,NULL, ListenThread,  this);
	if (err) {
		printf(" Create Pthread failed!! \n");
		return false;
	}
	memset(&param, 0, sizeof(param));
	sleep(1);
	err = pthread_getschedparam(m_hListenThread,&policy,&param);
	if (err) {
                printf(" Get Pthread priority failed %d \n",err);
                return false;
        }
	param.sched_priority = sched_get_priority_min(SCHED_OTHER);
	policy = SCHED_OTHER;//SCHED_RR;
	param.sched_priority = sched_get_priority_min(policy);

	err = pthread_setschedparam(m_hListenThread,policy,&param);
	if (err) {
		printf(" Set Pthread priority failed %d \n",err);
		return false;
	}
	printf(" Set Pthread priority to %d \n",param.sched_priority);
	return true;
}

bool MPU9250::CloseListenThread()
{
	if (false == s_bExit)
	{
		s_bExit = true;
		sleep(10);
		pthread_join(m_hListenThread,NULL);
	}
	return true;
}

extern void InitSensorFusion(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ, float timeDelta);
extern void GetEulerAngles(float* angles);
using namespace std;
#define ACC2G 9.81
#define GYRRADIAN 3.14159/180
#define sensor_BUF_SIZE 4*1024
const unsigned int SLEEP_TIME_INTERVAL = 1; //seconds

struct sensor_9axis_t sensor_9axis;

static float sensor_acc_spec[NUM_ACCEL_FSR] = {16384,8192,4096,2048};
static float sensor_gyr_spec[NUM_FSR] = {131,65.5,32.8,16.4};
static float sensor_geo_spec[NUM_MAG] = { 0.6,15};

static bool sensor_get_raw(MPU9250 *pSensor,signed short *s_data)
{
	time_t sys_now;
	struct tm* sys_tm;
	struct timeval tv;
	char tmp_name[20];
	static int in_count = 0;
	static FILE *fout = NULL;
	static int i_ms = 0;

	time(&sys_now);	
	gettimeofday(&tv,NULL);
	sys_tm = localtime(&sys_now);
	if (pSensor->sensor_cfg.log_enabled) {
		if (0 == in_count) {
			sprintf(tmp_name, "%02d%02d%02d%02d%02d.log"
				, sys_tm->tm_mon, sys_tm->tm_mday, sys_tm->tm_hour, sys_tm->tm_min, sys_tm->tm_sec);
			fout = fopen(tmp_name, "w");
			in_count++;
		}
	}

	i_ms = (tv.tv_usec/1000) - sensor_9axis.wMilliseconds;
	if (0 > i_ms)
		i_ms += 1000;
	pSensor->data_interval += i_ms;
	sensor_9axis.wYear = sys_tm->tm_year;
	sensor_9axis.wMonth = sys_tm->tm_mon;
	sensor_9axis.wDay = sys_tm->tm_mday;
	sensor_9axis.wHour = sys_tm->tm_hour;
	sensor_9axis.wMinute = sys_tm->tm_min;
	sensor_9axis.wSecond = sys_tm->tm_sec;
	sensor_9axis.wMilliseconds = tv.tv_usec/1000;
	
	sensor_9axis.f_acc_x = (float)(s_data[0] / sensor_acc_spec[pSensor->sensor_cfg.acc_range]) *ACC2G;
	sensor_9axis.f_acc_y = (float)(s_data[1] / sensor_acc_spec[pSensor->sensor_cfg.acc_range]) *ACC2G;
	sensor_9axis.f_acc_z = (float)(s_data[2] / sensor_acc_spec[pSensor->sensor_cfg.acc_range]) *ACC2G;
	sensor_9axis.f_gyr_x = (float)(s_data[3] / sensor_gyr_spec[pSensor->sensor_cfg.gyr_range]) *GYRRADIAN;
	sensor_9axis.f_gyr_y = (float)(s_data[4] / sensor_gyr_spec[pSensor->sensor_cfg.gyr_range]) *GYRRADIAN;
	sensor_9axis.f_gyr_z = (float)(s_data[5] / sensor_gyr_spec[pSensor->sensor_cfg.gyr_range]) *GYRRADIAN;
	if (pSensor->sensor_cfg.mag_enabled) {
		sensor_9axis.f_geo_x = (float)(s_data[6] * sensor_geo_spec[pSensor->sensor_cfg.mag_range]) / 1000;
		sensor_9axis.f_geo_y = (float)(s_data[7] * sensor_geo_spec[pSensor->sensor_cfg.mag_range]) / 1000;
		sensor_9axis.f_geo_z = (float)(s_data[8] * sensor_geo_spec[pSensor->sensor_cfg.mag_range]) / 1000;
	}

	pSensor->data_ptr[pSensor->current_num].f_acc_x = sensor_9axis.f_acc_x;
	pSensor->data_ptr[pSensor->current_num].f_acc_y = sensor_9axis.f_acc_y;
	pSensor->data_ptr[pSensor->current_num].f_acc_z = sensor_9axis.f_acc_z;
	pSensor->data_ptr[pSensor->current_num].f_gyr_x = sensor_9axis.f_gyr_x;
	pSensor->data_ptr[pSensor->current_num].f_gyr_y = sensor_9axis.f_gyr_y;
	pSensor->data_ptr[pSensor->current_num].f_gyr_z = sensor_9axis.f_gyr_z;
	pSensor->data_ptr[pSensor->current_num].wYear = sensor_9axis.wYear;
	pSensor->data_ptr[pSensor->current_num].wMonth = sensor_9axis.wMonth;
	pSensor->data_ptr[pSensor->current_num].wDay = sensor_9axis.wDay;
	pSensor->data_ptr[pSensor->current_num].wHour = sensor_9axis.wHour;
	pSensor->data_ptr[pSensor->current_num].wMinute = sensor_9axis.wMinute;
	pSensor->data_ptr[pSensor->current_num].wSecond = sensor_9axis.wSecond;
	pSensor->data_ptr[pSensor->current_num].wMilliseconds = sensor_9axis.wMilliseconds;
	pSensor->check_buf_num();//move pointer
	InitSensorFusion(sensor_9axis.f_gyr_x, sensor_9axis.f_gyr_y, sensor_9axis.f_gyr_z
		, sensor_9axis.f_acc_x, sensor_9axis.f_acc_y, sensor_9axis.f_acc_z, (float)i_ms/1000);
	//	InitSensorFusion(0.005, 0.005, 0.005, 0,9.81,0,0.01);
//	GetEulerAngles(sensor_9axis.angles);
	GetRotateMetrix(pSensor->data_ptr[pSensor->current_num].Rot);

	if (NULL != fout) {
		fprintf(fout, "%02d:%02d:%02d:%03d{%02d}"
			, sensor_9axis.wHour, sensor_9axis.wMinute, sensor_9axis.wSecond, sensor_9axis.wMilliseconds, i_ms);

		fprintf(fout, "[acc:%f %f %f]", sensor_9axis.f_acc_x, sensor_9axis.f_acc_y, sensor_9axis.f_acc_z);
		fprintf(fout, " [gyr:%f %f %f]", sensor_9axis.f_gyr_x, sensor_9axis.f_gyr_y, sensor_9axis.f_gyr_z);
		if (pSensor->sensor_cfg.mag_enabled)
			fprintf(fout, " [geo:%f %f %f]", sensor_9axis.f_geo_x, sensor_9axis.f_geo_y, sensor_9axis.f_geo_z);
//		fprintf(fout, " [angle:%f %f %f]", sensor_9axis.angles[0], sensor_9axis.angles[1], sensor_9axis.angles[2]);
		fprintf(fout, "\n");

		in_count++;
		if (499 < in_count) {
			fclose(fout);
			in_count = 0;
			fout = NULL;
		}
	}
	return true;
}


void* MPU9250::ListenThread(void *pParam)
{
	MPU9250 *pSerialPort = reinterpret_cast<MPU9250*>(pParam);
	signed short s_data[9];
	char BytesInQue;

	while (!s_bExit)
	{
		if (pSerialPort->sensor_cfg.mag_enabled)
			BytesInQue = pSerialPort->getMotion9(s_data);
		else
			BytesInQue = pSerialPort->getMotion6(s_data);
		if (0 > BytesInQue)
		{
			sleep(SLEEP_TIME_INTERVAL);
			continue;
		}	
		sensor_get_raw(pSerialPort,s_data);
	}
	return ((void*)0);

}
/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
bool MPU9250::initialize() {
	if (i2c_open())
		return false;
	USB2IO_SetI2cSpeed(i2c_fp,1);//1=160k
	setClockSource(MPU9250_CLOCK_PLL_XGYRO);
	setAccelXSelfTest(false);
	setAccelYSelfTest(false);
	setAccelZSelfTest(false);
	setDLPFMode(MPU9250_DLPF_BW_3600);
	setFchoice(MPU9250_FCHOICE_DLPF);
	setFullScaleGyroRange(MPU9250_GYRO_FS_250);
	setAccelFchoice(MPU9250_ACCEL_NO_DLP);
	setFchoice(MPU9250_FCHOICE_NO_DLPF);
	setFullScaleAccelRange(MPU9250_ACCEL_FS_2);
	setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
	return true;
}

void MPU9250::close() {
	i2c_close();
}
/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU9250::testConnection() {
	return 0x71 == getDeviceID();
}

void MPU9250::set_Exit()
{
	s_bExit = true;
}

bool MPU9250::is_Exit()
{
	return s_bExit;
}
// SMPLRT_DIV register

/** Get gyroscope output rate divider.
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current sample rate
 * @see MPU9250_RA_SMPLRT_DIV
 */
unsigned char MPU9250::getRate() {
	USB2IO_I2cRead(i2c_fp,devAddr, MPU9250_RA_SMPLRT_DIV,1,1, (char *)buffer);
    return buffer[0];
}
/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU9250_RA_SMPLRT_DIV
 */
void MPU9250::setRate(unsigned char rate) {
	buffer[0] = rate;
	USB2IO_I2cWrite(i2c_fp,devAddr, MPU9250_RA_SMPLRT_DIV,1,1, (char *)buffer);
}

// CONFIG register

/** Get external FSYNC configuration.
 * Configures the external Frame Synchronization (FSYNC) pin sampling. An
 * external signal connected to the FSYNC pin can be sampled by configuring
 * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
 * strobes may be captured. The latched FSYNC signal will be sampled at the
 * Sampling Rate, as defined in register 25. After sampling, the latch will
 * reset to the current FSYNC signal state.
 *
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table.
 *
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 *
 * @return FSYNC configuration value
 */
unsigned char MPU9250::getExternalFrameSync() {
    i2c_readBits(devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LENGTH, buffer);
    return buffer[0];
}
/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU9250_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void MPU9250::setExternalFrameSync(unsigned char sync) {
    i2c_writeBits(devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LENGTH, sync);
}
/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see MPU9250_RA_CONFIG
 * @see MPU9250_CFG_DLPF_CFG_BIT
 * @see MPU9250_CFG_DLPF_CFG_LENGTH
 */
unsigned char MPU9250::getDLPFMode() {
    i2c_readBits(devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LENGTH, buffer);
    return buffer[0];
}
/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU9250_DLPF_BW_256
 * @see MPU9250_RA_CONFIG
 * @see MPU9250_CFG_DLPF_CFG_BIT
 * @see MPU9250_CFG_DLPF_CFG_LENGTH
 */
void MPU9250::setDLPFMode(unsigned char mode) {
    i2c_writeBits(devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU9250_GYRO_FS_250
 * @see MPU9250_RA_GYRO_CONFIG
 * @see MPU9250_GCONFIG_FS_SEL_BIT
 * @see MPU9250_GCONFIG_FS_SEL_LENGTH
 */
unsigned char MPU9250::getFullScaleGyroRange() {
    i2c_readBits(devAddr, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, buffer);
    return buffer[0];
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU9250_GYRO_FS_250
 * @see MPU9250_RA_GYRO_CONFIG
 * @see MPU9250_GCONFIG_FS_SEL_BIT
 * @see MPU9250_GCONFIG_FS_SEL_LENGTH
 */
void MPU9250::setFullScaleGyroRange(unsigned char range) {
    i2c_writeBits(devAddr, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU9250::setFchoice(unsigned char mode) {
	i2c_writeBits(devAddr, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FCHOICE_B_BIT, MPU9250_GCONFIG_FCHOICE_B_LENGTH, mode);
}
// ACCEL_CONFIG register

/** Get self-test enabled setting for accelerometer X axis.
 * @return Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
unsigned char MPU9250::getAccelXSelfTest() {
    i2c_readBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_XA_ST_BIT, buffer);
    return buffer[0];
}
/** Get self-test enabled setting for accelerometer X axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
void MPU9250::setAccelXSelfTest(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_XA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Y axis.
 * @return Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
unsigned char MPU9250::getAccelYSelfTest() {
    i2c_readBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_YA_ST_BIT, buffer);
    return buffer[0];
}
/** Get self-test enabled value for accelerometer Y axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
void MPU9250::setAccelYSelfTest(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_YA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Z axis.
 * @return Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
unsigned char MPU9250::getAccelZSelfTest() {
    i2c_readBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ZA_ST_BIT, buffer);
    return buffer[0];
}
/** Set self-test enabled value for accelerometer Z axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_RA_ACCEL_CONFIG
 */
void MPU9250::setAccelZSelfTest(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ZA_ST_BIT, enabled);
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU9250_ACCEL_FS_2
 * @see MPU9250_RA_ACCEL_CONFIG
 * @see MPU9250_ACONFIG_AFS_SEL_BIT
 * @see MPU9250_ACONFIG_AFS_SEL_LENGTH
 */
unsigned char MPU9250::getFullScaleAccelRange() {
    i2c_readBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, buffer);
    return buffer[0];
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void MPU9250::setFullScaleAccelRange(unsigned char range) {
    i2c_writeBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU9250::setAccelFchoice(unsigned char mode) {
	i2c_writeBits(devAddr, MPU9250_RA_ACCEL_CONFIG2, MPU9250_ACONFIG_ACC_FCHOICE_BIT, MPU9250_ACONFIG_ACC_FCHOICE_LENGTH, mode);
}

/** Get the high-pass filter configuration.
 * The DHPF is a filter module in the path leading to motion detectors (Free
 * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
 * available to the data registers (see Figure in Section 8 of the MPU-6000/
 * MPU-6050 Product Specification document).
 *
 * The high pass filter has three modes:
 *
 * <pre>
 *    Reset: The filter output settles to zero within one sample. This
 *           effectively disables the high pass filter. This mode may be toggled
 *           to quickly settle the filter.
 *
 *    On:    The high pass filter will pass signals above the cut off frequency.
 *
 *    Hold:  When triggered, the filter holds the present sample. The filter
 *           output will be the difference between the input sample and the held
 *           sample.
 * </pre>
 *
 * <pre>
 * ACCEL_HPF | Filter Mode | Cut-off Frequency
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 * </pre>
 *
 * @return Current high-pass filter configuration
 * @see MPU9250_DHPF_RESET
 * @see MPU9250_RA_ACCEL_CONFIG
 */
unsigned char MPU9250::getDHPFMode() {
    i2c_readBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ACCEL_HPF_BIT, MPU9250_ACONFIG_ACCEL_HPF_LENGTH, buffer);
    return buffer[0];
}
/** Set the high-pass filter configuration.
 * @param bandwidth New high-pass filter configuration
 * @see setDHPFMode()
 * @see MPU9250_DHPF_RESET
 * @see MPU9250_RA_ACCEL_CONFIG
 */
void MPU9250::setDHPFMode(unsigned char bandwidth) {
    i2c_writeBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ACCEL_HPF_BIT, MPU9250_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}

// FIFO_EN register

/** Get temperature FIFO enabled value.
 * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
 * 66) to be written into the FIFO buffer.
 * @return Current temperature FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250::getTempFIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_TEMP_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set temperature FIFO enabled value.
 * @param enabled New temperature FIFO enabled value
 * @see getTempFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250::setTempFIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_TEMP_FIFO_EN_BIT, enabled);
}
/** Get gyroscope X-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
 * 68) to be written into the FIFO buffer.
 * @return Current gyroscope X-axis FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250::getXGyroFIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_XG_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set gyroscope X-axis FIFO enabled value.
 * @param enabled New gyroscope X-axis FIFO enabled value
 * @see getXGyroFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250::setXGyroFIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_XG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Y-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
 * 70) to be written into the FIFO buffer.
 * @return Current gyroscope Y-axis FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250::getYGyroFIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_YG_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set gyroscope Y-axis FIFO enabled value.
 * @param enabled New gyroscope Y-axis FIFO enabled value
 * @see getYGyroFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250::setYGyroFIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_YG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Z-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
 * 72) to be written into the FIFO buffer.
 * @return Current gyroscope Z-axis FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250::getZGyroFIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_ZG_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set gyroscope Z-axis FIFO enabled value.
 * @param enabled New gyroscope Z-axis FIFO enabled value
 * @see getZGyroFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250::setZGyroFIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_ZG_FIFO_EN_BIT, enabled);
}
/** Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
 * written into the FIFO buffer.
 * @return Current accelerometer FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250::getAccelFIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_ACCEL_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250::setAccelFIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_ACCEL_FIFO_EN_BIT, enabled);
}
/** Get Slave 2 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 2 to be written into the FIFO buffer.
 * @return Current Slave 2 FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250::getSlave2FIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV2_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set Slave 2 FIFO enabled value.
 * @param enabled New Slave 2 FIFO enabled value
 * @see getSlave2FIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250::setSlave2FIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV2_FIFO_EN_BIT, enabled);
}
/** Get Slave 1 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 1 to be written into the FIFO buffer.
 * @return Current Slave 1 FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250::getSlave1FIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV1_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set Slave 1 FIFO enabled value.
 * @param enabled New Slave 1 FIFO enabled value
 * @see getSlave1FIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250::setSlave1FIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV1_FIFO_EN_BIT, enabled);
}
/** Get Slave 0 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 0 to be written into the FIFO buffer.
 * @return Current Slave 0 FIFO enabled value
 * @see MPU9250_RA_FIFO_EN
 */
bool MPU9250::getSlave0FIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV0_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set Slave 0 FIFO enabled value.
 * @param enabled New Slave 0 FIFO enabled value
 * @see getSlave0FIFOEnabled()
 * @see MPU9250_RA_FIFO_EN
 */
void MPU9250::setSlave0FIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_FIFO_EN, MPU9250_SLV0_FIFO_EN_BIT, enabled);
}

// I2C_MST_CTRL register

/** Get multi-master enabled value.
 * Multi-master capability allows multiple I2C masters to operate on the same
 * bus. In circuits where multi-master capability is required, set MULT_MST_EN
 * to 1. This will increase current drawn by approximately 30uA.
 *
 * In circuits where multi-master capability is required, the state of the I2C
 * bus must always be monitored by each separate I2C Master. Before an I2C
 * Master can assume arbitration of the bus, it must first confirm that no other
 * I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
 * MPU-60X0's bus arbitration detection logic is turned on, enabling it to
 * detect when the bus is available.
 *
 * @return Current multi-master enabled value
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250::getMultiMasterEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_MULT_MST_EN_BIT, buffer);
    return buffer[0];
}
/** Set multi-master enabled value.
 * @param enabled New multi-master enabled value
 * @see getMultiMasterEnabled()
 * @see MPU9250_RA_I2C_MST_CTRL
 */
void MPU9250::setMultiMasterEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_MULT_MST_EN_BIT, enabled);
}
/** Get wait-for-external-sensor-data enabled value.
 * When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
 * delayed until External Sensor data from the Slave Devices are loaded into the
 * EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
 * data (i.e. from gyro and accel) and external sensor data have been loaded to
 * their respective data registers (i.e. the data is synced) when the Data Ready
 * interrupt is triggered.
 *
 * @return Current wait-for-external-sensor-data enabled value
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250::getWaitForExternalSensorEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_WAIT_FOR_ES_BIT, buffer);
    return buffer[0];
}
/** Set wait-for-external-sensor-data enabled value.
 * @param enabled New wait-for-external-sensor-data enabled value
 * @see getWaitForExternalSensorEnabled()
 * @see MPU9250_RA_I2C_MST_CTRL
 */
void MPU9250::setWaitForExternalSensorEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_WAIT_FOR_ES_BIT, enabled);
}
/** Get Slave 3 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 3 to be written into the FIFO buffer.
 * @return Current Slave 3 FIFO enabled value
 * @see MPU9250_RA_MST_CTRL
 */
bool MPU9250::getSlave3FIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_SLV_3_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set Slave 3 FIFO enabled value.
 * @param enabled New Slave 3 FIFO enabled value
 * @see getSlave3FIFOEnabled()
 * @see MPU9250_RA_MST_CTRL
 */
void MPU9250::setSlave3FIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_SLV_3_FIFO_EN_BIT, enabled);
}
/** Get slave read/write transition enabled value.
 * The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave
 * read to the next slave read. If the bit equals 0, there will be a restart
 * between reads. If the bit equals 1, there will be a stop followed by a start
 * of the following read. When a write transaction follows a read transaction,
 * the stop followed by a start of the successive write will be always used.
 *
 * @return Current slave read/write transition enabled value
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250::getSlaveReadWriteTransitionEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_BIT, buffer);
    return buffer[0];
}
/** Set slave read/write transition enabled value.
 * @param enabled New slave read/write transition enabled value
 * @see getSlaveReadWriteTransitionEnabled()
 * @see MPU9250_RA_I2C_MST_CTRL
 */
void MPU9250::setSlaveReadWriteTransitionEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_BIT, enabled);
}
/** Get I2C master clock speed.
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 *
 * @return Current I2C master clock speed
 * @see MPU9250_RA_I2C_MST_CTRL
 */
unsigned char MPU9250::getMasterClockSpeed() {
    i2c_readBits(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CLK_LENGTH, buffer);
    return buffer[0];
}
/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU9250_RA_I2C_MST_CTRL
 */
void MPU9250::setMasterClockSpeed(unsigned char speed) {
    i2c_writeBits(devAddr, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_BIT, MPU9250_I2C_MST_CLK_LENGTH, speed);
}

// I2C_SLV* registers (Slave 0-3)

/** Get the I2C address of the specified slave (0-3).
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * In read mode, the result of the read is placed in the lowest available 
 * EXT_SENS_DATA register. For further information regarding the allocation of
 * read results, please refer to the EXT_SENS_DATA register description
 * (Registers 73 - 96).
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions (getSlave4* and setSlave4*).
 *
 * I2C data transactions are performed at the Sample Rate, as defined in
 * Register 25. The user is responsible for ensuring that I2C data transactions
 * to and from each enabled Slave can be completed within a single period of the
 * Sample Rate.
 *
 * The I2C slave access rate can be reduced relative to the Sample Rate. This
 * reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
 * slave's access rate is reduced relative to the Sample Rate is determined by
 * I2C_MST_DELAY_CTRL (Register 103).
 *
 * The processing order for the slaves is fixed. The sequence followed for
 * processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a
 * particular Slave is disabled it will be skipped.
 *
 * Each slave can either be accessed at the sample rate or at a reduced sample
 * rate. In a case where some slaves are accessed at the Sample Rate and some
 * slaves are accessed at the reduced rate, the sequence of accessing the slaves
 * (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will
 * be skipped if their access rate dictates that they should not be accessed
 * during that particular cycle. For further information regarding the reduced
 * access rate, please refer to Register 52. Whether a slave is accessed at the
 * Sample Rate or at the reduced rate is determined by the Delay Enable bits in
 * Register 103.
 *
 * @param num Slave number (0-3)
 * @return Current address for specified slave
 * @see MPU9250_RA_I2C_SLV0_ADDR
 */
unsigned char MPU9250::getSlaveAddress(unsigned char num) {
    if (num > 3) return 0;
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_I2C_SLV0_ADDR + num * 3, 1, 1, (char *)buffer);

    return buffer[0];
}
/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU9250_RA_I2C_SLV0_ADDR
 */
void MPU9250::setSlaveAddress(unsigned char num, unsigned char address) {
    if (num > 3) return;
	buffer[0] = (char)address;
	USB2IO_I2cWrite(i2c_fp,devAddr, MPU9250_RA_I2C_SLV0_ADDR + num * 3, 1,1,(char *)buffer);
}
/** Get the active internal register for the specified slave (0-3).
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions.
 *
 * @param num Slave number (0-3)
 * @return Current active register for specified slave
 * @see MPU9250_RA_I2C_SLV0_REG
 */
unsigned char MPU9250::getSlaveRegister(unsigned char num) {
    if (num > 3) return 0;
	USB2IO_I2cRead(i2c_fp,devAddr, MPU9250_RA_I2C_SLV0_REG + num * 3,1,1,(char *)buffer);
    return buffer[0];
}
/** Set the active internal register for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param reg New active register for specified slave
 * @see getSlaveRegister()
 * @see MPU9250_RA_I2C_SLV0_REG
 */
void MPU9250::setSlaveRegister(unsigned char num, unsigned char reg) {
    if (num > 3) return;
	buffer[0] = (char)reg;
	USB2IO_I2cWrite(i2c_fp,devAddr, MPU9250_RA_I2C_SLV0_REG + num * 3,1,1, (char *)buffer);
}
/** Get the enabled value for the specified slave (0-3).
 * When set to 1, this bit enables Slave 0 for data transfer operations. When
 * cleared to 0, this bit disables Slave 0 from data transfer operations.
 * @param num Slave number (0-3)
 * @return Current enabled value for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::getSlaveEnabled(unsigned char num) {
    if (num > 3) return 0;
    i2c_readBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_EN_BIT, buffer);
    return buffer[0];
}
/** Set the enabled value for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New enabled value for specified slave
 * @see getSlaveEnabled()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250::setSlaveEnabled(unsigned char num, bool enabled) {
    if (num > 3) return;
    i2c_writeBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_EN_BIT, enabled);
}
/** Get word pair byte-swapping enabled for the specified slave (0-3).
 * When set to 1, this bit enables byte swapping. When byte swapping is enabled,
 * the high and low bytes of a word pair are swapped. Please refer to
 * I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
 * bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
 * registers in the order they were transferred.
 *
 * @param num Slave number (0-3)
 * @return Current word pair byte-swapping enabled value for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::getSlaveWordByteSwap(unsigned char num) {
    if (num > 3) return 0;
    i2c_readBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_BYTE_SW_BIT, buffer);
    return buffer[0];
}
/** Set word pair byte-swapping enabled for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair byte-swapping enabled value for specified slave
 * @see getSlaveWordByteSwap()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250::setSlaveWordByteSwap(unsigned char num, bool enabled) {
    if (num > 3) return;
    i2c_writeBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_BYTE_SW_BIT, enabled);
}
/** Get write mode for the specified slave (0-3).
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @param num Slave number (0-3)
 * @return Current write mode for specified slave (0 = register address + data, 1 = data only)
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::getSlaveWriteMode(unsigned char num) {
    if (num > 3) return 0;
    i2c_readBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_REG_DIS_BIT, buffer);
    return buffer[0];
}
/** Set write mode for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param mode New write mode for specified slave (0 = register address + data, 1 = data only)
 * @see getSlaveWriteMode()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250::setSlaveWriteMode(unsigned char num, bool mode) {
    if (num > 3) return;
    i2c_writeBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_REG_DIS_BIT, mode);
}
/** Get word pair grouping order offset for the specified slave (0-3).
 * This sets specifies the grouping order of word pairs received from registers.
 * When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
 * then odd register addresses) are paired to form a word. When set to 1, bytes
 * from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
 * register addresses) are paired to form a word.
 *
 * @param num Slave number (0-3)
 * @return Current word pair grouping order offset for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::getSlaveWordGroupOffset(unsigned char num) {
    if (num > 3) return 0;
    i2c_readBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_GRP_BIT, buffer);
    return buffer[0];
}
/** Set word pair grouping order offset for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair grouping order offset for specified slave
 * @see getSlaveWordGroupOffset()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250::setSlaveWordGroupOffset(unsigned char num, bool enabled) {
    if (num > 3) return;
    i2c_writeBit(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_GRP_BIT, enabled);
}
/** Get number of bytes to read for the specified slave (0-3).
 * Specifies the number of bytes transferred to and from Slave 0. Clearing this
 * bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN.
 * @param num Slave number (0-3)
 * @return Number of bytes to read for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
unsigned char MPU9250::getSlaveDataLength(unsigned char num) {
    if (num > 3) return 0;
    i2c_readBits(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_LEN_BIT, MPU9250_I2C_SLV_LEN_LENGTH, buffer);
    return buffer[0];
}
/** Set number of bytes to read for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see getSlaveDataLength()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
void MPU9250::setSlaveDataLength(unsigned char num, unsigned char length) {
    if (num > 3) return;
    i2c_writeBits(devAddr, MPU9250_RA_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV_LEN_BIT, MPU9250_I2C_SLV_LEN_LENGTH, length);
}

// I2C_SLV* registers (Slave 4)

/** Get the I2C address of Slave 4.
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * @return Current address for Slave 4
 * @see getSlaveAddress()
 * @see MPU9250_RA_I2C_SLV4_ADDR
 */
unsigned char MPU9250::getSlave4Address() {
	USB2IO_I2cRead(i2c_fp,devAddr, MPU9250_RA_I2C_SLV4_ADDR,1,1, (char *)buffer);
    return buffer[0];
}
/** Set the I2C address of Slave 4.
 * @param address New address for Slave 4
 * @see getSlave4Address()
 * @see MPU9250_RA_I2C_SLV4_ADDR
 */
void MPU9250::setSlave4Address(unsigned char address) {
	buffer[0] = (char)address;
	USB2IO_I2cWrite(i2c_fp,devAddr, MPU9250_RA_I2C_SLV4_ADDR,1,1, (char *)buffer);
}
/** Get the active internal register for the Slave 4.
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * @return Current active register for Slave 4
 * @see MPU9250_RA_I2C_SLV4_REG
 */
unsigned char MPU9250::getSlave4Register() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_I2C_SLV4_REG,1,1, (char *)buffer);
    return buffer[0];
}
/** Set the active internal register for Slave 4.
 * @param reg New active register for Slave 4
 * @see getSlave4Register()
 * @see MPU9250_RA_I2C_SLV4_REG
 */
void MPU9250::setSlave4Register(unsigned char reg) {
	buffer[0] = (char)reg;
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_I2C_SLV4_REG,1,1, (char *)buffer);
}
/** Set new byte to write to Slave 4.
 * This register stores the data to be written into the Slave 4. If I2C_SLV4_RW
 * is set 1 (set to read), this register has no effect.
 * @param data New byte to write to Slave 4
 * @see MPU9250_RA_I2C_SLV4_DO
 */
void MPU9250::setSlave4OutputByte(unsigned char data) {
	buffer[0] = (char)data;
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_I2C_SLV4_DO,1,1, (char *)buffer);
}
/** Get the enabled value for the Slave 4.
 * When set to 1, this bit enables Slave 4 for data transfer operations. When
 * cleared to 0, this bit disables Slave 4 from data transfer operations.
 * @return Current enabled value for Slave 4
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250::getSlave4Enabled() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_EN_BIT, buffer);
    return buffer[0];
}
/** Set the enabled value for Slave 4.
 * @param enabled New enabled value for Slave 4
 * @see getSlave4Enabled()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
void MPU9250::setSlave4Enabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_EN_BIT, enabled);
}
/** Get the enabled value for Slave 4 transaction interrupts.
 * When set to 1, this bit enables the generation of an interrupt signal upon
 * completion of a Slave 4 transaction. When cleared to 0, this bit disables the
 * generation of an interrupt signal upon completion of a Slave 4 transaction.
 * The interrupt status can be observed in Register 54.
 *
 * @return Current enabled value for Slave 4 transaction interrupts.
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250::getSlave4InterruptEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_INT_EN_BIT, buffer);
    return buffer[0];
}
/** Set the enabled value for Slave 4 transaction interrupts.
 * @param enabled New enabled value for Slave 4 transaction interrupts.
 * @see getSlave4InterruptEnabled()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
void MPU9250::setSlave4InterruptEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_INT_EN_BIT, enabled);
}
/** Get write mode for Slave 4.
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @return Current write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250::getSlave4WriteMode() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_REG_DIS_BIT, buffer);
    return buffer[0];
}
/** Set write mode for the Slave 4.
 * @param mode New write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see getSlave4WriteMode()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
void MPU9250::setSlave4WriteMode(bool mode) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_REG_DIS_BIT, mode);
}
/** Get Slave 4 master delay value.
 * This configures the reduced access rate of I2C slaves relative to the Sample
 * Rate. When a slave's access rate is decreased relative to the Sample Rate,
 * the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and
 * DLPF_CFG (register 26). Whether a slave's access rate is reduced relative to
 * the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103). For
 * further information regarding the Sample Rate, please refer to register 25.
 *
 * @return Current Slave 4 master delay value
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
unsigned char MPU9250::getSlave4MasterDelay() {
    i2c_readBits(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_MST_DLY_BIT, MPU9250_I2C_SLV4_MST_DLY_LENGTH, buffer);
    return buffer[0];
}
/** Set Slave 4 master delay value.
 * @param delay New Slave 4 master delay value
 * @see getSlave4MasterDelay()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
void MPU9250::setSlave4MasterDelay(unsigned char delay) {
    i2c_writeBits(devAddr, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_MST_DLY_BIT, MPU9250_I2C_SLV4_MST_DLY_LENGTH, delay);
}
/** Get last available byte read from Slave 4.
 * This register stores the data read from Slave 4. This field is populated
 * after a read transaction.
 * @return Last available byte read from to Slave 4
 * @see MPU9250_RA_I2C_SLV4_DI
 */
unsigned char MPU9250::getSlate4InputByte() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_I2C_SLV4_DI,1,1, (char *)buffer);
    return buffer[0];
}

// I2C_MST_STATUS register

/** Get FSYNC interrupt status.
 * This bit reflects the status of the FSYNC interrupt from an external device
 * into the MPU-60X0. This is used as a way to pass an external interrupt
 * through the MPU-60X0 to the host application processor. When set to 1, this
 * bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
 * (Register 55).
 * @return FSYNC interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getPassthroughStatus() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_PASS_THROUGH_BIT, buffer);
    return buffer[0];
}
/** Get Slave 4 transaction done status.
 * Automatically sets to 1 when a Slave 4 transaction has completed. This
 * triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
 * (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
 * I2C_SLV4_CTRL register (Register 52).
 * @return Slave 4 transaction done status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave4IsDone() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV4_DONE_BIT, buffer);
    return buffer[0];
}
/** Get master arbitration lost status.
 * This bit automatically sets to 1 when the I2C Master has lost arbitration of
 * the auxiliary I2C bus (an error condition). This triggers an interrupt if the
 * I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Master arbitration lost status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getLostArbitration() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_LOST_ARB_BIT, buffer);
    return buffer[0];
}
/** Get Slave 4 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 4 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave4Nack() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV4_NACK_BIT, buffer);
    return buffer[0];
}
/** Get Slave 3 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 3 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave3Nack() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV3_NACK_BIT, buffer);
    return buffer[0];
}
/** Get Slave 2 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 2 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave2Nack() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV2_NACK_BIT, buffer);
    return buffer[0];
}
/** Get Slave 1 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 1 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave1Nack() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV1_NACK_BIT, buffer);
    return buffer[0];
}
/** Get Slave 0 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 0 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave0Nack() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_STATUS, MPU9250_MST_I2C_SLV0_NACK_BIT, buffer);
    return buffer[0];
}

// INT_PIN_CFG register

/** Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_LEVEL_BIT
 */
bool MPU9250::getInterruptMode() {
    i2c_readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_LEVEL_BIT, buffer);
    return buffer[0];
}
/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_LEVEL_BIT
 */
void MPU9250::setInterruptMode(bool mode) {
   i2c_writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_LEVEL_BIT, mode);
}
/** Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 * @return Current interrupt drive mode (0=push-pull, 1=open-drain)
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_OPEN_BIT
 */
bool MPU9250::getInterruptDrive() {
    i2c_readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_OPEN_BIT, buffer);
    return buffer[0];
}
/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_OPEN_BIT
 */
void MPU9250::setInterruptDrive(bool drive) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_OPEN_BIT, drive);
}
/** Get interrupt latch mode.
 * Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
 * @return Current latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_LATCH_INT_EN_BIT
 */
bool MPU9250::getInterruptLatch() {
    i2c_readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_LATCH_INT_EN_BIT, buffer);
    return buffer[0];
}
/** Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_LATCH_INT_EN_BIT
 */
void MPU9250::setInterruptLatch(bool latch) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_LATCH_INT_EN_BIT, latch);
}
/** Get interrupt latch clear mode.
 * Will be set 0 for status-read-only, 1 for any-register-read.
 * @return Current latch clear mode (0=status-read-only, 1=any-register-read)
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_RD_CLEAR_BIT
 */
bool MPU9250::getInterruptLatchClear() {
    i2c_readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_RD_CLEAR_BIT, buffer);
    return buffer[0];
}
/** Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_INT_RD_CLEAR_BIT
 */
void MPU9250::setInterruptLatchClear(bool clear) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_RD_CLEAR_BIT, clear);
}
/** Get FSYNC interrupt logic level mode.
 * @return Current FSYNC interrupt mode (0=active-high, 1=active-low)
 * @see getFSyncInterruptMode()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT
 */
bool MPU9250::getFSyncInterruptLevel() {
    i2c_readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT, buffer);
    return buffer[0];
}
/** Set FSYNC interrupt logic level mode.
 * @param mode New FSYNC interrupt mode (0=active-high, 1=active-low)
 * @see getFSyncInterruptMode()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT
 */
void MPU9250::setFSyncInterruptLevel(bool level) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
/** Get FSYNC pin interrupt enabled setting.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled setting
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_FSYNC_INT_EN_BIT
 */
bool MPU9250::getFSyncInterruptEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_EN_BIT, buffer);
    return buffer[0];
}
/** Set FSYNC pin interrupt enabled setting.
 * @param enabled New FSYNC pin interrupt enabled setting
 * @see getFSyncInterruptEnabled()
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_FSYNC_INT_EN_BIT
 */
void MPU9250::setFSyncInterruptEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_FSYNC_INT_EN_BIT, enabled);
}
/** Get I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @return Current I2C bypass enabled status
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_I2C_BYPASS_EN_BIT
 */
bool MPU9250::getI2CBypassEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, buffer);
    return buffer[0];
}
/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_I2C_BYPASS_EN_BIT
 */
void MPU9250::setI2CBypassEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/** Get reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
 * further information regarding CLKOUT, please refer to the MPU-60X0 Product
 * Specification document.
 * @return Current reference clock output enabled status
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_CLKOUT_EN_BIT
 */
bool MPU9250::getClockOutputEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_CLKOUT_EN_BIT, buffer);
    return buffer[0];
}
/** Set reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
 * further information regarding CLKOUT, please refer to the MPU-60X0 Product
 * Specification document.
 * @param enabled New reference clock output enabled status
 * @see MPU9250_RA_INT_PIN_CFG
 * @see MPU9250_INTCFG_CLKOUT_EN_BIT
 */
void MPU9250::setClockOutputEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_CLKOUT_EN_BIT, enabled);
}

// INT_ENABLE register

/** Get full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit will be
 * set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
unsigned char MPU9250::getIntEnabled() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_INT_ENABLE,1,1, (char *)buffer);
    return buffer[0];
}
/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
void MPU9250::setIntEnabled(unsigned char enabled) {
	buffer[0] = (char)enabled;
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_INT_ENABLE,1,1, (char *)buffer);
}
/** Get Free Fall interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
bool MPU9250::getIntFreefallEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FF_BIT, buffer);
    return buffer[0];
}
/** Set Free Fall interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
void MPU9250::setIntFreefallEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FF_BIT, enabled);
}
/** Get Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_MOT_BIT
 **/
bool MPU9250::getIntMotionEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_MOT_BIT, buffer);
    return buffer[0];
}
/** Set Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntMotionEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_MOT_BIT
 **/
void MPU9250::setIntMotionEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_MOT_BIT, enabled);
}
/** Get Zero Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 **/
bool MPU9250::getIntZeroMotionEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_ZMOT_BIT, buffer);
    return buffer[0];
}
/** Set Zero Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntZeroMotionEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 **/
void MPU9250::setIntZeroMotionEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_ZMOT_BIT, enabled);
}
/** Get FIFO Buffer Overflow interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
 **/
bool MPU9250::getIntFIFOBufferOverflowEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    return buffer[0];
}
/** Set FIFO Buffer Overflow interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFIFOBufferOverflowEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
 **/
void MPU9250::setIntFIFOBufferOverflowEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
/** Get I2C Master interrupt enabled status.
 * This enables any of the I2C Master interrupt sources to generate an
 * interrupt. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
 **/
bool MPU9250::getIntI2CMasterEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_I2C_MST_INT_BIT, buffer);
    return buffer[0];
}
/** Set I2C Master interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntI2CMasterEnabled()
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
 **/
void MPU9250::setIntI2CMasterEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_I2C_MST_INT_BIT, enabled);
}
/** Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_RA_INT_ENABLE
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
bool MPU9250::getIntDataReadyEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, buffer);
    return buffer[0];
}
/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU9250_RA_INT_CFG
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
void MPU9250::setIntDataReadyEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, enabled);
}

// INT_STATUS register

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 */
unsigned char MPU9250::getIntStatus() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_INT_STATUS,1,1, (char *)buffer);
    return buffer[0];
}
/** Get Free Fall interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_FF_BIT
 */
bool MPU9250::getIntFreefallStatus() {
    i2c_readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_FF_BIT, buffer);
    return buffer[0];
}
/** Get Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Motion Detection interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_MOT_BIT
 */
bool MPU9250::getIntMotionStatus() {
    i2c_readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_MOT_BIT, buffer);
    return buffer[0];
}
/** Get Zero Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Zero Motion Detection interrupt has
 * been generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 */
bool MPU9250::getIntZeroMotionStatus() {
    i2c_readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_ZMOT_BIT, buffer);
    return buffer[0];
}
/** Get FIFO Buffer Overflow interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
 */
bool MPU9250::getIntFIFOBufferOverflowStatus() {
    i2c_readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    return buffer[0];
}
/** Get I2C Master interrupt status.
 * This bit automatically sets to 1 when an I2C Master interrupt has been
 * generated. For a list of I2C Master interrupts, please refer to Register 54.
 * The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
 */
bool MPU9250::getIntI2CMasterStatus() {
    i2c_readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_I2C_MST_INT_BIT, buffer);
    return buffer[0];
}
/** Get Data Ready interrupt status.
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
bool MPU9250::getIntDataReadyStatus() {
    i2c_readBit(devAddr, MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_DATA_RDY_BIT, buffer);
    return buffer[0];
}

// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 16-bit signed integer container for magnetometer X-axis value
 * @param my 16-bit signed integer container for magnetometer Y-axis value
 * @param mz 16-bit signed integer container for magnetometer Z-axis value
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
void MPU9250::getMotion9(signed short* ax, signed short* ay, signed short* az, signed short* gx, signed short* gy, signed short* gz, signed short* mx, signed short* my, signed short* mz) {
    
	//get accel and gyro
	getMotion6(ax, ay, az, gx, gy, gz);
	
	//read mag
	buffer[0] = 0x02;
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_INT_PIN_CFG,1,1, (char *)buffer); //set i2c bypass enable pin to true to access magnetometer
	sleep(10);
	buffer[0] = 0x01;
	USB2IO_I2cWrite(i2c_fp, MPU9250_RA_MAG_ADDRESS, 0x0A, 1,1,(char *)buffer); //enable the magnetometer
	sleep(10);
	USB2IO_I2cRead(i2c_fp, MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_XOUT_L, 1,6, (char *)buffer);
	*mx = (((signed short)buffer[0]) << 8) | buffer[1];
    *my = (((signed short)buffer[2]) << 8) | buffer[3];
    *mz = (((signed short)buffer[4]) << 8) | buffer[5];		
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
char MPU9250::getMotion6(signed short* ax, signed short* ay, signed short* az, signed short* gx, signed short* gy, signed short* gz)
{
	char read_len = USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_ACCEL_XOUT_H, 1, 14, (char *)buffer);
    
	if (0 > read_len)
		return read_len;
    *ax = (((signed short)buffer[0]) << 8) | buffer[1];
    *ay = (((signed short)buffer[2]) << 8) | buffer[3];
    *az = (((signed short)buffer[4]) << 8) | buffer[5];
    *gx = (((signed short)buffer[8]) << 8) | buffer[9];
    *gy = (((signed short)buffer[10]) << 8) | buffer[11];
    *gz = (((signed short)buffer[12]) << 8) | buffer[13];
	return read_len;
}
char MPU9250::getMotion6(signed short* all_data)
{
	char read_len;

	if (sensor_cfg.only_test_cap) {
		read_len = 14;
		for (int i = 0; i < read_len; i++)
			buffer[i] = 0;
	} else
		read_len = USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_ACCEL_XOUT_H, 1, 14, (char *)buffer);
	
	all_data[0] = (((signed short)buffer[0]) << 8) | buffer[1];
	all_data[1] = (((signed short)buffer[2]) << 8) | buffer[3];
	all_data[2] = (((signed short)buffer[4]) << 8) | buffer[5];
	all_data[3] = (((signed short)buffer[8]) << 8) | buffer[9];
	all_data[4] = (((signed short)buffer[10]) << 8) | buffer[11];
	all_data[5] = (((signed short)buffer[12]) << 8) | buffer[13];
	return read_len;
}

char MPU9250::enable_mag()
{
	//read mag
	buffer[0] = 0x02;
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_INT_PIN_CFG, 1, 1, (char *)buffer); //set i2c bypass enable pin to true to access magnetometer
	sleep(10);
	buffer[0] = 0x02/*continuous mode1*/ | 0x10/*16bits out,0x0=14bits out*/;
	USB2IO_I2cWrite(i2c_fp, MPU9250_RA_MAG_ADDRESS, 0x0A, 1, 1, (char *)buffer); //enable the magnetometer
	return 0;
}

char MPU9250::disable_mag()
{
	return 0;
}

char MPU9250::getMotion9(signed short* all_data)
{

	//get accel and gyro
	getMotion6(all_data);

	USB2IO_I2cRead(i2c_fp, MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_XOUT_L, 1, 6, (char *)buffer);
	all_data[6] = (((signed short)buffer[0]) << 8) | buffer[1];
	all_data[7] = (((signed short)buffer[2]) << 8) | buffer[3];
	all_data[8] = (((signed short)buffer[4]) << 8) | buffer[5];
	return 0;
}
/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU9250_RA_GYRO_XOUT_H
 */
void MPU9250::getAcceleration(signed short* x, signed short* y, signed short* z) {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_ACCEL_XOUT_H, 1,6, (char *)buffer);
    *x = (((signed short)buffer[0]) << 8) | buffer[1];
    *y = (((signed short)buffer[2]) << 8) | buffer[3];
    *z = (((signed short)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
signed short MPU9250::getAccelerationX() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_ACCEL_XOUT_H,1, 2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_YOUT_H
 */
signed short MPU9250::getAccelerationY() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_ACCEL_YOUT_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_ACCEL_ZOUT_H
 */
signed short MPU9250::getAccelerationZ() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_ACCEL_ZOUT_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}

// TEMP_OUT_* registers

/** Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see MPU9250_RA_TEMP_OUT_H
 */
signed short MPU9250::getTemperature() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_TEMP_OUT_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_XOUT_H
 */
void MPU9250::getRotation(signed short* x, signed short* y, signed short* z) {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_GYRO_XOUT_H, 1,6, (char *)buffer);
    *x = (((signed short)buffer[0]) << 8) | buffer[1];
    *y = (((signed short)buffer[2]) << 8) | buffer[3];
    *z = (((signed short)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_XOUT_H
 */
signed short MPU9250::getRotationX() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_GYRO_XOUT_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_YOUT_H
 */
signed short MPU9250::getRotationY() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_GYRO_YOUT_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_ZOUT_H
 */
signed short MPU9250::getRotationZ() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_GYRO_ZOUT_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}

// EXT_SENS_DATA_* registers

/** Read single byte from external sensor data register.
 * These registers store data read from external sensors by the Slave 0, 1, 2,
 * and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
 * I2C_SLV4_DI (Register 53).
 *
 * External sensor data is written to these registers at the Sample Rate as
 * defined in Register 25. This access rate can be reduced by using the Slave
 * Delay Enable registers (Register 103).
 *
 * External sensor data registers, along with the gyroscope measurement
 * registers, accelerometer measurement registers, and temperature measurement
 * registers, are composed of two sets of registers: an internal register set
 * and a user-facing read register set.
 *
 * The data within the external sensors' internal register set is always updated
 * at the Sample Rate (or the reduced access rate) whenever the serial interface
 * is idle. This guarantees that a burst read of sensor registers will read
 * measurements from the same sampling instant. Note that if burst reads are not
 * used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Data is placed in these external sensor data registers according to
 * I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39,
 * 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from
 * an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as
 * defined in Register 25) or delayed rate (if specified in Register 52 and
 * 103). During each Sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the
 * order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * Each enabled slave will have EXT_SENS_DATA registers associated with it by
 * number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from
 * EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may
 * change the higher numbered slaves' associated registers. Furthermore, if
 * fewer total bytes are being read from the external sensors as a result of
 * such a change, then the data remaining in the registers which no longer have
 * an associated slave device (i.e. high numbered registers) will remain in
 * these previously allocated registers unless reset.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of
 * available EXT_SENS_DATA registers, the excess bytes will be dropped. There
 * are 24 EXT_SENS_DATA registers and hence the total read lengths between all
 * the slaves cannot be greater than 24 or some bytes will be lost.
 *
 * Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further
 * information regarding the characteristics of Slave 4, please refer to
 * Registers 49 to 53.
 *
 * EXAMPLE:
 * Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and
 * I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that
 * I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00
 * through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05
 * will be associated with Slave 1. If Slave 2 is enabled as well, registers
 * starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
 *
 * If Slave 2 is disabled while Slave 3 is enabled in this same situation, then
 * registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3
 * instead.
 *
 * REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE:
 * If a slave is disabled at any time, the space initially allocated to the
 * slave in the EXT_SENS_DATA register, will remain associated with that slave.
 * This is to avoid dynamic adjustment of the register allocation.
 *
 * The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all
 * slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).
 *
 * This above is also true if one of the slaves gets NACKed and stops
 * functioning.
 *
 * @param position Starting position (0-23)
 * @return Byte read from register
 */
unsigned char MPU9250::getExternalSensorByte(int position) {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_EXT_SENS_DATA_00 + position,1,1, (char *)buffer);
    return buffer[0];
}
/** Read word (2 bytes) from external sensor data registers.
 * @param position Starting position (0-21)
 * @return Word read from register
 * @see getExternalSensorByte()
 */
unsigned short MPU9250::getExternalSensorWord(int position) {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_EXT_SENS_DATA_00 + position, 1,2, (char *)buffer);
    return (((unsigned short)buffer[0]) << 8) | buffer[1];
}
/** Read double word (4 bytes) from external sensor data registers.
 * @param position Starting position (0-20)
 * @return Double word read from registers
 * @see getExternalSensorByte()
 */
int MPU9250::getExternalSensorDWord(int position) {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_EXT_SENS_DATA_00 + position, 1,4, (char *)buffer);
    return (((unsigned int)buffer[0]) << 24) | (((unsigned int)buffer[1]) << 16) | (((unsigned short)buffer[2]) << 8) | buffer[3];
}

// MOT_DETECT_STATUS register

/** Get X-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_XNEG_BIT
 */
bool MPU9250::getXNegMotionDetected() {
    i2c_readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_XNEG_BIT, buffer);
    return buffer[0];
}
/** Get X-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_XPOS_BIT
 */
bool MPU9250::getXPosMotionDetected() {
    i2c_readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_XPOS_BIT, buffer);
    return buffer[0];
}
/** Get Y-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_YNEG_BIT
 */
bool MPU9250::getYNegMotionDetected() {
    i2c_readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_YNEG_BIT, buffer);
    return buffer[0];
}
/** Get Y-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_YPOS_BIT
 */
bool MPU9250::getYPosMotionDetected() {
    i2c_readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_YPOS_BIT, buffer);
    return buffer[0];
}
/** Get Z-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZNEG_BIT
 */
bool MPU9250::getZNegMotionDetected() {
    i2c_readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZNEG_BIT, buffer);
    return buffer[0];
}
/** Get Z-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZPOS_BIT
 */
bool MPU9250::getZPosMotionDetected() {
    i2c_readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZPOS_BIT, buffer);
    return buffer[0];
}
/** Get zero motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZRMOT_BIT
 */
bool MPU9250::getZeroMotionDetected() {
    i2c_readBit(devAddr, MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZRMOT_BIT, buffer);
    return buffer[0];
}

// I2C_SLV*_DO register

/** Write byte to Data Output container for specified slave.
 * This register holds the output data written into Slave when Slave is set to
 * write mode. For further information regarding Slave control, please
 * refer to Registers 37 to 39 and immediately following.
 * @param num Slave number (0-3)
 * @param data Byte to write
 * @see MPU9250_RA_I2C_SLV0_DO
 */
void MPU9250::setSlaveOutputByte(unsigned char num, unsigned char data) {
    if (num > 3) return;
	buffer[0] = (char)data;
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_I2C_SLV0_DO + num,1,1, (char *)buffer);
}

// I2C_MST_DELAY_CTRL register

/** Get external data shadow delay enabled status.
 * This register is used to specify the timing of external sensor data
 * shadowing. When DELAY_ES_SHADOW is set to 1, shadowing of external
 * sensor data is delayed until all data has been received.
 * @return Current external data shadow delay enabled status.
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT
 */
bool MPU9250::getExternalShadowDelayEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_DELAY_CTRL, MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT, buffer);
    return buffer[0];
}
/** Set external data shadow delay enabled status.
 * @param enabled New external data shadow delay enabled status.
 * @see getExternalShadowDelayEnabled()
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT
 */
void MPU9250::setExternalShadowDelayEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_MST_DELAY_CTRL, MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}
/** Get slave delay enabled status.
 * When a particular slave delay is enabled, the rate of access for the that
 * slave device is reduced. When a slave's access rate is decreased relative to
 * the Sample Rate, the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) Samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register  * 25)
 * and DLPF_CFG (register 26).
 *
 * For further information regarding I2C_MST_DLY, please refer to register 52.
 * For further information regarding the Sample Rate, please refer to register 25.
 *
 * @param num Slave number (0-4)
 * @return Current slave delay enabled status.
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
bool MPU9250::getSlaveDelayEnabled(unsigned char num) {
    // MPU9250_DELAYCTRL_I2C_SLV4_DLY_EN_BIT is 4, SLV3 is 3, etc.
    if (num > 4) return 0;
    i2c_readBit(devAddr, MPU9250_RA_I2C_MST_DELAY_CTRL, num, buffer);
    return buffer[0];
}
/** Set slave delay enabled status.
 * @param num Slave number (0-4)
 * @param enabled New slave delay enabled status.
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
void MPU9250::setSlaveDelayEnabled(unsigned char num, bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

// SIGNAL_PATH_RESET register

/** Reset gyroscope signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_RA_SIGNAL_PATH_RESET
 * @see MPU9250_PATHRESET_GYRO_RESET_BIT
 */
void MPU9250::resetGyroscopePath() {
    i2c_writeBit(devAddr, MPU9250_RA_SIGNAL_PATH_RESET, MPU9250_PATHRESET_GYRO_RESET_BIT, true);
}
/** Reset accelerometer signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_RA_SIGNAL_PATH_RESET
 * @see MPU9250_PATHRESET_ACCEL_RESET_BIT
 */
void MPU9250::resetAccelerometerPath() {
    i2c_writeBit(devAddr, MPU9250_RA_SIGNAL_PATH_RESET, MPU9250_PATHRESET_ACCEL_RESET_BIT, true);
}
/** Reset temperature sensor signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_RA_SIGNAL_PATH_RESET
 * @see MPU9250_PATHRESET_TEMP_RESET_BIT
 */
void MPU9250::resetTemperaturePath() {
    i2c_writeBit(devAddr, MPU9250_RA_SIGNAL_PATH_RESET, MPU9250_PATHRESET_TEMP_RESET_BIT, true);
}

// MOT_DETECT_CTRL register

/** Get accelerometer power-on delay.
 * The accelerometer data path provides samples to the sensor registers, Motion
 * detection, Zero Motion detection, and Free Fall detection modules. The
 * signal path contains filters which must be flushed on wake-up with new
 * samples before the detection modules begin operations. The default wake-up
 * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
 * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
 * any value above zero unless instructed otherwise by InvenSense. Please refer
 * to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
 * further information regarding the detection modules.
 * @return Current accelerometer power-on delay
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_ACCEL_ON_DELAY_BIT
 */
unsigned char MPU9250::getAccelerometerPowerOnDelay() {
    i2c_readBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_ACCEL_ON_DELAY_BIT, MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
    return buffer[0];
}
/** Set accelerometer power-on delay.
 * @param delay New accelerometer power-on delay (0-3)
 * @see getAccelerometerPowerOnDelay()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_ACCEL_ON_DELAY_BIT
 */
void MPU9250::setAccelerometerPowerOnDelay(unsigned char delay) {
    i2c_writeBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_ACCEL_ON_DELAY_BIT, MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}
/** Get Free Fall detection counter decrement configuration.
 * Detection is registered by the Free Fall detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring FF_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * FF_COUNT | Counter Decrement
 * ---------+------------------
 * 0        | Reset
 * 1        | 1
 * 2        | 2
 * 3        | 4
 * </pre>
 *
 * When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Free Fall detection,
 * please refer to Registers 29 to 32.
 *
 * @return Current decrement configuration
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_FF_COUNT_BIT
 */
unsigned char MPU9250::getFreefallDetectionCounterDecrement() {
    i2c_readBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_FF_COUNT_BIT, MPU9250_DETECT_FF_COUNT_LENGTH, buffer);
    return buffer[0];
}
/** Set Free Fall detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getFreefallDetectionCounterDecrement()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_FF_COUNT_BIT
 */
void MPU9250::setFreefallDetectionCounterDecrement(unsigned char decrement) {
    i2c_writeBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_FF_COUNT_BIT, MPU9250_DETECT_FF_COUNT_LENGTH, decrement);
}
/** Get Motion detection counter decrement configuration.
 * Detection is registered by the Motion detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring MOT_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * MOT_COUNT | Counter Decrement
 * ----------+------------------
 * 0         | Reset
 * 1         | 1
 * 2         | 2
 * 3         | 4
 * </pre>
 *
 * When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Motion detection,
 * please refer to Registers 29 to 32.
 *
 */
unsigned char MPU9250::getMotionDetectionCounterDecrement() {
    i2c_readBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, buffer);
    return buffer[0];
}
/** Set Motion detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getMotionDetectionCounterDecrement()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_MOT_COUNT_BIT
 */
void MPU9250::setMotionDetectionCounterDecrement(unsigned char decrement) {
    i2c_writeBits(devAddr, MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, decrement);
}

// USER_CTRL register

/** Get FIFO enabled status.
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 * @return Current FIFO enabled status
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_FIFO_EN_BIT
 */
bool MPU9250::getFIFOEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_FIFO_EN_BIT
 */
void MPU9250::setFIFOEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_EN_BIT, enabled);
}
/** Get I2C Master Mode enabled status.
 * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
 * external sensor slave devices on the auxiliary I2C bus. When this bit is
 * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
 * driven by the primary I2C bus (SDA and SCL). This is a precondition to
 * enabling Bypass Mode. For further information regarding Bypass Mode, please
 * refer to Register 55.
 * @return Current I2C Master Mode enabled status
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_EN_BIT
 */
bool MPU9250::getI2CMasterModeEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, buffer);
    return buffer[0];
}
/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_EN_BIT
 */
void MPU9250::setI2CMasterModeEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, enabled);
}
/** Switch from I2C to SPI mode (MPU-6000 only)
 * If this is set, the primary SPI interface will be enabled in place of the
 * disabled primary I2C interface.
 */
void MPU9250::switchSPIEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_IF_DIS_BIT, enabled);
}
/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_FIFO_RESET_BIT
 */
void MPU9250::resetFIFO() {
    i2c_writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_FIFO_RESET_BIT, true);
}
/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_RESET_BIT
 */
void MPU9250::resetI2CMaster() {
    i2c_writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_RESET_BIT, true);
}
/** Reset all sensor registers and signal paths.
 * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
 * accelerometers, and temperature sensor). This operation will also clear the
 * sensor registers. This bit automatically clears to 0 after the reset has been
 * triggered.
 *
 * When resetting only the signal path (and not the sensor registers), please
 * use Register 104, SIGNAL_PATH_RESET.
 *
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_SIG_COND_RESET_BIT
 */
void MPU9250::resetSensors() {
    i2c_writeBit(devAddr, MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_SIG_COND_RESET_BIT, true);
}

// PWR_MGMT_1 register

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_DEVICE_RESET_BIT
 */
void MPU9250::reset() {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT, true);
}
/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_SLEEP_BIT
 */
bool MPU9250::getSleepEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, buffer);
    return buffer[0];
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_SLEEP_BIT
 */
void MPU9250::setSleepEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, enabled);
}
/** Get wake cycle enabled status.
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 * @return Current sleep mode enabled status
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CYCLE_BIT
 */
bool MPU9250::getWakeCycleEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CYCLE_BIT, buffer);
    return buffer[0];
}
/** Set wake cycle enabled status.
 * @param enabled New sleep mode enabled status
 * @see getWakeCycleEnabled()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CYCLE_BIT
 */
void MPU9250::setWakeCycleEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CYCLE_BIT, enabled);
}
/** Get temperature sensor enabled status.
 * Control the usage of the internal temperature sensor.
 *
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @return Current temperature sensor enabled status
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_TEMP_DIS_BIT
 */
bool MPU9250::getTempSensorEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_TEMP_DIS_BIT, buffer);
    return buffer[0] == 0; // 1 is actually disabled here
}
/** Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param enabled New temperature sensor enabled status
 * @see getTempSensorEnabled()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_TEMP_DIS_BIT
 */
void MPU9250::setTempSensorEnabled(bool enabled) {
    // 1 is actually disabled here
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_TEMP_DIS_BIT, !enabled);
}
/** Get clock source setting.
 * @return Current clock source setting
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CLKSEL_BIT
 * @see MPU9250_PWR1_CLKSEL_LENGTH
 */
unsigned char MPU9250::getClockSource() {
    i2c_readBits(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, buffer);
    return buffer[0];
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CLKSEL_BIT
 * @see MPU9250_PWR1_CLKSEL_LENGTH
 */
void MPU9250::setClockSource(unsigned char source) {
    i2c_writeBits(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, source);
}

// PWR_MGMT_2 register

/** Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * <pre>
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 5 Hz
 * 3            | 10 Hz
 * <pre>
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *
 * @return Current wake frequency
 * @see MPU9250_RA_PWR_MGMT_2
 */
unsigned char MPU9250::getWakeFrequency() {
    i2c_readBits(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_LP_WAKE_CTRL_BIT, MPU9250_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
    return buffer[0];
}
/** Set wake frequency in Accel-Only Low Power Mode.
 * @param frequency New wake frequency
 * @see MPU9250_RA_PWR_MGMT_2
 */
void MPU9250::setWakeFrequency(unsigned char frequency) {
    i2c_writeBits(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_LP_WAKE_CTRL_BIT, MPU9250_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}

/** Get X-axis accelerometer standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XA_BIT
 */
bool MPU9250::getStandbyXAccelEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XA_BIT, buffer);
    return buffer[0];
}
/** Set X-axis accelerometer standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XA_BIT
 */
void MPU9250::setStandbyXAccelEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XA_BIT, enabled);
}
/** Get Y-axis accelerometer standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YA_BIT
 */
bool MPU9250::getStandbyYAccelEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YA_BIT, buffer);
    return buffer[0];
}
/** Set Y-axis accelerometer standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YA_BIT
 */
void MPU9250::setStandbyYAccelEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YA_BIT, enabled);
}
/** Get Z-axis accelerometer standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZA_BIT
 */
bool MPU9250::getStandbyZAccelEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZA_BIT, buffer);
    return buffer[0];
}
/** Set Z-axis accelerometer standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZA_BIT
 */
void MPU9250::setStandbyZAccelEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZA_BIT, enabled);
}
/** Get X-axis gyroscope standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XG_BIT
 */
bool MPU9250::getStandbyXGyroEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XG_BIT, buffer);
    return buffer[0];
}
/** Set X-axis gyroscope standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XG_BIT
 */
void MPU9250::setStandbyXGyroEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XG_BIT, enabled);
}
/** Get Y-axis gyroscope standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YG_BIT
 */
bool MPU9250::getStandbyYGyroEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YG_BIT, buffer);
    return buffer[0];
}
/** Set Y-axis gyroscope standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YG_BIT
 */
void MPU9250::setStandbyYGyroEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YG_BIT, enabled);
}
/** Get Z-axis gyroscope standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZG_BIT
 */
bool MPU9250::getStandbyZGyroEnabled() {
    i2c_readBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZG_BIT, buffer);
    return buffer[0];
}
/** Set Z-axis gyroscope standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZG_BIT
 */
void MPU9250::setStandbyZGyroEnabled(bool enabled) {
    i2c_writeBit(devAddr, MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZG_BIT, enabled);
}

// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
unsigned short MPU9250::getFIFOCount() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_FIFO_COUNTH, 1,2, (char *)buffer);
    return (((unsigned short)buffer[0]) << 8) | buffer[1];
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
unsigned char MPU9250::getFIFOByte() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_FIFO_R_W, 1,1,(char *)buffer);
    return buffer[0];
}
void MPU9250::getFIFOBytes(unsigned char *data, unsigned char length) {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_FIFO_R_W, 1,length, (char *)data);
}
/** Write byte to FIFO buffer.
 * @see getFIFOByte()
 * @see MPU9250_RA_FIFO_R_W
 */
void MPU9250::setFIFOByte(unsigned char data) {
	buffer[0] = (char)data;
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_FIFO_R_W,1,1,(char *)buffer);
}

// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0x71).
 * @return Device ID ( should be 0x71)
 * @see MPU9250_RA_WHO_AM_I
 * @see MPU9250_WHO_AM_I_BIT
 * @see MPU9250_WHO_AM_I_LENGTH
 */
unsigned char MPU9250::getDeviceID() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_WHO_AM_I, 1, 1, (char *)buffer);
    return buffer[0];
}
// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XA_OFFS_* registers

signed short MPU9250::getXAccelOffset() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_XA_OFFS_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setXAccelOffset(signed short offset) {
	buffer[0] = (unsigned char)((offset >> 8) && 0xFF);
	buffer[1] = (unsigned char)(offset && 0xFF);
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_XA_OFFS_H, 1,2, (char *)buffer);
}

// YA_OFFS_* register

signed short MPU9250::getYAccelOffset() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_YA_OFFS_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setYAccelOffset(signed short offset) {
	buffer[0] = (unsigned char)((offset >> 8) && 0xFF);
	buffer[1] = (unsigned char)(offset && 0xFF);
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_YA_OFFS_H, 1,2, (char *)buffer);
}

// ZA_OFFS_* register

signed short MPU9250::getZAccelOffset() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_ZA_OFFS_H, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setZAccelOffset(signed short offset) {
	buffer[0] = (unsigned char)((offset >> 8) && 0xFF);
	buffer[1] = (unsigned char)(offset && 0xFF);
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_ZA_OFFS_H, 1,2, (char *)buffer);
}

// XG_OFFS_USR* registers

signed short MPU9250::getXGyroOffsetUser() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_XG_OFFS_USRH, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setXGyroOffsetUser(signed short offset) {
	buffer[0] = (unsigned char)((offset >> 8) && 0xFF);
	buffer[1] = (unsigned char)(offset && 0xFF);
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_XG_OFFS_USRH, 1,2, (char *)buffer);
}

// YG_OFFS_USR* register

signed short MPU9250::getYGyroOffsetUser() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_YG_OFFS_USRH, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setYGyroOffsetUser(signed short offset) {
	buffer[0] = (unsigned char)((offset >> 8) && 0xFF);
	buffer[1] = (unsigned char)(offset && 0xFF);
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_YG_OFFS_USRH, 1,2, (char *)buffer);
}

// ZG_OFFS_USR* register

signed short MPU9250::getZGyroOffsetUser() {
	USB2IO_I2cRead(i2c_fp, devAddr, MPU9250_RA_ZG_OFFS_USRH, 1,2, (char *)buffer);
    return (((signed short)buffer[0]) << 8) | buffer[1];
}
void MPU9250::setZGyroOffsetUser(signed short offset) {
	buffer[0] = (unsigned char)((offset >> 8) && 0xFF);
	buffer[1] = (unsigned char)(offset && 0xFF);
	USB2IO_I2cWrite(i2c_fp, devAddr, MPU9250_RA_ZG_OFFS_USRH, 1,2, (char *)buffer);
}
