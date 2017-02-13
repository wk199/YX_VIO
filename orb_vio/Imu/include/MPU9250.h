// I2Cdev library collection - MPU9250 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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

#ifndef _MPU9250_H_
#define _MPU9250_H_

//Magnetometer Registers
#define MPU9250_RA_MAG_ADDRESS		0x0C
#define MPU9250_RA_MAG_XOUT_L		0x03
#define MPU9250_RA_MAG_XOUT_H		0x04
#define MPU9250_RA_MAG_YOUT_L		0x05
#define MPU9250_RA_MAG_YOUT_H		0x06
#define MPU9250_RA_MAG_ZOUT_L		0x07
#define MPU9250_RA_MAG_ZOUT_H		0x08

#define MPU9250_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU9250_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU9250_DEFAULT_ADDRESS     MPU9250_ADDRESS_AD0_LOW

#define MPU9250_RA_SELF_TEST_X_GYRO       0x00 //xg_st_data[7:0]
#define MPU9250_RA_SELF_TEST_Y_GYRO       0x01 
#define MPU9250_RA_SELF_TEST_Z_GYRO       0x02 
#define MPU9250_RA_SELF_TEST_X_ACCEL       0x0D //xa_st_data[7:0]
#define MPU9250_RA_SELF_TEST_Y_ACCEL       0x0E 
#define MPU9250_RA_SELF_TEST_Z_ACCEL       0x0F 

#define MPU9250_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU9250_RA_XG_OFFS_USRL     0x14
#define MPU9250_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU9250_RA_YG_OFFS_USRL     0x16
#define MPU9250_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU9250_RA_ZG_OFFS_USRL     0x18

#define MPU9250_RA_SMPLRT_DIV       0x19
#define MPU9250_RA_CONFIG           0x1A
#define MPU9250_RA_GYRO_CONFIG      0x1B
#define MPU9250_RA_ACCEL_CONFIG     0x1C
#define MPU9250_RA_ACCEL_CONFIG2    0x1D
#define MPU9250_RA_LP_ACCEL_ODR     0x1E

#define MPU9250_RA_FIFO_EN          0x23
#define MPU9250_RA_I2C_MST_CTRL     0x24
#define MPU9250_RA_I2C_SLV0_ADDR    0x25
#define MPU9250_RA_I2C_SLV0_REG     0x26
#define MPU9250_RA_I2C_SLV0_CTRL    0x27
#define MPU9250_RA_I2C_SLV1_ADDR    0x28
#define MPU9250_RA_I2C_SLV1_REG     0x29
#define MPU9250_RA_I2C_SLV1_CTRL    0x2A
#define MPU9250_RA_I2C_SLV2_ADDR    0x2B
#define MPU9250_RA_I2C_SLV2_REG     0x2C
#define MPU9250_RA_I2C_SLV2_CTRL    0x2D
#define MPU9250_RA_I2C_SLV3_ADDR    0x2E
#define MPU9250_RA_I2C_SLV3_REG     0x2F
#define MPU9250_RA_I2C_SLV3_CTRL    0x30
#define MPU9250_RA_I2C_SLV4_ADDR    0x31
#define MPU9250_RA_I2C_SLV4_REG     0x32
#define MPU9250_RA_I2C_SLV4_DO      0x33
#define MPU9250_RA_I2C_SLV4_CTRL    0x34
#define MPU9250_RA_I2C_SLV4_DI      0x35
#define MPU9250_RA_I2C_MST_STATUS   0x36
#define MPU9250_RA_INT_PIN_CFG      0x37
#define MPU9250_RA_INT_ENABLE       0x38
#define MPU9250_RA_DMP_INT_STATUS   0x39
#define MPU9250_RA_INT_STATUS       0x3A
#define MPU9250_RA_ACCEL_XOUT_H     0x3B
#define MPU9250_RA_ACCEL_XOUT_L     0x3C
#define MPU9250_RA_ACCEL_YOUT_H     0x3D
#define MPU9250_RA_ACCEL_YOUT_L     0x3E
#define MPU9250_RA_ACCEL_ZOUT_H     0x3F
#define MPU9250_RA_ACCEL_ZOUT_L     0x40
#define MPU9250_RA_TEMP_OUT_H       0x41
#define MPU9250_RA_TEMP_OUT_L       0x42
#define MPU9250_RA_GYRO_XOUT_H      0x43
#define MPU9250_RA_GYRO_XOUT_L      0x44
#define MPU9250_RA_GYRO_YOUT_H      0x45
#define MPU9250_RA_GYRO_YOUT_L      0x46
#define MPU9250_RA_GYRO_ZOUT_H      0x47
#define MPU9250_RA_GYRO_ZOUT_L      0x48
#define MPU9250_RA_EXT_SENS_DATA_00 0x49
#define MPU9250_RA_EXT_SENS_DATA_01 0x4A
#define MPU9250_RA_EXT_SENS_DATA_02 0x4B
#define MPU9250_RA_EXT_SENS_DATA_03 0x4C
#define MPU9250_RA_EXT_SENS_DATA_04 0x4D
#define MPU9250_RA_EXT_SENS_DATA_05 0x4E
#define MPU9250_RA_EXT_SENS_DATA_06 0x4F
#define MPU9250_RA_EXT_SENS_DATA_07 0x50
#define MPU9250_RA_EXT_SENS_DATA_08 0x51
#define MPU9250_RA_EXT_SENS_DATA_09 0x52
#define MPU9250_RA_EXT_SENS_DATA_10 0x53
#define MPU9250_RA_EXT_SENS_DATA_11 0x54
#define MPU9250_RA_EXT_SENS_DATA_12 0x55
#define MPU9250_RA_EXT_SENS_DATA_13 0x56
#define MPU9250_RA_EXT_SENS_DATA_14 0x57
#define MPU9250_RA_EXT_SENS_DATA_15 0x58
#define MPU9250_RA_EXT_SENS_DATA_16 0x59
#define MPU9250_RA_EXT_SENS_DATA_17 0x5A
#define MPU9250_RA_EXT_SENS_DATA_18 0x5B
#define MPU9250_RA_EXT_SENS_DATA_19 0x5C
#define MPU9250_RA_EXT_SENS_DATA_20 0x5D
#define MPU9250_RA_EXT_SENS_DATA_21 0x5E
#define MPU9250_RA_EXT_SENS_DATA_22 0x5F
#define MPU9250_RA_EXT_SENS_DATA_23 0x60
#define MPU9250_RA_MOT_DETECT_STATUS    0x61
#define MPU9250_RA_I2C_SLV0_DO      0x63
#define MPU9250_RA_I2C_SLV1_DO      0x64
#define MPU9250_RA_I2C_SLV2_DO      0x65
#define MPU9250_RA_I2C_SLV3_DO      0x66
#define MPU9250_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU9250_RA_SIGNAL_PATH_RESET    0x68
#define MPU9250_RA_MOT_DETECT_CTRL      0x69
#define MPU9250_RA_USER_CTRL        0x6A
#define MPU9250_RA_PWR_MGMT_1       0x6B
#define MPU9250_RA_PWR_MGMT_2       0x6C
#define MPU9250_RA_FIFO_COUNTH      0x72
#define MPU9250_RA_FIFO_COUNTL      0x73
#define MPU9250_RA_FIFO_R_W         0x74
#define MPU9250_RA_WHO_AM_I         0x75
#define MPU9250_RA_XA_OFFS_H        0x77 //[15:0] XA_OFFS
#define MPU9250_RA_XA_OFFS_L_TC     0x78
#define MPU9250_RA_YA_OFFS_H        0x7A //[15:0] YA_OFFS
#define MPU9250_RA_YA_OFFS_L_TC     0x7B
#define MPU9250_RA_ZA_OFFS_H        0x7D //[15:0] ZA_OFFS
#define MPU9250_RA_ZA_OFFS_L_TC     0x7E


#define MPU9250_TC_PWR_MODE_BIT     7
#define MPU9250_TC_OFFSET_BIT       6
#define MPU9250_TC_OFFSET_LENGTH    6
#define MPU9250_TC_OTP_BNK_VLD_BIT  0

#define MPU9250_VDDIO_LEVEL_VLOGIC  0
#define MPU9250_VDDIO_LEVEL_VDD     1

#define MPU9250_CFG_EXT_SYNC_SET_BIT    5
#define MPU9250_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU9250_CFG_DLPF_CFG_BIT    2
#define MPU9250_CFG_DLPF_CFG_LENGTH 3

#define MPU9250_EXT_SYNC_DISABLED       0x0
#define MPU9250_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU9250_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU9250_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU9250_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU9250_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU9250_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU9250_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU9250_DLPF_BW_256         0x00
#define MPU9250_DLPF_BW_188         0x01
#define MPU9250_DLPF_BW_98          0x02
#define MPU9250_DLPF_BW_42          0x03
#define MPU9250_DLPF_BW_20          0x04
#define MPU9250_DLPF_BW_10          0x05
#define MPU9250_DLPF_BW_5           0x06
#define MPU9250_DLPF_BW_3600        0x07

#define MPU9250_GCONFIG_FS_SEL_BIT      4
#define MPU9250_GCONFIG_FS_SEL_LENGTH   2

#define MPU9250_GYRO_FS_250         0x00
#define MPU9250_GYRO_FS_500         0x01
#define MPU9250_GYRO_FS_1000        0x02
#define MPU9250_GYRO_FS_2000        0x03

#define MPU9250_GCONFIG_FCHOICE_B_BIT      1
#define MPU9250_GCONFIG_FCHOICE_B_LENGTH   2

#define MPU9250_FCHOICE_NO_DLPF      0x02
#define MPU9250_FCHOICE_DLPF         0x00

#define MPU9250_ACONFIG_XA_ST_BIT           7
#define MPU9250_ACONFIG_YA_ST_BIT           6
#define MPU9250_ACONFIG_ZA_ST_BIT           5
#define MPU9250_ACONFIG_AFS_SEL_BIT         4
#define MPU9250_ACONFIG_AFS_SEL_LENGTH      2
#define MPU9250_ACONFIG_ACC_FCHOICE_BIT     3
#define MPU9250_ACONFIG_ACC_FCHOICE_LENGTH  1
#define MPU9250_ACONFIG_ACCEL_HPF_BIT       2
#define MPU9250_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU9250_ACCEL_FS_2          0x00
#define MPU9250_ACCEL_FS_4          0x01
#define MPU9250_ACCEL_FS_8          0x02
#define MPU9250_ACCEL_FS_16         0x03

#define MPU9250_ACCEL_USE_DLP       0x01
#define MPU9250_ACCEL_NO_DLP        0x00

#define MPU9250_DHPF_RESET          0x00
#define MPU9250_DHPF_5              0x01
#define MPU9250_DHPF_2P5            0x02
#define MPU9250_DHPF_1P25           0x03
#define MPU9250_DHPF_0P63           0x04
#define MPU9250_DHPF_HOLD           0x07

#define MPU9250_TEMP_FIFO_EN_BIT    7
#define MPU9250_XG_FIFO_EN_BIT      6
#define MPU9250_YG_FIFO_EN_BIT      5
#define MPU9250_ZG_FIFO_EN_BIT      4
#define MPU9250_ACCEL_FIFO_EN_BIT   3
#define MPU9250_SLV2_FIFO_EN_BIT    2
#define MPU9250_SLV1_FIFO_EN_BIT    1
#define MPU9250_SLV0_FIFO_EN_BIT    0

#define MPU9250_MULT_MST_EN_BIT     7
#define MPU9250_WAIT_FOR_ES_BIT     6
#define MPU9250_SLV_3_FIFO_EN_BIT   5
#define MPU9250_I2C_MST_P_NSR_BIT   4
#define MPU9250_I2C_MST_CLK_BIT     3
#define MPU9250_I2C_MST_CLK_LENGTH  4

#define MPU9250_CLOCK_DIV_348       0x0
#define MPU9250_CLOCK_DIV_333       0x1
#define MPU9250_CLOCK_DIV_320       0x2
#define MPU9250_CLOCK_DIV_308       0x3
#define MPU9250_CLOCK_DIV_296       0x4
#define MPU9250_CLOCK_DIV_286       0x5
#define MPU9250_CLOCK_DIV_276       0x6
#define MPU9250_CLOCK_DIV_267       0x7
#define MPU9250_CLOCK_DIV_258       0x8
#define MPU9250_CLOCK_DIV_500       0x9
#define MPU9250_CLOCK_DIV_471       0xA
#define MPU9250_CLOCK_DIV_444       0xB
#define MPU9250_CLOCK_DIV_421       0xC
#define MPU9250_CLOCK_DIV_400       0xD
#define MPU9250_CLOCK_DIV_381       0xE
#define MPU9250_CLOCK_DIV_364       0xF

#define MPU9250_I2C_SLV_RW_BIT      7
#define MPU9250_I2C_SLV_ADDR_BIT    6
#define MPU9250_I2C_SLV_ADDR_LENGTH 7
#define MPU9250_I2C_SLV_EN_BIT      7
#define MPU9250_I2C_SLV_BYTE_SW_BIT 6
#define MPU9250_I2C_SLV_REG_DIS_BIT 5
#define MPU9250_I2C_SLV_GRP_BIT     4
#define MPU9250_I2C_SLV_LEN_BIT     3
#define MPU9250_I2C_SLV_LEN_LENGTH  4

#define MPU9250_I2C_SLV4_RW_BIT         7
#define MPU9250_I2C_SLV4_ADDR_BIT       6
#define MPU9250_I2C_SLV4_ADDR_LENGTH    7
#define MPU9250_I2C_SLV4_EN_BIT         7
#define MPU9250_I2C_SLV4_INT_EN_BIT     6
#define MPU9250_I2C_SLV4_REG_DIS_BIT    5
#define MPU9250_I2C_SLV4_MST_DLY_BIT    4
#define MPU9250_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU9250_MST_PASS_THROUGH_BIT    7
#define MPU9250_MST_I2C_SLV4_DONE_BIT   6
#define MPU9250_MST_I2C_LOST_ARB_BIT    5
#define MPU9250_MST_I2C_SLV4_NACK_BIT   4
#define MPU9250_MST_I2C_SLV3_NACK_BIT   3
#define MPU9250_MST_I2C_SLV2_NACK_BIT   2
#define MPU9250_MST_I2C_SLV1_NACK_BIT   1
#define MPU9250_MST_I2C_SLV0_NACK_BIT   0

#define MPU9250_INTCFG_INT_LEVEL_BIT        7
#define MPU9250_INTCFG_INT_OPEN_BIT         6
#define MPU9250_INTCFG_LATCH_INT_EN_BIT     5
#define MPU9250_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU9250_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU9250_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU9250_INTCFG_CLKOUT_EN_BIT        0

#define MPU9250_INTMODE_ACTIVEHIGH  0x00
#define MPU9250_INTMODE_ACTIVELOW   0x01

#define MPU9250_INTDRV_PUSHPULL     0x00
#define MPU9250_INTDRV_OPENDRAIN    0x01

#define MPU9250_INTLATCH_50USPULSE  0x00
#define MPU9250_INTLATCH_WAITCLEAR  0x01

#define MPU9250_INTCLEAR_STATUSREAD 0x00
#define MPU9250_INTCLEAR_ANYREAD    0x01

#define MPU9250_INTERRUPT_FF_BIT            7
#define MPU9250_INTERRUPT_MOT_BIT           6
#define MPU9250_INTERRUPT_ZMOT_BIT          5
#define MPU9250_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU9250_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU9250_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU9250_INTERRUPT_DMP_INT_BIT       1
#define MPU9250_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU9250_DMPINT_5_BIT            5
#define MPU9250_DMPINT_4_BIT            4
#define MPU9250_DMPINT_3_BIT            3
#define MPU9250_DMPINT_2_BIT            2
#define MPU9250_DMPINT_1_BIT            1
#define MPU9250_DMPINT_0_BIT            0

#define MPU9250_MOTION_MOT_XNEG_BIT     7
#define MPU9250_MOTION_MOT_XPOS_BIT     6
#define MPU9250_MOTION_MOT_YNEG_BIT     5
#define MPU9250_MOTION_MOT_YPOS_BIT     4
#define MPU9250_MOTION_MOT_ZNEG_BIT     3
#define MPU9250_MOTION_MOT_ZPOS_BIT     2
#define MPU9250_MOTION_MOT_ZRMOT_BIT    0

#define MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU9250_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU9250_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU9250_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU9250_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU9250_PATHRESET_GYRO_RESET_BIT    2
#define MPU9250_PATHRESET_ACCEL_RESET_BIT   1
#define MPU9250_PATHRESET_TEMP_RESET_BIT    0

#define MPU9250_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU9250_DETECT_FF_COUNT_BIT             3
#define MPU9250_DETECT_FF_COUNT_LENGTH          2
#define MPU9250_DETECT_MOT_COUNT_BIT            1
#define MPU9250_DETECT_MOT_COUNT_LENGTH         2

#define MPU9250_DETECT_DECREMENT_RESET  0x0
#define MPU9250_DETECT_DECREMENT_1      0x1
#define MPU9250_DETECT_DECREMENT_2      0x2
#define MPU9250_DETECT_DECREMENT_4      0x3

#define MPU9250_USERCTRL_DMP_EN_BIT             7
#define MPU9250_USERCTRL_FIFO_EN_BIT            6
#define MPU9250_USERCTRL_I2C_MST_EN_BIT         5
#define MPU9250_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU9250_USERCTRL_DMP_RESET_BIT          3
#define MPU9250_USERCTRL_FIFO_RESET_BIT         2
#define MPU9250_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU9250_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU9250_PWR1_DEVICE_RESET_BIT   7
#define MPU9250_PWR1_SLEEP_BIT          6
#define MPU9250_PWR1_CYCLE_BIT          5
#define MPU9250_PWR1_TEMP_DIS_BIT       3
#define MPU9250_PWR1_CLKSEL_BIT         2
#define MPU9250_PWR1_CLKSEL_LENGTH      3

#define MPU9250_CLOCK_INTERNAL          0x00
#define MPU9250_CLOCK_PLL_XGYRO         0x01
#define MPU9250_CLOCK_PLL_YGYRO         0x02
#define MPU9250_CLOCK_PLL_ZGYRO         0x03
#define MPU9250_CLOCK_PLL_EXT32K        0x04
#define MPU9250_CLOCK_PLL_EXT19M        0x05
#define MPU9250_CLOCK_KEEP_RESET        0x07

#define MPU9250_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU9250_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU9250_PWR2_STBY_XA_BIT            5
#define MPU9250_PWR2_STBY_YA_BIT            4
#define MPU9250_PWR2_STBY_ZA_BIT            3
#define MPU9250_PWR2_STBY_XG_BIT            2
#define MPU9250_PWR2_STBY_YG_BIT            1
#define MPU9250_PWR2_STBY_ZG_BIT            0

#define MPU9250_WAKE_FREQ_1P25      0x0
#define MPU9250_WAKE_FREQ_2P5       0x1
#define MPU9250_WAKE_FREQ_5         0x2
#define MPU9250_WAKE_FREQ_10        0x3

#define MPU9250_BANKSEL_PRFTCH_EN_BIT       6
#define MPU9250_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU9250_BANKSEL_MEM_SEL_BIT         4
#define MPU9250_BANKSEL_MEM_SEL_LENGTH      5

#define MPU9250_WHO_AM_I_BIT        7
#define MPU9250_WHO_AM_I_LENGTH     7

#define MPU9250_DMP_MEMORY_BANKS        8
#define MPU9250_DMP_MEMORY_BANK_SIZE    256
#define MPU9250_DMP_MEMORY_CHUNK_SIZE   16

enum inv_slave_mode {
	INV_MODE_SUSPEND,
	INV_MODE_NORMAL,
};

enum inv_filter_e {
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};

enum inv_lposc_clksel_e {
	ACC_ODR_0P24HZ = 0,
	ACC_ODR_0P49HZ,
	ACC_ODR_0P98HZ,
	ACC_ODR_1P95HZ,
	ACC_ODR_3P91HZ,
	ACC_ODR_7P81HZ,
	ACC_ODR_15P63HZ,
	ACC_ODR_31P25HZ,
	ACC_ODR_62P50HZ,
	ACC_ODR_125HZ,
	ACC_ODR_250HZ,
	ACC_ODR_500HZ,
	ACC_ODR_MAX = ACC_ODR_500HZ,
};
enum inv_accel_fs_e {
	INV_FS_02G = 0,
	INV_FS_04G,
	INV_FS_08G,
	INV_FS_16G,
	NUM_ACCEL_FSR,
};

enum inv_fsr_e {
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_FSR,
};

enum inv_mag_e {
	INV_MAG_14BIT = 0,
	INV_MAG_16BIT,
	NUM_MAG,
};

enum inv_clock_sel_e {
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};

enum inv_acc_fchoice_b_sel {
	INV_ACC_F_B_DLP = 0,
	INV_ACC_F_B_1K,
	NUM_ACC_F_B
};

enum inv_gry_fchoice_b_sel {
	INV_GRY_F_B_DLP = 0,
	INV_GRY_F_B_8K,
	INV_GRY_F_B_3K,
	NUM_GRY_F_B
};
struct sensor_9axis_t {
	float f_acc_x;
	float f_acc_y;
	float f_acc_z;
	float f_gyr_x;
	float f_gyr_y;
	float f_gyr_z;
	float f_geo_x;
	float f_geo_y;
	float f_geo_z;
	float f_azimuth;
	float f_pitch;
	float f_roll;
	float f_yaw;
	float angles[3];
	int wYear;
	int wMonth;
	int wDay;
	int wHour;
	int wMinute;
	int wSecond;
	int wMilliseconds;
        
};

struct sensor_6axis_t {
	float f_acc_x;
	float f_acc_y;
	float f_acc_z;
	float f_gyr_x;
	float f_gyr_y;
	float f_gyr_z;
        float *Rot;
	int wYear;
	int wMonth;
	int wDay;
	int wHour;
	int wMinute;
	int wSecond;
	int wMilliseconds;
        
        public:
        sensor_6axis_t(){
           Rot = new float[9];
        }
};
#define SENSOR_BUF_SIZE 2000

struct sensor_cfg_t {
	inv_accel_fs_e acc_range;
	inv_fsr_e gyr_range;
	bool mag_enabled;
	bool log_enabled;
	inv_mag_e mag_range;
	signed short ax_offset;
	signed short ay_offset;
	signed short az_offset;
	signed short gx_offset;
	signed short gy_offset;
	signed short gz_offset;
	inv_acc_fchoice_b_sel acc_fb_mode;
	inv_gry_fchoice_b_sel gry_fb_mode;
	inv_filter_e lpf;
	inv_lposc_clksel_e lp_acc_odr;
	bool only_test_cap;
	int interval_max_ms;
};

class MPU9250 {
    public:
        MPU9250();
        MPU9250(int address);
	~MPU9250();

        bool initialize();
        bool testConnection();
	void close();

	bool OpenListenThread();
	bool CloseListenThread();
        // SMPLRT_DIV register
        unsigned char getRate();
        void setRate(unsigned char rate);

        // CONFIG register
        unsigned char getExternalFrameSync();
        void setExternalFrameSync(unsigned char sync);
        unsigned char getDLPFMode();
        void setDLPFMode(unsigned char bandwidth);

        // GYRO_CONFIG register
        unsigned char getFullScaleGyroRange();
        void setFullScaleGyroRange(unsigned char range);
	void setFchoice(unsigned char mode);

        // ACCEL_CONFIG register
        unsigned char getAccelXSelfTest();
        void setAccelXSelfTest(bool enabled);
        unsigned char getAccelYSelfTest();
        void setAccelYSelfTest(bool enabled);
        unsigned char getAccelZSelfTest();
        void setAccelZSelfTest(bool enabled);
        unsigned char getFullScaleAccelRange();
        void setFullScaleAccelRange(unsigned char range);
	void setAccelFchoice(unsigned char mode);
        unsigned char getDHPFMode();
        void setDHPFMode(unsigned char mode);

        // FF_THR register
        unsigned char getFreefallDetectionThreshold();
        void setFreefallDetectionThreshold(unsigned char threshold);

        // FF_DUR register
        unsigned char getFreefallDetectionDuration();
        void setFreefallDetectionDuration(unsigned char duration);

        // MOT_THR register
        unsigned char getMotionDetectionThreshold();
        void setMotionDetectionThreshold(unsigned char threshold);

        // MOT_DUR register
        unsigned char getMotionDetectionDuration();
        void setMotionDetectionDuration(unsigned char duration);

        // ZRMOT_THR register
        unsigned char getZeroMotionDetectionThreshold();
        void setZeroMotionDetectionThreshold(unsigned char threshold);

        // ZRMOT_DUR register
        unsigned char getZeroMotionDetectionDuration();
        void setZeroMotionDetectionDuration(unsigned char duration);

        // FIFO_EN register
        bool getTempFIFOEnabled();
        void setTempFIFOEnabled(bool enabled);
        bool getXGyroFIFOEnabled();
        void setXGyroFIFOEnabled(bool enabled);
        bool getYGyroFIFOEnabled();
        void setYGyroFIFOEnabled(bool enabled);
        bool getZGyroFIFOEnabled();
        void setZGyroFIFOEnabled(bool enabled);
        bool getAccelFIFOEnabled();
        void setAccelFIFOEnabled(bool enabled);
        bool getSlave2FIFOEnabled();
        void setSlave2FIFOEnabled(bool enabled);
        bool getSlave1FIFOEnabled();
        void setSlave1FIFOEnabled(bool enabled);
        bool getSlave0FIFOEnabled();
        void setSlave0FIFOEnabled(bool enabled);

        // I2C_MST_CTRL register
        bool getMultiMasterEnabled();
        void setMultiMasterEnabled(bool enabled);
        bool getWaitForExternalSensorEnabled();
        void setWaitForExternalSensorEnabled(bool enabled);
        bool getSlave3FIFOEnabled();
        void setSlave3FIFOEnabled(bool enabled);
        bool getSlaveReadWriteTransitionEnabled();
        void setSlaveReadWriteTransitionEnabled(bool enabled);
        unsigned char getMasterClockSpeed();
        void setMasterClockSpeed(unsigned char speed);

        // I2C_SLV* registers (Slave 0-3)
        unsigned char getSlaveAddress(unsigned char num);
        void setSlaveAddress(unsigned char num, unsigned char address);
        unsigned char getSlaveRegister(unsigned char num);
        void setSlaveRegister(unsigned char num, unsigned char reg);
        bool getSlaveEnabled(unsigned char num);
        void setSlaveEnabled(unsigned char num, bool enabled);
        bool getSlaveWordByteSwap(unsigned char num);
        void setSlaveWordByteSwap(unsigned char num, bool enabled);
        bool getSlaveWriteMode(unsigned char num);
        void setSlaveWriteMode(unsigned char num, bool mode);
        bool getSlaveWordGroupOffset(unsigned char num);
        void setSlaveWordGroupOffset(unsigned char num, bool enabled);
        unsigned char getSlaveDataLength(unsigned char num);
        void setSlaveDataLength(unsigned char num, unsigned char length);

        // I2C_SLV* registers (Slave 4)
        unsigned char getSlave4Address();
        void setSlave4Address(unsigned char address);
        unsigned char getSlave4Register();
        void setSlave4Register(unsigned char reg);
        void setSlave4OutputByte(unsigned char data);
        bool getSlave4Enabled();
        void setSlave4Enabled(bool enabled);
        bool getSlave4InterruptEnabled();
        void setSlave4InterruptEnabled(bool enabled);
        bool getSlave4WriteMode();
        void setSlave4WriteMode(bool mode);
        unsigned char getSlave4MasterDelay();
        void setSlave4MasterDelay(unsigned char delay);
        unsigned char getSlate4InputByte();

        // I2C_MST_STATUS register
        bool getPassthroughStatus();
        bool getSlave4IsDone();
        bool getLostArbitration();
        bool getSlave4Nack();
        bool getSlave3Nack();
        bool getSlave2Nack();
        bool getSlave1Nack();
        bool getSlave0Nack();

        // INT_PIN_CFG register
        bool getInterruptMode();
        void setInterruptMode(bool mode);
        bool getInterruptDrive();
        void setInterruptDrive(bool drive);
        bool getInterruptLatch();
        void setInterruptLatch(bool latch);
        bool getInterruptLatchClear();
        void setInterruptLatchClear(bool clear);
        bool getFSyncInterruptLevel();
        void setFSyncInterruptLevel(bool level);
        bool getFSyncInterruptEnabled();
        void setFSyncInterruptEnabled(bool enabled);
        bool getI2CBypassEnabled();
        void setI2CBypassEnabled(bool enabled);
        bool getClockOutputEnabled();
        void setClockOutputEnabled(bool enabled);

        // INT_ENABLE register
        unsigned char getIntEnabled();
        void setIntEnabled(unsigned char enabled);
        bool getIntFreefallEnabled();
        void setIntFreefallEnabled(bool enabled);
        bool getIntMotionEnabled();
        void setIntMotionEnabled(bool enabled);
        bool getIntZeroMotionEnabled();
        void setIntZeroMotionEnabled(bool enabled);
        bool getIntFIFOBufferOverflowEnabled();
        void setIntFIFOBufferOverflowEnabled(bool enabled);
        bool getIntI2CMasterEnabled();
        void setIntI2CMasterEnabled(bool enabled);
        bool getIntDataReadyEnabled();
        void setIntDataReadyEnabled(bool enabled);

        // INT_STATUS register
        unsigned char getIntStatus();
        bool getIntFreefallStatus();
        bool getIntMotionStatus();
        bool getIntZeroMotionStatus();
        bool getIntFIFOBufferOverflowStatus();
        bool getIntI2CMasterStatus();
        bool getIntDataReadyStatus();

	char enable_mag();
	char disable_mag();
        // ACCEL_*OUT_* registers
        void getMotion9(signed short* ax, signed short* ay, signed short* az, signed short* gx, signed short* gy, signed short* gz, signed short* mx, signed short* my, signed short* mz);
        char getMotion6(signed short* ax, signed short* ay, signed short* az, signed short* gx, signed short* gy, signed short* gz);
	char getMotion6(signed short* all_data);
	char getMotion9(signed short* all_data);
        void getAcceleration(signed short* x, signed short* y, signed short* z);
        signed short getAccelerationX();
        signed short getAccelerationY();
        signed short getAccelerationZ();

        // TEMP_OUT_* registers
        signed short getTemperature();

        // GYRO_*OUT_* registers
        void getRotation(signed short* x, signed short* y, signed short* z);
        signed short getRotationX();
        signed short getRotationY();
        signed short getRotationZ();

        // EXT_SENS_DATA_* registers
        unsigned char getExternalSensorByte(int position);
        unsigned short getExternalSensorWord(int position);
        int getExternalSensorDWord(int position);

        // MOT_DETECT_STATUS register
        bool getXNegMotionDetected();
        bool getXPosMotionDetected();
        bool getYNegMotionDetected();
        bool getYPosMotionDetected();
        bool getZNegMotionDetected();
        bool getZPosMotionDetected();
        bool getZeroMotionDetected();

        // I2C_SLV*_DO register
        void setSlaveOutputByte(unsigned char num, unsigned char data);

        // I2C_MST_DELAY_CTRL register
        bool getExternalShadowDelayEnabled();
        void setExternalShadowDelayEnabled(bool enabled);
        bool getSlaveDelayEnabled(unsigned char num);
        void setSlaveDelayEnabled(unsigned char num, bool enabled);

        // SIGNAL_PATH_RESET register
        void resetGyroscopePath();
        void resetAccelerometerPath();
        void resetTemperaturePath();

        // MOT_DETECT_CTRL register
        unsigned char getAccelerometerPowerOnDelay();
        void setAccelerometerPowerOnDelay(unsigned char delay);
        unsigned char getFreefallDetectionCounterDecrement();
        void setFreefallDetectionCounterDecrement(unsigned char decrement);
        unsigned char getMotionDetectionCounterDecrement();
        void setMotionDetectionCounterDecrement(unsigned char decrement);

        // USER_CTRL register
        bool getFIFOEnabled();
        void setFIFOEnabled(bool enabled);
        bool getI2CMasterModeEnabled();
        void setI2CMasterModeEnabled(bool enabled);
        void switchSPIEnabled(bool enabled);
        void resetFIFO();
        void resetI2CMaster();
        void resetSensors();

        // PWR_MGMT_1 register
        void reset();
        bool getSleepEnabled();
        void setSleepEnabled(bool enabled);
        bool getWakeCycleEnabled();
        void setWakeCycleEnabled(bool enabled);
        bool getTempSensorEnabled();
        void setTempSensorEnabled(bool enabled);
        unsigned char getClockSource();
        void setClockSource(unsigned char source);

        // PWR_MGMT_2 register
        unsigned char getWakeFrequency();
        void setWakeFrequency(unsigned char frequency);
        bool getStandbyXAccelEnabled();
        void setStandbyXAccelEnabled(bool enabled);
        bool getStandbyYAccelEnabled();
        void setStandbyYAccelEnabled(bool enabled);
        bool getStandbyZAccelEnabled();
        void setStandbyZAccelEnabled(bool enabled);
        bool getStandbyXGyroEnabled();
        void setStandbyXGyroEnabled(bool enabled);
        bool getStandbyYGyroEnabled();
        void setStandbyYGyroEnabled(bool enabled);
        bool getStandbyZGyroEnabled();
        void setStandbyZGyroEnabled(bool enabled);

        // FIFO_COUNT_* registers
        unsigned short getFIFOCount();

        // FIFO_R_W register
        unsigned char getFIFOByte();
        void setFIFOByte(unsigned char data);
        void getFIFOBytes(unsigned char *data, unsigned char length);

        // WHO_AM_I register
        unsigned char getDeviceID();
        
        // ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========
        
        // XG_OFFS_TC register
        unsigned char getOTPBankValid();
        void setOTPBankValid(bool enabled);
        char getXGyroOffset();
        void setXGyroOffset(char offset);

        // YG_OFFS_TC register
        char getYGyroOffset();
        void setYGyroOffset(char offset);

        // ZG_OFFS_TC register
        char getZGyroOffset();
        void setZGyroOffset(char offset);

        // X_FINE_GAIN register
        char getXFineGain();
        void setXFineGain(char gain);

        // Y_FINE_GAIN register
        char getYFineGain();
        void setYFineGain(char gain);

        // Z_FINE_GAIN register
        char getZFineGain();
        void setZFineGain(char gain);

        // XA_OFFS_* registers
        signed short getXAccelOffset();
        void setXAccelOffset(signed short offset);

        // YA_OFFS_* register
        signed short getYAccelOffset();
        void setYAccelOffset(signed short offset);

        // ZA_OFFS_* register
        signed short getZAccelOffset();
        void setZAccelOffset(signed short offset);

        // XG_OFFS_USR* registers
        signed short getXGyroOffsetUser();
        void setXGyroOffsetUser(signed short offset);

        // YG_OFFS_USR* register
        signed short getYGyroOffsetUser();
        void setYGyroOffsetUser(signed short offset);

        // ZG_OFFS_USR* register
        signed short getZGyroOffsetUser();
        void setZGyroOffsetUser(signed short offset);
	bool is_Exit();
	void set_Exit();
	void sensor_cap_start();
	bool sensor_data_is_ready();
	void switch_data_ptr();
	void check_buf_num();
	int get_data_num();
	void set_max_data_num(int);
	void sensor_cap_data(struct sensor_6axis_t *);
	bool sensor_set_buf(sensor_6axis_t(*)[SENSOR_BUF_SIZE]);
	sensor_cfg_t sensor_cfg;
	sensor_6axis_t data_ptr[SENSOR_BUF_SIZE];
	sensor_6axis_t(*data_deal_ptr)[SENSOR_BUF_SIZE];
	int current_num;
	int data_interval;
     private:
	bool openPort();
	bool openPort(unsigned char n_port);
	static void* ListenThread(void *pParam);
	int n_port;
	pthread_t m_hListenThread;
	pthread_mutex_t m_csCommunicationSync;
	unsigned char devAddr;
	unsigned char buffer[256];
	int end_num;
	int begin_num;
	int max_data_num;
	bool current_full;
};

#endif /* _MPU9250_H_ */
