/*
 * mpu6050_dmp.c
 *
 *  Created on: 8. 10. 2022
 *      Author: roman
 */


#include "mpu6050.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
//--------------------------------
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"  // displej
#include "fonts.h"
//--------------------------------
#include <math.h>

extern I2C_HandleTypeDef hi2c2;
extern uint8_t misto;
IMU_Parameter IMU_Data;

#define MPU_ADDR 0xD0
#define PI 3.14
float gyrox, gyroy, gyroz, accelx, accely, accelz, temp;
//---------------------------------------------------------------------------------
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{

	if(HAL_I2C_Mem_Write(&hi2c2,MPU_ADDR,reg,1,&data,1,0xff) == HAL_OK)
		return 0;
	else
		return 1;
}
uint8_t MPU_Read_Byte(uint8_t reg)
{

	if(HAL_I2C_Mem_Read(&hi2c2, 0xD1, reg, 1, &reg, 1, 0xff) == HAL_OK)
		return 0;
	else
		return 1;
}
// Set up MPU6050 Gyro sensor full scale range
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// Return value :0, Set up the success
//  other , Setup failed
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{

	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);// Set the gyro full scale range
}
// Set up MPU6050 Full scale range of acceleration sensor
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
// Return value :0, Set up the success
//  other , Setup failed
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{

	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);// Set the full-scale range of the acceleration sensor
}
// Set up MPU6050 Digital low pass filter
//lpf: Digital low-pass filter frequency (Hz)
// Return value :0, Set up the success
//  other , Setup failed
uint8_t MPU_Set_LPF(uint16_t lpf)
{

	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);// Set the digital low-pass filter
}
// Set up MPU6050 Sampling rate ( Assume Fs=1KHz)
//rate:4~1000(Hz)
// Return value :0, Set up the success
//  other , Setup failed
uint8_t MPU_Set_Rate(uint16_t rate)
{

	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);	// Set the digital low-pass filter
 	return MPU_Set_LPF(rate/2);	// Automatic setting LPF Half the sampling rate
}

uint8_t MPU6050_Init(void)
{

	uint8_t res[1];
	//....................................................................
    /* Check who I am */
	if (HAL_I2C_Mem_Read(&hi2c2,MPU_ADDR,0x75,1,(uint8_t*)res, 1, 5000) != HAL_OK)
		{
	      return 1;/* error */
		}
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	// Reset MPU6050
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	// Wake up the MPU6050
	MPU_Set_Gyro_Fsr(0);					// Gyroscope sensor ,±250dps
	MPU_Set_Accel_Fsr(0);					// Acceleration sensor ,±2g
	MPU_Set_Rate(75);						// Set the sampling rate 50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	// Turn off all interrupts
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C Main mode is off
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	// close FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT Pin low level active
	if(res[0] == 0x68)// device ID correct
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	// Set up CLKSEL,PLL X The axis is the reference
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	// Both acceleration and gyroscope work
		MPU_Set_Rate(75);						// Set the sampling rate to 75Hz
 	}else return 1;
	return 0;
}
void MPU6050_GET_Data(void)
{

	uint8_t buf[14]={
    0};

	HAL_I2C_Mem_Read(&hi2c2,MPU_ADDR,MPU_ACCEL_XOUTH_REG,1,buf,14,1000);

	accelx = (float) (((int16_t) (buf[0] << 8) + buf[1])/16384.0f);
	accely = (float) (((int16_t) (buf[2] << 8) + buf[3])/16384.0f);
	accelz = (float) (((int16_t) (buf[4] << 8) + buf[5])/16384.0f);
	temp = (float) (((int16_t) (buf[6] << 8) + buf[7])/340 + 36.53f);
	gyrox = (float) (((int16_t) (buf[8] << 8) + buf[9])/131.0f);
	gyroy = (float) (((int16_t) (buf[10] << 8) + buf[11])/131.0f);
	gyroz = (float) (((int16_t) (buf[12] << 8) + buf[13])/131.0f);

	IMU_Data.Accel_X = accelx - IMU_Data.offset_Accel_X;
	IMU_Data.Accel_Y = accely - IMU_Data.offset_Accel_Y;
	IMU_Data.Accel_Z = accelz - IMU_Data.offset_Accel_Z;
	IMU_Data.Temp = temp;
	IMU_Data.Gyro_X = gyrox - IMU_Data.offset_Gyro_X;
	IMU_Data.Gyro_Y = gyroy - IMU_Data.offset_Gyro_Y;
	IMU_Data.Gyro_Z = gyroz - IMU_Data.offset_Gyro_Z;
}

#define DEFAULT_MPU_HZ (100)// Define the output speed
//  Gyro direction setting
static signed char gyro_orientation[9] = {
     1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};
#define q30 1073741824.0f


// Direction change
unsigned short inv_row_2_scale(const signed char *row)
{

    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
//MPU6050 Self test
// Return value :0, normal
//  other , Failure
unsigned char run_self_test(void)
{

	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3];
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3)
	{


		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}else return 1;
}
// Gyroscope direction control
uint8_t inv_orientation_matrix_to_scalar(const signed char *mtx)
{

    unsigned short scalar;

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
uint8_t mpu_dmp_init(void)
{

	uint8_t res=0;
	if(mpu_init()==0)	// initialization MPU6050
	{

		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);// Set the required sensors
		if(res)return 1;
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);// Set up FIFO
		if(res)return 2;
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	// Set the sampling rate
		if(res)return 3;
		res=dmp_load_motion_driver_firmware();		// load dmp The firmware
		if(res)return 4;
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));// Set the gyro direction
		if(res)return 5;
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	// Set up dmp function
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6;
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	// Set up DMP Output rate ( No more than 200Hz)
		if(res)return 7;
		res=dmp_enable_gyro_cal(1);
		if(res)return 10;
		ILI9341_Draw_Text("CAL-GYRO(don't move 8 Sekund)", FONT4, 8,misto+= 20, RED, BLACK, 0);
		misto+= 10;
		for(uint8_t cas = 1; cas < 30; cas++) {
		ILI9341_Draw_Text(".", FONT8, (cas*10) ,misto, WHITE, BLACK, 0);
		HAL_Delay(500);
		}
		//res=dmp_enable_gyro_cal(0);
		//if(res)return 11;
		misto+= 10;
		ILI9341_Draw_Text("TEST-SLF(don't move)", FONT4, 8,misto+= 20, WHITE, BLACK, 0);
		HAL_Delay(3000);
		res=run_self_test();		// Self inspection
		if(res)return 8;
		res=mpu_set_dmp_state(1);	// Can make DMP
		if(res)return 9;
	}
	return 0;
}
// obtain dmp Processed data
//pitch: Pitch angle   precision :0.1°  Range :-90.0° <---> +90.0°
//roll: Roll angle   precision :0.1°  Range :-180.0°<---> +180.0°
//yaw: Heading angle   precision :0.1°  Range :-180.0°<---> +180.0°
// Return value :0, normal
//  other , Failure
uint8_t mpu_dmp_get_data(IMU_Parameter *IMU_Data)
{

	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	float a12 = 0, a22 = 0, a31 = 0, a32 = 0, a33 = 0;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;

	if(sensors&INV_WXYZ_QUAT) //INV_XYZ_GYRO
	{

		q0 = quat[0] / q30;	//q30 Convert format to floating point number
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;
		// Calculate the pitch angle / Roll angle / Heading angle
		IMU_Data->Angle_X = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		IMU_Data->Angle_Y = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		IMU_Data->Angle_Z  = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	    IMU_Data->Angle_Z   += 4.03f; // Declination
	    if(IMU_Data->Angle_Z < 0) IMU_Data->Angle_Z   += 360.0f; // Ensure yaw stays between 0 and 360
	    else if (IMU_Data->Angle_Z >= 360) IMU_Data->Angle_Z -= 360;

/*	    a12 =   2.0f * (q1 * q2 + q0 * q3);
	    a22 =   q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	    a31 =   2.0f * (q0 * q1 + q2 * q3);
	    a32 =   2.0f * (q1 * q3 - q0 * q2);
	    a33 =   q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	    IMU_Data->Angle_X = -asinf(a32);
	    IMU_Data->Angle_Y  = atan2f(a31, a33);
	    IMU_Data->Angle_Z   = atan2f(a12, a22);
	    IMU_Data->Angle_X *= 180.0f / PI;
	    IMU_Data->Angle_Z   *= 180.0f / PI;
	    IMU_Data->Angle_Z   += 4.03f; // Declination
	    if(IMU_Data->Angle_Z < 0) IMU_Data->Angle_Z   += 360.0f; // Ensure yaw stays between 0 and 360
	    else if (IMU_Data->Angle_Z >= 360) IMU_Data->Angle_Z -= 360;
	    IMU_Data->Angle_Y  *= 180.0f / PI; */
		//....................................................................................................
	}else return 2;
	return 0;
}

uint8_t HAL_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{

	HAL_I2C_Mem_Write(&hi2c2, ((slave_addr<<1)|0), reg_addr, 1, (unsigned char *)data, length, HAL_MAX_DELAY);
	return 0;
}
uint8_t HAL_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{

	HAL_I2C_Mem_Read(&hi2c2, ((slave_addr<<1)|1), reg_addr, 1, (unsigned char *)data, length, HAL_MAX_DELAY);
	return 0;
}

IMU_Parameter MPU6050_Calibrate(void)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0};
  int32_t accel_bias[3] = {0, 0, 0};

 // reset device
  MPU_Write_Byte( MPU_PWR_MGMT1_REG, 0x80);
 // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
 // else use the internal oscillator, bits 2:0 = 001
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x01);
  MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0x00);
  HAL_Delay(200);

   // Configure device for bias calculation
  MPU_Write_Byte(MPU_INT_EN_REG, 0x00);
   // Disable all interrupts
  MPU_Write_Byte(MPU_FIFO_EN_REG, 0x00);
      // Disable FIFO
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);
   // Turn on internal clock source
  MPU_Write_Byte(MPU_I2CMST_CTRL_REG, 0x00);
  // Disable I2C master
  MPU_Write_Byte(MPU_USER_CTRL_REG, 0x00);
  // Disable FIFO and I2C master modes
  MPU_Write_Byte(MPU_USER_CTRL_REG, 0x0C);
  // Reset FIFO and DMP
  HAL_Delay(15);
  // Configure MPU6050 gyro and accelerometer for bias calculation
  MPU_Write_Byte(MPU_CFG_REG, 0x01);
  // Set low-pass filter to 50 Hz
  MPU_Write_Byte(MPU_SAMPLE_RATE_REG, 0x13);
  // Set sample rate to 1 kHz
  MPU_Write_Byte( MPU_GYRO_CFG_REG, 0x08);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  MPU_Write_Byte(MPU_ACCEL_CFG_REG, 0x00);
  // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131.0f;   // = 16.5 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384.0f;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  MPU_Write_Byte(MPU_USER_CTRL_REG, 0x40);
   // Enable FIFO
  MPU_Write_Byte(MPU_FIFO_EN_REG, 0x78);
   // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
  // At end of sample accumulation, turn off FIFO sensor read
  MPU_Write_Byte(MPU_FIFO_EN_REG, 0x00);
        // Disable gyro and accelerometer sensors for FIFO
  HAL_I2C_Mem_Read(&hi2c2,MPU_ADDR, MPU_FIFO_CNTH_REG, 1, &data[0], 2, 5000);
 // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
     // read data for averaging
  HAL_I2C_Mem_Read(&hi2c2,MPU_ADDR, MPU_FIFO_RW_REG, 1, &data[0], 12, 5000);

    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
   // data[0] = (-gyro_bias[0]  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
   //data[1] = (-gyro_bias[0])       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
   //data[2] = (-gyro_bias[1]  >> 8) & 0xFF;
   //data[3] = (-gyro_bias[1])       & 0xFF;
   //data[4] = (-gyro_bias[2]  >> 8) & 0xFF;
   //data[5] = (-gyro_bias[2])       & 0xFF;

// Push gyro biases to hardware registers
 //HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x13, 1, &data[0] , 1, 5000);
 //HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x14, 1, &data[1] , 1, 5000);
 //HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x15, 1, &data[2] , 1, 5000);
 //HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x16, 1, &data[3] , 1, 5000);
 //HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x17, 1, &data[4] , 1, 5000);
 //HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x18, 1, &data[5] , 1, 5000);

// Output scaled gyro biases for display in the main program
  IMU_Data.Osgx = (float) gyro_bias[0]/gyrosensitivity;
  IMU_Data.Osgy = (float) gyro_bias[1]/gyrosensitivity;
  IMU_Data.Osgz = (float) gyro_bias[2]/gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  // Read factory accelerometer trim values
  HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x77, 1, &data[0], 2, 5000);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x7A, 1, &data[0], 2, 5000);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x7D, 1, &data[0], 2, 5000);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x77, 1, &data[0] , 1, 5000);
  HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x78, 1, &data[1] , 1, 5000);
  HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x7A, 1, &data[2] , 1, 5000);
  HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x7B, 1, &data[3] , 1, 5000);
  HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x7C, 1, &data[4] , 1, 5000);
  HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x7E, 1, &data[5] , 1, 5000);
// Output scaled accelerometer biases for display in the main program
  IMU_Data.offset_Accel_X = (float)accel_bias[0]/(float)accelsensitivity;
  IMU_Data.offset_Accel_Y= (float)accel_bias[1]/(float)accelsensitivity;
  IMU_Data.offset_Accel_Z = (float)accel_bias[2]/(float)accelsensitivity;

  return IMU_Data;
}
