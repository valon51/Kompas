/*
 * mpu6050.h
 *
 *  Created on: 8. 10. 2022
 *      Author: roman
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include "main.h"

#define MPU_SELF_TESTX_REG 0X0D // Self test register X
#define MPU_SELF_TESTY_REG 0X0E // Self test register Y
#define MPU_SELF_TESTZ_REG 0X0F // Self test register Z
#define MPU_SELF_TESTA_REG 0X10 // Self test register A
#define MPU_SAMPLE_RATE_REG 0X19 // Sampling frequency divider
#define MPU_CFG_REG 0X1A // Configuration register
#define MPU_GYRO_CFG_REG 0X1B // Gyro configuration register
#define MPU_ACCEL_CFG_REG 0X1C // Accelerometer configuration register
#define MPU_MOTION_DET_REG 0X1F // Motion detection threshold setting register
#define MPU_FIFO_EN_REG 0X23 //FIFO Enable register
#define MPU_I2CMST_CTRL_REG 0X24 //IIC Host control register
#define MPU_I2CSLV0_ADDR_REG 0X25 //IIC Slave 0 Device address register
#define MPU_I2CSLV0_REG 0X26 //IIC Slave 0 Data address register
#define MPU_I2CSLV0_CTRL_REG 0X27 //IIC Slave 0 Control register
#define MPU_I2CSLV1_ADDR_REG 0X28 //IIC Slave 1 Device address register
#define MPU_I2CSLV1_REG 0X29 //IIC Slave 1 Data address register
#define MPU_I2CSLV1_CTRL_REG 0X2A //IIC Slave 1 Control register
#define MPU_I2CSLV2_ADDR_REG 0X2B //IIC Slave 2 Device address register
#define MPU_I2CSLV2_REG 0X2C //IIC Slave 2 Data address register
#define MPU_I2CSLV2_CTRL_REG 0X2D //IIC Slave 2 Control register
#define MPU_I2CSLV3_ADDR_REG 0X2E //IIC Slave 3 Device address register
#define MPU_I2CSLV3_REG 0X2F //IIC Slave 3 Data address register
#define MPU_I2CSLV3_CTRL_REG 0X30 //IIC Slave 3 Control register
#define MPU_I2CSLV4_ADDR_REG 0X31 //IIC Slave 4 Device address register
#define MPU_I2CSLV4_REG 0X32 //IIC Slave 4 Data address register
#define MPU_I2CSLV4_DO_REG 0X33 //IIC Slave 4 Write data register
#define MPU_I2CSLV4_CTRL_REG 0X34 //IIC Slave 4 Control register
#define MPU_I2CSLV4_DI_REG 0X35 //IIC Slave 4 Read data register

#define MPU_I2CMST_STA_REG 0X36 //IIC Host status register
#define MPU_INTBP_CFG_REG 0X37 // interrupt / Bypass setting register
#define MPU_INT_EN_REG 0X38 // Interrupt enable register
#define MPU_INT_STA_REG 0X3A // Interrupt status register

#define MPU_ACCEL_XOUTH_REG 0X3B // Acceleration value ,X Shaft height 8 Bit register
#define MPU_ACCEL_XOUTL_REG 0X3C // Acceleration value ,X Shaft low 8 Bit register
#define MPU_ACCEL_YOUTH_REG 0X3D // Acceleration value ,Y Shaft height 8 Bit register
#define MPU_ACCEL_YOUTL_REG 0X3E // Acceleration value ,Y Shaft low 8 Bit register
#define MPU_ACCEL_ZOUTH_REG 0X3F // Acceleration value ,Z Shaft height 8 Bit register
#define MPU_ACCEL_ZOUTL_REG 0X40 // Acceleration value ,Z Shaft low 8 Bit register

#define MPU_TEMP_OUTH_REG 0X41 // Temperature value high octet register
#define MPU_TEMP_OUTL_REG 0X42 // The temperature value is low 8 Bit register

#define MPU_GYRO_XOUTH_REG 0X43 // Gyro value ,X Shaft height 8 Bit register
#define MPU_GYRO_XOUTL_REG 0X44 // Gyro value ,X Shaft low 8 Bit register
#define MPU_GYRO_YOUTH_REG 0X45 // Gyro value ,Y Shaft height 8 Bit register
#define MPU_GYRO_YOUTL_REG 0X46 // Gyro value ,Y Shaft low 8 Bit register
#define MPU_GYRO_ZOUTH_REG 0X47 // Gyro value ,Z Shaft height 8 Bit register
#define MPU_GYRO_ZOUTL_REG 0X48 // Gyro value ,Z Shaft low 8 Bit register

#define MPU_I2CSLV0_DO_REG 0X63 //IIC Slave 0 Data register
#define MPU_I2CSLV1_DO_REG 0X64 //IIC Slave 1 Data register
#define MPU_I2CSLV2_DO_REG 0X65 //IIC Slave 2 Data register
#define MPU_I2CSLV3_DO_REG 0X66 //IIC Slave 3 Data register

#define MPU_I2CMST_DELAY_REG 0X67 //IIC Host delay management register
#define MPU_SIGPATH_RST_REG 0X68 // Signal channel reset register
#define MPU_MDETECT_CTRL_REG 0X69 // Motion detection control register
#define MPU_USER_CTRL_REG 0X6A // User control register
#define MPU_PWR_MGMT1_REG 0X6B // Power management register 1
#define MPU_PWR_MGMT2_REG 0X6C // Power management register 2
#define MPU_FIFO_CNTH_REG 0X72 //FIFO High eight bits of the count register
#define MPU_FIFO_CNTL_REG 0X73 //FIFO The lower eight bits of the count register
#define MPU_FIFO_RW_REG 0X74 //FIFO Read and write registers
#define MPU_DEVICE_ID_REG 0X75 // device ID register

typedef struct
{

    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float offset_Accel_X;
    float offset_Accel_Y;
    float offset_Accel_Z;

    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    float offset_Gyro_X;
    float offset_Gyro_Y;
    float offset_Gyro_Z;

    float Angle_X;
    float Angle_Y;
    float Angle_Z;

    float Temp;
    float Osgx, Osgy, Osgz;

}IMU_Parameter;


uint8_t HAL_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
uint8_t HAL_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
unsigned char run_self_test(void);
uint8_t MPU6050_Init(void);
void MPU6050_GET_Data(void);
uint8_t mpu_dmp_init(void);
uint8_t mpu_dmp_get_data(IMU_Parameter *IMU_Data);
IMU_Parameter MPU6050_Calibrate(void);
#endif /* INC_MPU6050_H_ */
