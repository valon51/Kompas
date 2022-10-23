/*
 * hmc5833l.h
 *
 *  Created on: Nov 15, 2020
 *      Author: roman
 */

#ifndef INC_HMC5833L_H_
#define INC_HMC5833L_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define HMC5883l_ADDRESS 0x1E << 1
#define HMC5883l_ADD_DATAX_MSB 0x03
#define HMC5883l_ADD_DATAX_LSB 0x04
#define HMC5883l_ADD_DATAZ_MSB 0x05
#define HMC5883l_ADD_DATAZ_LSB 0x06
#define HMC5883l_ADD_DATAY_MSB 0x07
#define HMC5883l_ADD_DATAY_LSB 0x08

/************ 		MODE settings 		*********************/
#define HMC5883_MODE_SINGLE					1		/* Single measurement mode */
#define HMC5883_MODE_CONTINUOUS				0		/* Continuous measurement mode */



/************ 		CONFIG settings 		************/
/* Sampling rates */
#define HMC5883_CONFIG_RATE_0_75_HZ				(0 << 2)	/* 0.75 Hz */
#define HMC5883_CONFIG_RATE_1_5_HZ				(1 << 2)	/* 1.5 Hz */
#define HMC5883_CONFIG_RATE_3_HZ				(2 << 2)	/* 3 Hz */
#define HMC5883_CONFIG_RATE_7_5_HZ				(3 << 2)	/* 7.5 Hz */
#define HMC5883_CONFIG_RATE_15_HZ				(4 << 2)	/* 15 Hz */
#define HMC5883_CONFIG_RATE_30_HZ				(5 << 2)	/* 30 Hz */
#define HMC5883_CONFIG_RATE_75_HZ				(6 << 2)	/* 75 Hz */

/* Averaging */
#define HMC5883_CONFIG_AVG_1					(0 << 5)	/* No averaging */
#define HMC5883_CONFIG_AVG_2					(1 << 5)	/* Average 2 samples */
#define HMC5883_CONFIG_AVG_4					(2 << 5)	/* Average 4 samples */
#define HMC5883_CONFIG_AVG_8					(3 << 5)	/* Average 8 samples */

/* Measurement mode */
#define HMC5883_CONFIG_MODE_NORMAL				(0 << 0)
#define HMC5883_CONFIG_MODE_POS_BIAS			(1 << 0)
#define HMC5883_CONFIG_MODE_NEG_BIAS			(2 << 0)


/********** 		Gain Settings 		**************/
#define HMC5883_GAIN_0_88						(0 << 5)	/* Sensor Field range: +/- 0.88 Gauss */
#define HMC5883_GAIN_1_3						(1 << 5)	/* Sensor Field range: +/- 1.5 Gauss */
#define HMC5883_GAIN_1_9						(2 << 5)	/* Sensor Field range: +/- 1.9 Gauss */
#define HMC5883_GAIN_2_5						(3 << 5)	/* Sensor Field range: +/- 2.5 Gauss */
#define HMC5883_GAIN_4_0						(4 << 5)	/* Sensor Field range: +/- 4.0 Gauss */
#define HMC5883_GAIN_4_7						(5 << 5)	/* Sensor Field range: +/- 4.7 Gauss */
#define HMC5883_GAIN_5_6						(6 << 5)	/* Sensor Field range: +/- 5.6 Gauss */
#define HMC5883_GAIN_8_1						(7 << 5)	/* Sensor Field range: +/- 8.1 Gauss */

/**********  Default Values for Settings *************/
#define HMC5883_DEFAULT_CONFIG					(HMC5883_CONFIG_AVG_1|HMC5883_CONFIG_RATE_15_HZ|HMC5883_CONFIG_MODE_NORMAL)
#define HMC5883_DEFAULT_MODE					(HMC5883_MODE_SINGLE)
#define HMC5883_DEFAULT_GAIN					(HMC5883_GAIN_1_3)

I2C_HandleTypeDef hi2c2;
#define HMC5883L_I2C_PORT		&hi2c2
//---------------------------------------------------------------------------------------------------------------
typedef struct {
int16_t X;
int16_t Y;
int16_t Z;
float scale_x;
float scale_y;
float scale_z;
float offset_x;
float offset_y;
float offset_z;
uint8_t pripojeni;

}HMC5833L_Kompass;
// --------------------------------------------------------------------------------------------------------------

uint8_t          HMC5833L_Ini(uint8_t RegistrA, uint8_t RegistrB, uint8_t Mode);
HMC5833L_Kompass HMC5833L_ReadData (void);
uint8_t HMC_DataReady (void);
HMC5833L_Kompass HMC_Calibration(void);

#endif /* INC_HMC5833L_H_ */
