/*
 * hmc5833l.c
 *
 *  Created on: Nov 15, 2020
 *      Author: roman
 */
#include "hmc5833l.h"
HMC5833L_Kompass compas;

uint8_t HMC5833L_Ini(uint8_t RegistrA, uint8_t RegistrB, uint8_t Mode)
{
	uint8_t data[1];
     // Test pripojeni //
    if(HAL_I2C_IsDeviceReady(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 1, 1000) != HAL_OK)
        {
    	    return 0; //Chyba pripojeni //
        }
//.......................................................................................................
    // Konfigurace HMC5833 //
    data[0] = RegistrA;
	if(HAL_I2C_Mem_Write(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x00 , 1, data  , 1, 1000) != HAL_OK) // konfigurace registru A
	    {
	        return 0; // Chyba prenosu //
	    }
	data[0] = RegistrB;
	if(HAL_I2C_Mem_Write(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x01 , 1, data , 1, 1000)  != HAL_OK) // konfigurace registru B
	    {
		    return 0; // Chyba prenosu //
	    }
	data[0] = Mode;
	if(HAL_I2C_Mem_Write(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x02 , 1, data , 1, 1000) != HAL_OK) // konfigurace modu
		{
		    return 0; // Chyba prenosu //
		}
//.......................................................................................................
	return 1; // Inicializace proběhla uspěsně //
}
// Vycteni dat z chipu //
HMC5833L_Kompass HMC5833L_ReadData (void)
{
uint8_t dataXM[1];
uint8_t dataXL[1];
uint8_t dataYM[1];
uint8_t dataYL[1];
uint8_t dataZM[1];
uint8_t dataZL[1];

    if(HAL_I2C_Mem_Read(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x03 , 1, dataXM, 1, 1000) != HAL_OK)
	     {
	       compas.pripojeni = 0;
	     }
    if(HAL_I2C_Mem_Read(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x04 , 1, dataXL, 1, 1000) != HAL_OK)
   	     {
   	       compas.pripojeni = 0;
   	     }
    if(HAL_I2C_Mem_Read(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x05 , 1, dataZM, 1, 1000) != HAL_OK)
   	     {
   	       compas.pripojeni = 0;
   	     }
    if(HAL_I2C_Mem_Read(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x06 , 1, dataZL, 1, 1000) != HAL_OK)
   	     {
   	       compas.pripojeni = 0;
   	     }
    if(HAL_I2C_Mem_Read(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x07 , 1, dataYM, 1, 1000) != HAL_OK)
   	     {
   	       compas.pripojeni = 0;
   	     }
    if(HAL_I2C_Mem_Read(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x08 , 1, dataYL, 1, 1000) != HAL_OK)
   	     {
   	       compas.pripojeni = 0;
   	     }

    	 compas.X         = ((int16_t)dataXM[0] << 8) | dataXL[0];
         compas.Z         = ((int16_t)dataZM[0] << 8) | dataZL[0];
         compas.Y         = ((int16_t)dataYM[0] << 8) | dataYL[0];
         compas.pripojeni = 1;


 return compas;
}
//........................................................................................................
// Data pripravena ke cteni?//
//........................................................................................................
uint8_t HMC_DataReady (void)
{
	uint8_t data[1];

    if(HAL_I2C_Mem_Read(HMC5883L_I2C_PORT, HMC5883l_ADDRESS, 0x09 , 1, (uint8_t*)data, 1, 1000) != HAL_OK)
	    {
	       return 0;
	    }
    if((data[0] == 0x01)||(data[0] == 0x11))
    {
    	   return 1;
    }
           return 0;
}
//........................................................................................................
// Kalibrace
//........................................................................................................
HMC5833L_Kompass HMC_Calibration()
{
	if(HMC5833L_Ini(HMC5883_CONFIG_AVG_8|HMC5883_CONFIG_RATE_75_HZ|HMC5883_CONFIG_MODE_NORMAL, HMC5883_GAIN_2_5, HMC5883_MODE_CONTINUOUS) != 1)
	    {
	  		while(1);
	  	}

 uint16_t ii = 0, sample_count = 0;
 float mag_scale[3] = {0, 0, 0};
 int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

 HAL_Delay(4000);

 // shoot for 30 seconds of mag data

 sample_count = 2460; //at 75 Hz ODR, new mag data is available every 13 ms
 for(ii = 0; ii < sample_count; ii++) {
	 // Read the mag data
 while(HMC_DataReady () == 0){;}
	 compas =  HMC5833L_ReadData ();
	 mag_temp[0] = compas.X;
	 mag_temp[1] = compas.Y;
	 mag_temp[2] = compas.Z;

 for (int jj = 0; jj < 3; jj++) {
 if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
 if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
 }
 HAL_Delay(14); // at 75 Hz ODR, new mag data is available every 13 ms
 }
 // Get hard iron correction
 compas.offset_x  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
 compas.offset_y  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
 compas.offset_z  = (mag_max[2] + mag_min[2])/2;  // get average y mag bias in counts

 // Get soft iron correction estimate

 mag_scale[0] = (mag_max[0] - mag_min[0])/2; // get average x axis max chord length in counts
 mag_scale[1] = (mag_max[1] - mag_min[1])/2; // get average y axis max chord length in counts
 mag_scale[2] = (mag_max[2] - mag_min[2])/2; // get average z axis max chord length in counts

 float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
 avg_rad /= 3.0;

 compas.scale_x = avg_rad/((float)mag_scale[0]);
 compas.scale_y = avg_rad/((float)mag_scale[1]);
 compas.scale_z = avg_rad/((float)mag_scale[2]);
 return compas;

}
