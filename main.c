/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "hmc5833l.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_Touch_cfg.h"
#include "ILI9341_Touch_Driver.h"
#include "obrazek.h"
#include "ILI9341_GFX.h"
#include "fonts.h"
#include "TinyGPS.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI              3.141592653f
#define true 1
#define false 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
IMU_Parameter MPU6050;
//Compass vysledek;
HMC5833L_Kompass ergebniss;
EEPROM_DATA bunky;
volatile uint8_t screen = 1, blokovani_mimo_menu = 0;

uint8_t priznak = 0;
char temp_Buffer_text[70];
//
float teplota, rychlost, distance, nadvyska;
uint8_t len;
volatile uint8_t misto = 10;
uint8_t dataRX[1];
float flat, flon;
unsigned long age;
// EEPROM proměné
unsigned long velikost=sizeof(float);
//
#define DELKA velikost *6
uint16_t Adress1 = 0;
//--------------------------------
float   self[6];
double  azimuth;
float   heading;
float   pitch;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable.
float   roll;                                    //Calculate the traveled roll angle and add this to the angle_roll variable.
float  Y_h, X_h;
float mag_x,mag_y,mag_z;
long   accoffset[3], gyrooffset[3];
int16_t x_old = 50, y_old = 140;
//--------------------------------
int16_t x;
int16_t y;
uint32_t _millis = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float getBearing(int azimuth){
	unsigned long a = azimuth / 22.5;
	unsigned long r = a - (int)a;
	byte sexdec = 0;
	sexdec = ( r >= .5 ) ? ceil(a) : floor(a);
	return sexdec;
}
//........................................................................................
//The following subrouting calculates the smallest difference between two heading values.
//........................................................................................
float course_deviation(float course_b, float course_c)
{
  float base_course_mirrored,actual_course_mirrored, course_a;
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}
//......................................................................................
//         Výpo ?et Azimuthu
//......................................................................................
void Azimuth_calculation(void)
{
//Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
//Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
//pitch = atan2((double)-acc_x, sqrt((double)acc_z*(double)acc_z + (double)acc_y*(double)acc_y));
//roll = atan2((double)acc_y, sqrt((double)acc_z*(double)acc_z  + (double)acc_x*(double)acc_x));
//.......................................................................................................................................................................
X_h = (double)mag_x*cos(pitch * -0.0174533) + (double)mag_y*sin(roll* 0.0174533)*sin(pitch * -0.0174533) - (double)mag_z*cos(roll * 0.0174533)*sin(pitch * -0.0174533);
Y_h = (double)mag_y*cos(roll * 0.0174533) - (double)mag_z*sin(roll * 0.0174533);
//.......................................................................................................................................................................
//X_h = (double)mag_x*cos(pitch) + (double)mag_y*sin(roll)*sin(pitch) + (double)mag_z*cos(roll)*sin(pitch);
//Y_h = (double)mag_y*cos(roll) - (double)mag_z*sin(roll);
//Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
//Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.

if (Y_h < 0)azimuth = 180 + (180 + ((atan2(Y_h, X_h)) * (180 / 3.14)));
else azimuth = (atan2(Y_h, X_h)) * (180 / 3.14);

azimuth += 4.03;                         //Add the declination to the magnetic compass heading to get the geographic north.
if (azimuth < 0) azimuth += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
else if (azimuth >= 360) azimuth -= 360; //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
}
//............................................................................
//                          Čtení dat z kompasu a mpu6050
//............................................................................

void Read_Kompas_Daten(void)
{
  while(HMC_DataReady () == 0){;}
  ergebniss =  HMC5833L_ReadData ();

  mag_x = (ergebniss.X - bunky.OFF_x) * bunky.SC_x;
  mag_y = (ergebniss.Y - bunky.OFF_y) * bunky.SC_y;
  mag_z = (ergebniss.Z - bunky.OFF_z) * bunky.SC_z;
//.................................................................
  while(mpu_dmp_get_data(&MPU6050) != 0){;}
//.................................................................
  pitch = MPU6050.Angle_X;
  roll = MPU6050.Angle_Y;
}
//............................................................................
//                         Zápis  ?ást do EEPROM
//............................................................................
void EEPROM_Write(float Offset_xx, float Offset_yy, float Offset_zz, float Scale_xx, float Scale_yy, float Scale_zz)
{
	uint8_t TXBUFFER[DELKA+1];	// alokace bufferu v pameti
	// Plmime do bufferu postupne vsechny hodnoty s posunem o velikost double
    memcpy((float *) TXBUFFER,              &Offset_xx,velikost);
    memcpy((float *)(TXBUFFER+velikost),    &Offset_yy,velikost);
    memcpy((float *)(TXBUFFER+(2*velikost)),&Offset_zz,velikost);
    memcpy((float *)(TXBUFFER+(3*velikost)),&Scale_xx,velikost);
    memcpy((float *)(TXBUFFER+(4*velikost)),&Scale_yy,velikost);
    memcpy((float *)(TXBUFFER+(5*velikost)),&Scale_zz,velikost);

if(HAL_I2C_IsDeviceReady(&hi2c2, 0xA0 , 1, 10) == HAL_OK)
  {
	  if (HAL_I2C_Mem_Write(&hi2c2,  0xA0, Adress1, 2, TXBUFFER, velikost*6, 5000) != HAL_OK)
	  {
		  ILI9341_Draw_Text("CHYBA ZAPIS EEPROM", FONT4, 8, (misto+= 20), RED, BLACK, 0);
		  while (1);
	  }
	      ILI9341_Draw_Text("OK WRITE EEPROM", FONT4, 8, (misto+= 20), WHITE, BLACK, 0);
  }
}
//...............................................................................................................
//                                      Číst z EEPROM
//...............................................................................................................
EEPROM_DATA EEPROM_Read(void)
{
	uint8_t RXBUFFER[DELKA+1];	// alokace bufferu v pameti
	if(HAL_I2C_IsDeviceReady(&hi2c2, 0xA0 , 1, 10) == HAL_OK)
	  {
		if(HAL_I2C_Mem_Read(&hi2c2, 0xA0, Adress1, 2, RXBUFFER , velikost*6 , 5000) != HAL_OK)
		{
			  ILI9341_Draw_Text("CHYBA CTENI EEPROM", FONT4, 8, (misto+= 20), RED, BLACK, 0);
			  while (1);
		}
	          ILI9341_Draw_Text("OK READ EEPROM", FONT4, 8, (misto+= 20), WHITE, BLACK, 0);

	          memcpy(&bunky.OFF_x, (float *)RXBUFFER,		        velikost);
	          memcpy(&bunky.OFF_y, (float *)(RXBUFFER+velikost),	    velikost);
	          memcpy(&bunky.OFF_z, (float *)(RXBUFFER+(2*velikost)), velikost);
	          memcpy(&bunky.SC_x, (float *)(RXBUFFER+(3*velikost)),velikost);
	          memcpy(&bunky.SC_y, (float *)(RXBUFFER+(4*velikost)),velikost);
	          memcpy(&bunky.SC_z, (float *)(RXBUFFER+(5*velikost)),velikost);

	          HAL_Delay(2000);
	          ILI9341_Fill_Screen(BLACK);
	     	  sprintf(temp_Buffer_text, "%0.1f %0.1f %0.1f %0.5f %0.5f %0.5f", bunky.OFF_x, bunky.OFF_y, bunky.OFF_z,  bunky.SC_x, bunky.SC_y, bunky.SC_z);
	     	  ILI9341_Draw_Text(temp_Buffer_text, FONT1, 8, 10, WHITE, BLACK, 0);

	 }        return bunky;
}
//...................................................................................
// po ?ítání délky řetězce
//...................................................................................
uint8_t length(uint8_t *data)
{
	uint8_t i;
	for(i=0; data[i]!='\0'; i++)
	{
	;
	}
	return i;
}
//..............................................................................................................
// Příjem znaků z uart
//..............................................................................................................
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
     if (huart->Instance == USART6){
    	 if(encode((char)dataRX[0])) {
          nadvyska = f_altitude();
          rychlost = f_speed_kmph();
    	  f_get_position(&flat, &flon, &age);
    	 }
	 HAL_UART_Receive_IT(&huart6, dataRX, sizeof(dataRX));
 }
}
//..............................................................................................................
// Scaner I2C linky
//..............................................................................................................
void I2C_Scanner()
{
    uint8_t pozice = misto + 20;
	for(uint8_t adresse = 0; adresse < 127; adresse ++ ) {
     if(HAL_I2C_IsDeviceReady(&hi2c2, adresse << 1, 1, 10) == HAL_OK)
      {
    	 sprintf(temp_Buffer_text, "I2C Scan = %2Xh", adresse);
    	 ILI9341_Draw_Text(temp_Buffer_text, FONT1, 8, pozice, WHITE, BLACK, 0);
    	 pozice += 12;

      }
	}
	misto = pozice-14;
}
//................................................................................................................
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  ILI9341_Init();
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
  ILI9341_Fill_Screen(BLACK);
  // ......Vypnutí přerušení od Uart6(GPS).................
  NVIC_DisableIRQ(USART6_IRQn);
  NVIC_DisableIRQ(EXTI1_IRQn);
  //.......................................................
  //...........................................................................................
  ILI9341_Draw_Text("CAL-ACC", FONT4, 8,misto, WHITE, BLACK, 0);
  //..........................................................................................
  get_st_biases(gyrooffset, accoffset, 1);
  if(mpu_set_accel_bias(accoffset) !=0)
  {
	   ILI9341_Draw_Text("ERROR CAL-ACC", FONT4, 8, (misto += 20), RED, BLACK, 0);
	   while (1);
  }
  //MPU6050_Calibrate();   // kalibrace acce

  //...........................................................................................
  ILI9341_Draw_Text("INI-MPU6050", FONT4, 8, (misto += 20), WHITE, BLACK, 0);
   if( MPU6050_Init() == 1)
     {
	   ILI9341_Draw_Text("ERROR INI-MPU6050", FONT4, 8, (misto += 20), RED, BLACK, 0);
	   while (1);
     }
  //...........................................................................................
  ILI9341_Draw_Text("INI-DMP-MPU6050", FONT4, 8, (misto += 20), WHITE, BLACK, 0);
  if(mpu_dmp_init() != 0)
     {
	  ILI9341_Draw_Text("ERROR INI-MPU.DMP", FONT4, 8, (misto += 20), RED, BLACK, 0);
	  while (1);
     }
  //.............................................................................................................................................
  I2C_Scanner();
  //.............................................................................................................................................
  ILI9341_Draw_Text("INI-HMC588L", FONT4, 8, (misto += 20), WHITE, BLACK, 0);
  if(HMC5833L_Ini(HMC5883_CONFIG_AVG_8|HMC5883_CONFIG_RATE_75_HZ|HMC5883_CONFIG_MODE_NORMAL, HMC5883_GAIN_2_5, HMC5883_MODE_CONTINUOUS) != 1) {
	  ILI9341_Draw_Text("ERROR INI-HMC", FONT4, 8, (misto += 20), RED, BLACK, 0);
  		while(1);
  	}
 GPIOC->BSRR = GPIO_BSRR_BS13; // informa ?ní ledka..
 HAL_UART_Receive_IT(&huart6, dataRX, sizeof(dataRX));
 //----------------------------------------------------------------------------------------------
 EEPROM_Read();
 HAL_Delay(3000);
 // .......Zapnutí přerušení od Uart6(GPS)............
 NVIC_EnableIRQ(USART6_IRQn);
 NVIC_EnableIRQ(EXTI1_IRQn);
 //...................................................
 ILI9341_Fill_Screen(BLACK);
  //---------------------------------------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

 switch (screen){                                // Menu System
 case 1:
	     if(priznak != 1){
 //.....................................................................
          ILI9341_Fill_Screen(BLACK);
          ILI9341_Draw_Hollow_Rectangle_Coord(1, 1, 319, 239, WHITE);
          ILI9341_Draw_Text("Roll:", FONT4, 8, 5, RED, BLACK, 0);
          ILI9341_Draw_Text("Pitch:", FONT4, 8, 23, RED, BLACK, 0);
//......................................................................
          ILI9341_Draw_Filled_Circle(104, 140, 96, BLUE);
          ILI9341_Draw_Filled_Circle(104, 140, 92, DARKCYAN);
//......................................................................
          ILI9341_Draw_Filled_Rectangle_Coord(209, 7, 315, 60, YELLOW);
          ILI9341_Draw_Hollow_Rectangle_Coord(207, 5, 316, 61, WHITE);
          ILI9341_Draw_Text("Calibration", FONT4, 219, 26, BLACK, YELLOW, 0);
          ILI9341_Draw_Filled_Rectangle_Coord(209, 65, 315, 119, GREEN);
          ILI9341_Draw_Hollow_Rectangle_Coord(207, 63, 316, 120, WHITE);
          ILI9341_Draw_Text("Autopilot", FONT4, 228, 83, BLACK, GREEN, 0);
          ILI9341_Draw_Filled_Rectangle_Coord(209, 124, 315, 178, BLUE);
          ILI9341_Draw_Hollow_Rectangle_Coord(207, 122, 316, 179, WHITE);
          ILI9341_Draw_Text("GPS", FONT4, 244, 142, BLACK, BLUE, 0);
          ILI9341_Draw_Filled_Rectangle_Coord(209, 183, 315, 234, RED);
          ILI9341_Draw_Hollow_Rectangle_Coord(207, 181, 316, 235, WHITE);
          priznak = 1;
          blokovani_mimo_menu = 0;
	   }
          Read_Kompas_Daten();
          Azimuth_calculation();

          x = 104 + 80 * sin(atan2(Y_h, X_h));
          y = 140 - 80 * cos(atan2(Y_h, X_h));
//----------------------------------------------------------------------------------------------------------------
          ILI9341_Draw_Text("  ", FONT6, 95, 118, BLACK, DARKCYAN, 1);
	      sprintf(temp_Buffer_text, "%d",((uint16_t)azimuth));
		  ILI9341_Draw_Text(temp_Buffer_text, FONT6, 65, 118, BLACK, DARKCYAN, 1);

		  sprintf(temp_Buffer_text, "%0.1f ", roll);
		  ILI9341_Draw_Text(temp_Buffer_text, FONT4, 55, 5, RED, BLACK, 1);

		  sprintf(temp_Buffer_text, "%0.1f ", pitch);
		  ILI9341_Draw_Text(temp_Buffer_text, FONT4, 55, 23, RED, BLACK, 1);

		  if((x != x_old) || (y != y_old)) {
		  ILI9341_Draw_Filled_Circle(x_old, y_old, 11, DARKCYAN);
		  ILI9341_Draw_Filled_Circle(x, y, 11, RED);
		  }
			x_old = x;
			y_old = y;
			break;
 case 2:
	     if(priznak != 2){
	     ILI9341_Fill_Screen(BLACK);
	     ILI9341_Draw_Hollow_Rectangle_Coord(1, 1, 319, 239, WHITE);
	     ILI9341_Draw_Hollow_Rectangle_Coord(68, 10, 256, 40, WHITE);
	     ILI9341_Draw_Text("AUTOPILOT", FONT8, 73, 15, YELLOW, BLACK, 0);
	     ILI9341_Draw_Text("To Uart send date  ->>>>>>", FONT4, 5, 50, RED, BLACK, 0);
	     //.......................................................................................................
	     //         Zobrazí kalibrační hodnoty pro kompas
	     //.........................................................................................................................................................
	     ILI9341_Draw_Text("CAL-Values: ", FONT4, 5, 72, RED, BLACK, 0);
    	 sprintf(temp_Buffer_text, "%d %d %d %0.3f %0.3f %0.3f", (int16_t)bunky.OFF_x, (int16_t)bunky.OFF_y, (int16_t)bunky.OFF_z,  bunky.SC_x, bunky.SC_y, bunky.SC_z);
    	 ILI9341_Draw_Text(temp_Buffer_text, FONT1, 115, 78, YELLOW, BLACK, 0);
    	 sprintf(temp_Buffer_text, "%ld %ld %ld", accoffset[0], accoffset[1], accoffset[2]);
    	 ILI9341_Draw_Text(temp_Buffer_text, FONT1, 115, 88, YELLOW, BLACK, 0);
    	 sprintf(temp_Buffer_text, "%ld %ld %ld", gyrooffset[0], gyrooffset[1], gyrooffset[2]);
    	 ILI9341_Draw_Text(temp_Buffer_text, FONT1, 115, 98, YELLOW, BLACK, 0);
    	 //.........................................................................................................................................................
		 ILI9341_Draw_Filled_Rectangle_Coord(210, 186, 315, 235, BLUE);
		 ILI9341_Draw_Hollow_Rectangle_Coord(208, 184, 316, 236, WHITE);
		 ILI9341_Draw_Text("Menu", FONT4, 241, 203, BLACK, BLACK, 0);
	     priznak = 2;
	     }
	     Read_Kompas_Daten();
	     Azimuth_calculation();

	     sprintf(temp_Buffer_text, " Roll: %0.1f | Pitch: %0.1f ||",roll,pitch);
		 len = length((unsigned char*)temp_Buffer_text);
		 HAL_UART_Transmit(&huart1,(unsigned char*)temp_Buffer_text,len, 10);

		 sprintf(temp_Buffer_text," Heading = %0.2f ||\n",azimuth);
		 len = length((unsigned char*)temp_Buffer_text);
		 HAL_UART_Transmit(&huart1,(unsigned char*)temp_Buffer_text,len, 10);
		 blokovani_mimo_menu = 1;
         break;

 case 3:
	     ILI9341_Fill_Screen(BLACK);
	     ILI9341_Draw_Hollow_Rectangle_Coord(1, 1, 319, 239, WHITE);
	     ILI9341_Draw_Text("CALIBRATION", FONT8, 50, 15, YELLOW, BLACK, 0);
	     ILI9341_Draw_Hollow_Rectangle_Coord(46, 10, 270, 40, WHITE);
	     // ......Vypnutí přerušení od Uart6(GPS).................
	     NVIC_DisableIRQ(USART6_IRQn);
	     NVIC_DisableIRQ(EXTI1_IRQn);
	     //.......................................................
         HAL_Delay(3000);
	     ergebniss = HMC_Calibration();
	     sprintf(temp_Buffer_text, "SX = %f", ergebniss.scale_x );
	     ILI9341_Draw_Text(temp_Buffer_text, FONT4, 5, 45, RED, BLACK, 1);

	     sprintf(temp_Buffer_text, "SY = %f", ergebniss.scale_y );
	     ILI9341_Draw_Text(temp_Buffer_text, FONT4, 5, 65, RED, BLACK, 1);

	     sprintf(temp_Buffer_text, "SZ = %f", ergebniss.scale_z );
	     ILI9341_Draw_Text(temp_Buffer_text, FONT4, 5, 85, RED, BLACK, 1);

	     sprintf(temp_Buffer_text, "OX = %0.1f", ergebniss.offset_x );
	     ILI9341_Draw_Text(temp_Buffer_text, FONT4, 5, 105, WHITE, BLACK, 1);

	     sprintf(temp_Buffer_text, "OY = %0.1f", ergebniss.offset_y );
	     ILI9341_Draw_Text(temp_Buffer_text, FONT4, 5, 125, WHITE, BLACK, 1);

	     sprintf(temp_Buffer_text, "OZ = %0.1f", ergebniss.offset_z );
	     ILI9341_Draw_Text(temp_Buffer_text, FONT4, 5, 145, WHITE, BLACK, 1);
	     ILI9341_Draw_Text("Value save to EEPROM", FONT4, 5, 165, RED, BLACK, 0);
	     misto = 165;
	     EEPROM_Write(ergebniss.offset_x, ergebniss.offset_y, ergebniss.offset_z, ergebniss.scale_x, ergebniss.scale_y, ergebniss.scale_z);
	     //  Ulozeni do RAM paměti hned, aby se nemuselo restartovat................
		 bunky.SC_x = ergebniss.scale_x;
	     bunky.SC_y = ergebniss.scale_y;
	     bunky.SC_z = ergebniss.scale_z;
	     //...................................................
	     bunky.OFF_x = ergebniss.offset_x;
	     bunky.OFF_y = ergebniss.offset_y;
	     bunky.OFF_z = ergebniss.offset_z;
	     // .......Zapnutí přerušení od Uart6(GPS)............
	     NVIC_EnableIRQ(USART6_IRQn);
	     NVIC_EnableIRQ(EXTI1_IRQn);
	     //...................................................
	     HAL_Delay(5000);
	     ILI9341_Draw_Text("Calibration is SAVE", FONT4, 5, (misto+= 20), RED, BLACK, 0);
	     HAL_Delay(5000);
	     misto = 10;
	     screen = 1;
	     priznak = 0;
         break;
 case 4:
	    if(priznak != 2){
        ILI9341_Fill_Screen(BLACK);
        ILI9341_Draw_Hollow_Rectangle_Coord(1, 1, 319, 239, WHITE);
        ILI9341_Draw_Hollow_Rectangle_Coord(61, 10, 262, 40, WHITE);
        ILI9341_Draw_Text("GPS MODUL", FONT8, 65, 15, YELLOW, BLACK, 0);
	    ILI9341_Draw_Filled_Rectangle_Coord(210, 186, 315, 235, BLUE);
	    ILI9341_Draw_Hollow_Rectangle_Coord(208, 184, 316, 236, WHITE);
	    ILI9341_Draw_Text("Menu", FONT4, 241, 203, BLACK, BLACK, 0);
        priznak = 2;
	    }
	    blokovani_mimo_menu = 1;
		sprintf(temp_Buffer_text, "%0.6f", flat);
		ILI9341_Draw_Text(temp_Buffer_text, FONT3, 10, 100,  RED, BLACK, 1);

		sprintf(temp_Buffer_text, "%0.6f ",flon);
		ILI9341_Draw_Text(temp_Buffer_text, FONT3, 10, 130,  BLUE, BLACK, 1);

		sprintf(temp_Buffer_text, "%2.1f  ", rychlost);
		ILI9341_Draw_Text(temp_Buffer_text, FONT11, 10, 42, WHITE, BLACK, 1);

		sprintf(temp_Buffer_text, "%2.1f  ", nadvyska);
		ILI9341_Draw_Text(temp_Buffer_text, FONT3, 10, 160, PINK, BLACK, 1);
	    break;
     }
	  //------------------------------------------------------------
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|DC_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_Touch_GPIO_Port, CS_Touch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Toutch_Pin */
  GPIO_InitStruct.Pin = IRQ_Toutch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_Toutch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = RST_Pin|DC_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Touch_Pin */
  GPIO_InitStruct.Pin = CS_Touch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_Touch_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
