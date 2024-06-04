#include "MKL46Z4.h"
#include "i2c.h"
#include "mag.h"
#include "slcd.h"
#include <math.h>

/* Function to initialise the Magnetometer*/
void Magnetometer_Init(void)
// Initialize local variables here
{	
	unsigned char MAG_DEVICE_ID;
	int flag_device_id_match = 0;
	unsigned char control_reg1;
	unsigned char control_reg2;

	/* Configuring the settings for the Magnetometer*/
	/*
	Use 16-bit full-resolution mode for output data (X, Y and Zaxis)
    Set to ‘continuous measurements mode’ with ‘ODR=80Hz’ and ‘OSR=1’
	Set to ACTIVE Mode in ’CTRL_REG1’ register
	*/
	unsigned char DATA_WRITE[] = {0x01,0xA0};
	MAG_DEVICE_ID = I2C_SingleByteRead(MAG_DEVICE_ADDRESS, MAG_DEVICE_ID_REGISTER_ADDRESS);
	
	/*checking the device ID (‘0xC4’)of magnetometer */
	while(MAG_DEVICE_ID != 0xC4)
		/*If the device ID is inaccessible or incorrect, the function should return an error.*/
	{
		SLCD_WriteMsg((unsigned char *)"Err");
	}
	flag_device_id_match = 1;
	control_reg1 = I2C_SingleByteRead(MAG_DEVICE_ADDRESS,MAG_CTRL_REG1);


		/*Writing corresponding data to control registers to configure settings(enable the active mode with ODR=80Hz, OSR=1)*/
	if(control_reg1!=0x01)
		{
			I2C_MultipleByteWrite(MAG_DEVICE_ADDRESS,MAG_CTRL_REG1,2,DATA_WRITE);
		}
}

/* Function to acquire the Magnetometer data and find maximum and minimum values*/
void Magnetometer_Acq()
{	// Initialize local variables here
	int loop_count = 0;
	//int flag_first_data1 =0;
	// Reading data from magnetometer along X, Y, Z axes
	I2C_MultipleByteRead(MAG_DEVICE_ADDRESS, MAG_OUT_X_MSB, 6);
	
	// Combining upper and lower bytes of data for each axis to get complete data along each axis
	for(loop_count=0;loop_count<3;loop_count++)
	{
		MAG_DATA_READ_AXIS[loop_count]=((short int)((DATA_READ[2*loop_count]<<8)| DATA_READ[2*loop_count+1]));
	}
	
	// Find the maximum and minimum values along X, Y, Z axes for calibration purposes
	for(loop_count=0;loop_count<3;loop_count++)
	{
		if((MAG_DATA_MAX_AXIS[loop_count] == 0)&&(MAG_DATA_MIN_AXIS[loop_count]==0))
		{   //Set the first reading to be the maximum and minimum value along each axis
			MAG_DATA_MAX_AXIS[loop_count]=MAG_DATA_READ_AXIS[loop_count];
			MAG_DATA_MIN_AXIS[loop_count]=MAG_DATA_READ_AXIS[loop_count];
			//Finding the Maximum value along each axis
		}
		else if (MAG_DATA_READ_AXIS[loop_count] > MAG_DATA_MAX_AXIS[loop_count])
		{
			MAG_DATA_MAX_AXIS[loop_count] = MAG_DATA_READ_AXIS[loop_count];
			//Finding the Minimum value along each axis
		}
		else if (MAG_DATA_READ_AXIS[loop_count] < MAG_DATA_MIN_AXIS[loop_count])
		{
			MAG_DATA_MIN_AXIS[loop_count] = MAG_DATA_READ_AXIS[loop_count];
		}
	}
}

/* Function to calibrate the Magnetometer for Hard Iron losses*/
void Magnetometer_Cal()
{/*Calibrate the magnetometer for hard-iron effects based on the maximum and minimum X, Y, and Z-axis data
	found during ‘MAG_ACQ’ mode */
for(int loop_count = 0;loop_count < 3; loop_count++)
	{   
		MAG_DATA_AVERAGE_AXIS[loop_count] = (MAG_DATA_MAX_AXIS[loop_count] + MAG_DATA_MIN_AXIS[loop_count])/2;
	}
}

/* Function to acquire the correct Magnetometer data after calibration and also calculate the angle for e-compass 
i.e the angle subtended wrt magnetic north pole*/
void Magnetometer_Run()
{
	// Initialize local variables here
	int loop_count = 0;float TEMP_ANGLE1;short int TEMP_ANGLE;
	
	// Reading data from magnetometer along X, Y, Z axes
	I2C_MultipleByteRead(MAG_DEVICE_ADDRESS, MAG_OUT_X_MSB, 6);
	
	// Combining upper and lower bytes of data for each axis to get complete data along each axis
	for(loop_count=0;loop_count<3;loop_count++)
		{
			MAG_DATA_READ_AXIS[loop_count]=((short int)((DATA_READ[2*loop_count]<<8)|DATA_READ[2*loop_count+1]));
		}
	/*Calculating the calibrated magnetometer data by subtracting the data offset from the raw uncalibrated data*/
	for(loop_count=0;loop_count<3;loop_count++)
	{
		MAG_DATA_HI_CALIBRATED[loop_count] = MAG_DATA_READ_AXIS[loop_count] - MAG_DATA_AVERAGE_AXIS[loop_count];
	}
	/*calculate the angle for e-compass i.e the angle subtended wrt magnetic north pole*/

	if((MAG_DATA_HI_CALIBRATED[1]==0)&&(MAG_DATA_HI_CALIBRATED[0]>0))
	{
		ANGLE = 0;
	}
	else if((MAG_DATA_HI_CALIBRATED[1]==0)&&(MAG_DATA_HI_CALIBRATED[0]<0))
	{
		ANGLE = 180;
	}
	else if(MAG_DATA_HI_CALIBRATED[1]<0)
	{
		ANGLE = 270-(atan(((double)MAG_DATA_HI_CALIBRATED[0]/(double)MAG_DATA_HI_CALIBRATED[1]))*57.29);
	}
	else
	{
		ANGLE = 90-(atan(((double)MAG_DATA_HI_CALIBRATED[0]/(double)MAG_DATA_HI_CALIBRATED[1]))*57.29);
	}
}