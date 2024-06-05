#include "MKL46Z4.h"
#include "slcd.h"
#include "mag.h"
#include "i2c.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define PIN(x)                 (1 << x)
// Green LED is connected to PTD5 and define functions for Green LED
#define GREEN_LED             (5)
#define GREEN_LED_ON()       PTD->PCOR |= PIN(GREEN_LED) ;// Define function to turn Green LED ON
#define GREEN_LED_OFF()      PTD->PSOR |= PIN(GREEN_LED) ;// Define function to turn Green LED OFF
#define GREEN_LED_TOGGLE()   PTD->PTOR |= PIN(GREEN_LED) ;// Define function to TOGGLE Green LED

// Red LED is connected to PTE29 and define functions for Red LED
#define RED_LED             (29)
#define RED_LED_ON()       PTE->PCOR |= PIN(RED_LED) ; //Define function to turn Red LED ON
#define RED_LED_OFF()      PTE->PSOR |= PIN(RED_LED) ; //Define function to turn Red LED OFF
#define RED_LED_TOGGLE()   PTE->PTOR |= PIN(RED_LED) ; //Define function to TOGGLE Red LED

// SW1 is connected to PTC3
#define SW1             (3)
// SW2 is connected to PTC12
#define SW2             (12)
// This is used to set the internal pull up resistor of the corresponding pin.
// Setting PE = 1 and PS = 1 for the corresponding pin to which the switches are connected.
#define ENABLE_PULLUP_RESISTOR  (3)

// Defining variables to be used for acquiring and processing Magnetometer Data
unsigned char DATA_READ[6];
short int MAG_DATA_READ_AXIS[3];
short int MAG_DATA_MAX_AXIS[3];
short int MAG_DATA_MIN_AXIS[3];
short int MAG_DATA_AVERAGE_AXIS[3];
short int MAG_DATA_HI_CALIBRATED[3];
unsigned char DR_STATUS_DATA;
short int ANGLE;

typedef enum {
	STOP,
	RUN,
	MAG_ACQ,
	MAG_CAL,
	ACC_CAL
} enumECompassOperationState;

enumECompassOperationState enumECompassState = STOP;

bool bSW3Pressed = false;
bool bSW1Pressed = false;

bool bIsTimerExpired = false;

unsigned char    ucSecond = 0;
unsigned char    ucHundredsMilliSecond = 0;
unsigned char    ucMinute = 0;
unsigned short   usTimeElapsed = 0;

unsigned char    ucaryLCDMsg[5] = "";
/* Define and initialise variables*/
// --------------------------------------------------------------------
// Defining variables to be used for calculation of TPM0 MOD Count for Timer Overflow every 0.1 second
int prescalar;
int prescalar_factor;//to be
int resolution;
int original_clock_frequency; //in kHz
int new_clock_frequency ;
int timer_overflow_count;
int reset_count;

// Defining variables to be used for acquiring and processing Accelerometer Data
unsigned char DR_STATUS_DATA_ACC;

// Defining variables to be used as flags
int flag_first_data = 0;
int flag_initialisation = 0;
int flag_first_data_acquire =0;
int flag_first_data_run =0;
int flag_first_data1 =0;

void LED_Init(void)
{
	/*   Turn on clock to Port D and E module respectively*/
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	/*Set configuration for Green LED (Output Mode)*/
	/*Set the PTD5 pin multiplexer to GPIO mode*/
	PORTD->PCR[GREEN_LED] = PORT_PCR_MUX(1) ;
	/*Set the initial output state to low*/
	PTD->PCOR |= PIN(GREEN_LED) ;
	/*Set the pins direction to output*/
	PTD->PDDR |= PIN(GREEN_LED);
	/*Set configuration for Red LED (Output Mode)*/
	/*Set the PTE29 pin multiplexer to GPIO mode*/
	PORTE->PCR[RED_LED] = PORT_PCR_MUX(1) ;
	/*Set the initial output state to low*/
	PTE->PCOR |= PIN(RED_LED) ;
	/*Set the pins direction to output*/
	PTE->PDDR |= PIN(RED_LED);
}

/* Function to initialise Switches*/
void SWITCH_Init(void)
{
	/*   Turn on clock to Port C module respectively*/
		SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	/*Set configuration for Switch 1 (Input Mode)*/
	
	/*PTC3 pin set to GPIO mode, PE(bit0) and PS(bit1) set to 1 by ENABLE_PULLUP_RESISTOR,
	 * PTC_IRQC(Interrupt Configuration) set to 1010 to detect interrupt on falling edge  */
		PORTC->PCR[SW1] |= PORT_PCR_MUX(1) | ENABLE_PULLUP_RESISTOR | PIN(17) | PIN(19);
	
	/*Set configuration for Switch 2 (Input Mode)*/
	/*PTC3 pin set to GPIO mode, PE(bit0) and PS(bit1) set to 1 by ENABLE_PULLUP_RESISTOR,
	 * PTC_IRQC(Interrupt Configuration) set to 1010 to detect interrupt on falling edge  */
		PORTC->PCR[SW2] |= PORT_PCR_MUX(1) | ENABLE_PULLUP_RESISTOR | PIN(17) | PIN(19);

	/*Set the pins direction to input (Set the corresponding bit to 0) */
	PTC->PDDR &= (~PIN(SW1))&(~PIN(SW2));
}

/* Function to initialise Timer*/
void TIMER_Init(void)
{
	/*Calculation of the MOD Value or upper limit for the TPM0 Counter to trigger a Timer Overflow Interrupt:*/
	/*User can change the prescalar factor and resolution if needed and the upper count limit will be calculated automatically*/
	//TPM counter increments on every TPM counter clock
	//Prescale Factor set to 6, It divides clock frequency by 2^6 = 64.New Frequency = 8MHz/64 = 0.125 MHz = 125kHz
    //Time period = 1/New Frequency.
	//timer_overflow_count = resolution/time period = resolution * original_clock_frequency / prescalar = 12500
	
	prescalar_factor = 6;//user specified prescalar factor
	prescalar = 1<<(prescalar_factor);//Left shift to find the prescalar
	resolution = 100; //The lowest count in milliseconds (Can be modified by user)
	original_clock_frequency = 8000; //The Frequency of the onboard crystal oscillator in kHz
	timer_overflow_count = resolution * original_clock_frequency / prescalar;//Calculating the MOD value after which the Timer Overflow occurs and an interrupt is triggered
	reset_count = 0;//Reset value for TPM0
	
	/*Connect Clock Source to the TPM0 */
	OSC0->CR |= PIN(7); //Enabling the external reference clock through ERCLKEN bit in OSC0 Control Register
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; //Enable TPM0 clock using System Clock Gating Control Register 6
	SIM->SOPT2 |= PIN(25); //Selecting the OSCERCLK as source of TPM0 by setting TPMSRC to 10 in System Options Register 2
	
	/*Resetting TPM0 Count Values and Specifying the upper limit(MOD value) for the TPM0 Counter */
	TPM0->CNT = reset_count; //The CNT register of TPM0 is reset to 0
	TPM0->MOD = timer_overflow_count; //Setting the maximum count value. TPM0 counter reaches this modulo value and increments, the overflow flag (TOF) is set.

	/*Status and Control register configuration for the TPM0 */
	TPM0->SC &=(~(TPM_SC_CPWMS_MASK));//CPWMS is set to 0 so that TPM0 Counter operates in upcounting mode
	TPM0->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(prescalar_factor);//CMOD is set to 01 so that TPM counter increments on every TPM counter clock and PS is set to user defined prescalar value
	TPM0->SC |= TPM_SC_TOIE_MASK; //Enabling the timer to raise an interrupt on overflow every hundred-millisecond
	TPM0->CONF |= PIN(7) | PIN(6); //Setting DBGMODE to be 11  in TPM0_CONF so that TPM counter continues in debug mode
}

/* Function for Timer Interrupt Handler*/
void TPM0_IRQHandler(void)
{
/*Update the Timer Expired state variable and Clearing the Timer Overflow Flag for the TPM0 */
	bIsTimerExpired = true;//This indicates that 1 unit lowest count/resolution(hundred-millisecond) has reached
	TPM0->SC |= TPM_SC_TOF_MASK; //Clears the overflow flag TOF so that clock can start counting the next lowest count or resolution (hundred-millisecond)
}

/* Function for PORT C and PORT D Interrupt Handler*/
void PORTC_PORTD_IRQHandler(void)
{
	int SW1_Status, SW2_Status; //Indicates the status of SW1 and SW2 (if they are pressed or not)

	/*Read Interrupt Status Flag for Switch 1 and Switch 2 and storing the status after right shifting to indicate a 0 or 1 value*/
	SW1_Status=(PORTC->PCR[SW1] & PIN(24))>>24;
	SW2_Status=(PORTC->PCR[SW2] & PIN(24))>>24;
	
	/*Update the corresponding State variables and Clearing Interrupt Status Flags for the corresponding switches for detecting future interrupts */
	if (SW1_Status==1)
	{
		bSW1Pressed=1;      //Update the state variable for Reset Switch (Switch 1)
		PORTC->PCR[SW1] |= PIN(24); //Clearing the Interrupt Status Flag for Switch 1
	}
	if (SW2_Status==1)
	{
		bSW3Pressed=1;  //Update the state variable for Start/Stop Toggle Switch (Switch 2)
		PORTC->PCR[SW2] |= PIN(24); //Clearing the Interrupt Status Flag for Switch 2
	}
}

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
int main(void)
{
    __disable_irq();

    /* Peripheral initialization */
    SLCD_Init();
    // (Re)initialize variables
            	/* Peripheral initialization */
		BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
            	    LED_Init();
            	    SWITCH_Init();
            	    TIMER_Init();
            	    I2C0_Init();
            	    /* Enable individual interrupt */
            	    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
            	    NVIC_EnableIRQ(TPM0_IRQn);
            	    /* Enable global interrupt */
            	    __enable_irq();
            	    /* Set interrupt priority */
            	    NVIC_SetPriority(PORTC_PORTD_IRQn, 0); // Highest priority for switches
            	    NVIC_SetPriority(TPM0_IRQn, 1); // Lower priority for timer

	
	//	PRINTF("Hello world!");
    while(1){
      /* State transition upon a switch-press */
      // Check if SW3 is pressed
        if(bSW3Pressed == true){
          // Clear the flag
            bSW3Pressed = false;
            if(enumECompassState == STOP){
                RED_LED_OFF();
                GREEN_LED_OFF();
                enumECompassState = MAG_ACQ;
            }else if(enumECompassState == MAG_ACQ){
                enumECompassState = MAG_CAL;
            }else if(enumECompassState == MAG_CAL){
                RED_LED_OFF();
                GREEN_LED_OFF();
                enumECompassState = RUN;
            }
        // Check if SW1 is pressed
        }else if(bSW1Pressed == true){
          // Clear the flag
            bSW1Pressed = false;
            if(enumECompassState == STOP){
                // Nothing to be done
            }else if(enumECompassState == MAG_ACQ){
                // Nothing to be done
            }else if(enumECompassState == MAG_CAL){
                // Nothing to be done
            }else if(enumECompassState == RUN){
				//Stop if SW1 is pressed
                enumECompassState = STOP;
            }
        }
        /* Carry out the given tasks defined in the current state */
        if(enumECompassState == STOP){

        		//Initialising all flags and data variables
        	    RED_LED_ON();
        	    GREEN_LED_OFF();
        	    flag_first_data = 0;
        	    flag_initialisation = 0;
        	    flag_first_data_acquire =0;
        	    flag_first_data_run =0;
        	    for(int i =0;i<3;i++)
        	    	{MAG_DATA_MAX_AXIS[i]=0;
        	    	 MAG_DATA_MIN_AXIS[i]=0;
        	    	 MAG_DATA_READ_AXIS[i]=0;}
        	    //Display STOP Message
        	    SLCD_WriteMsg((unsigned char *)"STOP");
        }
				else if(enumECompassState == RUN)
				{
			//The e-compass heading is updated every 100ms using the timer IRQ

				//Polling the DR_Status register data to read Magnetometer whenever new data is available 
        		DR_STATUS_DATA_ACC = I2C_SingleByteRead(MAG_DEVICE_ADDRESS,MAG_DR_STATUS);
            	if((DR_STATUS_DATA_ACC!=0)||(flag_first_data_run ==0))
					//Update Magnetometer Data
            		{
									Magnetometer_Run();
									flag_first_data_run =1;
            		}
            		// Displaying current magnetometer heading angle wrt magnetic north on LCD in degrees
				while(bIsTimerExpired == true)
					//clear bIsTimerExpired Flag
					{
								bIsTimerExpired = false;
            		snprintf(ucaryLCDMsg,5,"%4d",ANGLE);
            		SLCD_WriteMsg(ucaryLCDMsg);

					/* The green LED should be turned on between 345° to 15° (± 15° tolerance) as the e-compass is heading towards
					north magnetic pole (Otherwise, the green LED should be turned off).*/
            		if(((ANGLE >= 0)&&(ANGLE <= 15))||((ANGLE >= 345)&&(ANGLE <= 360)))
            		{
            			GREEN_LED_ON();
            		}
            		else
            		{
            			GREEN_LED_OFF();
            		}

            	}//endwhile
        }
			else if(enumECompassState == MAG_ACQ)
				{
            if(flag_initialisation == 0)
            {   //initialise the Magnetometer once
            	Magnetometer_Init();
            	flag_initialisation = 1;
            }
			//Polling the DR_Status register data to read Magnetometer whenever new data is available 
            DR_STATUS_DATA = I2C_SingleByteRead(MAG_DEVICE_ADDRESS,MAG_DR_STATUS);
            if((DR_STATUS_DATA!=0)||((flag_first_data_acquire ==0)))
               	{//Acquire Magnetometer data for calibration purpose
            	 Magnetometer_Acq();
            	 flag_first_data_acquire =1;
            	 //clear bIsTimerExpired Flag
            	 //bIsTimerExpired = false;
            	 }
			//Display MACQ Message
        	SLCD_WriteMsg((unsigned char *)"MACQ");

       }
			else if(enumECompassState == MAG_CAL)
			{
			//Calibrate Magnetometer
        	Magnetometer_Cal();
			//Display MCAL Message
        	SLCD_WriteMsg((unsigned char *)"MCAL");

      }
    }
}
