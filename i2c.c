#include "MKL46Z4.h"
#include "i2c.h"
#include "delay.h"

#define READ_MASK                          0x01
#define WRITE_MASK                         0xFE
#define DATA_SHIFT                         1
#define ACC_DEVICE_ADDRESS                 0x1D
#define RESET_MASK                         0x00
#define RIGHT_SHIFT(x,y)                   (x >> y)
#define LEFT_SHIFT(x,y)                    (x << y)
#define I2C0_SCL                           (24)
#define I2C0_SDA                           (25)
#define READ(x)                            ((x<<1)|(0x01))
#define WRITE(x)                           ((x<<1)&(0xFE))

void I2C0_Init(void)
{
		//Enable clock for I2C0 and PORTE
		SIM->SCGC4|=SIM_SCGC4_I2C0_MASK;
		SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;
	
		//Config SCL and SDA pin
		PORTE->PCR[24]|= PORT_PCR_MUX(5);
		PORTE->PCR[25]|= PORT_PCR_MUX(5);
	
		//Enable pullup resistor SCL
		PORTE->PCR[24]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
		//Enable pullup resistor SDA
		PORTE->PCR[25]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
	
		//Config clock freq 100kHz
		I2C0->F |= I2C_F_MULT(0) | I2C_F_ICR(0x14);
		//Enable I2C0
		I2C0->C1 |= I2C_C1_IICEN_MASK;
}


unsigned char I2C_SingleByteRead(unsigned char DEV_ADR, unsigned char REG_ADR)
{   
		unsigned char dummy_read = 0;
		unsigned char data = 0;
		//Select transmit and start I2C
		I2C0->C1 |= I2C_C1_TX_MASK;
		I2C0->C1 |= I2C_C1_MST_MASK;
		
		//Send MAG device address with write bit
		I2C0->D = DEV_ADR << 1;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send MAG register address
		I2C0->D = REG_ADR;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Repeated start to change to read mode
		I2C0->C1 |= I2C_C1_RSTA_MASK;
			
		//Send MAG device address and a Read Bit
		I2C0->D = (DEV_ADR << 1) | 1;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S|= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
		
		//Sending NAK to ensure right after the data is read, NAK signal is sent
		I2C0->C1 |= I2C_C1_TXAK_MASK;
		//Set the I2C in Receiver Mode to read data from MAG3110
		I2C0->C1 &= (~I2C_C1_TX_MASK);
				
		//Read dummy data
		dummy_read = I2C0->D;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		
		//Read real data
		data = I2C0->D;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
			
		//Stop I2C
		I2C0->C1 &= (~I2C_C1_MST_MASK);
		// Clear Transmit Nack by setting TXAK to 0
		I2C0->C1 &= ~(I2C_C1_TXAK_MASK);
			
		delay();
		return data;
}

void I2C_MultipleByteRead(unsigned char DEV_ADR,unsigned char REG_ADR, int num_bytes)
{   
		unsigned char dummy_data = 0;
		//Select transmit and start I2C
		I2C0->C1 |= I2C_C1_TX_MASK;
		I2C0->C1 |= I2C_C1_MST_MASK;
		
		//Send MAG address with write bit
		I2C0->D = DEV_ADR << 1;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send MAG register address 
		I2C0->D = REG_ADR;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		//Wait ACK reg address from MAG
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Repeated Start to start read data
		I2C0->C1|=I2C_C1_RSTA_MASK;
			
		//Send MAG device address and a Read Bit
		I2C0->D = (DEV_ADR << 1) | 1;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
		
		//Set the I2C in Receiver Mode
		I2C0->C1&=(~I2C_C1_TX_MASK);
		//Read Dummy Magnetometer Data
		dummy_data = I2C0->D;
		for(int i = 0; i < num_bytes; i++)
		{
			if(i < (num_bytes - 2))//read normally
					{
					 while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
					 I2C0->S|= I2C_S_IICIF_MASK;
						DATA_READ[i] = I2C0->D;
					}
			else
					{//Read two final bytes differently to ensure the NAK signal
						
						//Sending NAK to ensure right after the data is read, NAK signal is sent
						I2C0->C1 |= (I2C_C1_TXAK_MASK);
						
						while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
						I2C0->S |= I2C_S_IICIF_MASK;
						
						DATA_READ[i] = I2C0->D;
						while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
						I2C0->S|= I2C_S_IICIF_MASK;
						
						i++;
						DATA_READ[i]=I2C0->D;
						while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
						I2C0->S|= I2C_S_IICIF_MASK;
							
						//Stop
						I2C0->C1 &= (~I2C_C1_MST_MASK);
						// Clear Transmit Nack by setting TXAK to 0
						I2C0->C1 &= (~I2C_C1_TXAK_MASK);
					}
		}
		delay();
}

void I2C_SingleByteWrite(unsigned char DEV_ADR, unsigned char REG_ADR, unsigned char DATA)
{
		//Select transmit and start I2C
		I2C0->C1 |= I2C_C1_TX_MASK;
		I2C0->C1 |= I2C_C1_MST_MASK;
		
		//Send MAG device address with write bit
		I2C0->D = DEV_ADR << 1;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		//Wait ACK device address from MAG
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send MAG register address 
		I2C0->D = REG_ADR;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		//Wait ACK reg address from MAG
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send data
		I2C0->D = DATA;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		//Wait ACK reg address from MAG
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
		
		//Stop
		I2C0->C1&=(~I2C_C1_MST_MASK);
		
		delay();
}

void I2C_MultipleByteWrite(unsigned char DEV_ADR, unsigned char REG_ADR, int num_bytes, unsigned char data_bytes[])
{
		//Select transmit and start I2C
		I2C0->C1 |= I2C_C1_TX_MASK;
		I2C0->C1 |= I2C_C1_MST_MASK;
		
		//Send MAG address with write bit
		I2C0->D = DEV_ADR << 1;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send MAG register address 
		I2C0->D = REG_ADR;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		// Send the multiple bytes of data
		for(int i = 0;i < num_bytes; i++)
		{
			I2C0->D = data_bytes[i];
			while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
			I2C0->S|= I2C_S_IICIF_MASK;
			while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
		}
		//Stop 
		I2C0->C1&=(~I2C_C1_MST_MASK);
		delay();
}