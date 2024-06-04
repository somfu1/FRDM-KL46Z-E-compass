#ifndef __I2C_H__
#define __I2C_H__

extern unsigned char DATA_READ[6];

void I2C0_Init(void);
unsigned char I2C_SingleByteRead(unsigned char DEV_ADR, unsigned char REG_ADR);
void I2C_MultipleByteRead(unsigned char DEV_ADR,unsigned char REG_ADR, int max_count);
void I2C_SingleByteWrite(unsigned char DEV_ADR, unsigned char REG_ADR, unsigned char DATA);
void I2C_MultipleByteWrite(unsigned char DEV_ADR, unsigned char REG_ADR, int max_count, unsigned char data_wr[]);

#endif