/*
 * hmc5883l_raw.c
 *
 *  Created on: 21 maj 2026
 *      Author: 254710
 */
#include "stdio.h"
#include <stdint.h>
#include "i2c.h"
#include "uart.h"
#include "timer32.h"

#define HMC_ADDR (0x0D << 1)

void HMC_Init(void)
{
	HMC_WriteReg(0x09, 0x01);
}

void HMC_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    I2CWrite(HMC_ADDR, buf, 2);
}

void HMC_ReadXYZ(int16_t *mag)
{
    uint8_t reg = 0x00;
    uint8_t buf[6];

    I2CWrite(HMC_ADDR, &reg, 1);
    I2CRead(HMC_ADDR, buf, 6);


    mag[0] = (int16_t)((buf[0] << 8) | buf[1]); // X
    mag[2] = (int16_t)((buf[2] << 8) | buf[3]); // Z
    mag[1] = (int16_t)((buf[4] << 8) | buf[5]);
}
