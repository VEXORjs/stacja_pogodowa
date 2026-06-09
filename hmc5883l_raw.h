/*
 * hmc5883l_raw.h
 *
 *  Created on: 21 maj 2026
 *      Author: 254710
 */
#include <stdint.h>

#ifndef INC_HMC5883L_RAW_H_
#define INC_HMC5883L_RAW_H_

void HMC_Init(void);
void HMC_WriteReg(uint8_t reg, uint8_t data);
void HMC_Read(uint8_t reg, uint8_t *data, uint8_t len);
void HMC_ReadXYZ(int16_t *mag);

#endif /* INC_HMC5883L_RAW_H_ */
