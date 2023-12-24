/*
 * software_i2c.h
 *
 *  Created on: Dec 16, 2023
 *      Author: alexey
 */

#ifndef INC_SOFTWARE_I2C_H_
#define INC_SOFTWARE_I2C_H_

#include <stdint.h>


void SW_I2C_Initialize(int stateDelay);

/*
void SW_I2C_MasterTransmit(uint8_t devAddress, uint8_t const *pData, uint16_t dataSize);
void SW_I2C_MasterReceive(uint8_t devAddress, uint8_t *pData, uint16_t dataSize);
*/

void SW_I2C_MemWrite(uint8_t devAddress, uint8_t memAddress, uint8_t const *pData, uint16_t dataSize);
void SW_I2C_MemRead(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t dataSize);

#endif /* INC_SOFTWARE_I2C_H_ */
