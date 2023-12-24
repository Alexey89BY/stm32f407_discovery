/*
 * software_spi.h
 *
 *  Created on: Dec 8, 2023
 *      Author: alexey
 */

#ifndef INC_SOFTWARE_SPI_H_
#define INC_SOFTWARE_SPI_H_

#include <stdint.h>
#include <stdbool.h>


void SW_SPI_Initialize(bool isInvertedNSS, int stateDelay);

void SW_SPI_TransmitReceive(uint8_t const *pTxData, uint8_t *pRxData, uint16_t Size);

bool SW_SPI_IsBusy(void);

bool SW_SPI_sequentialStart(uint8_t const *pTxData, uint8_t *pRxData, uint16_t Size);
bool SW_SPI_sequentialOnState(void);


#endif /* INC_SOFTWARE_SPI_H_ */
