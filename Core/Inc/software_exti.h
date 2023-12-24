/*
 * software_exti.h
 *
 *  Created on: Dec 24, 2023
 *      Author: alexey
 */

#ifndef INC_SOFTWARE_EXTI_H_
#define INC_SOFTWARE_EXTI_H_

#include <stdint.h>
#include <stdbool.h>


// typedef void (*SW_EXTI_CALLBACK)(uint16_t); // like void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

typedef struct _SW_EXTI_INSTANCE
{
  bool eventState;
  bool oldState;
} SW_EXTI_INSTANCE;


void SW_EXTI_Initialize(SW_EXTI_INSTANCE *this, bool resetPinState, bool eventPinState);
bool SW_EXTI_Update(SW_EXTI_INSTANCE *this, bool pinState);


#endif /* INC_SOFTWARE_EXTI_H_ */
