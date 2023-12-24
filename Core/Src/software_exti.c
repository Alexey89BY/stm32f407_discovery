/*
 * software_exti.c
 *
 *  Created on: Dec 24, 2023
 *      Author: alexey
 */

#include "software_exti.h"


void SW_EXTI_Initialize(SW_EXTI_INSTANCE *this, bool resetPinState, bool eventPinState)
{
  this->eventState = eventPinState;
  this->oldState = resetPinState;
}


bool SW_EXTI_Update(SW_EXTI_INSTANCE *this, bool pinState)
{
  if (pinState == this->oldState)
  {
    return (false);
  }

  this->oldState = pinState;

  if (pinState == this->eventState)
  {
    return (true);
  }

  return (false);
}
