/*
 * software_i2c.c
 *
 *  Created on: Dec 16, 2023
 *      Author: alexey
 */

#include "software_i2c.h"
#include <stdbool.h>
#include "main.h"


enum _STATE
{
  STATE_IDLE = 0,
  STATE_BUSY,
};

struct _I2C_SW
{
  int state_delay;
  enum _STATE state;
};

static struct _I2C_SW i2c_sw;


static void IO_SET_SDA(bool state)
{
#ifdef SW_I2C_SDA_GPIO_Port
  GPIO_PinState pinState = (state)? GPIO_PIN_SET: GPIO_PIN_RESET;
  HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, pinState);
#endif // SW_I2C_SDA_GPIO_Port
}


static bool IO_GET_SDA(void)
{
#ifdef SW_I2C_SDA_GPIO_Port
  GPIO_PinState pinState = HAL_GPIO_ReadPin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin);
  return (pinState != GPIO_PIN_RESET);
#else // SW_I2C_SDA_GPIO_Port
  return (false);
#endif // SW_I2C_SDA_GPIO_Port
}


static void IO_SET_SCL(bool state)
{
#ifdef SW_I2C_SCL_GPIO_Port
  GPIO_PinState pinState = (state)? GPIO_PIN_SET: GPIO_PIN_RESET;
  HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, pinState);
#endif // SW_I2C_SCL_GPIO_Port
}

/*
static bool IO_GET_SCL(void)
{
#ifdef SW_I2C_SCL_GPIO_Port
  GPIO_PinState pinState = HAL_GPIO_ReadPin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin);
  return (pinState != GPIO_PIN_RESET);
#else // SW_I2C_SDA_GPIO_Port
  return (false);
#endif // SW_I2C_SDA_GPIO_Port
}
*/

void SW_I2C_Initialize(int stateDelay)
{
  i2c_sw.state_delay = stateDelay;

  IO_SET_SCL(true);
  IO_SET_SDA(true);
}


static void STATE_DELAY(void)
{
  if (i2c_sw.state_delay < 0)
  {
    int delay = -i2c_sw.state_delay;
    while (--delay != 0);
  }
  else
  {
    HAL_Delay(i2c_sw.state_delay);
  }
}


static void SW_I2C_SendStart(void)
{
  // is repeated???
  IO_SET_SDA(true);
  STATE_DELAY();

  IO_SET_SCL(true);
  STATE_DELAY();

  IO_SET_SDA(false);
  STATE_DELAY();

  IO_SET_SCL(false);
}


static void SW_I2C_SendStop(void)
{
  // is scl low???
  IO_SET_SDA(false);
  STATE_DELAY();

  IO_SET_SCL(true);
  STATE_DELAY();

  IO_SET_SDA(true);
}


static bool SW_I2C_ExchangeBit(bool write_bit)
{
  IO_SET_SDA(write_bit);
  STATE_DELAY();

  IO_SET_SCL(true);
  STATE_DELAY();

  bool read_bit = IO_GET_SDA();
  IO_SET_SCL(false);

  return (read_bit);
}

/*
static bool SW_I2C_ExchangeBit(bool sda_bit, bool is_start_stop)
{
  // SDA = 0 / 1
  IO_SET_SDA(sda_bit);
  STATE_DELAY();

  // SCL = 1
  IO_SET_SCL(true);
  STATE_DELAY();

  // 0 – read / 1 – invert sda
  bool sda_state = IO_GET_SDA();

  bool is_release_bus = false;

  if (is_start_stop)
  {
    is_release_bus = ! sda_bit;

    IO_SET_SDA(is_release_bus);
    STATE_DELAY();
  }

  // SCL = 0 / 1
  IO_SET_SCL(is_release_bus);

  return (sda_state);
}
*/

static bool SW_I2C_SendByte(uint8_t write_byte)
{
  for (int mask = 0x80; mask != 0; mask >>= 1)
  {
    bool write_bit = (write_byte & mask) != 0;
    SW_I2C_ExchangeBit(write_bit);
  }

  bool read_ack_bit = SW_I2C_ExchangeBit(true);

  return (read_ack_bit);
}


static uint8_t SW_I2C_ReceiveByte(bool write_ack_bit)
{
  uint8_t read_byte = 0;

  for (int mask = 0x80; mask != 0; mask >>= 1)
  {
    bool read_bit = SW_I2C_ExchangeBit(true);
    (read_bit)? (read_byte |= mask): (0);
  }

  SW_I2C_ExchangeBit(write_ack_bit);

  return (read_byte);
}

/*
void SW_I2C_MasterTransmit(uint8_t devAddress, uint8_t const *pData, uint16_t dataSize)
{
  if (i2c_sw.state != STATE_IDLE)
  {
    return;
  }

  i2c_sw.state = STATE_BUSY;

  SW_I2C_SendStart();

  uint8_t address_write = (devAddress << 1) | 0;
  SW_I2C_SendByte(address_write);

  for (int idx = 0; idx < dataSize; ++idx)
  {
    SW_I2C_SendByte(pData[idx]);
  }

  SW_I2C_SendStop();

  i2c_sw.state = STATE_IDLE;
}


void SW_I2C_MasterReceive(uint8_t devAddress, uint8_t *pData, uint16_t dataSize)
{
  if (i2c_sw.state != STATE_IDLE)
  {
    return;
  }

  i2c_sw.state = STATE_BUSY;

  SW_I2C_SendStart();

  uint8_t address_read = (devAddress << 1) | 0x01;
  SW_I2C_SendByte(address_read);

  for (int idx = 0; idx < dataSize; ++idx)
  {
    bool is_nack = idx >= (dataSize - 1);
    pData[idx] = SW_I2C_ReceiveByte(is_nack);
  }

  SW_I2C_SendStop();

  i2c_sw.state = STATE_IDLE;
}
*/

void SW_I2C_MemWrite(uint8_t devAddress, uint8_t memAddress, uint8_t const *pData, uint16_t dataSize)
{
  if (i2c_sw.state != STATE_IDLE)
  {
    return;
  }

  i2c_sw.state = STATE_BUSY;

  SW_I2C_SendStart();

  uint8_t address_write = (devAddress << 1) | 0;
  SW_I2C_SendByte(address_write);

  SW_I2C_SendByte(memAddress);

  for (int idx = 0; idx < dataSize; ++idx)
  {
    SW_I2C_SendByte(pData[idx]);
  }

  SW_I2C_SendStop();

  i2c_sw.state = STATE_IDLE;
}


void SW_I2C_MemRead(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t dataSize)
{
  if (i2c_sw.state != STATE_IDLE)
  {
    return;
  }

  i2c_sw.state = STATE_BUSY;

  SW_I2C_SendStart();

  uint8_t address_write = (devAddress << 1) | 0;
  SW_I2C_SendByte(address_write);

  SW_I2C_SendByte(memAddress);

  SW_I2C_SendStart();

  uint8_t address_read = (devAddress << 1) | 0x01;
  SW_I2C_SendByte(address_read);

  for (int idx = 0; idx < dataSize; ++idx)
  {
    bool is_nack = idx >= (dataSize - 1);
    pData[idx] = SW_I2C_ReceiveByte(is_nack);
  }

  SW_I2C_SendStop();

  i2c_sw.state = STATE_IDLE;
}
