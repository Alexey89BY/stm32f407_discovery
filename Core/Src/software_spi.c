/*
 * software_spi.c
 *
 *  Created on: Dec 8, 2023
 *      Author: alexey
 */

#include "software_spi.h"
#include "main.h"


enum _STATE
{
  STATE_IDLE = 0,
  STATE_START,
  STATE_CLOCK_ON,
  STATE_CLOCK_OFF,
  STATE_STOP,
};

struct _SPI_SW
{
  uint8_t const *tx_data;
  uint8_t *rx_data;
  uint16_t bits_count;
  uint16_t bit_index;
  int state_delay;
  enum _STATE state;
  bool is_inverted_nss;
};

static struct _SPI_SW spi_sw;


static void IO_SET_NSS(bool state)
{
#ifdef SW_SPI_NSS_GPIO_Port
  GPIO_PinState pinState = (state != spi_sw.is_inverted_nss)? GPIO_PIN_SET: GPIO_PIN_RESET;
  HAL_GPIO_WritePin(SW_SPI_NSS_GPIO_Port, SW_SPI_NSS_Pin, pinState);
#endif // SW_SPI_CS_GPIO_Port
}


static void IO_SET_CLK(bool state)
{
#ifdef SW_SPI_CLK_GPIO_Port
  GPIO_PinState pinState = (state)? GPIO_PIN_SET: GPIO_PIN_RESET;
  HAL_GPIO_WritePin(SW_SPI_CLK_GPIO_Port, SW_SPI_CLK_Pin, pinState);
#endif // SW_SPI_CLK_GPIO_Port
}


static void IO_SET_MOSI(bool state)
{
#ifdef SW_SPI_MOSI_GPIO_Port
  GPIO_PinState pinState = (state)? GPIO_PIN_SET: GPIO_PIN_RESET;
  HAL_GPIO_WritePin(SW_SPI_MOSI_GPIO_Port, SW_SPI_MOSI_Pin, pinState);
#endif // SW_SPI_MOSI_GPIO_Port
}


static bool IO_GET_MISO(void)
{
#ifdef SW_SPI_MISO_GPIO_Port
  GPIO_PinState pinState = HAL_GPIO_ReadPin(SW_SPI_MISO_GPIO_Port, SW_SPI_MISO_Pin);
  return (pinState != GPIO_PIN_RESET);
#else // SW_SPI_MISO_GPIO_Port
  return (false);
#endif // SW_SPI_MISO_GPIO_Port
}


#define BIT_BYTE_INDEX(bit_index) (bitIndex >> 3)
#define BIT_BYTE_MASK(bit_index)  (0x80 >> (bitIndex & 7)) // MSB first
//#define BIT_BYTE_MASK(bit_index)  (0x01 << (bitIndex & 7)) // LSB first


static bool GetDataBit(uint8_t const *pData, int bitIndex)
{
  int byteIndex = BIT_BYTE_INDEX(bitIndex);
  uint8_t bitMask = BIT_BYTE_MASK(bitIndex);
  return ((pData[byteIndex] & bitMask) != 0);
}


static void SetDataBit(uint8_t *pData, int bitIndex, bool bitState)
{
  int byteIndex = BIT_BYTE_INDEX(bitIndex);
  uint8_t bitMask = BIT_BYTE_MASK(bitIndex);
  (bitState)? (pData[byteIndex] |= bitMask): (pData[byteIndex] &= ~bitMask);
}


void SW_SPI_Initialize(bool isInvertedNSS, int stateDelay)
{
  spi_sw.is_inverted_nss = isInvertedNSS;
  spi_sw.state_delay = stateDelay;

  IO_SET_NSS(true);
  IO_SET_CLK(true);
}


bool SW_SPI_IsBusy(void)
{
  return (spi_sw.state != STATE_IDLE);
}


bool SW_SPI_sequentialStart(uint8_t const *pTxData, uint8_t *pRxData, uint16_t Size)
{
  if (SW_SPI_IsBusy())
  {
    return (false);
  }

  spi_sw.state = STATE_START;
  spi_sw.tx_data = pTxData;
  spi_sw.rx_data = pRxData;
  spi_sw.bits_count = Size * 8;
  spi_sw.bit_index = 0;

  return (true);
}


bool SW_SPI_sequentialOnState(void)
{
  switch (spi_sw.state)
  {
    case STATE_CLOCK_ON:
      {
	IO_SET_CLK(false);
	bool data_bit = GetDataBit(spi_sw.tx_data, spi_sw.bit_index);
	IO_SET_MOSI(data_bit);
	spi_sw.state = STATE_CLOCK_OFF;
      }
      return (true);

    case STATE_CLOCK_OFF:
      {
	IO_SET_CLK(true);
	bool data_bit = IO_GET_MISO();
	SetDataBit(spi_sw.rx_data, spi_sw.bit_index, data_bit);
	++spi_sw.bit_index;
	spi_sw.state = (spi_sw.bit_index < spi_sw.bits_count)? STATE_CLOCK_ON: STATE_STOP;
      }
      return (true);

    case STATE_START:
      IO_SET_NSS(false);
      spi_sw.state = STATE_CLOCK_ON;
      return (true);

    default: // STATE_STOP, STATE_IDLE
      IO_SET_NSS(true);
      spi_sw.tx_data = NULL;
      spi_sw.rx_data = NULL;
      spi_sw.bits_count = 0;
      spi_sw.bit_index = 0;
      spi_sw.state = STATE_IDLE;
      break;
  }

  return (false);
}


static void STATE_DELAY(void)
{
  if (spi_sw.state_delay < 0)
  {
    int delay = -spi_sw.state_delay;
    while (--delay != 0);
  }
  else
  {
    HAL_Delay(spi_sw.state_delay);
  }
}


void SW_SPI_TransmitReceive(uint8_t const *pTxData, uint8_t *pRxData, uint16_t Size)
{
  if (SW_SPI_sequentialStart(pTxData, pRxData, Size))
  {
    do
    {
      STATE_DELAY();
    }
    while (SW_SPI_sequentialOnState());
  }
}
