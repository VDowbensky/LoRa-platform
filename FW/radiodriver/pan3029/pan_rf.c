/**
 * @file      pan_rf.c
 * @brief     PAN3029/PAN3060 driver implementation
 * @version   V1.0.1
 * @date      2025-08-18
 * @copyright Panchip Microelectronics Co., Ltd. All rights reserved.
 * @code
 *             ____              ____ _     _
 *            |  _ \ __ _ _ __  / ___| |__ (_)_ __
 *            | |_) / _` | '_ \| |   | '_ \| | '_ \
 *            |  __/ (_| | | | | |___| | | | | |_) |
 *            |_|   \__,_|_| |_|\____|_| |_|_| .__/
 *                                           |_|
 *            (C)2009-2025 PanChip
 * @endcode
 * @author    PanChip
 * @note      The encoding of this file is utf-8.
 */
#include <stdio.h>
#include <string.h>
#include "pan_param.h"
#include "pan_rf.h"

/**
* @brief PAN3029/3060 Received Data Packet Structure
* @note This structure is used to store received data packets, including data length, data buffer, SNR, and RSSI information.
*/
volatile RfRxPkt_t g_RfRxPkt;

/**
* @brief PAN3029/3060 Configuration Parameter Structure
* @note This structure is used to store PAN3029/3060 configuration parameters, including transmit power, frequency, spreading factor, bandwidth, and coding rate.
*/
static volatile RfConfig_t g_RfCfgParams;

/**
* @brief Save the current RF operation state
*/
static volatile RfOpState_t g_RfOperatetate;


/**
* @brief Select register page
* @param Page The register page to be selected, page range 0-3
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
* @note If the current page is already the required page, no register configuration is required
*/
int8_t PAN_SetPage(uint8_t Page)
{
  static uint8_t gCurrPage = 0xFF;
  if(gCurrPage == Page) return PAN_OK;
  gCurrPage = Page;
  PAN_ASSERT(PAN_WriteReg(0x00, gCurrPage)); /* Select register page */
  return PAN_OK;
}

/**
* @brief Write a single byte to the register of the specified page
* @param Page Register page to be written, page range 0-3
* @param Addr Register address to be written
* @param Value Single byte of data to be written to the register
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_WritePageReg(uint8_t Page, uint8_t Addr, uint8_t Value)
{
  PAN_SetPage(Page);
  PAN_WriteReg(Addr, Value);
  return PAN_OK;
}

/**
* @brief Writes multiple bytes to the register range of a specified page
* @param Page Register page to be written, page range 0-3
* @param Addr Register address to be written
* @param Buffer Buffer pointer to the register to be written
* @param Size Number of bytes to be written
*/
void PAN_WritePageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
  PAN_SetPage(Page);                 /* Select register page */
  PAN_WriteRegs(Addr, Buffer, Size); /* Write register value */
}

/**
* @brief Reads a single byte from a register in the specified page.
* @param Page The register page to be read, page range 0-3
* @param Addr The register address to be read
* @return uint8_t The value read from the register
*/
uint8_t PAN_ReadPageReg(uint8_t Page, uint8_t Addr)
{
  PAN_SetPage(Page);
  return PAN_ReadReg(Addr);
}

/**
* @brief Reads multiple bytes from a register range in the specified page.
* @param Page The register page to be read, page range 0-3
* @param Addr The register address to be read
* @param Buffer Pointer to the buffer storing the read data
* @param Size Number of bytes to read
*/
void PAN_ReadPageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
  PAN_SetPage(Page);                /* Select register page */
  PAN_ReadRegs(Addr, Buffer, Size); /* Read register value */
}

/**
* @brief Set register bits in the specified page
* @param Page Register page to read, page range 0-3
* @param Addr Register address to set
* @param Mask Bit mask to set
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_SetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask)
{
  uint8_t Temp;

  PAN_SetPage(Page);
  Temp = PAN_ReadReg(Addr);
  PAN_WriteReg(Addr, Temp | Mask);
  return PAN_OK;
}

/**
* @brief Resets register bits in the specified page
* @param Page Register page to read, page range 0-3
* @param Addr Register address to reset
* @param Mask Bit mask to reset
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_ResetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask)
{
  uint8_t Temp;

  PAN_SetPage(Page);                  /* Select register page */
  Temp = PAN_ReadReg(Addr);           /* Read register value */
  PAN_WriteReg(Addr, Temp & (~Mask)); /* Clear the register bits corresponding to the mask */
  return PAN_OK;
}

/**
* @brief Write register bits in the specified page
* @param Page Register page to be read, page range 0-3
* @param Addr Register address to be written
* @param Value Value to be written
* @param Mask Bit mask to be written
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
* @note This function first clears the register bits corresponding to the mask and then sets the new value.
* @note For example, to set bits 2 and 3 of register 0x08 on page 1 to 0b10, leaving other bits unchanged, you can call
* PAN_WritePageRegBits(1, 0x08, 0x02, 0x0C);
* Where Value = 0x02, Mask = 0x0C, Value does not need to be left-shifted because the mask already specifies the bits to be set.
*/
int8_t PAN_WritePageRegBits(uint8_t Page, uint8_t Addr, uint8_t Value, uint8_t Mask)
{
  uint8_t Temp;
  uint8_t shift = __ctz(Mask); /* Get the shift value of the mask */

  Value <<= shift; /* Shift the value left to the position corresponding to the mask */
  Value &= Mask;   /* AND the value with the mask to ensure that only the bits corresponding to the mask are set */
  PAN_SetPage(Page);                            /* Select the register page */
  Temp = PAN_ReadReg(Addr);                     /* Read the register value */
  PAN_WriteReg( Addr, (Temp & (~Mask)) | Value); /* Clear the register bits corresponding to the mask and then set the new value */
  return PAN_OK;
}

/**
* @brief Configure GPIO mode
* @param <GpioPin> Pin number
* <GpioMode> GPIO mode
* - PAN_GPIO_MODE_INPUT: Input mode
* - PAN_GPIO_MODE_OUTPUT: Output mode
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_ConfigGpio(uint8_t GpioPin, uint8_t GpioMode)
{
  if(GpioMode == PAN_GPIO_MODE_INPUT)
  {
    if(GpioPin < 8) PAN_ASSERT(PAN_SetPageRegBits(0, 0x63, (1 << GpioPin)));
    else PAN_ASSERT(PAN_SetPageRegBits(0, 0x64, (1 << (GpioPin - 8))));
  }
  else if(GpioMode == PAN_GPIO_MODE_OUTPUT)
  {
    if(GpioPin < 8) PAN_SetPageRegBits(0, 0x65, (1 << GpioPin));
    else PAN_SetPageRegBits(0, 0x66, (1 << (GpioPin - 8)));
  }
  else return PAN_FAIL;
  return PAN_OK;
}

/**
* @brief Control GPIO output level
* @param <GpioPin> Pin number
* <Level> GPIO level
* - 0: Low level
* - 1: High level
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_WriteGpioLevel(uint8_t GpioPin, uint8_t Level)
{
  if(GpioPin < 8) PAN_WritePageRegBits(0, 0x67, Level, (1 << GpioPin));
  else PAN_WritePageRegBits(0, 0x68, Level, (1 << (GpioPin - 8)));
  return PAN_OK;
}

/**
* @brief Read GPIO level
* @param <GpioPin> Pin number
* @return Read GPIO level
* - 0: Low level
* - 1: High level
*/
uint8_t PAN_ReadGpioLevel(uint8_t GpioPin)
{
  uint8_t Temp;

  if(GpioPin < 6) Temp = PAN_ReadPageReg(0, 0x74);
  else
  {
    Temp = PAN_ReadPageReg(0, 0x75);
    GpioPin -= 6;
  }
  return (bool)((Temp >> GpioPin) & 0x01);
}

/**
* @brief Initializes the PAN3029/3060 antenna control GPIO
* @note This function initializes the PAN3029/3060 antenna control GPIO, configures it in output mode, and sets the initial level to low.
* @note If you use the MCU's GPIO to control the antenna switch, you need to re-adapt this function.
*/
void PAN_InitAntGpio(void)
{
  PAN_ConfigGpio(MODULE_GPIO_RX, PAN_GPIO_MODE_OUTPUT);
  PAN_ConfigGpio(MODULE_GPIO_TX, PAN_GPIO_MODE_OUTPUT);
  PAN_WriteGpioLevel(MODULE_GPIO_RX, 0);
  PAN_WriteGpioLevel(MODULE_GPIO_TX, 0);
}

/**
* @brief Turn on the PAN3029/3060 transmit antenna
* @note This function turns on the PAN3029/3060 transmit antenna, setting the TX pin high and the RX pin low.
* @note If you use the MCU's GPIO to control the antenna switch, you need to re-adapt this function.
*/
void PAN_TurnonTxAnt(void)
{
  PAN_WriteGpioLevel(MODULE_GPIO_RX, 0);
  PAN_WriteGpioLevel(MODULE_GPIO_TX, 1);
}

/**
* @brief Turn on the PAN3029/3060 receive antenna
* @note This function turns on the PAN3029/3060 receiving antenna, setting the RX pin high and the TX pin low.
* @note If you use the MCU's GPIO to control the antenna, you need to reconfigure this function.
*/
void PAN_TurnonRxAnt(void)
{
  PAN_WriteGpioLevel(MODULE_GPIO_TX, 0);
  PAN_WriteGpioLevel(MODULE_GPIO_RX, 1);
}

/**
* @brief Turns off the PAN3029/3060 antenna.
* @note This function turns off the PAN3029/3060 antenna, setting both the RX and TX pins low.
* @note If you use the MCU's GPIO to control the antenna, you need to reconfigure this function.
*/
void PAN_ShutdownAnt(void)
{
  PAN_WriteGpioLevel(MODULE_GPIO_RX, 0);
  PAN_WriteGpioLevel(MODULE_GPIO_TX, 0);
}

/**
* @brief Initializes the TCXO control GPIO
* @note This function initializes the PAN3029/3060 TCXO control GPIO, configures it in output mode, and sets the initial level to high.
* @note If you use the MCU's GPIO to control the TCXO on/off, you need to re-adapt this function.
*/
void PAN_InitTcxoGpio(void)
{
    PAN_ConfigGpio(MODULE_GPIO_TCXO, PAN_GPIO_MODE_OUTPUT);
    PAN_WriteGpioLevel(MODULE_GPIO_TCXO, 1);
}

/**
* @brief Turns on the TCXO power supply
* @note This function turns on the PAN3029/3060 TCXO and sets the TCXO pin to high.
* @note If you use the MCU's GPIO to control the TCXO on/off, you need to reconfigure this function.
*/
void PAN_TurnonTcxo(void)
{
  PAN_WriteGpioLevel(MODULE_GPIO_TCXO, 1);
}

/**
* @brief Turns off the TCXO power supply.
* @note This function turns off the PAN3029/3060's TCXO and sets the TCXO pin to low.
* @note If you use the MCU's GPIO to control the TCXO on/off, you need to reconfigure this function.
*/
void PAN_TurnoffTcxo(void)
{
  PAN_WriteGpioLevel(MODULE_GPIO_TCXO, 0);
}

/**
* @brief Enable LDO PA
*/
void PAN_TurnonLdoPA(void)
{    
  PAN_SetPageRegBits(0, 0x4F, 0x08);
}

/*
* @brief Disable LDO PA
*/
void PAN_TurnoffLdoPA(void)
{
  PAN_ResetPageRegBits(0, 0x4F, 0x08);
}

/**
* @brief Turn off internal and external PAs
*/
void PAN_TurnoffPA(void)
{
  PAN_TurnoffLdoPA(); /* Turn off internal PAs */
  PAN_ShutdownAnt();  /* Turn off external PAs */
  /* After transmission is complete, if configured in DCDC power mode, switch back to DCDC power mode. */
  if(g_RfCfgParams.RegulatorMode == USE_DCDC) PAN_WritePageReg(3, 0x24, 0x08);
}

/**
* @brief Turn on internal and external PAs
*/
void PAN_TurnonPA(void)
{
  /* If the current power mode is DCDC, switch to LDO power mode before transmitting. */
  if(g_RfCfgParams.RegulatorMode == USE_DCDC) PAN_WritePageReg(3, 0x24, 0x00);
  PAN_TurnonLdoPA(); /* Turn on the internal PA. */
  PAN_TurnonTxAnt(); /* Turn on the external PA. */
}

/**
* @brief Set chip mode.
* @param <ChipMode> Chip mode.
* - CHIPMODE_MODE0
* - CHIPMODE_MODE1
*/
void PAN_SetChipMode(RfChipMode_t ChipMode)
{
  if(ChipMode == CHIPMODE_MODE0)
  {
    /* Mode0 config */
    PAN_WritePageRegBits(1, 0x25, 0, 0xF0);
    PAN_WritePageRegBits(1, 0x25, 0, 0x08);
    PAN_WritePageRegBits(3, 0x12, 1, 0x04);
    PAN_WritePageRegBits(3, 0x12, 1, 0x10);
    PAN_WritePageRegBits(0, 0x58, 1, 0x04); /* Enable crc interrupt */
  }
  else
  {
    /* Mode1 config */
    PAN_WritePageRegBits(1, 0x25, 4, 0xF0);
    PAN_WritePageRegBits(1, 0x25, 1, 0x08);
    PAN_WritePageRegBits(3, 0x12, 0, 0x04);
    PAN_WritePageRegBits(3, 0x12, 0, 0x10);
    PAN_WritePageRegBits(0, 0x58, 0, 0x04); /* Disable crc interrupt */
  }
  g_RfCfgParams.ChipMode = ChipMode;
}

/**
* @brief Get chip mode
* @param -
*/
RfChipMode_t PAN_GetChipMode(void)
{
  return g_RfCfgParams.ChipMode;
}

/**
* @brief Read byte from information area
* @param <Addr> Register address
* <Pattern> Pattern match value
* <InfoAddr> Information area address
* @return Byte value read from information area
*/
uint8_t PAN_ReadInfoByte(uint8_t Addr, uint16_t Pattern, uint8_t InfoAddr)
{
  uint8_t Value;
  uint8_t Buffer[3];
  uint16_t Timeout = 10000;

  Buffer[0] = Pattern >> 8;
  Buffer[1] = Pattern & 0xFF;
  Buffer[2] = InfoAddr << 1;
  PAN_WritePageRegs(2, Addr, Buffer, sizeof(Buffer));
  do
  {
    if (PAN_ReadPageReg(0, 0x6C) & 0x80) break;
  } while (Timeout--);
  Value = PAN_ReadPageReg(2, Addr);
  return Value;
}

/**
* @brief Calibrate RF related parameters
* @return int8_t Return operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_Calibrate(void)
{
  int i;
  uint8_t Temp[3] = {0};

  /* Temp[0]: efuse[0x1E] - DCDCIMAX
  Temp[1]: efuse[0x1F] - DCDCREF
  Temp[2]: efuse[0x20] - PABIAS */
  PAN_ResetPageRegBits(2, 0x3E, 0x08); // Unlock info
  for (i = 0; i < sizeof(Temp); i++) Temp[i] = PAN_ReadInfoByte(0x3B, 0x5AA5, 0x1E + i);
  if (PAN_ReadInfoByte(0x3B, 0x5AA5, 0x1C) == 0x5A)
  {
    PAN_WritePageReg(2, 0x3D, 0xFD);
    if (Temp[2] != 0) PAN_WritePageReg(0, 0x45, Temp[2]); /* Write PABIAS */
    PAN_WritePageReg(3, 0x1C, (0xC0 | (Temp[0] & 0x1F))); /* Write DCDCIMAX */
    PAN_WritePageReg(3, 0x1D, Temp[1]);                   /* Write DCDCREF */
  }
  PAN_ASSERT(PAN_SetPageRegBits(2, 0x3E, 0x08)); // Lock info
  return PAN_OK;
}

/**
* @brief Configure AGC function
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_ConfigAgc(void)
{
  /* Enable AGC function
  - [Page2][0x06][Bit0] equal to 0 means enable AGC function
  - [Page2][0x06][Bit1] equal to 1 means disable AGC function */
  PAN_ASSERT(PAN_ResetPageRegBits(2, 0x06, 0x01));

#if REGION_DEFAULT == REGION_CN470_510 //change here according to actual frequencies!!!
  PAN_WritePageRegs(2, 0x0A, (uint8_t *)g_LowFreqAgcCfg, 40);
#elif REGION_DEFAULT == REGION_EU_863_870 || REGION_DEFAULT == REGION_US_902_928 
  PAN_WritePageRegs(2, 0x0A, (uint8_t *)g_HighFreqAgcCfg, 40);
#endif
  PAN_ASSERT(PAN_WritePageReg(2, 0x34, 0xEF));
  return PAN_OK;
}

/**
* @brief Configures the default parameters of the RF registers
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_ConfigDefaultParams(void)
{
  int i;
  for(i = 0; i < sizeof(g_RfDefaultConfig)/sizeof(PAN_RegCfg_t); i++)
  {
    PAN_WritePageReg(g_RfDefaultConfig[i].Page, g_RfDefaultConfig[i].Addr, g_RfDefaultConfig[i].Value);
  }
  return PAN_OK;
}

/**
* @brief Initializes the RF transceiver to STB3 state after power-up
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
* @note Before calling this function, you must configure the MCU's SPI and related GPIO pins.
*/
int8_t PAN_Init(void)
{
#if USE_PAN_RST_GPIO == 1
  PAN_Reset();
#endif
  /* [Pagex][0x04][BIT4] is the reset control. When it is 0, the chip is reset. When it is 1, the reset is released */
  PAN_WriteReg(0x04, 0x06); /* Start POR reset chip */
  //PAN_DelayUs(100);         /* Ensure that the actual delay is above 100us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
#if INTERFACE_MODE == USE_SPI_4LINE
  PAN_WriteReg(0x00, 0x03); /* Select register page 3 */
  PAN_WriteReg(0x1A, 0x03); /* Enable 4-line SPI */
#elif INTERFACE_MODE == USE_SPI_3LINE
  PAN_WriteReg(0x00, 0x03); /* Select register page 3 */
  PAN_WriteReg(0x1A, 0x83); /* Enable 3-line SPI */
#endif
  PAN_SetPage(0);                                    /* Select register page 0 */
  PAN_ASSERT(PAN_WriteReg(0x02, PAN_STATE_DEEPSLEEP)); /* Enter deepsleep state */
  //PAN_DelayUs(10);                                   /* Ensure actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_ASSERT(PAN_WriteReg(0x02, PAN_STATE_SLEEP));     /* Enter sleep state */
  //PAN_DelayUs(10);                                   /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_ASSERT(PAN_WritePageReg(3, 0x06, 0x20));        /* Enable ISO */
  //PAN_DelayUs(10);                                   /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_ASSERT(PAN_WriteReg(0x02, PAN_STATE_STB1));      /* Enter stb1 state */
 //PAN_DelayUs(10);                                   /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
#if USE_ACTIVE_CRYSTAL == 1                           /* If using an active crystal oscillator, you need to configure the TCXO GPIO pin */
  PAN_ASSERT(PAN_WritePageReg(3, 0x26, 0xA0));        /* Enable core power and turn on the active crystal oscillator channel */
  PAN_DelayUs(100);                                  /* Ensure that the actual delay is above 100us */
  PAN_ASSERT(PAN_WriteReg(0x04, 0x36));               /* Enable LFT and release POR reset */
  PAN_DelayMs(1);                                    /* Ensure that the actual delay is above 1ms */
  PAN_InitTcxoGpio(pan);                                /* Initialize TCXO GPIO pin */
#else
  PAN_ASSERT(PAN_WritePageReg(3, 0x26, 0x20));        /* Enable core power */
  //PAN_DelayUs(100);                                  /* Ensure that the actual delay is above 100us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_ASSERT(PAN_WriteReg(0x04, 0x36));               /* Enable LFT and release POR reset */
  PAN_DelayMs(1);                                    /* Ensure that the actual delay is above 1ms */
#endif
  PAN_ASSERT(PAN_WriteReg(0x02, PAN_STATE_STB2));      /* Enter stb2 state */
  PAN_DelayMs(1);                                    /* Ensure that the actual delay is above 1ms */
  PAN_ASSERT(PAN_WriteReg(0x02, PAN_STATE_STB3));      /* Enter stb3 state */
  //PAN_DelayUs(100);                                  /* Ensure that the actual delay is above 100us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_ASSERT(PAN_ConfigDefaultParams());              /* Configure the default parameters of the RF register */
  PAN_ASSERT(PAN_Calibrate());                        /* Calibrate RF related parameters */
  PAN_ASSERT(PAN_ConfigAgc());                        /* Configure the AGC function */
  PAN_InitAntGpio();                                 /* Initialize the antenna GPIO pinS */
  g_RfOperatetate = PAN_STATE_STB3;                  /* Set the current working state to STB3 */
  return PAN_OK;
}

/**
* @brief Configure the user parameters of the RF chip
*/
void PAN_ConfigUserParams(void)
{
  PAN_SetTxPower(22);                    /* Set the power level */
  PAN_SetFreq(PAN_FREQ_DEFAULT);          /* Set the frequency */
  PAN_SetBW(PAN_BW_DEFAULT);              /* Set the bandwidth */
  PAN_SetSF(PAN_SF_DEFAULT);              /* Set the spreading factor */
  PAN_SetCR(PAN_CR_DEFAULT);              /* Set the channel coding rate */
  PAN_SetCRC(PAN_CRC_DEFAULT);            /* Set the CRC check */
  PAN_SetLDR(PAN_LDR_DEFAULT);            /* Set the low-rate mode */
  PAN_SetPreamLen(PAN_PREAMBLE_DEFAULT);  /* Set the preamble length */
  PAN_SetInvertIQ(PAN_IQ_INVERT_DEFAULT); /* Set IQ to non-inverted */
  PAN_SetRegulatorMode(USE_LDO);         /* Set the chip to LDO power mode */
  PAN_SetChipMode(CHIPMODE_MODE0);       /* Set the chip mode to MODE0 */
}

/**
* @brief Software reset of the RF chip control logic
*/
void PAN_ResetLogic(void)
{
  PAN_WriteReg(0x00, 0x80);
  PAN_WriteReg(0x00, 0x00);
  (void)PAN_ReadReg(0x00); /* A dummy read of register 0x00 is required for the reset to take effect */ 
}

/**
* @brief Get the operating status of the RF chip
* @return RfOpState_t Current operating state
* - PAN_STATE_SLEEP: Chip is in sleep mode
* - PAN_STATE_STB3: Chip is in standby mode
* - PAN_STATE_TX: Chip is in transmit mode
* - PAN_STATE_RX: Chip is in receive mode
*/
RfOpState_t PAN_GetOperateState(void)
{
  return g_RfOperatetate;
}

/**
* @brief Set the operating state of the RF chip
* @param <RfState> Operating state
* - PAN_STATE_SLEEP: Chip is in sleep mode
* - PAN_STATE_STB3: Chip is in standby mode
* - PAN_STATE_TX: Chip is in transmit mode
* - PAN_STATE_RX: Chip is in receive mode
*/
void PAN_SetOperateState(RfOpState_t RfState)
{
  g_RfOperatetate = RfState;
}

/**
* @brief Set the RF chip's operating state.
* @param <RfState>
* - PAN_STATE_DEEPSLEEP
* - PAN_STATE_SLEEP
* - PAN_STATE_STB3
* - PAN_STATE_TX
* - PAN_STATE_RX
*/
void PAN_SetRfState(uint8_t RfState)
{
  PAN_WriteReg(0x02, RfState);
  g_RfOperatetate = (RfOpState_t)RfState;
}

/**
* @brief Enter deep sleep mode.
* @note This function puts the RF chip into deep sleep mode, turning off the antenna power supply and TCXO power supply.
* @note This function sets the chip's operating state to MODE_DEEPSLEEP.
* @note After executing this function, if you need to wake up the RF chip, you need to call the PAN_Init() function to wake up the chip.
*/
void PAN_EnterDeepsleepState(void)
{
  PAN_ShutdownAnt();                      /* Turn off the antenna */
  PAN_WriteReg(0x02, PAN_STATE_STB3);      /* Enter STB3 state */
  //PAN_DelayUs(150);                       /* Ensure that the actual delay is above 150us */
	PAN_DelayMs(1);                       /* Ensure that the actual delay is above 150us */
  PAN_WriteReg(0x02, PAN_STATE_STB2);      /* Enter STB2 state */
  //PAN_DelayUs(10);                        /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WriteReg(0x02, PAN_STATE_STB1);      /* Enter STB1 state */
  //PAN_DelayUs(10);                        /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
#if USE_ACTIVE_CRYSTAL == 1                /* If an active crystal oscillator is used, the TCXO power supply needs to be turned off */
  PAN_TurnoffTcxo(pan);                      /* Turn off the TCXO power supply */
#endif
  PAN_WriteReg(0x04, 0x06);               /* Turn off LFT */
  //PAN_DelayUs(10);                        /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WriteReg(0x02, PAN_STATE_SLEEP);     /* Enter the SLEEP state */
  //PAN_DelayUs(10);                        /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WritePageReg(3, 0x06, 0x00);        /* Turn off ISO */
  //PAN_DelayUs(10);                        /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WritePageReg(3, 0x26, 0x00);        /* Turn off the internal power supply */
  //PAN_DelayUs(10);                        /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WriteReg(0x02, PAN_STATE_DEEPSLEEP); /* Enter the DEEPSLEEP state */
  g_RfOperatetate = PAN_STATE_DEEPSLEEP;
}

/**
* @brief Enter sleep mode
* @note This function is used to put the RF chip into sleep mode, turn off the antenna power supply and TCXO power supply
* @note This function will set the chip's working state to MODE_SLEEP
* @note After executing this function, if you need to wake up the RF chip, you need to call the PAN_ExitSleepState() function
*/
void PAN_EnterSleepState(void)
{
  PAN_ShutdownAnt();                   /* Turn off the antenna */
  PAN_WriteReg(0x02, PAN_STATE_STB3);   /* Enter the STB3 state */
  //PAN_DelayUs(150);                    /* Ensure that the actual delay is above 150us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WriteReg(0x02, PAN_STATE_STB2);   /* Enter STB2 state */
  //PAN_DelayUs(10);                     /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WriteReg(0x02, PAN_STATE_STB1);   /* Enter STB1 state */
  //PAN_DelayUs(10);                     /* Ensure that the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
#if USE_ACTIVE_CRYSTAL == 1             /* If using an active crystal oscillator, you need to turn off the TCXO power supply */
  PAN_TurnoffTcxo(pan);                   /* Turn off the TCXO power supply */
#endif
  PAN_WriteReg(0x04, 0x16);            /* Turn off LFT */
  //PAN_DelayUs(10);                     /* Ensure the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WriteReg(0x02, PAN_STATE_SLEEP);  /* Enter SLEEP state */
  //PAN_DelayUs(10);                     /* Ensure the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_ResetPageRegBits(3, 0x06, 0x20); /* Turn off ISO */
  //PAN_DelayUs(10);                     /* Ensure the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WritePageReg(3, 0x26, 0x00);     /* Turn off internal power */
  g_RfOperatetate = PAN_STATE_SLEEP;
}

/**
* @brief Exit sleep mode
* @note This function is used to exit the RF chip from sleep mode and turn on the antenna power supply and TCXO power supply.
* @note This function will set the chip's operating state to MODE_STDBY
*/
void PAN_ExitSleepState(void)
{
  PAN_SetPageRegBits(3, 0x06, 0x20); /* Enable ISO */
  //PAN_DelayUs(10);                   /* Ensure the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WriteReg(0x02, PAN_STATE_STB1); /* Enter STB1 state */
  //PAN_DelayUs(10);                   /* Ensure the actual delay is above 10us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
#if USE_ACTIVE_CRYSTAL == 1
  PAN_WritePageReg(3, 0x26, 0xA0);   /* Enable core power and turn on active crystal oscillator channel */
  PAN_DelayUs(100);                  /* Ensure the actual delay is above 100us */
  PAN_WriteReg(0x04, 0x36);          /* Enable LFT */
  PAN_DelayUs(100);                  /* Ensure the actual delay is above 100us */
  PAN_TurnonTcxo(pan);                  /* Turn on TCXO */
#else
  PAN_WritePageReg(3, 0x26, 0x20);   /* Enable core power */
  //PAN_DelayUs(100);                  /* Ensure the actual delay is above 100us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  PAN_WriteReg(0x04, 0x36);          /* Enable LFT */
  //PAN_DelayUs(100);                  /* Ensure the actual delay is above 100us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
#endif
  PAN_WriteReg(0x02, PAN_STATE_STB2); /* Enter STB2 state */
  PAN_DelayMs(1);                    /* Ensure the actual delay is above 1ms */
  PAN_WriteReg(0x02, PAN_STATE_STB3); /* Enter STB3 state */
  //PAN_DelayUs(100);                  /* Ensure the actual delay is above 100us */
	PAN_DelayMs(1);                        /* Ensure that the actual delay is above 10us */
  g_RfOperatetate = PAN_STATE_STB3;
}

/**
* @brief Enter standby mode
* @note This function sets the chip's operating state to MODE_STDBY.
*/
void PAN_EnterStandbyState(void)
{
  PAN_SetRfState(PAN_STATE_STB3);
  PAN_SetOperateState(PAN_STATE_STB3);
}

/**
* @brief Checks whether the RF chip is in sleep mode.
* @note This function checks whether the RF chip is in sleep mode.
* @note If it is in sleep mode, it exits sleep mode and enters standby mode.
* @note This function sets the chip's operating state to MODE_STDBY.
*/
void PAN_CheckDeviceReady(void)
{
  if (PAN_GetOperateState() == PAN_STATE_SLEEP) PAN_ExitSleepState();
}

/**
* @brief Sets the chip's power supply mode.
* @param <RegulatorMode> Power supply mode.
* - USE_LDO: Use LDO for power supply.
* - USE_DCDC: Use DCDC power supply.
* @note: In transmit mode, the RF must use LDO power supply mode.
* In other modes, any power supply mode can be selected.
*/
void PAN_SetRegulatorMode(RfRegulatorMode_t RegulatorMode)
{
  PAN_WritePageReg(3, 0x24, (RegulatorMode == USE_DCDC) ? 0x08 : 0x00);
  g_RfCfgParams.RegulatorMode = RegulatorMode;
}

/**
* @brief Set the RF chip frequency
* @param <Frequency> Communication frequency (Hz)
* @note Supported frequency range:
* Low frequency band:
* - 138.33MHz to 282.5MHz
* - 405.00MHz to 565.00MHz
* High frequency band:
* - 810.00MHz to 1080.00MHz
*/
int8_t PAN_SetFreq(uint32_t Frequency)
{
  int i;
  uint32_t Fa, Fb;
  uint32_t Temp = 0;
  uint32_t IntegerPart;
  uint8_t FreqReg[4], Fab[3];
  uint32_t FreqTableNum = (sizeof(g_RfFreqTable) / sizeof(RadioFreqTable_t));
   
  if (Frequency < g_RfFreqTable[0].StartFreq || Frequency > g_RfFreqTable[FreqTableNum - 1].StopFreq) return PAN_FAIL;
  /* Traverse the frequency table and find the matching frequency segment */
  for (i = 0; i < FreqTableNum; i++)
  {
    if (Frequency > g_RfFreqTable[i].StartFreq && Frequency <= g_RfFreqTable[i].StopFreq)
    {
      uint8_t LoMux = (g_RfFreqTable[i].LoParam & 0x70) >> 4;
      Temp = Frequency * g_VcoDivTable[LoMux];
      PAN_WritePageRegs(0, 0x40, (uint8_t *)&g_RfFreqTable[i].VcoParam, 2);
      PAN_WriteReg(0x3D, g_RfFreqTable[i].LoParam);
      break;
    }
  }
  /* No frequency range matched */
  if (i >= FreqTableNum) return PAN_FAIL;
  IntegerPart = Temp / 32000000;
  Fa = IntegerPart - 20; //640MHz
  Fb = (Temp % 32000000) / 40000;
  FreqReg[0] = (uint8_t)Frequency;
  FreqReg[1] = (uint8_t)(Frequency >> 8);
  FreqReg[2] = (uint8_t)(Frequency >> 16);
  FreqReg[3] = (uint8_t)(Frequency >> 24);
  PAN_WritePageRegs(3, 0x09, FreqReg, 4);
  Fab[0] = (uint8_t)(Fa); 
  Fab[1] = (uint8_t)(Fb); //10kHz 
  Fab[2] = (uint8_t)((Fb >> 8) & 0x0F); //2.56MHz
  PAN_WritePageRegs(3, 0x15, Fab, 3);
  g_RfCfgParams.Frequency = Frequency;
  return PAN_OK;
}

/**
* @brief Set IQ inversion
* @param <NewState> Enable or disable IQ inversion
* - true: Enable IQ inversion 
* - false: disable IQ inversion 
*/
void PAN_SetInvertIQ(bool NewState)
{
  if (NewState)
  {
    /*
    * BIT6 = 0: invert rx IQ
    * BIT5 = 1: invert tx IQ
    */
    PAN_WritePageRegBits(1, 0x0E, 0x01, 0x40 | 0x20);
    g_RfCfgParams.InvertIQ = PAN_IQ_INVERTED;
  }
  else
  {
    /*
    * BIT6 = 1: non-invert rx IQ
    * BIT5 = 0: non-invert tx IQ
    */
    PAN_WritePageRegBits(1, 0x0E, 0x02, 0x40 | 0x20);
    g_RfCfgParams.InvertIQ = PAN_IQ_NORMAL;
  }
}

/**
* @brief Set the preamble length
* @param <PreamLen> Preamble length value
* Range is 4 - 65535
*/
void PAN_SetPreamLen(uint16_t PreamLen)
{
  uint8_t Temp[2] = {(uint8_t)(PreamLen), (uint8_t)((PreamLen >> 8))};
  PAN_WritePageRegs(3, 0x13, Temp, 2);
  g_RfCfgParams.PreambleLen = PreamLen;
}

/**
* @brief Set the synchronization word
* @param <syncWord> Synchronization word value
* @note The synchronization word size supported by PAN3029/3060 is 1 byte
* @note The sync word is used for synchronization detection when receiving data packets. Typically, the same sync word should be set when sending and receiving data packets.
* For example, if the sync word is set to 0x12 when sending a data packet, the sync word should also be set to 0x12 when receiving a data packet.
*/
void PAN_SetSyncWord(uint8_t SyncWord)
{
  PAN_WritePageReg(3, 0x0F, SyncWord);
  g_RfCfgParams.SyncWord = SyncWord;
}

/**
* @brief Set the transmit power.
* @param <TxPower> Transmit power level, range: 1-22
* @note The power values corresponding to the power levels are shown in the following table:
 *|------|------------------|------------------|------------------|------------------|
 *|Level |410MHz Power (dBm)|430MHz Power (dBm)|450MHz Power (dBm)|460MHz Power (dBm)|
 *|------|------------------|------------------|------------------|------------------|
 *|  1   |       -18.7      |      -18.2       |       -18.8      |       -19.7      |
 *|  2   |       -8.3       |      -7.9        |       -8.6       |       -9.5       |
 *|  3   |        1.6       |       1.7        |        0.9       |       -0.1       |
 *|  4   |        3.9       |       4.5        |        4.1       |        3.3       |
 *|  5   |        4.9       |       5.3        |        4.8       |        3.9       |
 *|  6   |        5.4       |       5.5        |        4.8       |        3.8       |
 *|  7   |        7.1       |       7.6        |        7.3       |        6.8       |
 *|  8   |        7.7       |       8.2        |        8.0       |        7.5       |
 *|  9   |        8.6       |       9.2        |        8.8       |        8.1       |
 *|  10  |        9.0       |       9.3        |        9.1       |        8.9       |
 *|  11  |        10.3      |       10.7       |        10.6      |        10.3      |
 *|  12  |        10.8      |       11.0       |        10.8      |        10.6      |
 *|  13  |        11.6      |       11.8       |        11.6      |        11.5      |
 *|  14  |        12.8      |       13.2       |        13.1      |        12.9      |
 *|  15  |        13.8      |       14.3       |        14.2      |        14.0      |
 *|  16  |        14.9      |       15.4       |        15.4      |        15.0      |
 *|  17  |        15.4      |       16.0       |        15.8      |        15.3      |
 *|  18  |        16.1      |       16.6       |        16.5      |        16.3      |
 *|  19  |        16.5      |       17.0       |        16.9      |        16.7      |
 *|  20  |        17.4      |       17.9       |        17.8      |        17.6      |
 *|  21  |        18.2      |       18.3       |        18.0      |        17.7      |
 *|  22  |        19.4      |       19.4       |        19.1      |        18.7      |
 *|------|------------------|------------------|------------------|------------------|
 */
void PAN_SetTxPower(uint8_t TxPower)
{
  int Index;
  uint8_t Temp1, Temp2;
  static bool PaBiasReadFlag = false; // for read efuse only once
  static uint8_t PaBiasVal = 0;
    
  TxPower = (TxPower > PAN_MAX_RAMP ? PAN_MAX_RAMP : TxPower);
  TxPower = (TxPower < PAN_MIN_RAMP ? PAN_MIN_RAMP : TxPower);
  Index = TxPower - 1;
  /* Modulate wave ramp mode */
  PAN_WritePageReg(3, 0x22, g_RfPowerRampCfg[Index].Ldo & 0x01);
  PAN_WritePageReg(0, 0x1E, g_RfPowerRampCfg[Index].Ramp);
  PAN_WritePageReg(0, 0x4B, g_RfPowerRampCfg[Index].Ldo >> 4);
  if (g_RfPowerRampCfg[Index].PAbias != 0x70) PAN_SetPageRegBits(0, 0x46, 0x04); // page0, reg0x46, bit2=1
  else PAN_ResetPageRegBits(0, 0x46, 0x04); // page0, reg0x46, bit2=0
  if(!PaBiasReadFlag)
  {
    PaBiasReadFlag = true;
    PAN_ResetPageRegBits(2, 0x3E, 0x08); /* RF unlock info */
    PaBiasVal = PAN_ReadInfoByte(0x3B, 0x5AA5, 0x20);
    PAN_SetPageRegBits(2, 0x3E, 0x08);  /* RF lock info */
    if (PaBiasVal == 0) PaBiasVal = 8;
  }
  Temp1 = PaBiasVal - (g_RfPowerRampCfg[Index].PAbias & 0x0F);
  Temp2 = (g_RfPowerRampCfg[Index].PAbias & 0xF0) | Temp1;
  PAN_WritePageReg(0, 0x45, Temp2);
  g_RfCfgParams.TxPower = TxPower; /* save current TxPower value */
}

/**
* @brief Set modulation bandwidth
* @param <BandWidth> Modulation bandwidth value
* - PAN_BW_062K / PAN_BW_125K / PAN_BW_250K / PAN_BW_500K
* @note: A larger modulation bandwidth increases the data rate but shortens the transmission distance.
* @note: The modulation bandwidth range of the PAN3029 chip is PAN_BW_062K - PAN_BW_500K
* @note: The modulation bandwidth range of the PAN3060 chip is PAN_BW_125K - PAN_BW_500K
*/
void PAN_SetBW(uint8_t BandWidth)
{
  /* Page 3, Reg 0x0D, Bit[7:4] = BandWidth */
  PAN_WritePageRegBits(3, 0x0D, BandWidth, 0xF0);
  if (BandWidth != PAN_BW_500K) PAN_SetPageRegBits(2, 0x3F, 0x02);
  else PAN_ResetPageRegBits(2, 0x3F, 0x02);
  g_RfCfgParams.Bandwidth = (RfBandwidths_t)BandWidth; // save current BW value
}

/**
* @brief Set the spreading factor
* @param <SpreadFactor> Spreading factor value
* - PAN_SF5 / PAN_SF6 / PAN_SF7 / PAN_SF8 / PAN_SF9 / PAN_SF10 / PAN_SF11 / PAN_SF12
* @note A larger spreading factor increases the transmission distance but reduces the data rate.
* @note The spreading factor range for the PAN3029 chip is PAN_SF5 - PAN_SF12
* @note The spreading factor range for the PAN3060 chip is PAN_SF5 - PAN_SF9
*/
void PAN_SetSF(uint8_t SpreadFactor)
{
  /* Page 3, Reg 0x0E, Bit[7:4] = SpreadFactor */
  PAN_WritePageRegBits(3, 0x0E, SpreadFactor, 0xF0);
  g_RfCfgParams.SpreadingFactor = (RfSpreadFactor_t)SpreadFactor; // save current SF value
}

/**
* @brief Set channel coding rate
* @param <CodingRate> Channel coding rate value
* - PAN_CR_4_5 / PAN_CR_4_6 / PAN_CR_4_7 / PAN_CR_4_8
*/
void PAN_SetCR(uint8_t CodingRate)
{
  /* Page 3, Reg 0x0D, Bit[3:1] = CodingRate */
  PAN_WritePageRegBits(3, 0x0D, CodingRate, 0x0E);
  g_RfCfgParams.CodingRate = (RfCodingRates_t)CodingRate; // save current CR value
}

/**
* @brief Set CRC check
* @param <CrcMode> Enable or disable CRC check
* - PAN_CRC_ON: Enable CRC check
* - PAN_CRC_OFF: Disable CRC check
*/
void PAN_SetCRC(uint8_t CrcMode)
{
  /* Page 3, Reg 0x0D, Bit[0] = CRC */
  PAN_WritePageRegBits(3, 0x0E, CrcMode, 0x08);
  g_RfCfgParams.CrcMode = (RfCrcModes_t)CrcMode; // save current CRC value
}

/**
* @brief Set low data rate mode
* @param <LdrMode> Low data rate mode value
* - PAN_LDR_ON: Enable low data rate mode
* - PAN_LDR_OFF: Disable low data rate mode
*/
void PAN_SetLDR(uint8_t LdrMode)
{
  /* Page 3, Reg 0x12, Bit[3] = LDR */
  PAN_WritePageRegBits(3, 0x12, LdrMode, 0x08);
  g_RfCfgParams.LowDatarateOptimize = LdrMode; // save current LDR value
}

/**
* @brief Set modem mode
* @param <modem_mode>
* - MODEM_MODE_NORMAL
* - MODEM_MOdDE_MULTI_SECTOR 
* @note This function should be called after PAN_SetSF(uint8_t SpreadFactor) 
*/
void PAN_SetModemMode(uint8_t ModemMode)
{
  if (ModemMode == MODEM_MODE_NORMAL) PAN_WritePageReg(1, 0x0B, 0x08);
  else if (ModemMode == MODEM_MODE_MULTI_SECTOR)
  {
    PAN_WritePageReg(1, 0x0B, 0x18);
    if( g_RfCfgParams.SpreadingFactor <= PAN_SF6 )
    {
      PAN_WritePageReg(1, 0x2F, 0x74);
      PAN_WritePageReg(1, 0x30, 0x01);
    }
    else
    {
      PAN_WritePageReg(1, 0x2F, 0x54);
      PAN_WritePageReg(1, 0x30, 0x40);
    }
  }
}

/**
* @brief Set the transmit mode
* @param Buffer Data buffer to be sent
* @param Size Number of data bytes to be sent
* @note Ensure that the RF is in standby (STB3) mode before calling this function
* @note This function is in single-shot transmit mode and will automatically enter standby (STB3) mode after the transmission is complete.
* @note A TX_DONE interrupt will be triggered after the transmission is complete.
*/
void PAN_SetTx(uint8_t *Buffer, uint8_t Size)
{
  PAN_TxSinglePkt(Buffer, Size);
  g_RfOperatetate = PAN_STATE_TX;
}

/**
* @brief Set the receive mode
* @param TimeoutMs Receive timeout in milliseconds
* - 0: Continuous Receive Mode
* - >0: Single Receive Mode. After a timeout, a timeout interrupt is generated and the device automatically enters Standby (STB3) mode.
* @note: Ensure that the RF is already in Standby (STB3) mode before calling this function.
*/
void PAN_SetRx(uint32_t TimeoutMs)
{
  if (TimeoutMs == 0) PAN_EnterContinousRxState();
  else PAN_EnterSingleRxWithTimeout(TimeoutMs);
  g_RfOperatetate = PAN_STATE_RX;
}

/**
* @brief Start CAD detection
* @param <Threshold>
- PAN_CAD_THRESHOLD_0A
- PAN_CAD_THRESHOLD_10
- PAN_CAD_THRESHOLD_15
- PAN_CAD_THRESHOLD_20 
<Chirps> 
- PAN_CAD_01_SYMBOL 
- PAN_CAD_02_SYMBOL 
- PAN_CAD_03_SYMBOL 
- PAN_CAD_04_SYMBOL 
*/
void PAN_StartCad(uint8_t Threshold, uint8_t Chirps)
{
  /* Configure GPIO11 as output for CAD indication */
  PAN_ConfigGpio(MODULE_GPIO_CAD_IRQ, PAN_GPIO_MODE_OUTPUT);
  /* [Page0][Reg0x5E][Bit6] = 0, enable GPIO11 CAD indication */
  PAN_ResetPageRegBits(0, 0x5E, 0x40);
  PAN_WritePageReg(1, 0x0F, Threshold);            /* [Page1][Reg0x0F] = Threshold */
  PAN_WritePageRegBits(1, 0x25, Chirps - 1, 0x03); /* [Page1][Reg0x25][Bit[1:0]] = Chirps - 1 */
  PAN_WritePageReg(1, 0x35, 0xFE);                 /* [Page1][Reg0x35] payload cad config */
  PAN_EnterContinousRxState();                     /* Enter continous RX state */
}

/** 
* @brief Set CAD detection threshold 
* @param <Threshold> CAD detection threshold 
* - PAN_CAD_THRESHOLD_0A 
* - PAN_CAD_THRESHOLD_10 
* - PAN_CAD_THRESHOLD_15 
* - PAN_CAD_THRESHOLD_20 
*/
void PAN_SetCadThreshold(uint8_t Threshold)
{
  PAN_WritePageReg(1, 0x0F, Threshold);
}

/**
* @brief Set the number of CAD detection symbols
* @param <Chirps> Number of CAD detection symbols
* - PAN_CAD_01_SYMBOL
* - PAN_CAD_02_SYMBOL
* - PAN_CAD_03_SYMBOL
* - PAN_CAD_04_SYMBOL
*/
void PAN_SetCadChirps(uint8_t Chirps)
{
  PAN_WritePageRegBits(1, 0x25, Chirps - 1, 0x03); /* [Page1][Reg0x25][Bit[1:0]] = Chirps */
}

/**
* @brief Stop CAD detection
*/
void PAN_StopCad(void)
{
    PAN_SetPageRegBits(0, 0x5E, 0x40); /* [Page0][Reg0x5E][Bit6] = 1, disable GPIO11 CAD indication */
    PAN_WritePageReg(1, 0x0F, 0x0A);   /* Reset CAD threshold */
    PAN_WritePageReg(1, 0x35, 0xF4);   /* Reset payload cad config */
    PAN_SetRfState(PAN_STATE_STB3);     /* Enter standby state */
    PAN_ResetLogic();                  /* Soft reset the RF chip, clear cad state */
    g_RfOperatetate = PAN_STATE_STB3;
}

/**
* @brief Set the transmit mode
* @param <TxMode>
* - PAN_TX_SINGLE: Single transmit mode
* - PAN_TX_CONTINOUS: Continuous transmit mode
* @note This function is only used to set the transmit mode and does not change the chip's operating state.
*/
void PAN_SetTxMode(uint8_t TxMode)
{
  PAN_WritePageRegBits(3, 0x06, TxMode, 0x04); /* [Page3][Reg0x06][Bit[2]] = TxMode */
}

/**
* @brief Send a single data packet
* @param <Buffer> Data buffer to be sent
* @param <Size> Number of data bytes to be sent
* @note TX_DONE interrupt will be triggered after transmission is completed
* @note In the transmission completion interrupt, the PAN_TurnoffPA() function needs to be called to turn off the chip's internal and external PAs.
*/
void PAN_TxSinglePkt(uint8_t *Buffer, uint8_t Size)
{
  PAN_WriteReg(0x02, PAN_STATE_STB3); /* Enter standby state */
  PAN_SetTxMode(PAN_TX_MODE_SINGLE);  /* Set single transmit mode */
  PAN_WritePageReg(1, 0x0C, Size);   /* Set transmit data length */
  PAN_TurnonPA();                    /* Enable the PA before transmitting data */
  PAN_SetRfState(PAN_STATE_TX);       /* Set the chip to transmit mode */
  PAN_WriteRegs(0x01, Buffer, Size); /* Write data to the FIFO. After writing the data, the chip starts transmitting data. 0x01 is the FIFO register address */
}

/**
* @brief Set the receive mode
* @param <RxMode>
* - PAN_RX_SINGLE: Single receive mode. Automatically enters standby mode after receiving a packet of data.
* - PAN_RX_SINGLE_TIMEOUT: Single receive mode with timeout. Automatically enters standby mode after timeout.
* - PAN_RX_CONTINOUS: Continuous receive mode, continues receiving after receiving a packet of data.
* @note This function only sets the receive mode and does not change the chip's operating state.
*/
void PAN_SetRxMode(uint8_t RxMode)
{
  PAN_WritePageRegBits(3, 0x06, RxMode, 0x03); /* [Page3][Reg0x06][Bit[1:0]] = RxMode */
}

/**
* @brief Puts the chip into continuous receive mode.
* @note After calling this function, the chip enters continuous receive mode.
* @note This function sets the chip's operating state to MODE_RX.
*/
void PAN_EnterContinousRxState(void)
{
  PAN_SetRfState(PAN_STATE_STB3);       /* Enter standby mode. */
  PAN_TurnonRxAnt();                   /* Turn on the receive antenna. */
  PAN_TurnoffLdoPA();                  /* Turn off the internal PA */
  PAN_SetRxMode(PAN_RX_MODE_CONTINOUS); /* Set the receive mode to continuous receive */
  PAN_SetRfState(PAN_STATE_RX);         /* Enter receive state */
}

/**
* @brief Set the receive timeout
* @param <TimeoutMs> Timeout in milliseconds
* Timeout range: 0~65535ms
* @note This function is only used to set the receive timeout and does not change the chip's working state.
*/
void PAN_SetRxTimeout(uint16_t TimeoutMs)
{
  uint8_t Temp[2] = {(uint8_t)TimeoutMs, (uint8_t)(TimeoutMs >> 8)};
  PAN_WritePageRegs(3, 0x07, Temp, 2);
}

/**
* @brief Let the chip enter the single receive state with timeout
* @param <TimeoutMs> Timeout time, in ms
* Timeout range: 1~65535ms
* @note After calling this function, the chip will enter the single receive state with timeout
* @note This function will set the chip's working state to MODE_RX
*/
void PAN_EnterSingleRxWithTimeout(uint16_t TimeoutMs)
{
  PAN_SetRfState(PAN_STATE_STB3);            /* Enter standby state */
  PAN_TurnonRxAnt();                        /* Turn on the receive antenna */
  PAN_TurnoffLdoPA();                       /* Turn off the internal PA */
  PAN_SetRxTimeout(TimeoutMs);              /* Set the receive timeout */
  PAN_SetRxMode(PAN_RX_MODE_SINGLE_TIMEOUT); /* Set the receive mode to single receive mode */
  PAN_SetRfState(PAN_STATE_RX);              /* Enter receive state */
}

/**
* @brief Get the received data length
* @return Received data length
* @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the receive length register
*/
uint8_t PAN_GetRxPayloadLen(void)
{
  return PAN_ReadPageReg(1, 0x7D);
}

/**
* @brief Function used to obtain the length and content of received data
* @param *Buffer Pointer address of the data area to be received
* @return Received data length
* @note This function must be called before the receive interrupt is cleared, as clearing the receive interrupt will clear the receive length register.
*/
uint8_t PAN_GetRecvPayload(uint8_t *Buffer)
{
  uint8_t Size;
  
  Size = PAN_GetRxPayloadLen();     /* Get the received data length */
  PAN_ReadRegs(0x01, Buffer, Size); /* Read the received data from the FIFO, where 0x01 is the FIFO register address */
  return Size;
}

/**
* @brief Get the RSSI value of the received data packet
* @return RSSI value
* @note RSSI value range: -125 to -10, unit: dBm, RSSI value is less than or equal to the sensitivity value
* @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the signal strength register to zero.
*/
int8_t PAN_GetPktRssi(void)
{
  return PAN_ReadPageReg(1, 0x7F);
}

/**
* @brief Get real-time RSSI value
* @return RSSI value
* @note RSSI value range: -125~-10, unit: dBm
* @note Before calling this function, ensure that the RF is in the receive state.
*/
int8_t PAN_GetRealTimeRssi(void)
{
  /* Clear Bit[2] of register 0x06 to 0 and then set it to 1 to update the value of [Page1][Reg0x7E] */
  PAN_ResetPageRegBits(2, 0x06, 0x04);
  PAN_SetPageRegBits(2, 0x06, 0x04);
  return (int8_t)PAN_ReadPageReg(1, 0x7E);
}

/**
* @brief Get the SNR value of the received data packet
* @return SNR value
* @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the SNR register
* @note SNR value range: -20~10, unit dB
*/
int32_t PAN_GetPktSnr(void)
{
  int32_t PktSnr = 0, SnrVal;
  uint8_t i, Temp[6];
  uint32_t NoiseStrength;
  uint32_t SingalStrength;

  PAN_ReadPageRegs(2, 0x71, &Temp[0], 3); // Noise strength
  PAN_ReadPageRegs(1, 0x74, &Temp[3], 3); // Singal strength
  SingalStrength = (((uint32_t)Temp[5] << 16) | ((uint32_t)Temp[4] << 8) | Temp[3]);
  NoiseStrength = (((uint32_t)Temp[2] << 16) | ((uint32_t)Temp[1] << 8) | Temp[0]);
  if (NoiseStrength == 0) NoiseStrength = 1;
  if(g_RfCfgParams.SpreadingFactor <= 9) SnrVal = (SingalStrength << (9 - g_RfCfgParams.SpreadingFactor)) / NoiseStrength;
  else SnrVal = (SingalStrength >> (g_RfCfgParams.SpreadingFactor - 9)) / NoiseStrength;
  for (i = 0; i < 31; i++)
  {
    if (SnrVal <= g_SnrLog10Talbe[i])
    {
      PktSnr = (int32_t)i - (int32_t)20;
      break;
    }
  }
  return PktSnr;
}

/**
* @brief Get interrupt flag
* @return IRQ flag
* - 0x00: No interrupt
* - 0x01: PAN_IRQ_TX_DONE
* - 0x02: PAN_IRQ_RX_TIMEOUT
* - 0x04: PAN_IRQ_CRC_ERR
* - 0x08: PAN_IRQ_RX_DONE
* - 0x40: PAN_IRQ_MAPM_DONE
*/
uint8_t PAN_GetIRQFlag(void)
{
  return (PAN_ReadPageReg(0, 0x6C) & 0x7F);
}

/**
* @brief Clear the interrupt flag
* @param <IRQFlag> Interrupt flag
*/
void PAN_ClrIRQFlag(uint8_t IRQFlag)
{
  PAN_WritePageReg(0, 0x6C, IRQFlag);
}

/**
* @brief Get the current frequency setting
*/
uint32_t PAN_GetFreq(void)
{
  return g_RfCfgParams.Frequency;
}

/**
* @brief Get the current IQ inversion value
*/
RfIQModes_t PAN_GetInvertIQ(void)
{
  return g_RfCfgParams.InvertIQ;
}

/**
* @brief Get the current preamble length setting
*/
uint16_t PAN_GetPreamLen(void)
{
  return g_RfCfgParams.PreambleLen;
}

/**
* @brief Get the current transmit power setting
*/
uint8_t PAN_GetTxPower(void)
{
  return g_RfCfgParams.TxPower;
}

/**
* @brief Get the current modulation bandwidth setting
*/
uint8_t PAN_GetBandWidth(void)
{
  return g_RfCfgParams.Bandwidth;
}

/**
* @brief Get the current spreading factor setting
*/
uint8_t PAN_GetSF(void)
{
  return g_RfCfgParams.SpreadingFactor;
}

/**
* @brief Get the current CRC check setting
*/
uint8_t PAN_GetCRC(void)
{
  return g_RfCfgParams.CrcMode;
}

/**
* @brief Get the current coding rate setting
*/
uint8_t PAN_GetCR(void)
{
    return g_RfCfgParams.CodingRate;
}

/**
* @brief Get the current sync word setting
*/
uint8_t PAN_GetSyncWord(void)
{
  return g_RfCfgParams.SyncWord;
}

/**
* @brief Get the current transmit mode setting
*/
uint8_t PAN_GetLDR(void)
{
  return g_RfCfgParams.LowDatarateOptimize;
}

/**
* @brief Get the time of a single symbol
* @param <bw> Bandwidth
* - PAN_BW_062K / PAN_BW_125K / PAN_BW_250K / PAN_BW_500K
* @param <sf> Spreading factor
* - PAN_SF5 / PAN_SF6 / PAN_SF7 / PAN_SF8 / PAN_SF9 / PAN_SF10 / PAN_SF11 / PAN_SF12
* @return Single symbol time, in us
* @note This function is used to calculate the time of a single symbol
*/
uint32_t PAN_GetOneSymbolTime(uint8_t bw, uint8_t sf)
{
  const uint32_t BwTable[4] = {62500, 125000, 250000, 500000};

  if(bw < PAN_BW_062K || bw > PAN_BW_500K) return 0;
  return (1000000 * (1 << sf) / BwTable[bw - PAN_BW_062K]);
}

/**
* @brief Calculates the time to send a packet.
* @param <Size> The size of the packet to be sent, in bytes.
* @return The time to send the packet, in milliseconds.
*/
uint32_t PAN_GetTxTimeMs(uint8_t Size)
{
  uint8_t sf, cr, bw, ldr;
  uint16_t PreambleLen; /* Preamble length */
  float SymbolTime;     /* Symbol time:ms */
  float PreambleTime;   /* Preamble time:ms */
  float PayloadTime;    /* Payload time:ms */
  float TotalTime;      /* Total time:ms */
  const float BwTable[4] = {62.5, 125, 250, 500};

  sf = PAN_GetSF();
  cr = PAN_GetCR();
  bw = PAN_GetBandWidth();
  ldr = PAN_GetLDR();
	PreambleLen = PAN_GetPreamLen();
  SymbolTime = (float)(1 << sf) / BwTable[bw - PAN_BW_062K]; /* Symbol time: ms */
  if (sf < 7)
  {
    PreambleTime = (PreambleLen + 6.25f) * SymbolTime;
    PayloadTime = ceil((float)(Size * 8 - sf * 4 + 36) / ((sf - ldr * 2) * 4));
  }
  else
  {
    PreambleTime = (PreambleLen + 4.25f) * SymbolTime;
    PayloadTime = ceil((float)(Size * 8 - sf * 4 + 44) / ((sf - ldr * 2) * 4));
  }
  TotalTime = PreambleTime + (PayloadTime * (cr + 4) + 8) * SymbolTime;
  if(TotalTime < 1) TotalTime = 1; /* When less than 1ms, treat it as 1ms */
  return (uint32_t)TotalTime;
}

/**
* @brief Enable MPM mode
*/
void PAN_EnableMapm(void)
{
  PAN_SetPageRegBits(1, 0x38, 0x01);      /* Enable mapm mode */
  PAN_WritePageRegBits(0, 0x58, 0, 0x40); /* Enable mapm interrupt */
}

/**
* @brief Disable MPM mode
*/
void PAN_DisableMapm(void)
{
  PAN_ResetPageRegBits(1, 0x38, 0x01);    /* Disable mapm mode */
  PAN_WritePageRegBits(0, 0x58, 1, 0x40); /* Disable mapm interrupt */
}

/**
* @brief Configure MPM parameters
* @param <pMapmCfg>
* fn: Number of fields in the mapm, totaling fn*(2^fnm) fields sent (usually fnm=0)
* fnm: Number of times the same field is sent repeatedly (default fnm=0)
* - When fnm=0, the same field is sent once, for a total of fn*1 fields sent
* - When fnm=1, the same field is sent twice, for a total of fn*2 fields sent
* - When fnm=2, the same field is sent four times, for a total of fn*4 fields sent
* - When fnm=3, the same field is sent eight times, for a total of fn*8 fields sent
* gfs: The payload function of the last group in each field
* - When gfs=0, the payload function of the last group is the address
* - When gfs=1, the payload function of the last group is the number of remaining fields (including the current field)
* gn: Number of groups in a field
* pg1: The number of preambles in the first group in the field, ranging from 8 to 255.
* - When pg1 = 8, it indicates that the first group has 8 preambles.
* - When pg1 = 200, it indicates that the first group has 200 preambles.
* pgn: The number of preambles in all groups except the first group in the field, ranging from 0 to 255.
* - When pgn = 8, it indicates that the first group has 8 preambles.
* - When pgn = 200, it indicates that the first group has 200 preambles.
* pn: The number of preambles in the packet, ranging from 1 to 65535.
* @note: The address or count value in the group occupies 2 chirps.
*/
void PAN_ConfigMapm(PAN_MapmCfg_t *pMapmCfg)
{
  uint8_t reg_fn, fn_h, fn_l;

  fn_h = pMapmCfg->fn / 15 + 1;
  fn_l = pMapmCfg->fn % 15 + 1;
  reg_fn = (fn_h << 4) + fn_l;
  PAN_WritePageReg(1, 0x3D, reg_fn);                            /* set the number of fields */
  PAN_WritePageRegBits(1, 0x37, pMapmCfg->fnm, 0x80 | 0x40);    /* set the unit code word of the field counter represents several fields */
  PAN_WritePageRegBits(1, 0x38, pMapmCfg->gfs, 0x02);           /* set the last group function selection */
  PAN_WritePageRegBits(1, 0x38, pMapmCfg->gn - 1, 0x08 | 0x04); /* set the number of groups in Field */
  PAN_WritePageReg(1, 0x3B, pMapmCfg->pg1);                     /* set the number of Preambles in first groups */
  /* set the number of preambles for groups other than the first group */
  PAN_WritePageReg(1, 0x3C, pMapmCfg->pgn);
  /* set the number of preamble between the last group and the sync word */
  PAN_WritePageRegBits(1, 0x39, (uint8_t)(pMapmCfg->pn >> 8), 0x0F);
  PAN_WritePageReg(1, 0x3A, (uint8_t)(pMapmCfg->pn));
}

/**
* @brief Set the group address in mapm mode
* @param <MapmAddr> mapm group address
* <AddrWidth> Address width, range 1 to 4
* @note The MapmAddr[0] on the receiving end must be consistent with the MapmAddr[0] on the sending end.
* Otherwise, the receiving end will not trigger a mapm interrupt.
* @note Mapm address register description:
* [Page1][Reg0x3E] is MapmAddr[0]
* [Page1][Reg0x3F] is MapmAddr[1]
* [Page1][Reg0x40] is MapmAddr[2]
* [Page1][Reg0x41] is MapmAddr[3]
*/
void PAN_SetMapmAddr(uint8_t *MapmAddr, uint8_t AddrWidth)
{
  PAN_WritePageRegs(1, 0x3E, MapmAddr, AddrWidth);
}

/**
* @brief Calculate the time it takes to calculate a field (ms)
* @param <pMapmCfg> mapm configuration parameters
* <SymbolTime> Single symbol (chirp) time
* @note The number of chirps in Group1 is (pg1 + 2), where pg1 is the number of preambles in Group 1, and 2 is the number of chirps occupied by the addresses in Group 1.
* @note The number of chirps in other groups is (pgn + 2) * (gn - 1), where pgn is the number of preambles in other individual groups,
* 2 is the number of chirps occupied by the addresses (or count values) in other individual groups, and (gn - 1) is the number of groups remaining after removing Group 1.
*/
uint32_t PAN_GetMapmOneFieldTime(PAN_MapmCfg_t *pMapmCfg, uint32_t SymbolTime)
{
  uint8_t pgn = pMapmCfg->pgn;
  uint8_t pg1 = pMapmCfg->pg1;
  uint8_t gn = pMapmCfg->gn;

  uint16_t ChirpNumInOneField = (pg1 + 2) + (pgn + 2) * (gn - 1);
  return ChirpNumInOneField * SymbolTime / 1000;
}

/**
* @brief Get the remaining Mapm time in mapm mode
* @param <pMapmCfg> mapm configuration parameter
* <SymbolTime> Single symbol (chirp) time
* @return Remaining Mapm time, in milliseconds
* @note The remaining Mapm time is the time from the current moment to the completion of sending the remaining fields.
* @note: The remaining field count includes the current field.
*/
uint32_t PAN_GetLeftMapmTime(PAN_MapmCfg_t *pMapmCfg, uint32_t SymbolTime)
{
  uint8_t fnm, gn, pgn, pg1, fn;
  uint16_t ChirpNumInOneField;
  uint16_t NumberOfLeftChirps;
  uint32_t LeftMapmTime;

  pgn = pMapmCfg->pgn;
  pg1 = pMapmCfg->pg1;
  gn = pMapmCfg->gn;
  fnm = pMapmCfg->fnm;
  fn = pMapmCfg->fn;
  /**
  * @brief Calculate the number of chirps in a field
  * @note The number of chirps in Group 1 is (pg1 + 2), where pg1 is the number of preambles in Group 1, and 2 is the number of chirps occupied by the addresses in Group 1.
  * @note The number of chirps in other groups is (pgn + 2) * (gn - 1), where pgn is the number of preambles in other groups,
  * (gn - 1) is the number of groups remaining after removing Group 1, and 2 is the number of chirps occupied by the addresses (or count values) in other groups.
  */
  ChirpNumInOneField = (pg1 + 2) + (pgn + 2) * (gn - 1);
  /**
  * @brief Calculate the number of remaining chirps
  * @note fn is the number of fields in mapm. If fnm > 0, the number of fields actually sent is fn * (2^fnm).
  * @note pn is the number of preambles between a field and the sync word, resulting in pn chirps in the air.
  * @note The number of chirps per field is subtracted to account for the time spent on the field itself.
  */
  NumberOfLeftChirps = (1 << fnm) * fn * ChirpNumInOneField - ChirpNumInOneField;
  /* The remaining time is the number of remaining chirps multiplied by the time of a single chirp. */
  LeftMapmTime = SymbolTime * NumberOfLeftChirps;
  return LeftMapmTime / 1000; /* Convert microseconds to milliseconds. */
}

/**
* @brief Start transmitting continuous carrier wave.
* @note The transmit power and frequency must be set before calling this function.
* @note After calling this function, the chip will remain in the transmitting state until the PAN_StopTxContinuousWave() function is called to stop transmitting.
*/
void PAN_StartTxContinuousWave(void)
{
  PAN_WriteReg(0x02, PAN_STATE_STB3);   /* Set the chip to idle state. */
  PAN_WritePageReg(0, 0x58, 0x00);     /* Disable all RF interrupts */
  PAN_SetTxMode(PAN_TX_MODE_CONTINOUS); /* Set continuous transmit mode */
  PAN_WritePageReg(1, 0x0C, 1);        /* Set transmit data length to 1 byte */
  PAN_TurnonPA();                      /* Enable the PA before transmitting data */
  PAN_WriteReg(0x02, PAN_STATE_TX);     /* Set the chip to transmit mode */
  PAN_WriteReg(0x01, 0xFF);            /* 0x01 is the FIFO register address. After writing data, start transmitting the carrier on the rising edge of CS */
  g_RfOperatetate = PAN_STATE_TX;
}

/**
* @brief Stop transmitting continuous carrier waves
* @note After calling this function, the chip stops transmitting and enters standby mode.
*/
void PAN_StopTxContinuousWave(void)
{
  PAN_WriteReg(0x02, PAN_STATE_STB3); /* Set the chip to idle state */
  PAN_TurnoffPA();                   /* Turn off the PA after transmission is complete */
  PAN_WritePageReg(0, 0x58, 0x0F);   /* Restore the RF default interrupt */
  g_RfOperatetate = PAN_STATE_STB3;
}

/**
* @brief Handle RF interrupt events
* @note This function can be called from an interrupt service routine;
* It can also be called in a polling manner to handle RF interrupt events.
*/
void PAN_IRQ_Process(void)
{
  if (CHECK_RF_IRQ()) /* RF interrupt detected, a high level indicates an interrupt */
  {
    uint8_t IRQFlag;

    IRQFlag = PAN_GetIRQFlag();    /* Get interrupt flags */
    if (IRQFlag & PAN_IRQ_TX_DONE) /* Transmit complete interrupt */
    {
      PAN_TurnoffPA();                /* Turn off PA after transmission completes */
      PAN_ClrIRQFlag(PAN_IRQ_TX_DONE); /* Clear transmit complete interrupt flag */
      IRQFlag &= ~PAN_IRQ_TX_DONE;
    }
    if (IRQFlag & PAN_IRQ_RX_DONE) /* Receive complete interrupt */
    {
      g_RfRxPkt.Snr = PAN_GetPktSnr();   /* Get the SNR value of the received packet */
      g_RfRxPkt.Rssi = PAN_GetPktRssi(); /* Get the RSSI value of the received packet */
      /* Get the received data and length */
      g_RfRxPkt.RxLen = PAN_GetRecvPayload((uint8_t *)g_RfRxPkt.RxBuf);
      PAN_ClrIRQFlag(PAN_IRQ_RX_DONE); /* Clear the receive complete interrupt flag */
      IRQFlag &= ~PAN_IRQ_RX_DONE;
    }
    if (IRQFlag & PAN_IRQ_MAPM_DONE) /* MAPM receive complete interrupt */
    {
      uint8_t MapmAddr = PAN_ReadPageReg(0, 0x6E);
      g_RfRxPkt.MapmRxBuf[g_RfRxPkt.MapmRxIndex++] = MapmAddr;
      PAN_ClrIRQFlag(PAN_IRQ_MAPM_DONE); /* Clear the MAMP receive completion interrupt flag */
      IRQFlag &= ~PAN_IRQ_MAPM_DONE;
    }
    if (IRQFlag & PAN_IRQ_CRC_ERR) /* CRC error interrupt */
    {
      PAN_ClrIRQFlag(PAN_IRQ_CRC_ERR); /* Clear the CRC error interrupt flag */
      IRQFlag &= ~PAN_IRQ_CRC_ERR;
    }
    if (IRQFlag & PAN_IRQ_RX_TIMEOUT) /* Receive timeout interrupt */
    {
      /* rf_refresh(); */
      IRQFlag &= ~PAN_IRQ_RX_TIMEOUT;
      PAN_ClrIRQFlag(PAN_IRQ_RX_TIMEOUT); /* Clear the receive timeout interrupt flag */
    }
    if (IRQFlag) PAN_ClrIRQFlag(IRQFlag); /* Clear the remaining interrupt flag */
  }
}

