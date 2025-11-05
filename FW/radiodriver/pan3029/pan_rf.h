/**
 * @file      pan_rf.h
 * @brief     PAN3029/PAN3060 driver implementation
 * @version   V1.0.1
 * @date      2025-08-18
 * @copyright Panchip Microelectronics Co., Ltd. All rights reserved.
 * @code
 *              ____              ____ _     _
 *             |  _ \ __ _ _ __  / ___| |__ (_)_ __
 *             | |_) / _` | '_ \| |   | '_ \| | '_ \
 *             |  __/ (_| | | | | |___| | | | | |_) |
 *             |_|   \__,_|_| |_|\____|_| |_|_| .__/
 *                                            |_|
 *              (C)2009-2025 PanChip
 * @endcode
 * @author    PanChip
 * @addtogroup ChirpIOT
 * @{
 * @defgroup  PAN3029/3060 Radio Driver API
 * @{
 */
#ifndef __PAN_PAN_H__
#define __PAN_PAN_H__

#include <stdint.h>
#include <math.h>
#include "pan_interface.h"


/**TODO: Adjust according to your hardware:
* @brief PAN3029/3060 GPIO Pin Definitions
*/
#define MODULE_GPIO_TX             0  /* PAN3029/3060 antenna transmit control pin. A high level turns the transmit antenna on, a low level turns it off. */
#define MODULE_GPIO_RX             10 /* PAN3029/3060 antenna receive control pin. A high level turns the receive antenna on, a low level turns it off. */
#define MODULE_GPIO_TCXO           3  /* PAN3029/3060 TCXO control pin, used to turn the TCXO power on and off. */
#define MODULE_GPIO_CAD_IRQ        11 /* PAN3029/3060 CAD interrupt pin. Outputs high when detecting a CAD signal, otherwise outputs low. */

/**
* @brief PAN3029/3060 frequency band definition
*/
#define REGION_CN470_510           0x00 /* China 470-510MHz band */
#define REGION_EU_863_870          0x01 /* Europe 863-870MHz band */
#define REGION_US_902_928          0x02 /* United States 902-928MHz band */
#define REGION_DEFAULT             REGION_CN470_510 /* Default frequency band is China 470-510MHz band */

/**
* @brief Defines the default parameters for the PAN3029/3060. Users can modify these RF parameters as needed.
*/
#define PAN_FREQ_DEFAULT            490000000   /* Default frequency configuration */
#define PAN_SF_DEFAULT              PAN_SF7      /* Default spreading factor configuration */
#define PAN_BW_DEFAULT              PAN_BW_500K  /* Default bandwidth configuration */
#define PAN_CR_DEFAULT              PAN_CR_4_5   /* Default channel coding rate configuration */
#define PAN_CRC_DEFAULT             PAN_CRC_ON   /* Default CRC checksum configuration */
#define PAN_LDR_DEFAULT             PAN_LDR_OFF  /* Default low-rate optimization configuration */
#define PAN_PREAMBLE_DEFAULT        8           /* Default preamble length configuration */
#define PAN_IQ_INVERT_DEFAULT       FALSE       /* Default IQ modulation configuration */

/**
* @brief PAN3029/3060 external crystal and reset pin configuration
*/
#define USE_ACTIVE_CRYSTAL         0 /* Whether to use an external active crystal (0: not used, 1: used) */
#define USE_PAN_RST_GPIO            0 /* Whether to use an MCU pin to control the RF reset function (0: not used, 1: used) */

/**
* @brief PAN3029/3060 register readback confirmation function on/off
* @note This function is used to confirm the success of a register write by reading the register value.
*/
#define USE_PAN_REG_CHECK           0 /* Whether to use the register readback confirmation function (0: not used, 1: used) */
#if USE_PAN_REG_CHECK
#define PAN_ASSERT(fn)        \
  do                       \
  {                        \
    if (PAN_OK != fn)     \
    {                    \
      return PAN_FAIL;  \
    }                    \
  } while (0);
#else
#define PAN_ASSERT(fn)  fn
#endif

/**
* @brief PAN3029/3060 interrupt flag definition
*/
#define PAN_IRQ_TX_DONE                  0x01 /* Transmit completion interrupt flag */
#define PAN_IRQ_RX_TIMEOUT               0x02 /* Single receive timeout interrupt flag */
#define PAN_IRQ_CRC_ERR                  0x04 /* CRC error interrupt flag */
#define PAN_IRQ_RX_DONE                  0x08 /* Receive complete interrupt flag */
#define PAN_IRQ_MAPM_DONE                0x40 /* MAPM interrupt flag */

/**
* @brief PAN3029/3060 modulation mode definition
*/
#define MODEM_MODE_NORMAL               0x00 /* Normal modulation mode */
#define MODEM_MODE_MULTI_SECTOR         0x01 /* Multi-segment modulation mode */

/**
* @brief PAN3029/3060 synchronization word definition
* @note: The PAN3029/3060 synchronization word is a single byte, ranging from 0x00 to 0xFF.
*/
#define PAN_MAC_PRIVATE_SYNCWORD         0x12 /* Private network synchronization word. The default value is 0x12. */
#define PAN_MAC_PUBLIC_SYNCWORD          0x34 /* Public network synchronization word */

/**
* @brief PAN3029/3060 power range definition
*/
#define PAN_MIN_RAMP                     1  /* Minimum power range */
#define PAN_MAX_RAMP                     22 /* Maximum power range */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**
* @brief RF-related operation return value definition
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/

#define PAN_OK			0
#define PAN_FAIL		-1

/**
* @brief Working status definition of PAN3029/3060 
*/
typedef enum
{
  PAN_STATE_DEEPSLEEP = 0x00, //!< The radio is in deep sleep mode
  PAN_STATE_SLEEP = 0x01,     //!< The radio is in sleep mode
  PAN_STATE_STB1 = 0x02,      //!< The radio is in standby mode 1
  PAN_STATE_STB2 = 0x03,      //!< The radio is in standby mode 2
  PAN_STATE_STB3 = 0x04,      //!< The radio is in standby mode 3
  PAN_STATE_TX = 0x05,        //!< The radio is in transmit mode
  PAN_STATE_RX = 0x06,        //!< The radio is in receive mode
} RfOpState_t;

/** 
*@brief PAN3029/3060 GPIO Mode Definitions
* - PAN_GPIO_MODE_INPUT: GPIO input mode
* - PAN_GPIO_MODE_OUTPUT: GPIO output mode
*/
typedef enum
{
  PAN_GPIO_MODE_INPUT = 0x00, //!< GPIO input mode
  PAN_GPIO_MODE_OUTPUT = 0x01,//!< GPIO output mode
} RfGpioMode_t;

/**
* @brief PAN3029/3060 Transmission Mode Definitions
* - PAN_TX_MODE_SINGLE: Single transmission mode, used in most cases
* - PAN_TX_MODE_CONTINOUS: Continuous transmission mode, commonly used for carrier signal transmission
*/
typedef enum
{
  PAN_TX_MODE_SINGLE = 0x00,    //!< Single transmission mode
  PAN_TX_MODE_CONTINOUS = 0x01, //!< Continuous transmission mode
} RfTxMode_t;

/**
* @brief PAN3029/3060 receive mode definitions
* - PAN_RX_MODE_SINGLE: Single receive mode. Automatically enters standby mode after receiving a packet. This mode is generally not used.
* - PAN_RX_MODE_SINGLE_TIMEOUT: Single receive mode with timeout. Automatically enters standby mode after timeout.
* - PAN_RX_MODE_CONTINOUS: Continuous receive mode. Continues receiving after receiving a packet.
*/
typedef enum
{
  PAN_RX_MODE_SINGLE = 0x00,         //!< Single reception mode
  PAN_RX_MODE_SINGLE_TIMEOUT = 0x01, //!< Single reception with timeout mode
  PAN_RX_MODE_CONTINOUS = 0x02,      //!< Continuous reception mode
} RfRxMode_t;

/**
* @brief PAN3029/3060 power supply mode definitions
* - USE_LDO: Use LDO for power supply. Current is slightly higher, but receiver sensitivity is slightly better.
* - USE_DCDC: Use DCDC for power supply. Current is slightly lower, but receiver sensitivity is slightly worse by 1-2 dB.
*/
typedef enum
{
  USE_LDO = 0x00,  //!< Use LDO for power regulation
  USE_DCDC = 0x01, //!< Use DCDC for power regulation
} RfRegulatorMode_t;

/**
* @brief PAN3029/3060 chip mode definitions
* - CHIPMODE_MODE0
* - CHIPMODE_MODE1
*/
typedef enum
{
  CHIPMODE_MODE0 = 0, //!< Mode 0
  CHIPMODE_MODE1 = 1, //!< Mode 1
} RfChipMode_t;

/**
* @brief CAD detection threshold configuration definition
* - A smaller threshold value allows detection of weaker signals, but the false alarm rate increases.
* - A larger threshold value reduces the false alarm rate, but the detectable signal strength range decreases.
*/
typedef enum
{
  PAN_CAD_THRESHOLD_0A = 0x0A,
  PAN_CAD_THRESHOLD_10 = 0x10,
  PAN_CAD_THRESHOLD_15 = 0x15,
  PAN_CAD_THRESHOLD_20 = 0x20,
} RfCadThreshold_t;

/**
* @brief CAD detection symbol count configuration definition
*/
typedef enum
{
  PAN_CAD_01_SYMBOL = 0x01, //!< The actual detection time may be longer than the time for one symbol.
  PAN_CAD_02_SYMBOL = 0x02, //!< Actual detection time may be greater than 2 symbols
  PAN_CAD_03_SYMBOL = 0x03, //!< Actual detection time may be greater than 3 symbols
  PAN_CAD_04_SYMBOL = 0x04, //!< Actual detection time may be greater than 4 symbols
} RfCadSymbols_t;

/**
 * @brief Represents the possible spreading factor values in ChirpIot packet types
 */
typedef enum
{
  PAN_SF5 = 0x05, //!< 5 spreading factor
  PAN_SF6 = 0x06, //!< 6 spreading factor
  PAN_SF7 = 0x07, //!< 7 spreading factor
  PAN_SF8 = 0x08, //!< 8 spreading factor
  PAN_SF9 = 0x09, //!< 9 spreading factor
  PAN_SF10 = 0x0A, //!< 10 spreading factor 
  PAN_SF11 = 0x0B, //!< 11 spreading factor 
  PAN_SF12 = 0x0C, //!< 12 spreading factor
} RfSpreadFactor_t;

/**
 * @brief Represents the bandwidth values for ChirpIot packet type
 */
typedef enum
{
  PAN_BW_062K = 6, //!< 62.5KHz bandwidth
  PAN_BW_125K = 7, //!< 125KHz bandwidth
  PAN_BW_250K = 8, //!< 250KHz bandwidth
  PAN_BW_500K = 9, //!< 500KHz bandwidth
} RfBandwidths_t;

/**
 * @brief Represents the coding rate values for ChirpIot packet type
 * @note  The coding rate is expressed as 4/x where x is the value below
 */
typedef enum
{
  PAN_CR_4_5 = 0x01, //!< 4/5 coding rate
  PAN_CR_4_6 = 0x02, //!< 4/6 coding rate
  PAN_CR_4_7 = 0x03, //!< 4/7 coding rate
  PAN_CR_4_8 = 0x04, //!< 4/8 coding rate
} RfCodingRates_t;

/**
* @brief CRC mode
* @note PAN_CRC_ON: Enable CRC check
* @note PAN_CRC_OFF: Disable CRC check, requiring the user to verify data integrity
*/
typedef enum
{
  PAN_CRC_OFF = 0x00, //!< CRC not used
  PAN_CRC_ON = 0x01,  //!< CRC activated
} RfCrcModes_t;

/**
* @brief Low rate optimization mode 
* @note PAN_LDR_ON: Low data rate optimization activated 
* @note PAN_LDR_OFF: Low data rate optimization not used 
*/
typedef enum
{
  PAN_LDR_OFF = 0x00, //!< Low data rate optimization not used
  PAN_LDR_ON = 0x01,  //!< Low data rate optimization activated
} RfLdr_t;

/**
 * @brief Represents the IQ mode for ChirpIot packet type
 */
typedef enum
{
  PAN_IQ_NORMAL = 0x00,   //!< Normal IQ mode,default
  PAN_IQ_INVERTED = 0x01, //!< Inverted IQ mode
} RfIQModes_t;

typedef enum
{
  PAN_MAPM_GRP_ADDR = 0x00,    //!< Address group
  PAN_MAPM_GRP_COUNTER = 0x01, //!< Counter group
} RfMapmGrpType_t;

/**
 * @brief  RF configuration structure of mapm
 * @member Addr: the address of mapm, which is 4 bytes
 * @member fn: the number of field before the standard preamble
 * @member fnm: the clone number of one field, which can be 0, 1, 2, 3, 
 *              fnm=0 indicates clone the field 1 times
 *              fnm=1 indicates clone the field 2 times
 *              fnm=2 indicates clone the field 4 times
 *              fnm=3 indicates clone the field 8 times
 * @member gfs: the function select of last address, which can be 0 or 1
 *              0: the last address is ordinary address
 *              1: the last address is field counter
 * @member gn: the number of group in one filed, which can be 1, 2, 3, 4
 *              gn=1 indicates 1 group(when gn=1, the field has only one group, fgs must be 0)
 *              gn=2 indicates 2 groups
 *              gn=3 indicates 3 groups
 *              gn=4 indicates 4 groups
 * @member pg1: the number of preamble in the first group, which can be 8~255
 *              pg1=8 indicates 8 chirps in the first group
 *              pg1=255 indicates 255 chirps in the first group
 * @member pgn: the number of preamble in the other group, which can be 0~255
 *              pgn=0 indicates 0 chirp in the other group, addr2~addr3 will be combined to addr1
 *              pgn=255 indicates 255 chirps in the other group
 * @member pn: the number of preamble between fields and syncword, which can be 1~65535
 * @note   the total numer of chirp before syncword: Pl=(pg1+1+pgn+1)*gn*fn*Fmux+pn
 * @note The mapm frame structure is as follows:
 *       1. The number of fields is fn, so N in the following figure equals fn;
 *       2. Preamble length in the figure is pn which is equal to the preamble length of normal
 *          ChirpIot packet;
 * |<---------------------------------- Mapm Frame------------------------------------------>|
 * | Field1 | Field2 | Field3 | ... | FieldN | Preamble | SyncWord | Header | Payload | CRC  |
 * |-----------------------------------------------------------------------------------------|
 * 
 * @note The mapm field structure is as follows(gn=4 && gfs=1):
 *       1. Each field has 4 groups, the preamble len of the fist group must be bigger than 8 
 *          and less than 255; the preamble len of the other groups can be 0~255;
 *       2. Each group carries 1byte payload, the type of first three groups is address, and
 *          the type of last(fourth) group is counter which indicates the number of left fields;
 *       3. The coding rate of the address and counter is 4/8, so the number of chirps in the 
 *          first group is pg1 + 2, and the number of chirps in the other groups is pgn + 2;
 * |<---------------------------------- Field ---------------------------------------------->|
 * |        Group1       |         Group2       |         Group3       |        Group4       |
 * |-----------------------------------------------------------------------------------------|
 * | Preamble | Address1 | Preamble | Address2  | Preamble | Address3  | Preamble | Counter  |
 * |-----------------------------------------------------------------------------------------|
 * 
 * @note The mapm field structure is as follows(gn=3 && gfs=1):
 *       1. Each field has 4 groups, the preamble len of the fist group must be bigger than 8 
 *          and less than 255; the preamble len of the other groups can be 0~255;
 *       2. Each group carries 1byte payload, the type of first two groups is address, and
 *          the type of last(third) group is counter which indicates the number of left fields;
 *       3. The coding rate of the address and counter is 4/8, so the number of chirps in the 
 *          first group is pg1 + 2, and the number of chirps in the other groups is pgn + 2;
 * |<---------------------------------- Field ----------------------->|
 * |        Group1       |         Group2       |        Group3       |
 * |------------------------------------------------------------------|
 * | Preamble | Address1 | Preamble | Address2  | Preamble | Counter  |
 * |------------------------------------------------------------------|
 */
typedef struct
{
  uint8_t Addr[4];      //!< the address of mapm, which is 4 bytes
  uint8_t fn;           //!< the number of field before the standard preamble
  uint8_t fnm;          //!< the clone number of one field, which can be 0, 1, 2, 3
  uint8_t gn;           //!< the number of group in one filed
  uint8_t gfs;          //!< the function select of last address
  uint8_t pg1;          //!< the number of preamble in the first group
  uint8_t pgn;          //!< the number of preamble in the other group
  uint16_t pn;          //!< the number of preamble between fields and syncword
} PAN_MapmCfg_t;

/**
 * @brief Represents the possible operating states of the radio
 */
typedef struct
{
  uint8_t RxLen;          //!< Size of the payload in the ChirpIot packet
  uint8_t RxBuf[255];     //!< Buffer to store the received payload
  uint8_t MapmRxIndex;    //!< Index of the received payload in the ChirpIot packet
  uint8_t MapmRxBuf[16];  //!< Buffer to store the received payload in mapm mode
  int8_t Rssi;            //!< The RSSI of the received packet
  int8_t Snr;             //!< The SNR of the received packet
} RfRxPkt_t;

/**
* @brief PAN3029/3060 configuration parameter structure
* @note: Convenient for users to quickly retrieve previously set parameters
*/
typedef struct
{
    uint8_t TxPower;                  //!< The power level to be used for transmission
    uint32_t Frequency;               //!< The frequency to be used for transmission and reception
    RfSpreadFactor_t SpreadingFactor; //!< Spreading Factor for the ChirpIot modulation
    RfBandwidths_t Bandwidth;         //!< Bandwidth for the ChirpIot modulation
    RfCodingRates_t CodingRate;       //!< Coding rate for the ChirpIot modulation
    RfCrcModes_t CrcMode;             //!< Size of CRC block in ChirpIot packet
    RfIQModes_t InvertIQ;             //!< Allows to swap IQ for ChirpIot packet
    uint8_t SyncWord;                 //!< Sync word byte
    uint8_t LowDatarateOptimize;      //!< Indicates if the modem uses the low datarate optimization
    uint16_t PreambleLen;             //!< The preamble length is the number of ChirpIot symbols in the preamble
    RfChipMode_t ChipMode;            //!< The Chip for Communication with This Device
    RfRegulatorMode_t RegulatorMode;  //!< The regulator mode for the radio
} RfConfig_t;

/**
* @brief PAN3029/3060 received packet structure
* @note This structure is used to store received packets, including data length, data buffer, SNR, and RSSI information.
*/
extern volatile RfRxPkt_t g_RfRxPkt;

/**
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */

/**
* @brief Select register page
* @param Page The register page to be selected, page range 0-3
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
* @note If the current page is already the required page, no register configuration is required
*/
int8_t PAN_SetPage(uint8_t Page);
/**
* @brief Write a single byte to the register of the specified page
* @param Page Register page to be written, page range 0-3
* @param Addr Register address to be written
* @param Value Single byte of data to be written to the register
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_WritePageReg(uint8_t Page, uint8_t Addr, uint8_t Value);
/**
* @brief Writes multiple bytes to the register range of a specified page
* @param Page Register page to be written, page range 0-3
* @param Addr Register address to be written
* @param Buffer Buffer pointer to the register to be written
* @param Size Number of bytes to be written
*/
void PAN_WritePageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size);
/**
* @brief Reads a single byte from a register in the specified page.
* @param Page The register page to be read, page range 0-3
* @param Addr The register address to be read
* @return uint8_t The value read from the register
*/
uint8_t PAN_ReadPageReg(uint8_t Page, uint8_t Addr);
/**
* @brief Reads multiple bytes from a register range in the specified page.
* @param Page The register page to be read, page range 0-3
* @param Addr The register address to be read
* @param Buffer Pointer to the buffer storing the read data
* @param Size Number of bytes to read
*/
void PAN_ReadPageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size);
/**
* @brief Set register bits in the specified page
* @param Page Register page to read, page range 0-3
* @param Addr Register address to set
* @param Mask Bit mask to set
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_SetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask);
/**
* @brief Resets register bits in the specified page
* @param Page Register page to read, page range 0-3
* @param Addr Register address to reset
* @param Mask Bit mask to reset
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_ResetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask);
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
int8_t PAN_WritePageRegBits(uint8_t Page, uint8_t Addr, uint8_t Value, uint8_t Mask);
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
int8_t PAN_ConfigGpio(uint8_t GpioPin, uint8_t GpioMode);
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
int8_t PAN_WriteGpioLevel(uint8_t GpioPin, uint8_t Level);
/**
* @brief Read GPIO level
* @param <GpioPin> Pin number
* @return Read GPIO level
* - 0: Low level
* - 1: High level
*/
uint8_t PAN_ReadGpioLevel(uint8_t GpioPin);
/**
* @brief Initializes the PAN3029/3060 antenna control GPIO
* @note This function initializes the PAN3029/3060 antenna control GPIO, configures it in output mode, and sets the initial level to low.
* @note If you use the MCU's GPIO to control the antenna switch, you need to re-adapt this function.
*/
void PAN_InitAntGpio(void);
/**
* @brief Turn on the PAN3029/3060 transmit antenna
* @note This function turns on the PAN3029/3060 transmit antenna, setting the TX pin high and the RX pin low.
* @note If you use the MCU's GPIO to control the antenna switch, you need to re-adapt this function.
*/
void PAN_TurnonTxAnt(void);
/**
* @brief Turn on the PAN3029/3060 receive antenna
* @note This function turns on the PAN3029/3060 receiving antenna, setting the RX pin high and the TX pin low.
* @note If you use the MCU's GPIO to control the antenna, you need to reconfigure this function.
*/
void PAN_TurnonRxAnt(void);
/**
* @brief Turns off the PAN3029/3060 antenna.
* @note This function turns off the PAN3029/3060 antenna, setting both the RX and TX pins low.
* @note If you use the MCU's GPIO to control the antenna, you need to reconfigure this function.
*/
void PAN_ShutdownAnt(void);
/**
* @brief Initializes the TCXO control GPIO
* @note This function initializes the PAN3029/3060 TCXO control GPIO, configures it in output mode, and sets the initial level to high.
* @note If you use the MCU's GPIO to control the TCXO on/off, you need to re-adapt this function.
*/
void PAN_InitTcxoGpio(void);
/**
* @brief Turns on the TCXO power supply
* @note This function turns on the PAN3029/3060 TCXO and sets the TCXO pin to high.
* @note If you use the MCU's GPIO to control the TCXO on/off, you need to reconfigure this function.
*/
void PAN_TurnonTcxo(void);
/**
* @brief Turns off the TCXO power supply.
* @note This function turns off the PAN3029/3060's TCXO and sets the TCXO pin to low.
* @note If you use the MCU's GPIO to control the TCXO on/off, you need to reconfigure this function.
*/
void PAN_TurnoffTcxo(void);
/**
* @brief Enable LDO PA
*/
void PAN_TurnonLdoPA(void);
/*
* @brief Disable LDO PA
*/
void PAN_TurnoffLdoPA(void);
/**
* @brief Turn off internal and external PAs
*/
void PAN_TurnoffPA(void);
/**
* @brief Turn on internal and external PAs
*/
void PAN_TurnonPA(void);
/**
* @brief Set chip mode.
* @param <ChipMode> Chip mode.
* - CHIPMODE_MODE0
* - CHIPMODE_MODE1
*/
void PAN_SetChipMode(RfChipMode_t ChipMode);
/**
* @brief Get chip mode
* @param -
*/
RfChipMode_t PAN_GetChipMode(void);
/**
* @brief Read byte from information area
* @param <Addr> Register address
* <Pattern> Pattern match value
* <InfoAddr> Information area address
* @return Byte value read from information area
*/
uint8_t PAN_ReadInfoByte(uint8_t Addr, uint16_t Pattern, uint8_t InfoAddr);
/**
* @brief Calibrate RF related parameters
* @return int8_t Return operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_Calibrate(void);
/**
* @brief Configure AGC function
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_ConfigAgc(void);
/**
* @brief Configures the default parameters of the RF registers
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
*/
int8_t PAN_ConfigDefaultParams(void);
/**
* @brief Initializes the RF transceiver to STB3 state after power-up
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
* @note Before calling this function, you must configure the MCU's SPI and related GPIO pins.
*/
int8_t PAN_Init(void);
/**
* @brief Configure the user parameters of the RF chip
*/
void PAN_ConfigUserParams(void);
/**
* @brief Software reset of the RF chip control logic
*/
void PAN_ResetLogic(void);
/**
* @brief Get the operating status of the RF chip
* @return RfOpState_t Current operating state
* - PAN_STATE_SLEEP: Chip is in sleep mode
* - PAN_STATE_STB3: Chip is in standby mode
* - PAN_STATE_TX: Chip is in transmit mode
* - PAN_STATE_RX: Chip is in receive mode
*/
RfOpState_t PAN_GetOperateState(void);
/**
* @brief Set the operating state of the RF chip
* @param <RfState> Operating state
* - PAN_STATE_SLEEP: Chip is in sleep mode
* - PAN_STATE_STB3: Chip is in standby mode
* - PAN_STATE_TX: Chip is in transmit mode
* - PAN_STATE_RX: Chip is in receive mode
*/
void PAN_SetOperateState(RfOpState_t RfState);
/**
* @brief Set the RF chip's operating state.
* @param <RfState>
* - PAN_STATE_DEEPSLEEP
* - PAN_STATE_SLEEP
* - PAN_STATE_STB3
* - PAN_STATE_TX
* - PAN_STATE_RX
*/
void PAN_SetRfState(uint8_t RfState);
/**
* @brief Enter deep sleep mode.
* @note This function puts the RF chip into deep sleep mode, turning off the antenna power supply and TCXO power supply.
* @note This function sets the chip's operating state to MODE_DEEPSLEEP.
* @note After executing this function, if you need to wake up the RF chip, you need to call the PAN_Init() function to wake up the chip.
*/
void PAN_EnterDeepsleepState(void);
/**
* @brief Enter sleep mode
* @note This function is used to put the RF chip into sleep mode, turn off the antenna power supply and TCXO power supply
* @note This function will set the chip's working state to MODE_SLEEP
* @note After executing this function, if you need to wake up the RF chip, you need to call the PAN_ExitSleepState() function
*/
void PAN_EnterSleepState(void);
/**
* @brief Exit sleep mode
* @note This function is used to exit the RF chip from sleep mode and turn on the antenna power supply and TCXO power supply.
* @note This function will set the chip's operating state to MODE_STDBY
*/
void PAN_ExitSleepState(void);
/**
* @brief Enter standby mode
* @note This function sets the chip's operating state to MODE_STDBY.
*/
void PAN_EnterStandbyState(void);
/**
* @brief Checks whether the RF chip is in sleep mode.
* @note This function checks whether the RF chip is in sleep mode.
* @note If it is in sleep mode, it exits sleep mode and enters standby mode.
* @note This function sets the chip's operating state to MODE_STDBY.
*/
void PAN_CheckDeviceReady(void);
/**
* @brief Sets the chip's power supply mode.
* @param <RegulatorMode> Power supply mode.
* - USE_LDO: Use LDO for power supply.
* - USE_DCDC: Use DCDC power supply.
* @note: In transmit mode, the RF must use LDO power supply mode.
* In other modes, any power supply mode can be selected.
*/
void PAN_SetRegulatorMode(RfRegulatorMode_t RegulatorMode);
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
int8_t PAN_SetFreq(uint32_t Frequency);
/**
* @brief Set IQ inversion
* @param <NewState> Enable or disable IQ inversion
* - true: Enable IQ inversion 
* - false: disable IQ inversion 
*/
void PAN_SetInvertIQ(bool NewState);
/**
* @brief Set the preamble length
* @param <PreamLen> Preamble length value
* Range is 4 - 65535
*/
void PAN_SetPreamLen(uint16_t PreamLen);
/**
* @brief Set the synchronization word
* @param <syncWord> Synchronization word value
* @note The synchronization word size supported by PAN3029/3060 is 1 byte
* @note The sync word is used for synchronization detection when receiving data packets. Typically, the same sync word should be set when sending and receiving data packets.
* For example, if the sync word is set to 0x12 when sending a data packet, the sync word should also be set to 0x12 when receiving a data packet.
*/
void PAN_SetSyncWord(uint8_t SyncWord);
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
void PAN_SetTxPower(uint8_t TxPower);
/**
* @brief Set modulation bandwidth
* @param <BandWidth> Modulation bandwidth value
* - PAN_BW_062K / PAN_BW_125K / PAN_BW_250K / PAN_BW_500K
* @note: A larger modulation bandwidth increases the data rate but shortens the transmission distance.
* @note: The modulation bandwidth range of the PAN3029 chip is PAN_BW_062K - PAN_BW_500K
* @note: The modulation bandwidth range of the PAN3060 chip is PAN_BW_125K - PAN_BW_500K
*/
void PAN_SetBW(uint8_t BandWidth);
/**
* @brief Set the spreading factor
* @param <SpreadFactor> Spreading factor value
* - PAN_SF5 / PAN_SF6 / PAN_SF7 / PAN_SF8 / PAN_SF9 / PAN_SF10 / PAN_SF11 / PAN_SF12
* @note A larger spreading factor increases the transmission distance but reduces the data rate.
* @note The spreading factor range for the PAN3029 chip is PAN_SF5 - PAN_SF12
* @note The spreading factor range for the PAN3060 chip is PAN_SF5 - PAN_SF9
*/
void PAN_SetSF(uint8_t SpreadFactor);
/**
* @brief Set channel coding rate
* @param <CodingRate> Channel coding rate value
* - PAN_CR_4_5 / PAN_CR_4_6 / PAN_CR_4_7 / PAN_CR_4_8
*/
void PAN_SetCR(uint8_t CodingRate);
/**
* @brief Set CRC check
* @param <CrcMode> Enable or disable CRC check
* - PAN_CRC_ON: Enable CRC check
* - PAN_CRC_OFF: Disable CRC check
*/
void PAN_SetCRC(uint8_t CrcMode);
/**
* @brief Set low data rate mode
* @param <LdrMode> Low data rate mode value
* - PAN_LDR_ON: Enable low data rate mode
* - PAN_LDR_OFF: Disable low data rate mode
*/
void PAN_SetLDR(uint8_t LdrMode);
/**
* @brief Set modem mode
* @param <modem_mode>
* - MODEM_MODE_NORMAL
* - MODEM_MOdDE_MULTI_SECTOR 
* @note This function should be called after PAN_SetSF(uint8_t SpreadFactor) 
*/
void PAN_SetModemMode(uint8_t ModemMode);
/**
* @brief Set the transmit mode
* @param Buffer Data buffer to be sent
* @param Size Number of data bytes to be sent
* @note Ensure that the RF is in standby (STB3) mode before calling this function
* @note This function is in single-shot transmit mode and will automatically enter standby (STB3) mode after the transmission is complete.
* @note A TX_DONE interrupt will be triggered after the transmission is complete.
*/
void PAN_SetTx(uint8_t *Buffer, uint8_t Size);
/**
* @brief Set the receive mode
* @param TimeoutMs Receive timeout in milliseconds
* - 0: Continuous Receive Mode
* - >0: Single Receive Mode. After a timeout, a timeout interrupt is generated and the device automatically enters Standby (STB3) mode.
* @note: Ensure that the RF is already in Standby (STB3) mode before calling this function.
*/
void PAN_SetRx(uint32_t TimeoutMs);
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
void PAN_StartCad(uint8_t Threshold, uint8_t Chirps);
/** 
* @brief Set CAD detection threshold 
* @param <Threshold> CAD detection threshold 
* - PAN_CAD_THRESHOLD_0A 
* - PAN_CAD_THRESHOLD_10 
* - PAN_CAD_THRESHOLD_15 
* - PAN_CAD_THRESHOLD_20 
*/
void PAN_SetCadThreshold(uint8_t Threshold);
/**
* @brief Set the number of CAD detection symbols
* @param <Chirps> Number of CAD detection symbols
* - PAN_CAD_01_SYMBOL
* - PAN_CAD_02_SYMBOL
* - PAN_CAD_03_SYMBOL
* - PAN_CAD_04_SYMBOL
*/
void PAN_SetCadChirps(uint8_t Chirps);
/**
* @brief Stop CAD detection
*/
void PAN_StopCad(void);
/**
* @brief Set the transmit mode
* @param <TxMode>
* - PAN_TX_SINGLE: Single transmit mode
* - PAN_TX_CONTINOUS: Continuous transmit mode
* @note This function is only used to set the transmit mode and does not change the chip's operating state.
*/
void PAN_SetTxMode(uint8_t TxMode);
/**
* @brief Send a single data packet
* @param <Buffer> Data buffer to be sent
* @param <Size> Number of data bytes to be sent
* @note TX_DONE interrupt will be triggered after transmission is completed
* @note In the transmission completion interrupt, the PAN_TurnoffPA() function needs to be called to turn off the chip's internal and external PAs.
*/
void PAN_TxSinglePkt(uint8_t *Buffer, uint8_t Size);
/**
* @brief Set the receive mode
* @param <RxMode>
* - PAN_RX_SINGLE: Single receive mode. Automatically enters standby mode after receiving a packet of data.
* - PAN_RX_SINGLE_TIMEOUT: Single receive mode with timeout. Automatically enters standby mode after timeout.
* - PAN_RX_CONTINOUS: Continuous receive mode, continues receiving after receiving a packet of data.
* @note This function only sets the receive mode and does not change the chip's operating state.
*/
void PAN_SetRxMode(uint8_t RxMode);
/**
* @brief Puts the chip into continuous receive mode.
* @note After calling this function, the chip enters continuous receive mode.
* @note This function sets the chip's operating state to MODE_RX.
*/
void PAN_EnterContinousRxState(void);
/**
* @brief Set the receive timeout
* @param <TimeoutMs> Timeout in milliseconds
* Timeout range: 0~65535ms
* @note This function is only used to set the receive timeout and does not change the chip's working state.
*/
void PAN_SetRxTimeout(uint16_t TimeoutMs);
/**
* @brief Let the chip enter the single receive state with timeout
* @param <TimeoutMs> Timeout time, in ms
* Timeout range: 1~65535ms
* @note After calling this function, the chip will enter the single receive state with timeout
* @note This function will set the chip's working state to MODE_RX
*/
void PAN_EnterSingleRxWithTimeout(uint16_t TimeoutMs);
/**
* @brief Get the received data length
* @return Received data length
* @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the receive length register
*/
uint8_t PAN_GetRxPayloadLen(void);
/**
* @brief Function used to obtain the length and content of received data
* @param *Buffer Pointer address of the data area to be received
* @return Received data length
* @note This function must be called before the receive interrupt is cleared, as clearing the receive interrupt will clear the receive length register.
*/
uint8_t PAN_GetRecvPayload(uint8_t *Buffer);
/**
* @brief Get the RSSI value of the received data packet
* @return RSSI value
* @note RSSI value range: -125 to -10, unit: dBm, RSSI value is less than or equal to the sensitivity value
* @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the signal strength register to zero.
*/
int8_t PAN_GetPktRssi(void);
/**
* @brief Get real-time RSSI value
* @return RSSI value
* @note RSSI value range: -125~-10, unit: dBm
* @note Before calling this function, ensure that the RF is in the receive state.
*/
int8_t PAN_GetRealTimeRssi(void);
/**
* @brief Get the SNR value of the received data packet
* @return SNR value
* @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the SNR register
* @note SNR value range: -20~10, unit dB
*/
int32_t PAN_GetPktSnr(void);
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
uint8_t PAN_GetIRQFlag(void);
/**
* @brief Clear the interrupt flag
* @param <IRQFlag> Interrupt flag
*/
void PAN_ClrIRQFlag(uint8_t IRQFlag);
/**
* @brief Get the current frequency setting
*/
uint32_t PAN_GetFreq(void);
/**
* @brief Get the current IQ inversion value
*/
RfIQModes_t PAN_GetInvertIQ(void);
/**
* @brief Get the current preamble length setting
*/
uint16_t PAN_GetPreamLen(void);
/**
* @brief Get the current transmit power setting
*/
uint8_t PAN_GetTxPower(void);
/**
* @brief Get the current modulation bandwidth setting
*/
uint8_t PAN_GetBandWidth(void);
/**
* @brief Get the current spreading factor setting
*/
uint8_t PAN_GetSF(void);
/**
* @brief Get the current CRC check setting
*/
uint8_t PAN_GetCRC(void);
/**
* @brief Get the current coding rate setting
*/
uint8_t PAN_GetCR(void);
/**
* @brief Get the current sync word setting
*/
uint8_t PAN_GetSyncWord(void);
/**
* @brief Get the current transmit mode setting
*/
uint8_t PAN_GetLDR(void);
/**
* @brief Get the time of a single symbol
* @param <bw> Bandwidth
* - PAN_BW_062K / PAN_BW_125K / PAN_BW_250K / PAN_BW_500K
* @param <sf> Spreading factor
* - PAN_SF5 / PAN_SF6 / PAN_SF7 / PAN_SF8 / PAN_SF9 / PAN_SF10 / PAN_SF11 / PAN_SF12
* @return Single symbol time, in us
* @note This function is used to calculate the time of a single symbol
*/
uint32_t PAN_GetOneSymbolTime(uint8_t bw, uint8_t sf);
/**
* @brief Calculates the time to send a packet.
* @param <Size> The size of the packet to be sent, in bytes.
* @return The time to send the packet, in milliseconds.
*/
uint32_t PAN_GetTxTimeMs(uint8_t Size);
/**
* @brief Enable MPM mode
*/
void PAN_EnableMapm(void);
/**
* @brief Disable MPM mode
*/
void PAN_DisableMapm(void);
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
void PAN_ConfigMapm(PAN_MapmCfg_t *pMapmCfg);
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
void PAN_SetMapmAddr(uint8_t *MapmAddr, uint8_t AddrWidth);
/**
* @brief Calculate the time it takes to calculate a field (ms)
* @param <pMapmCfg> mapm configuration parameters
* <SymbolTime> Single symbol (chirp) time
* @note The number of chirps in Group1 is (pg1 + 2), where pg1 is the number of preambles in Group 1, and 2 is the number of chirps occupied by the addresses in Group 1.
* @note The number of chirps in other groups is (pgn + 2) * (gn - 1), where pgn is the number of preambles in other individual groups,
* 2 is the number of chirps occupied by the addresses (or count values) in other individual groups, and (gn - 1) is the number of groups remaining after removing Group 1.
*/
uint32_t PAN_GetMapmOneFieldTime(PAN_MapmCfg_t *pMapmCfg, uint32_t SymbolTime);
/**
* @brief Get the remaining Mapm time in mapm mode
* @param <pMapmCfg> mapm configuration parameter
* <SymbolTime> Single symbol (chirp) time
* @return Remaining Mapm time, in milliseconds
* @note The remaining Mapm time is the time from the current moment to the completion of sending the remaining fields.
* @note: The remaining field count includes the current field.
*/
uint32_t PAN_GetLeftMapmTime(PAN_MapmCfg_t *pMapmCfg, uint32_t SymbolTime);
/**
* @brief Start transmitting continuous carrier wave.
* @note The transmit power and frequency must be set before calling this function.
* @note After calling this function, the chip will remain in the transmitting state until the PAN_StopTxContinuousWave() function is called to stop transmitting.
*/
void PAN_StartTxContinuousWave(void);
/**
* @brief Stop transmitting continuous carrier waves
* @note After calling this function, the chip stops transmitting and enters standby mode.
*/
void PAN_StopTxContinuousWave(void);
/**
* @brief Handle RF interrupt events
* @note This function can be called from an interrupt service routine;
* It can also be called in a polling manner to handle RF interrupt events.
*/
void PAN_IRQ_Process(void);

/** \} defgroup PAN3029/3060 */
/** \} addtogroup ChirpIOT */

#endif // __PAN_PAN_H__
