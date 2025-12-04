#ifndef _PAN3029_DEFS_H_
#define _PAN3029_DEFS_H_

/**
* @brief PAN3029/3060 interrupt flag definition
*/
#define PAN3029_IRQ_TX_DONE 0x01 /* Transmit completion interrupt flag */
#define PAN3029_IRQ_RX_TIMEOUT 0x02 /* Single receive timeout interrupt flag */
#define PAN3029_IRQ_CRC_ERR 0x04 /* CRC error interrupt flag */
#define PAN3029_IRQ_RX_DONE 0x08 /* Receive completion interrupt flag */
#define PAN3029_IRQ_MAPM_DONE 0x40 /* MAPM interrupt flag */

/**
* @brief PAN3029/3060 modulation mode definition
*/
#define MODEM_MODE_NORMAL 0x00 /* Normal modulation mode */
#define MODEM_MODE_MULTI_SECTOR 0x01 /* Multi-segment modulation mode */

/**
* @brief PAN3029/3060 Sync Word Definitions
* @note: The PAN3029/3060 sync word is a single byte, ranging from 0x00 to 0xFF
*/
#define PAN3029_MAC_PRIVATE_SYNCWORD 0x12 /* Private network sync word, default value is 0x12 */
#define PAN3029_MAC_PUBLIC_SYNCWORD 0x34 /* Public network sync word */

/**
* @brief PAN3029/3060 Power Range Definitions
*/
#define PAN3029_MIN_RAMP 1 /* Minimum power range */
#define PAN3029_MAX_RAMP 22 /* Maximum power level */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**
* @brief RF-related operation return value definitions
* - PAN3029_OK: Operation successful
* - PAN3029_FAIL: Operation failed
*/
typedef enum
{
  PAN3029_OK, //!< Operate ok
  PAN3029_FAIL, //!< Operate fail
} PAN3029_Err_t;

/**
* @brief PAN3029/3060 operating state definitions
*/
typedef enum
{
  PAN3029_STATE_DEEPSLEEP = 0x00, //!< The radio is in deep sleep mode
  PAN3029_STATE_SLEEP = 0x01, //!< The radio is in sleep mode
  PAN3029_STATE_STB1 = 0x02, //!< The radio is in standby mode 1 
  PAN3029_STATE_STB2 = 0x03, //!< The radio is in standby mode 2 
  PAN3029_STATE_STB3 = 0x04, //!< The radio is in standby mode 3 
  PAN3029_STATE_TX = 0x05, //!< The radio is in transmit mode 
  PAN3029_STATE_RX = 0x06, //!< The radio is in receive mode
} RfOpState_t;

/** 
* @brief GPIO mode definition of PAN3029/3060 
* - PAN_GPIO_MODE_INPUT: GPIO input mode 
* - PAN_GPIO_MODE_OUTPUT: GPIO output mode 
*/
typedef enum
{ 
  PAN_GPIO_MODE_INPUT = 0x00, //!< GPIO input mode 
  PAN_GPIO_MODE_OUTPUT = 0x01, //!< GPIO output mode
} RfGpioMode_t;

/**
* @brief PAN3029/3060 Transmit Mode Definitions
* - PAN3029_TX_MODE_SINGLE: Single transmit mode, used in most cases
* - PAN3029_TX_MODE_CONTINOUS: Continuous transmit mode, commonly used for carrier signal transmission
*/
typedef enum
{
  PAN3029_TX_MODE_SINGLE = 0x00, //!< Single transmission mode
  PAN3029_TX_MODE_CONTINOUS = 0x01, //!< Continuous transmission mode
} RfTxMode_t;

/**
* @brief PAN3029/3060 Receive Mode Definitions
* - PAN3029_RX_MODE_SINGLE: Single receive mode, automatically enters standby mode after receiving a packet of data. This mode is generally not used.
* - PAN3029_RX_MODE_SINGLE_TIMEOUT: Single reception mode with timeout, automatically enters standby mode after timeout.
* - PAN3029_RX_MODE_CONTINOUS: Continuous reception mode, continues receiving after receiving a packet.
*/
typedef enum
{
  PAN3029_RX_MODE_SINGLE = 0x00, //!< Single reception mode
  PAN3029_RX_MODE_SINGLE_TIMEOUT = 0x01, //!< Single reception mode with timeout
  PAN3029_RX_MODE_CONTINOUS = 0x02, //!< Continuous reception mode
} RfRxMode_t;

/**
* @brief PAN3029/3060 power supply mode definition
* - USE_LDO: Use LDO for power supply; current is slightly higher, but the receiver sensitivity is slightly better.
* - USE_DCDC: Use DCDC for power supply; current is slightly lower, but the receiver sensitivity is slightly worse by 1-2dB.
*/
typedef enum
{
  USE_LDO = 0x00, //!< Use LDO for power regulation
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
* @brief CAD detection threshold configuration definitions
* - A smaller threshold value allows detection of weaker signals, but the false alarm rate increases.
* - A larger threshold value reduces the false alarm rate, but the detectable signal strength range decreases.
*/
typedef enum
{
  PAN3029_CAD_THRESHOLD_0A = 0x0A,
  PAN3029_CAD_THRESHOLD_10 = 0x10,
  PAN3029_CAD_THRESHOLD_15 = 0x15,
  PAN3029_CAD_THRESHOLD_20 = 0x20,
} RfCadThreshold_t;

/**
* @brief CAD detection symbol count configuration definition
*/
typedef enum
{
  PAN3029_CAD_01_SYMBOL = 0x01, //!< Actual detection time may be greater than 1 symbol
  PAN3029_CAD_02_SYMBOL = 0x02, //!< Actual detection time may be greater than 2 symbols
  PAN3029_CAD_03_SYMBOL = 0x03, //!< Actual detection time may be greater than 3 symbols
  PAN3029_CAD_04_SYMBOL = 0x04, //!< Actual detection time may be greater than 4 symbols
} RfCadSymbols_t;

/**
* @brief Represents the possible spreading factor values in ChirpIot packet types 
*/
typedef enum
{ 
  PAN3029_SF5 = 0x05, //!< 5 spreading factor 
  PAN3029_SF6 = 0x06, //!< 6 spreading factor 
  PAN3029_SF7 = 0x07, //!< 7 spreading factor 
  PAN3029_SF8 = 0x08, //!< 8 spreading factor 
  PAN3029_SF9 = 0x09, //!< 9 spreading factor 
  PAN3029_SF10 = 0x0A, //!< 10 spreading factor 
  PAN3029_SF11 = 0x0B, //!< 11 spreading factor 
  PAN3029_SF12 = 0x0C, //!< 12 spreading factor
} RfSpreadFactor_t;

/** 
* @brief Represents the bandwidth values for ChirpIot packet type 
*/
typedef enum
{ 
  PAN3029_BW_062K = 6, //!< 62.5KHz bandwidth 
  PAN3029_BW_125K = 7, //!< 125KHz bandwidth 
  PAN3029_BW_250K = 8, //!< 250KHz bandwidth 
  PAN3029_BW_500K = 9, //!< 500KHz bandwidth
} RfBandwidths_t;

/** 
* @brief Represents the coding rate values for ChirpIot packet type 
* @note The coding rate is expressed as 4/x where x is the value below 
*/
typedef enum
{ 
  PAN3029_CR_4_5 = 0x01, //!< 4/5 coding rate 
  PAN3029_CR_4_6 = 0x02, //!< 4/6 coding rate 
  PAN3029_CR_4_7 = 0x03, //!< 4/7 coding rate 
  PAN3029_CR_4_8 = 0x04, //!< 4/8 coding rate
} RfCodingRates_t;

/**
* @brief CRC mode
* @note PAN3029_CRC_ON: Enables CRC checking
* @note PAN3029_CRC_OFF: Disables CRC checking, requiring the user to verify data integrity
*/
typedef enum
{ 
  PAN3029_CRC_OFF = 0x00, //!< CRC not used 
  PAN3029_CRC_ON = 0x01, //!< CRC activated
} RfCrcModes_t;

/** 
* @brief low rate optimization mode 
* @note PAN3029_LDR_ON: Low data rate optimization activated 
* @note PAN3029_LDR_OFF: Low data rate optimization not used 
*/
typedef enum
{ 
  PAN3029_LDR_OFF = 0x00, //!< Low data rate optimization not used 
  PAN3029_LDR_ON = 0x01, //!< Low data rate optimization activated
} RfLdr_t;

/** 
* @brief Represents the IQ mode for ChirpIot packet type 
*/
typedef enum
{ 
  PAN3029_IQ_NORMAL = 0x00, //!< Normal IQ mode,default 
  PAN3029_IQ_INVERTED = 0x01, //!< Inverted IQ mode
} RfIQModes_t;

typedef enum
{ 
  PAN3029_MAPM_GRP_ADDR = 0x00, //!< Address group 
  PAN3029_MAPM_GRP_COUNTER = 0x01, //!< Counter group
} RfMapmGrpType_t;


#endif
