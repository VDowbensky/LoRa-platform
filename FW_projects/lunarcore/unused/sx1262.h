#ifndef _SX1262_H_
#define _SX1262_H_

// Delay function declaration
extern void delay_ms(uint32_t ms);

// Opcode constants
#define OPCODE_SET_SLEEP                    0x84
#define OPCODE_SET_STANDBY                  0x80
#define OPCODE_SET_FS                       0xC1
#define OPCODE_SET_TX                       0x83
#define OPCODE_SET_RX                       0x82
#define OPCODE_STOP_TIMER_ON_PREAMBLE       0x9F
#define OPCODE_SET_RX_DUTY_CYCLE            0x94
#define OPCODE_SET_CAD                      0xC5
#define OPCODE_SET_TX_CONTINUOUS_WAVE       0xD1
#define OPCODE_SET_TX_INFINITE_PREAMBLE     0xD2
#define OPCODE_SET_REGULATOR_MODE           0x96
#define OPCODE_CALIBRATE                    0x89
#define OPCODE_CALIBRATE_IMAGE              0x98
#define OPCODE_SET_PA_CONFIG                0x95
#define OPCODE_SET_RX_TX_FALLBACK_MODE      0x93
#define OPCODE_WRITE_REGISTER               0x0D
#define OPCODE_READ_REGISTER                0x1D
#define OPCODE_WRITE_BUFFER                 0x0E
#define OPCODE_READ_BUFFER                  0x1E
#define OPCODE_SET_DIO_IRQ_PARAMS           0x08
#define OPCODE_GET_IRQ_STATUS               0x12
#define OPCODE_CLEAR_IRQ_STATUS             0x02
#define OPCODE_SET_DIO2_AS_RF_SWITCH_CTRL   0x9D
#define OPCODE_SET_DIO3_AS_TCXO_CTRL        0x97
#define OPCODE_SET_RF_FREQUENCY             0x86
#define OPCODE_SET_PACKET_TYPE              0x8A
#define OPCODE_GET_PACKET_TYPE              0x11
#define OPCODE_SET_TX_PARAMS                0x8E
#define OPCODE_SET_MODULATION_PARAMS        0x8B
#define OPCODE_SET_PACKET_PARAMS            0x8C
#define OPCODE_SET_CAD_PARAMS               0x88
#define OPCODE_SET_BUFFER_BASE_ADDRESS      0x8F
#define OPCODE_SET_LORA_SYMB_NUM_TIMEOUT    0xA0
#define OPCODE_GET_STATUS                   0xC0
#define OPCODE_GET_RX_BUFFER_STATUS         0x13
#define OPCODE_GET_PACKET_STATUS            0x14
#define OPCODE_GET_RSSI_INST                0x15
#define OPCODE_GET_STATS                    0x10
#define OPCODE_RESET_STATS                  0x00
#define OPCODE_GET_DEVICE_ERRORS            0x17
#define OPCODE_CLEAR_DEVICE_ERRORS          0x07

// Register constants
#define REGISTER_WHITENING_INITIAL_MSB      0x06B8
#define REGISTER_WHITENING_INITIAL_LSB      0x06B9
#define REGISTER_CRC_INITIAL_MSB            0x06BC
#define REGISTER_CRC_INITIAL_LSB            0x06BD
#define REGISTER_CRC_POLYNOMIAL_MSB         0x06BE
#define REGISTER_CRC_POLYNOMIAL_LSB         0x06BF
#define REGISTER_SYNC_WORD_0                0x06C0
#define REGISTER_SYNC_WORD_1                0x06C1
#define REGISTER_NODE_ADDRESS               0x06CD
#define REGISTER_BROADCAST_ADDRESS          0x06CE
#define REGISTER_LORA_SYNC_WORD_MSB         0x0740
#define REGISTER_LORA_SYNC_WORD_LSB         0x0741
#define REGISTER_RANDOM_NUMBER_0            0x0819
#define REGISTER_RANDOM_NUMBER_1            0x081A
#define REGISTER_RANDOM_NUMBER_2            0x081B
#define REGISTER_RANDOM_NUMBER_3            0x081C
#define REGISTER_RX_GAIN                    0x08AC
#define REGISTER_OCP_CONFIGURATION          0x08E7
#define REGISTER_XTA_TRIM                   0x0911
#define REGISTER_XTB_TRIM                   0x0912

// IRQ constants
#define IRQ_TX_DONE             (1 << 0)
#define IRQ_RX_DONE             (1 << 1)
#define IRQ_PREAMBLE_DETECTED   (1 << 2)
#define IRQ_SYNC_WORD_VALID     (1 << 3)
#define IRQ_HEADER_VALID        (1 << 4)
#define IRQ_HEADER_ERR          (1 << 5)
#define IRQ_CRC_ERR             (1 << 6)
#define IRQ_CAD_DONE            (1 << 7)
#define IRQ_CAD_DETECTED        (1 << 8)
#define IRQ_TIMEOUT             (1 << 9)
#define IRQ_ALL                 0x03FF

// RadioConfig structure
typedef struct 
{
  uint32_t frequency;
  uint8_t spreading_factor;
  uint8_t bandwidth;
  uint8_t coding_rate;
  int8_t tx_power;
  uint8_t sync_word;
  uint16_t preamble_length;
  bool crc_enabled;
  bool implicit_header;
  bool ldro;
} RadioConfig;

// Default RadioConfig initializer
static inline RadioConfig radio_config_default(void) 
{
  RadioConfig config = 
  {
    .frequency = 868100000,
    .spreading_factor = 9,
    .bandwidth = 0,
    .coding_rate = 1,
    .tx_power = 14,
    .sync_word = 0x12,
    .preamble_length = 8,
    .crc_enabled = true,
    .implicit_header = false,
    .ldro = false,
  };
  return config;
}

// RadioState enumeration
typedef enum 
{
  RADIO_STATE_SLEEP,
  RADIO_STATE_STANDBY,
  RADIO_STATE_TX,
  RADIO_STATE_RX,
  RADIO_STATE_CAD,
} RadioState;

// RadioError enumeration
typedef enum 
{
  RADIO_ERROR_NONE = 0,
  RADIO_ERROR_SPI,
  RADIO_ERROR_BUSY_TIMEOUT,
  RADIO_ERROR_INVALID_CONFIG,
  RADIO_ERROR_TX_TIMEOUT,
  RADIO_ERROR_RX_TIMEOUT,
  RADIO_ERROR_CRC_ERROR,
  RADIO_ERROR_BUFFER_OVERFLOW,
} RadioError;

// HeaplessVec structure (fixed-size vector with length tracking)
typedef struct 
{
  uint8_t data[256];
  size_t len;
} HeaplessVec;

// RxResult structure for receive operation results
typedef struct 
{
  HeaplessVec data;
  int16_t rssi;
  int8_t snr;
} RxResult;

// Sx1262 structure
typedef struct 
{
  SpiDevice* spi;
  OutputPin* nss;
  OutputPin* reset;
  InputPin* busy;
  InputPin* dio1;
  RadioConfig config;
  RadioState state;
} Sx1262;

Sx1262 sx1262_new(SpiDevice* spi, OutputPin* nss, OutputPin* reset, InputPin* busy, InputPin* dio1);
RadioError sx1262_init(Sx1262* radio);
RadioError sx1262_set_standby(Sx1262* radio);
RadioError sx1262_configure(Sx1262* radio, const RadioConfig* config);
RadioError sx1262_transmit(Sx1262* radio, const uint8_t* data, size_t len);
RadioError sx1262_get_irq_status(Sx1262* radio, uint16_t* irq);
RadioError sx1262_clear_irq(Sx1262* radio, uint16_t flags);
RadioError sx1262_start_rx(Sx1262* radio, uint32_t timeout_ms);
RadioError sx1262_check_rx(Sx1262* radio, RxResult* result, bool* has_data);
RadioError sx1262_cad(Sx1262* radio, bool* detected);
RadioState sx1262_state(const Sx1262* radio);
RadioError sx1262_get_rssi(Sx1262* radio, int16_t* rssi);
RadioError sx1262_random(Sx1262* radio, uint32_t* random_value);

#endif
