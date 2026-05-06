#include "sx1262.h"

// Forward declarations of internal functions
static RadioError sx1262_reset(Sx1262* radio);
static RadioError sx1262_wait_busy(Sx1262* radio);
static RadioError sx1262_wait_busy_extended(Sx1262* radio);
static void sx1262_delay_ms(const Sx1262* radio, uint32_t ms);
static RadioError sx1262_write_command(Sx1262* radio, const uint8_t* data, size_t len);
static RadioError sx1262_transfer(Sx1262* radio, const uint8_t* tx, uint8_t* rx, size_t len);
static RadioError sx1262_write_register(Sx1262* radio, uint16_t addr, uint8_t value);
static RadioError sx1262_wait_tx_done(Sx1262* radio);

// Constructor function
Sx1262 sx1262_new(SpiDevice* spi, OutputPin* nss, OutputPin* reset, InputPin* busy, InputPin* dio1) 
{
  Sx1262 radio = 
  {
    .spi = spi,
    .nss = nss,
    .reset = reset,
    .busy = busy,
    .dio1 = dio1,
    .config = radio_config_default(),
    .state = RADIO_STATE_SLEEP,
  };
  return radio;
}

// Initialize the radio
RadioError sx1262_init(Sx1262* radio) 
{
  RadioError err;
  // Reset the radio
  err = sx1262_reset(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Wait for busy to clear
  err = sx1262_wait_busy_extended(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Set DIO3 as TCXO control
  uint8_t tcxo_cmd[] = 
  {
    OPCODE_SET_DIO3_AS_TCXO_CTRL,
    0x02,
    0x00,
    0x01,
    0x40,
  };
  err = sx1262_write_command(radio, tcxo_cmd, sizeof(tcxo_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  sx1262_delay_ms(radio, 10);
  err = sx1262_wait_busy_extended(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Set standby mode
  uint8_t standby_cmd[] = {OPCODE_SET_STANDBY, 0x01};
  err = sx1262_write_command(radio, standby_cmd, sizeof(standby_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  radio->state = RADIO_STATE_STANDBY;
  err = sx1262_wait_busy(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Set regulator mode
  uint8_t regulator_cmd[] = {OPCODE_SET_REGULATOR_MODE, 0x01};
  err = sx1262_write_command(radio, regulator_cmd, sizeof(regulator_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  err = sx1262_wait_busy(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Calibrate
  uint8_t calibrate_cmd[] = {OPCODE_CALIBRATE, 0x7F};
  err = sx1262_write_command(radio, calibrate_cmd, sizeof(calibrate_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  err = sx1262_wait_busy_extended(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Calibrate image
  uint8_t calibrate_image_cmd[] = 
  {
    OPCODE_CALIBRATE_IMAGE,
    0xE1,
    0xE9,
  };
  err = sx1262_write_command(radio, calibrate_image_cmd, sizeof(calibrate_image_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  err = sx1262_wait_busy(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Set DIO2 as RF switch control
  uint8_t dio2_cmd[] = {OPCODE_SET_DIO2_AS_RF_SWITCH_CTRL, 0x01};
  err = sx1262_write_command(radio, dio2_cmd, sizeof(dio2_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Set packet type to LoRa
  uint8_t packet_type_cmd[] = {OPCODE_SET_PACKET_TYPE, 0x01};
  err = sx1262_write_command(radio, packet_type_cmd, sizeof(packet_type_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Configure with default settings
  RadioConfig config = radio->config;
  err = sx1262_configure(radio, &config);
  if (err != RADIO_ERROR_NONE) return err;
  return RADIO_ERROR_NONE;
}

// Reset the radio
static RadioError sx1262_reset(Sx1262* radio) 
{
  // Set reset pin low
  radio->reset->set_low(radio->reset);
  delay_ms(1);
  // Set reset pin high
  radio->reset->set_high(radio->reset);
  delay_ms(10);
  return RADIO_ERROR_NONE;
}

// Wait for busy pin to go low
static RadioError sx1262_wait_busy(Sx1262* radio) 
{
  for (int i = 0; i < 100; i++) 
  {
    bool is_high;
    int result = radio->busy->is_high(radio->busy, &is_high);
    if (result == 0 && !is_high) return RADIO_ERROR_NONE;
    delay_ms(1);
  }
  return RADIO_ERROR_BUSY_TIMEOUT;
}

// Wait for busy pin to go low (extended timeout)
static RadioError sx1262_wait_busy_extended(Sx1262* radio) 
{
  for (int i = 0; i < 500; i++) 
  {
    bool is_high;
    int result = radio->busy->is_high(radio->busy, &is_high);
    if (result == 0 && !is_high) return RADIO_ERROR_NONE;
    delay_ms(1);
  }
  return RADIO_ERROR_BUSY_TIMEOUT;
}

// Delay function
static void sx1262_delay_ms(const Sx1262* radio, uint32_t ms) 
{
  (void)radio; // Unused parameter
  delay_ms(ms);
}

// Write command to radio
static RadioError sx1262_write_command(Sx1262* radio, const uint8_t* data, size_t len) 
{
  RadioError err = sx1262_wait_busy(radio);
  if (err != RADIO_ERROR_NONE) return err;
  radio->nss->set_low(radio->nss);
  int result = radio->spi->write(radio->spi, data, len);
  radio->nss->set_high(radio->nss);
  if (result != 0) return RADIO_ERROR_SPI;
  return RADIO_ERROR_NONE;
}

// Transfer data with radio (SPI transfer)
static RadioError sx1262_transfer(Sx1262* radio, const uint8_t* tx, uint8_t* rx, size_t len) 
{
  RadioError err = sx1262_wait_busy(radio);
  if (err != RADIO_ERROR_NONE) return err;
  radio->nss->set_low(radio->nss);
  int result = radio->spi->transfer(radio->spi, rx, tx, len);
  radio->nss->set_high(radio->nss);
  
  if (result != 0) return RADIO_ERROR_SPI;
  return RADIO_ERROR_NONE;
}

// Set radio to standby mode
RadioError sx1262_set_standby(Sx1262* radio) 
{
  uint8_t cmd[] = {OPCODE_SET_STANDBY, 0x01};
  RadioError err = sx1262_write_command(radio, cmd, sizeof(cmd));
  if (err != RADIO_ERROR_NONE) 
  radio->state = RADIO_STATE_STANDBY;
  return RADIO_ERROR_NONE;
}

// Configure the radio
RadioError sx1262_configure(Sx1262* radio, const RadioConfig* config) 
{
  RadioError err;
  // Validate configuration
  if (config->spreading_factor < 7 || config->spreading_factor > 12) return RADIO_ERROR_INVALID_CONFIG;
  if (config->bandwidth > 2) return RADIO_ERROR_INVALID_CONFIG;
  // Set RF frequency
  uint32_t freq_reg = (uint32_t)(((uint64_t)config->frequency * (1ULL << 25)) / 32000000);
  uint8_t freq_cmd[] = 
  {
    OPCODE_SET_RF_FREQUENCY,
    (uint8_t)(freq_reg >> 24),
    (uint8_t)(freq_reg >> 16),
    (uint8_t)(freq_reg >> 8),
    (uint8_t)freq_reg,
  };
  err = sx1262_write_command(radio, freq_cmd, sizeof(freq_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Set PA configuration
  uint8_t pa_cmd[] = 
  {
    OPCODE_SET_PA_CONFIG,
    0x04,
    0x07,
    0x00,
    0x01,
  };
  err = sx1262_write_command(radio, pa_cmd, sizeof(pa_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Set TX parameters
  int8_t power = config->tx_power;
  if (power < -9) power = -9;
  if (power > 22) power = 22;
  uint8_t power_byte = (uint8_t)(power + 9);
  uint8_t tx_params_cmd[] = 
  {
    OPCODE_SET_TX_PARAMS,
    power_byte,
    0x04,
  };
  err = sx1262_write_command(radio, tx_params_cmd, sizeof(tx_params_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Calculate LDRO setting
  uint32_t bw_hz;
  switch (config->bandwidth) 
  {
    case 0: bw_hz = 125000; break;
    case 1: bw_hz = 250000; break;
    case 2: bw_hz = 500000; break;
    default: bw_hz = 125000; break;
  }
  uint32_t symbol_time_us = ((1U << config->spreading_factor) * 1000000) / bw_hz;
  bool ldro_required = symbol_time_us > 16380;
  uint8_t ldro = (config->ldro || ldro_required) ? 0x01 : 0x00;
  // Set modulation parameters
  uint8_t mod_params_cmd[] = 
  {
    OPCODE_SET_MODULATION_PARAMS,
    config->spreading_factor,
    config->bandwidth,
    config->coding_rate,
    ldro,
  };
  err = sx1262_write_command(radio, mod_params_cmd, sizeof(mod_params_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Set packet parameters
  uint8_t header_type = config->implicit_header ? 0x01 : 0x00;
  uint8_t crc_type = config->crc_enabled ? 0x01 : 0x00;
  uint8_t pkt_params_cmd[] = 
  {
    OPCODE_SET_PACKET_PARAMS,
    (uint8_t)(config->preamble_length >> 8),
    (uint8_t)config->preamble_length,
    header_type,
    255,
    crc_type,
    0x00,
  };
  err = sx1262_write_command(radio, pkt_params_cmd, sizeof(pkt_params_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Set sync word
  uint8_t sync_msb = (config->sync_word >> 4) | 0x40;
  uint8_t sync_lsb = (config->sync_word << 4) | 0x04;
  err = sx1262_write_register(radio, REGISTER_LORA_SYNC_WORD_MSB, sync_msb);
  if (err != RADIO_ERROR_NONE) return err;
  err = sx1262_write_register(radio, REGISTER_LORA_SYNC_WORD_LSB, sync_lsb);
  if (err != RADIO_ERROR_NONE) return err;
  // Set buffer base address
  uint8_t buf_addr_cmd[] = {OPCODE_SET_BUFFER_BASE_ADDRESS, 0x00, 0x00};
  err = sx1262_write_command(radio, buf_addr_cmd, sizeof(buf_addr_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Set DIO IRQ parameters
  uint8_t irq_cmd[] = 
  {
    OPCODE_SET_DIO_IRQ_PARAMS,
    (uint8_t)(IRQ_ALL >> 8),
    (uint8_t)IRQ_ALL,
    (uint8_t)(IRQ_ALL >> 8),
    (uint8_t)IRQ_ALL,
    0x00, 0x00,
    0x00, 0x00,
  };
  err = sx1262_write_command(radio, irq_cmd, sizeof(irq_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Store configuration
  radio->config = *config;
  return RADIO_ERROR_NONE;
}

// Write to a register
static RadioError sx1262_write_register(Sx1262* radio, uint16_t addr, uint8_t value) 
{
  uint8_t cmd[] = 
  {
    OPCODE_WRITE_REGISTER,
    (uint8_t)(addr >> 8),
    (uint8_t)addr,
    value,
  };
  return sx1262_write_command(radio, cmd, sizeof(cmd));
}

// Transmit data
RadioError sx1262_transmit(Sx1262* radio, const uint8_t* data, size_t len) 
{
  RadioError err;
  if (len > 255) return RADIO_ERROR_BUFFER_OVERFLOW;
  // Set to standby
  err = sx1262_set_standby(radio);
  if (err != RADIO_ERROR_NONE)  return err;
  // Write buffer
  uint8_t cmd[258];
  cmd[0] = OPCODE_WRITE_BUFFER;
  cmd[1] = 0x00;
  memcpy(&cmd[2], data, len);
  err = sx1262_write_command(radio, cmd, len + 2);
  if (err != RADIO_ERROR_NONE) return err;
  // Set packet parameters with actual payload length
  uint8_t header_type = radio->config.implicit_header ? 0x01 : 0x00;
  uint8_t crc_type = radio->config.crc_enabled ? 0x01 : 0x00;
  uint8_t pkt_params_cmd[] = 
  {
    OPCODE_SET_PACKET_PARAMS,
    (uint8_t)(radio->config.preamble_length >> 8),
    (uint8_t)radio->config.preamble_length,
    header_type,
    (uint8_t)len,
    crc_type,
    0x00,
  };
  err = sx1262_write_command(radio, pkt_params_cmd, sizeof(pkt_params_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Clear IRQ status
  uint8_t clear_irq_cmd[] = {OPCODE_CLEAR_IRQ_STATUS, 0x03, 0xFF};
  err = sx1262_write_command(radio, clear_irq_cmd, sizeof(clear_irq_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Set TX mode
  uint8_t tx_cmd[] = {OPCODE_SET_TX, 0x00, 0x00, 0x00};
  err = sx1262_write_command(radio, tx_cmd, sizeof(tx_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  radio->state = RADIO_STATE_TX;
  // Wait for TX done
  err = sx1262_wait_tx_done(radio);
  if (err != RADIO_ERROR_NONE) return err;
  radio->state = RADIO_STATE_STANDBY;
  return RADIO_ERROR_NONE;
}

// Wait for TX done
static RadioError sx1262_wait_tx_done(Sx1262* radio) 
{
  for (int i = 0; i < 10000; i++) 
  {
    bool is_high = false;
    radio->dio1->is_high(radio->dio1, &is_high);
    if (is_high) 
    {
      uint16_t irq;
      RadioError err = sx1262_get_irq_status(radio, &irq);
      if (err != RADIO_ERROR_NONE) continue;
      if (irq & IRQ_TX_DONE) 
      {
        uint8_t clear_cmd[] = {OPCODE_CLEAR_IRQ_STATUS, 0x03, 0xFF};
        sx1262_write_command(radio, clear_cmd, sizeof(clear_cmd));
        return RADIO_ERROR_NONE;
      }
    }
    delay_ms(1);
  }
  sx1262_set_standby(radio);
  return RADIO_ERROR_TX_TIMEOUT;
}

// Get IRQ status
RadioError sx1262_get_irq_status(Sx1262* radio, uint16_t* irq) 
{
  uint8_t tx[4] = {OPCODE_GET_IRQ_STATUS, 0, 0, 0};
  uint8_t rx[4] = {0};
  RadioError err = sx1262_transfer(radio, tx, rx, 4);
  if (err != RADIO_ERROR_NONE) return err;
  *irq = ((uint16_t)rx[2] << 8) | (uint16_t)rx[3];
  return RADIO_ERROR_NONE;
}

// Clear IRQ flags
RadioError sx1262_clear_irq(Sx1262* radio, uint16_t flags) 
{
  uint8_t cmd[] = 
  {
    OPCODE_CLEAR_IRQ_STATUS,
    (uint8_t)(flags >> 8),
    (uint8_t)flags,
  };
  return sx1262_write_command(radio, cmd, sizeof(cmd));
}

// Start RX mode
RadioError sx1262_start_rx(Sx1262* radio, uint32_t timeout_ms) 
{
  RadioError err;
  // Set to standby
  err = sx1262_set_standby(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Clear IRQ status
  uint8_t clear_cmd[] = {OPCODE_CLEAR_IRQ_STATUS, 0x03, 0xFF};
  err = sx1262_write_command(radio, clear_cmd, sizeof(clear_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Calculate timeout ticks
  uint32_t timeout_ticks;
  if (timeout_ms == 0) timeout_ticks = 0xFFFFFF;
  else 
  {
    timeout_ticks = (uint32_t)(((uint64_t)timeout_ms * 64) / 1000);
    if (timeout_ticks > 0xFFFFFF) timeout_ticks = 0xFFFFFF;
  }
  // Set RX mode
  uint8_t rx_cmd[] = 
  {
    OPCODE_SET_RX,
    (uint8_t)(timeout_ticks >> 16),
    (uint8_t)(timeout_ticks >> 8),
    (uint8_t)timeout_ticks,
  };
  err = sx1262_write_command(radio, rx_cmd, sizeof(rx_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  radio->state = RADIO_STATE_RX;
  return RADIO_ERROR_NONE;
}

// Check for received data
RadioError sx1262_check_rx(Sx1262* radio, RxResult* result, bool* has_data) 
{
  RadioError err;
  *has_data = false;
  // Check DIO1 pin
  bool is_high = false;
  radio->dio1->is_high(radio->dio1, &is_high);
  if (!is_high) return RADIO_ERROR_NONE;
  // Get IRQ status
  uint16_t irq;
  err = sx1262_get_irq_status(radio, &irq);
  if (err != RADIO_ERROR_NONE) return err;
  // Check for CRC error
  if (irq & IRQ_CRC_ERR) 
  {
    uint8_t clear_cmd[] = {OPCODE_CLEAR_IRQ_STATUS, 0x03, 0xFF};
    sx1262_write_command(radio, clear_cmd, sizeof(clear_cmd));
    return RADIO_ERROR_CRC_ERROR;
  }
  // Check for timeout
  if (irq & IRQ_TIMEOUT) 
  {
    uint8_t clear_cmd[] = {OPCODE_CLEAR_IRQ_STATUS, 0x03, 0xFF};
    sx1262_write_command(radio, clear_cmd, sizeof(clear_cmd));
    return RADIO_ERROR_RX_TIMEOUT;
  }
  
  // Check for RX done
  if (irq & IRQ_RX_DONE) 
  {
    // Get RX buffer status
    uint8_t buf_status_tx[4] = {OPCODE_GET_RX_BUFFER_STATUS, 0, 0, 0};
    uint8_t buf_status_rx[4] = {0};
    err = sx1262_transfer(radio, buf_status_tx, buf_status_rx, 4);
    if (err != RADIO_ERROR_NONE) return err;
    uint8_t payload_len = buf_status_rx[2];
    uint8_t start_offset = buf_status_rx[3];
    // Get packet status
    uint8_t pkt_status_tx[5] = {OPCODE_GET_PACKET_STATUS, 0, 0, 0, 0};
    uint8_t pkt_status_rx[5] = {0};
    err = sx1262_transfer(radio, pkt_status_tx, pkt_status_rx, 5);
    if (err != RADIO_ERROR_NONE) return err;
    int16_t rssi = -(int16_t)(pkt_status_rx[2] / 2);
    int8_t snr = (int8_t)pkt_status_rx[3] / 4;
    // Read buffer
    uint8_t read_cmd[258] = {0};
    read_cmd[0] = OPCODE_READ_BUFFER;
    read_cmd[1] = start_offset;
    read_cmd[2] = 0;
    size_t read_len = payload_len + 3;
    uint8_t rx_buf[258] = {0};
    err = sx1262_transfer(radio, read_cmd, rx_buf, read_len);
    if (err != RADIO_ERROR_NONE) return err;
    // Copy data to result
    result->data.len = payload_len;
    memcpy(result->data.data, &rx_buf[3], payload_len);
    result->rssi = rssi;
    result->snr = snr;
    // Clear IRQ status
    uint8_t clear_cmd[] = {OPCODE_CLEAR_IRQ_STATUS, 0x03, 0xFF};
    sx1262_write_command(radio, clear_cmd, sizeof(clear_cmd));
    *has_data = true;
    return RADIO_ERROR_NONE;
  }
  return RADIO_ERROR_NONE;
}

// Perform Channel Activity Detection
RadioError sx1262_cad(Sx1262* radio, bool* detected) 
{
  RadioError err;
  
  // Set to standby
  err = sx1262_set_standby(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Set CAD parameters
  uint8_t cad_params_cmd[] = 
  {
    OPCODE_SET_CAD_PARAMS,
    0x04,
    24,
    10,
    0x00,
    0x00, 0x00, 0x00,
  };
  err = sx1262_write_command(radio, cad_params_cmd, sizeof(cad_params_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Clear IRQ status
  uint8_t clear_cmd[] = {OPCODE_CLEAR_IRQ_STATUS, 0x03, 0xFF};
  err = sx1262_write_command(radio, clear_cmd, sizeof(clear_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  // Start CAD
  uint8_t cad_cmd[] = {OPCODE_SET_CAD};
  err = sx1262_write_command(radio, cad_cmd, sizeof(cad_cmd));
  if (err != RADIO_ERROR_NONE) return err;
  radio->state = RADIO_STATE_CAD;
  // Wait for CAD done
  for (int i = 0; i < 1000; i++) 
  {
    bool is_high = false;
    radio->dio1->is_high(radio->dio1, &is_high);
    
    if (is_high) 
    {
      uint16_t irq;
      err = sx1262_get_irq_status(radio, &irq);
      if (err != RADIO_ERROR_NONE) continue;
      if (irq & IRQ_CAD_DONE) 
      {
        *detected = (irq & IRQ_CAD_DETECTED) != 0;
        uint8_t clear_cmd2[] = {OPCODE_CLEAR_IRQ_STATUS, 0x03, 0xFF};
        sx1262_write_command(radio, clear_cmd2, sizeof(clear_cmd2));
        radio->state = RADIO_STATE_STANDBY;
        return RADIO_ERROR_NONE;
      }
    }
    delay_ms(1);
  }
  sx1262_set_standby(radio);
  return RADIO_ERROR_BUSY_TIMEOUT;
}

// Get current radio state
RadioState sx1262_state(const Sx1262* radio) 
{
  return radio->state;
}

// Get instantaneous RSSI
RadioError sx1262_get_rssi(Sx1262* radio, int16_t* rssi) 
{
  uint8_t tx[3] = {OPCODE_GET_RSSI_INST, 0, 0};
  uint8_t rx[3] = {0};
  RadioError err = sx1262_transfer(radio, tx, rx, 3);
  if (err != RADIO_ERROR_NONE) return err;
  *rssi = -(int16_t)(rx[2] / 2);
  return RADIO_ERROR_NONE;
}

// Get random number
RadioError sx1262_random(Sx1262* radio, uint32_t* random_value) 
{
  RadioError err;
  // Start RX to generate random numbers
  err = sx1262_start_rx(radio, 0);
  if (err != RADIO_ERROR_NONE) return err;
  delay_ms(10);
  err = sx1262_set_standby(radio);
  if (err != RADIO_ERROR_NONE) return err;
  // Read random number registers
  uint32_t random = 0;
  uint16_t addrs[] = 
  {
    REGISTER_RANDOM_NUMBER_0,
    REGISTER_RANDOM_NUMBER_1,
    REGISTER_RANDOM_NUMBER_2,
    REGISTER_RANDOM_NUMBER_3,
  };
  for (int i = 0; i < 4; i++) 
  {
    uint8_t tx[3] = 
    {
      OPCODE_READ_REGISTER,
      (uint8_t)(addrs[i] >> 8),
      (uint8_t)addrs[i],
    };
    uint8_t rx[3] = {0};
    err = sx1262_transfer(radio, tx, rx, 3);
    if (err != RADIO_ERROR_NONE) return err;
    random |= ((uint32_t)rx[2]) << (i * 8);
  }
  *random_value = random;
  return RADIO_ERROR_NONE;
}
