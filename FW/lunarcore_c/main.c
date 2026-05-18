

// Global atomic state variables
static atomic_bool dio1_triggered = ATOMIC_VAR_INIT(false);
static atomic_bool packet_pending = ATOMIC_VAR_INIT(false);
static atomic_bool tx_complete = ATOMIC_VAR_INIT(false);
static atomic_uint_least32_t system_ticks = ATOMIC_VAR_INIT(0);
static atomic_uint_least32_t last_activity = ATOMIC_VAR_INIT(0);
static atomic_uint_least32_t error_count = ATOMIC_VAR_INIT(0);



// Function declarations for NodeIdentity
static void node_identity_read_mac_address(uint8_t mac[6]);
static void node_identity_read_hardware_serial(uint8_t serial[8]);
static void node_identity_load_or_create_identity(const uint8_t hardware_serial[8], 
                                                   uint32_t *node_id, uint8_t private_key[32]);
static uint32_t node_identity_generate_random_node_id(void);
static void node_identity_generate_random_private_key(const uint8_t hardware_serial[8], 
                                                       uint8_t private_key[32]);
static NodeIdentity node_identity_from_hardware(void);
static NodeIdentity* node_identity_factory_reset(void);
static void node_identity_x25519_pubkey(const NodeIdentity *self, uint8_t pubkey[32]);


// Function declarations for BatteryState
static void battery_state_new(BatteryState *battery);
static void battery_state_update(BatteryState *battery, uint32_t adc_value);
static uint8_t battery_state_voltage_to_percentage(uint32_t mv);



// Function declarations for Stats
static void stats_new(Stats *stats);
static void stats_record_rx(Stats *stats, size_t len, int16_t rssi, int8_t snr);
static void stats_record_tx(Stats *stats, size_t len, uint32_t airtime_ms);


// Function declarations for LedController
static void led_controller_new(LedController *led);
static void led_controller_set_idle(LedController *led);
static void led_controller_set_active(LedController *led);
static void led_controller_set_error(LedController *led);
static void led_controller_flash(LedController *led, uint8_t count);
static bool led_controller_update(LedController *led, uint32_t current_time);



// Function declarations for LunarCore
void lunar_core_new(LunarCore *core, Sx1262 *radio, const NodeIdentity *identity);
const NodeIdentity* lunar_core_identity(const LunarCore *core);
void lunar_core_update_battery(LunarCore *core, uint32_t adc_value);
void lunar_core_handle_dio1_interrupt(LunarCore *core);
void lunar_core_process_radio_events(LunarCore *core);
void lunar_core_process_at_command(LunarCore *core, uart_port_t uart);
void lunar_core_process_serial_byte(LunarCore *core, uint8_t byte, uart_port_t uart);
void lunar_core_configure_radio_for_protocol(LunarCore *core, Protocol protocol);
void lunar_core_handle_meshcore_frame(LunarCore *core, const Frame *frame, uart_port_t uart);
void lunar_core_handle_meshtastic_frame(LunarCore *core, const MeshtasticFrame *frame, uart_port_t uart);
void lunar_core_handle_rnode_frame(LunarCore *core, const KissFrame *frame, uart_port_t uart);

// Helper function declarations
static void write_hex8(uart_port_t uart, uint8_t value);
static void write_hex32(uart_port_t uart, uint32_t value);
static void write_u32(uart_port_t uart, uint32_t value);
static void write_i16(uart_port_t uart, int16_t value);
static const char* format_battery(const BatteryState *battery, char *buf);
static bool parse_u32_from_cmd(const uint8_t *cmd, size_t len, uint32_t *result);
static bool parse_i8_from_cmd(const uint8_t *cmd, size_t len, int8_t *result);

// Implementation: NodeIdentity functions

static void node_identity_read_mac_address(uint8_t mac[6]) 
{
  // Read MAC address from ESP32 eFuse
  esp_efuse_mac_get_default(mac); //to be changed!!! I never use ESP
}

void node_identity_read_hardware_serial(uint8_t serial[8]) 
{
  // Read hardware serial from eFuse registers
  volatile uint32_t *efuse_base = (volatile uint32_t *)0x6001A044; //to be changed!!! I never use ESP
  uint32_t word0 = *efuse_base;
  uint32_t word1 = *(efuse_base + 1);
    
  memcpy(&serial[0], &word0, 4);
  memcpy(&serial[4], &word1, 4);
}

uint32_t node_identity_generate_random_node_id(void) //to be changed!!! I never use ESP 
{
  uint8_t random_bytes[4];
  esp_fill_random(random_bytes, 4);
    
  uint32_t id;
  memcpy(&id, random_bytes, 4);
  // Set high bit to indicate random ID
  id |= 0x80000000;
  return id;
}

void node_identity_generate_random_private_key(const uint8_t hardware_serial[8],uint8_t private_key[32]) 
{
  uint8_t random_bytes[32];
  esp_fill_random(random_bytes, 32);
    
  // Combine random data with hardware serial
  uint8_t seed_input[40];
  memcpy(&seed_input[0], random_bytes, 32);
  memcpy(&seed_input[32], hardware_serial, 8);
    
  // Hash to generate private key
  sha256_hash(seed_input, 40, private_key);
  
  // Clamp for Curve25519
  private_key[0] &= 248;
  private_key[31] &= 127;
  private_key[31] |= 64;
}

static void node_identity_load_or_create_identity(const uint8_t hardware_serial[8],uint32_t *node_id, uint8_t private_key[32]) 
{
  nvs_handle_t handle;
  esp_err_t err;
    
  // Try to open NVS
  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) 
  {
    // Initialize NVS flash and retry
    nvs_flash_init();
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) 
    {
      // NVS not available, use ephemeral identity
      ESP_LOGW("NodeIdentity", "NVS not available, using ephemeral identity");
      *node_id = node_identity_generate_random_node_id();
      node_identity_generate_random_private_key(hardware_serial, private_key);
      return;
    }
  }
    
  // Try to load existing identity
  uint32_t stored_node_id;
  size_t key_len = 32;
  bool has_node_id = (nvs_get_u32(handle, NVS_KEY_NODE_ID, &stored_node_id) == ESP_OK);
  bool has_private_key = (nvs_get_blob(handle, NVS_KEY_PRIVATE_KEY, private_key, &key_len) == ESP_OK) && (key_len == 32);
    
  if (has_node_id && has_private_key) 
  {
    // Found existing identity
    ESP_LOGI("NodeIdentity", "Loaded existing node identity from NVS");
    *node_id = stored_node_id;
    nvs_close(handle);
    return;
  }
    
  // Create new identity
  ESP_LOGI("NodeIdentity", "Creating new random node identity (privacy-first)");
  *node_id = node_identity_generate_random_node_id();
  node_identity_generate_random_private_key(hardware_serial, private_key);
    
  // Store in NVS
  nvs_set_u32(handle, NVS_KEY_NODE_ID, *node_id);
  nvs_set_blob(handle, NVS_KEY_PRIVATE_KEY, private_key, 32);
  nvs_commit(handle);
  nvs_close(handle);
    
  ESP_LOGI("NodeIdentity", "Stored new identity in NVS");
}

NodeIdentity node_identity_from_hardware(void) 
{
  NodeIdentity identity;
    
  // Read hardware info
  node_identity_read_mac_address(identity.mac_address);
  node_identity_read_hardware_serial(identity.hardware_serial);
    
  // Load or create identity
  node_identity_load_or_create_identity(identity.hardware_serial, 
                                          &identity.node_id, 
                                          identity.private_key);
    
  // Generate public key from private key
  ed25519_public_key(identity.private_key, identity.public_key);
  
  return identity;
}

NodeIdentity* node_identity_factory_reset(void) 
{
  // Erase NVS partition
  nvs_flash_erase_partition(NVS_NAMESPACE);
    
  ESP_LOGI("NodeIdentity", "Factory reset: erased old identity, generating new one");
    
  // Create new identity
  NodeIdentity *identity = malloc(sizeof(NodeIdentity));
  if (identity) *identity = node_identity_from_hardware();
  return identity;
}

void node_identity_x25519_pubkey(const NodeIdentity *self, uint8_t pubkey[32]) 
{
  x25519_base(self->private_key, pubkey);
}



// Implementation: Stats functions

void stats_new(Stats *stats) 
{
  stats->tx_packets = 0;
  stats->rx_packets = 0;
  stats->tx_errors = 0;
  stats->rx_errors = 0;
  stats->protocol_switches = 0;
  stats->tx_bytes = 0;
  stats->rx_bytes = 0;
  stats->uptime_seconds = 0;
  stats->last_rssi = 0;
  stats->last_snr = 0;
  stats->airtime_ms = 0;
}

void stats_record_rx(Stats *stats, size_t len, int16_t rssi, int8_t snr) 
{
  stats->rx_packets++;
  stats->rx_bytes += len;
  stats->last_rssi = rssi;
  stats->last_snr = snr;
}

void stats_record_tx(Stats *stats, size_t len, uint32_t airtime_ms) 
{
  stats->tx_packets++;
  stats->tx_bytes += len;
  stats->airtime_ms += airtime_ms;
}


// Implementation: LunarCore functions

void lunar_core_new(LunarCore *core, Sx1262 *radio, const NodeIdentity *identity) 
{
  // Initialize radio
  memcpy(&core->radio, radio, sizeof(Sx1262));
    
  // Initialize protocol handlers
  protocol_router_new(&core->router);
  frame_parser_new(&core->meshcore_parser);
  meshtastic_handler_new(&core->meshtastic, identity->node_id);
  rnode_handler_new(&core->rnode);
  //ble_manager_new(&core->ble);
    
  // Initialize stats and state
  stats_new(&core->stats);
  memcpy(&core->identity, identity, sizeof(NodeIdentity));
  battery_state_new(&core->battery);
  led_controller_new(&core->led);
    
  core->rx_active = false;
  core->serial_protocol = PROTOCOL_UNKNOWN;
  core->vext_enabled = true;
  core->last_battery_check = 0;
  core->at_buffer_len = 0;
    
  // Prepare X25519 private key
  uint8_t x25519_private[32];
  memcpy(x25519_private, identity->private_key, 32);
  x25519_private[0] &= 248;
  x25519_private[31] &= 127;
  x25519_private[31] |= 64;
    
  // Initialize session and onion routing
  session_manager_new(&core->session_manager);
  onion_router_new(&core->onion_router, x25519_private);
  route_builder_new(&core->route_builder);
    
  // Generate our universal address
  address_translator_from_public_key(identity->public_key, &core->our_address);
}

static const NodeIdentity* lunar_core_identity(const LunarCore *core) 
{
  return &core->identity;
}



static void lunar_core_handle_dio1_interrupt(LunarCore *core) 
{
  // Clear interrupt flag
  dio1_triggered = false;
  // Read interrupt status from radio
  uint16_t irq_status;
  if (sx1262_get_irq_status(&core->radio, &irq_status) == 0) 
  {
    // TX complete
    if (irq_status & 0x01) 
    {
      tx_complete = true;
      last_activity = system_ticks;
    }
    // RX done
    if (irq_status & 0x02) 
    {
      packet_pending = true;
      last_activity = system_ticks;
    }
    // Preamble detected (0x04) - no action needed
    // CRC error
    if (irq_status & 0x40) 
    {
      core->stats.rx_errors++;
      error_count++;
    }
    // Clear interrupt
    sx1262_clear_irq(&core->radio, irq_status);
  }
}

void lunar_core_process_radio_events(LunarCore *core) 
{
  // Handle TX complete
  if (atomic_exchange(&tx_complete, false)) 
  {
    // Restart RX mode
    if (sx1262_start_rx(&core->radio, 0) == 0) core->rx_active = true;
    led_controller_flash(&core->led, 1);
  }
  // Handle RX packet
  if (atomic_exchange(&packet_pending, false)) 
  {
    led_controller_flash(&core->led, 2);
  }
}



static void lunar_core_configure_radio_for_protocol(LunarCore *core, Protocol protocol) 
{
  // Configure radio parameters based on detected protocol
  // This would contain protocol-specific configuration
  // Implementation depends on protocol specifics
}

static void lunar_core_handle_meshcore_frame(LunarCore *core, const Frame *frame, uart_port_t uart) 
{
  // Handle MeshCore protocol frame
  // Implementation depends on MeshCore protocol specifics
}

static void lunar_core_handle_meshtastic_frame(LunarCore *core, const MeshtasticFrame *frame, uart_port_t uart) 
{
  // Handle Meshtastic protocol frame
  // Implementation depends on Meshtastic protocol specifics
}

static void lunar_core_handle_rnode_frame(LunarCore *core, const KissFrame *frame, uart_port_t uart) 
{
  // Handle RNode/KISS protocol frame
  // Implementation depends on RNode protocol specifics
}

static void lunar_core_process_serial_byte(LunarCore *core, uint8_t byte, uart_port_t uart) 
{
  // Protocol detection
  if (core->serial_protocol == PROTOCOL_UNKNOWN) 
  {
    TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
    Protocol detected = protocol_detector_feed(&transport->detector, byte);
    if (detected != PROTOCOL_UNKNOWN) 
    {
      core->serial_protocol = detected;
      core->stats.protocol_switches++;
      ESP_LOGI("LunarCore", "Protocol detected: %s", protocol_name(detected));
      lunar_core_configure_radio_for_protocol(core, detected);
    }
  }
  // Process byte according to detected protocol
  switch (core->serial_protocol) 
  {
    case PROTOCOL_MESHCORE: 
    {
      Frame *frame = frame_parser_feed(&core->meshcore_parser, byte);
      if (frame != NULL) 
      {
        lunar_core_handle_meshcore_frame(core, frame, uart);
        TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
        protocol_detector_confirm_frame(&transport->detector);
      }
      break;
    }
        
    case PROTOCOL_MESHTASTIC: 
    {
      MeshtasticFrame *frame = meshtastic_handler_feed_serial(&core->meshtastic, byte);
      if (frame != NULL) 
      {
        lunar_core_handle_meshtastic_frame(core, frame, uart);
        TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
        protocol_detector_confirm_frame(&transport->detector);
      }
      break;
    }
        
    case PROTOCOL_RNODE: 
    {
      KissFrame *frame = rnode_handler_feed_serial(&core->rnode, byte);
      if (frame != NULL) 
      {
        lunar_core_handle_rnode_frame(core, frame, uart);
        TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
        protocol_detector_confirm_frame(&transport->detector);
      }
      break;
    }
        
    default:
    break;
  }
}






/////////////////////////////////////////////////////////////////////////////////////

// Forward declaration of the external function run_lunarcore
extern void run_lunarcore(void);



//Main lunarcore function

void main(void) 
{
  run_lunarcore();
}



/* Atomic variables for thread-safe counters and timestamps */
static _Atomic uint32_t DIO1_COUNT = 0;
static _Atomic uint32_t LAST_ACTIVITY = 0;

/* Helper functions to simulate Rust functionality */
uint32_t millis() 
{
  return (uint32_t)(esp_timer_get_time() / 1000);
}

void IRAM_ATTR dio1_isr(void* arg) 
{
  atomic_fetch_add(&DIO1_COUNT, 1);
}

/* Function prototypes for external logic referenced in run_lunarcore */
node_identity_t node_identity_from_hardware(void);
bool load_repeater_setting(void);
void init_watchdog(void);
void feed_watchdog(void);
esp_err_t status_display_init(status_display_t* disp);
esp_err_t status_display_boot_animation(status_display_t* disp, void (*delay_fn)(uint32_t));
esp_err_t status_display_show_status(status_display_t* disp, const status_content_t* status);
esp_err_t sx1262_init(sx1262_t* radio);
esp_err_t sx1262_configure(sx1262_t* radio, const radio_config_t* config);
esp_err_t sx1262_start_rx(sx1262_t* radio, uint32_t timeout);
esp_err_t sx1262_transmit(sx1262_t* radio, const uint8_t* data, size_t len);
esp_err_t sx1262_get_irq_status(sx1262_t* radio, uint16_t* irq);
esp_err_t sx1262_clear_irq(sx1262_t* radio, uint16_t irq);
esp_err_t sx1262_read_packet(sx1262_t* radio, uint8_t* buffer, size_t* len, int16_t* rssi, int8_t* snr);
esp_err_t sx1262_get_status(sx1262_t* radio, uint8_t* chip_mode, uint8_t* cmd_status);
uint16_t sx1262_get_errors(sx1262_t* radio);
esp_err_t ble_init(ble_stack_t* ble, const char* name);
esp_err_t ble_start_advertising(ble_stack_t* ble);
uint32_t ble_connection_count(ble_stack_t* ble);
void lunar_core_process_radio_events(lunar_core_t* core, uart_port_t uart);
void lunar_core_process_serial_byte(lunar_core_t* core, uint8_t byte, uart_port_t uart);
void lunar_core_route_rx_packet(lunar_core_t* core, const uint8_t* data, size_t len, int16_t rssi, int8_t snr, uart_port_t uart);
bool lunar_core_maybe_relay_packet(lunar_core_t* core, const uint8_t* data, size_t len, uint32_t now);
bool lunar_core_app_connected(lunar_core_t* core);
bool lunar_core_repeater_active(lunar_core_t* core);
void lunar_core_reset_protocol(lunar_core_t* core);
void lunar_core_update_battery(lunar_core_t* core, uint32_t adc_value);
bool led_update(led_controller_t* led, uint32_t now);
void led_flash(led_controller_t* led, uint32_t count);

/* Main Entry Point: run_lunarcore */
void run_lunarcore(void) 
{
  // identity logic
  node_identity_t identity = node_identity_from_hardware();
  printf("[INIT] Node ID: %08X\r\n",(uint32_t)identity.node_id);
  // GPIO Setup (Peripherals::take equivalent)
  gpio_init();
  // OLED Reset and I2C Setup
  i2c_init();
  oled_init();
  //display starting animation
  status_display_boot_animation(&status_display, (void (*)(uint32_t))vTaskDelay);
  // SPI and LoRa Radio Setup
  spi_init();
  sx1262_t radio = 
  {
    .spi = spi_handle,
    .nss = GPIO_NUM_8,
    .reset = GPIO_NUM_12,
    .busy = GPIO_NUM_13,
    .dio1 = GPIO_NUM_14
  };
  radio_init();
  retarget_init();  
  lunarcore_logic_init();

  // LunarCore Logic Init
  lunar_core_t lunarcore;
  memset(&lunarcore, 0, sizeof(lunarcore));
  lunarcore.radio = radio;
  lunarcore.identity = identity;
  lunarcore.repeater_enabled = load_repeater_setting();
  printf("[INIT] LunarCore OK (repeater: %s)\r\n", lunarcore.repeater_enabled ? "ON" : "OFF");
  //ble_init(); //not used
  // UART Init
  uart_init();
  init_watchdog();
  // ADC Init
  adc_init();

  printf("========================================\r\n");
  printf("  Protocols: MeshCore, Meshtastic, KISS\r\n");
  printf("  Waiting for protocol detection...\r\n");
  printf("========================================\r\n");

  radio_config();

  radio_start_rx();

  uint32_t last_second = 0;
  uint32_t battery_check_interval = 0;
  uint32_t last_beacon_time = 0;

  uint32_t beacon_interval = 3000 + (lunarcore.identity.public_key[0] % 2000);

  uint8_t beacon_data[8];
  memcpy(&beacon_data[0], "TEST", 4);
  memcpy(&beacon_data[4], &lunarcore.identity.public_key[0], 4);

  uint16_t current_irq = 0;
  uint16_t last_nonzero_irq = 0;

  // Infinite Loop
  while (1) 
  {
    uint32_t now = millis();
    // Beacon Logic
    if (!lunar_core_app_connected(&lunarcore) && (now - last_beacon_time) >= beacon_interval) 
    {
      last_beacon_time = now;
      lunarcore.rx_active = false;
      if (sx1262_transmit(&lunarcore.radio, beacon_data, sizeof(beacon_data)) == ESP_OK) 
      {
        lunarcore.stats.tx_packets += 1;
        lunarcore.rx_active = true;
      }
    }
    feed_watchdog();
    lunar_core_process_radio_events(&lunarcore, UART_NUM_0);
    // Serial Processing
    uint8_t byte;
    while (uart_read_bytes(UART_NUM_0, &byte, 1, 0) > 0) 
    {
      lunar_core_process_serial_byte(&lunarcore, byte, UART_NUM_0);
      atomic_store_explicit(&LAST_ACTIVITY, now, memory_order_relaxed);
    }
    // Radio IRQ Processing
    uint16_t irq;
    if (sx1262_get_irq_status(&lunarcore.radio, &irq) == ESP_OK) 
    {
      current_irq = irq;
      if (irq != 0) 
      {
        last_nonzero_irq = irq;
        // RxError, CRC error
        if (irq & 0x40) lunarcore.stats.rx_errors++;
        // RxDone and no error, process RX packet
        if ((irq & 0x02) && !(irq & 0x40)) 
        {
          uint8_t rx_buffer[256];
          size_t rx_len = 0;
          int16_t rssi;
          int8_t snr;
          if (sx1262_read_packet(&lunarcore.radio, rx_buffer, &rx_len, &rssi, &snr) == ESP_OK) 
          {
            lunarcore.stats.rx_packets += 1;
            lunarcore.stats.last_rssi = rssi;
            lunar_core_route_rx_packet(&lunarcore, rx_buffer, rx_len, rssi, snr, UART_NUM_0);
            if (lunar_core_maybe_relay_packet(&lunarcore, rx_buffer, rx_len, now)) led_flash(&lunarcore.led, 3);
            else led_flash(&lunarcore.led, 2);
          } 
          else lunarcore.stats.rx_errors += 1;
        }
        sx1262_clear_irq(&lunarcore.radio, irq);
        // TxDone or RxDone -> restart RX
        if ((irq & 0x01) || (irq & 0x02)) 
        {
          sx1262_start_rx(&lunarcore.radio, 0);
          lunarcore.rx_active = true;
        }
      }
    }
    // LED Update
    if (led_update(&lunarcore.led, now)) 
    {
      if (lunarcore.led.is_on) gpio_set_level(GPIO_NUM_35, 1);
      else gpio_set_level(GPIO_NUM_35, 0);
    }
    // Battery Check (10s interval)
    if ((now - battery_check_interval) >= 10000) 
    {
      battery_check_interval = now;
      int adc_raw;
      if (adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &adc_raw) == ESP_OK) lunar_core_update_battery(&lunarcore, (uint32_t)adc_raw);
    }
    // BLE and Protocol Logic
/*     {
      uint32_t ble_now = ble_connection_count(&lunarcore.ble);
      if (lunarcore.serial_protocol != Protocol_Unknown && ble_now == 0 && lunarcore.last_serial_rx > 0 && (now - lunarcore.last_serial_rx) > 30000) 
      {
        ESP_LOGI(TAG, "Serial idle 30s, resetting protocol");
        lunar_core_reset_protocol(&lunarcore);
      }
      if (lunarcore.prev_ble_connections > 0 && ble_now == 0 && (lunarcore.last_serial_rx == 0 || (now - lunarcore.last_serial_rx) > 5000)) 
      {
        ESP_LOGI(TAG, "BLE disconnected, resetting protocol");
        lunar_core_reset_protocol(&lunarcore);
      }
      lunarcore.prev_ble_connections = ble_now;
    } */
    // Second-based periodic tasks
    uint32_t current_second = now / 1000;
    if (current_second > last_second) 
    {
      last_second = current_second;
      lunarcore.stats.uptime_seconds = current_second;
      if (current_second % 2 == 0) 
      {
        const char* protocol_name;
        switch (lunarcore.serial_protocol) 
        {
          case Protocol_MeshCore:   protocol_name = "MeshCore"; break;
          case Protocol_Meshtastic: protocol_name = "Meshtastic"; break;
          case Protocol_RNode:      protocol_name = "RNode/KISS"; break;
          case Protocol_AtCommand:  protocol_name = "AT Command"; break;
          default:                  protocol_name = "Detecting..."; break;
        }
        uint8_t chip_mode = 0, cmd_status = 0;
        sx1262_get_status(&lunarcore.radio, &chip_mode, &cmd_status);
        uint16_t device_errors = sx1262_get_errors(&lunarcore.radio);
        status_content_t status = 
        {
          .protocol = protocol_name,
          .node_id = lunarcore.identity.node_id,
          .battery_pct = lunarcore.battery.percentage,
          .rx_count = lunarcore.stats.rx_packets,
          .tx_count = lunarcore.stats.tx_packets,
          .rssi = lunarcore.stats.last_rssi,
          .connected = (lunarcore.serial_protocol != Protocol_Unknown),
          .irq_status = current_irq,
          .last_irq = last_nonzero_irq,
          .dio1_count = atomic_load_explicit(&DIO1_COUNT, memory_order_relaxed),
          .chip_mode = chip_mode,
          .device_errors = device_errors,
          .repeater_active = lunar_core_repeater_active(&lunarcore),
          .relay_count = lunarcore.relay_count
        };
        status_display_show_status(&status_display, &status);
      }
      if (current_second % 60 == 0) 
      {
/*         ESP_LOGI(TAG, "Uptime: %lus, RX: %lu, TX: %lu",
        (unsigned long)current_second,
        (unsigned long)lunarcore.stats.rx_packets,
        (unsigned long)lunarcore.stats.tx_packets); */
      }
    }
  vTaskDelay(pdMS_TO_TICKS(1));
  }
}
