
/* Main Entry Point: run_lunarcore */
void run_lunarcore(void) 
{
  // identity logic
  node_identity_t identity = node_identity_from_hardware();
  
  read_identity();
  read_config();
  init_clocks(0;
  init_peripherals(); //gpio, i2c, spi, uart's, display, adc, wdt, ...
  show_boot_animation();
  //status_display_boot_animation(&status_display, (void (*)(uint32_t))vTaskDelay);
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


  // Infinite Loop
  while (1) 
  {
    radio_proc();
    health_proc(); //on flags
    if(app_connected) app_proc();
    else check_serial_connect();
    feed_watchdog();
    ui_proc(); //keyboard, display, LES's
    
    // Second-based periodic tasks
  
    uint32_t current_second = now / 1000;
    if (current_second > last_second) 
    {
      update_status();
      
      last_second = current_second;
      lunarcore.stats.uptime_seconds = current_second;
/*       if (current_second % 2 == 0) 
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
      } */
    }
  }
}
