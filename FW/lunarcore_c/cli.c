#include "cli.h"

void cli_getversion(int argc, char **argv);
void cli_status(int argc, char **argv);
void cli_nodeid(int argc, char **argv);
void cli_mac(int argc, char **argv);
void cli_initconfig(int argc, char **argv);
void cli_storeconfig(int argc, char **argv);

CommandEntry_t commands[] =
  {
		//General
		COMMAND_ENTRY("AT+VERSION", "", cli_getversion, "Enter test mode"),
    COMMAND_ENTRY("AT+STATUS", "", cli_status, "Enter test mode"),
    COMMAND_ENTRY("AT+NODEID", "", cli_nodeid, "Enter test mode"),
    COMMAND_ENTRY("AT+MAC", "", cli_mac, "Enter test mode"),
    COMMAND_ENTRY("AT+FREQ=", "", cli_test, "Enter test mode"),
    COMMAND_ENTRY("AT+SF=", "", cli_test, "Enter test mode"),
    COMMAND_ENTRY("AT+TXPOWER=", "", cli_test, "Enter test mode"),
    COMMAND_ENTRY("AT+RESET", "", cli_test, "Enter test mode"),
    
    COMMAND_ENTRY("AT+RX", "", cli_test, "Enter test mode"),
    COMMAND_ENTRY("AT+RSSI", "", cli_test, "Enter test mode"),
    COMMAND_ENTRY("AT", "", cli_test, "Enter test mode"),
    
    COMMAND_ENTRY("AT+SF=", "", cli_test, "Enter test mode"),
    COMMAND_ENTRY("AT+SF=", "", cli_test, "Enter test mode"),
    

		//End marker
		COMMAND_ENTRY(NULL, NULL, NULL, NULL),
  };
  
  void cli_getversion(int argc, char **argv)
  {
    printf("LunarCore v1.0.0\r\n");
    printf("Unified Mesh Bridge Firmware\r\n");
    printf("Protocols: MeshCore, Meshtastic, RNode/KISS\r\n");
    printf("OK\r\n");
  }
  
  void cli_status(int argc, char **argv)
  {
     printf("Status: ");
    if (core->rx_active) printf("RX Active\r\n");
    else printf("Idle\r\n");
    char buf[32];
    const char *battery_str = format_battery(&core->battery,buf);
    printf("%s\r\n",battery_str);
    printf("TX: %u, RX: %u\r\n",core->stats.tx_packets,core->stats.rx_packets);
    printf("OK\r\n");
  }
  
  void cli_nodeid(int argc, char **argv)
  {
    printf("Node ID: 0x%08X\r\n",core->identity.node_id);
    printf("OK\r\n");
  }
  
  void cli_mac(int argc, char **argv)
  {
    printf(uart, "MAC: ", 5);
    for (int i = 0; i < 6; i++) 
    {
      write_hex8(uart, core->identity.mac_address[i]);
      if (i < 5) uart_write_bytes(uart, ":", 1);
    }
    uart_write_bytes(uart, "\r\n", 2);
    printf("OK\r\n");
  }

static void lunar_core_process_at_command(LunarCore *core, uart_port_t uart) 
{


  else if (memcmp(cmd_upper, "AT+FREQ=", 8) == 0) 
  {
    uint32_t freq;
    if (parse_u32_from_cmd(&cmd_upper[8], len - 8, &freq)) 
    {
      ESP_LOGI("LunarCore", "Setting frequency to %d Hz", freq);
      RadioConfig config = core->radio.config;
      config.frequency = freq;
      if (sx1262_configure(&core->radio, &config) == 0) uart_write_bytes(uart, "OK\r\n", 4);
      else uart_write_bytes(uart, "ERROR\r\n", 7);
    } 
    else uart_write_bytes(uart, "ERROR: Invalid frequency\r\n", 26);
  }
  else if (memcmp(cmd_upper, "AT+SF=", 6) == 0) 
  {
    uint32_t sf;
    if (parse_u32_from_cmd(&cmd_upper[6], len - 6, &sf)) 
    {
      if (sf >= 7 && sf <= 12) 
      {
        ESP_LOGI("LunarCore", "Setting SF to %d", sf);
        RadioConfig config = core->radio.config;
        config.spreading_factor = (uint8_t)sf;
        if (sx1262_configure(&core->radio, &config) == 0) uart_write_bytes(uart, "OK\r\n", 4);
        else uart_write_bytes(uart, "ERROR\r\n", 7);
      } 
      else uart_write_bytes(uart, "ERROR: SF must be 7-12\r\n", 24);
    } 
    else uart_write_bytes(uart, "ERROR: Invalid SF\r\n", 19);
  }
  else if (memcmp(cmd_upper, "AT+TXPOWER=", 11) == 0) 
  {
    int8_t power;
    if (parse_i8_from_cmd(&cmd_upper[11], len - 11, &power)) 
    {
      if (power >= -9 && power <= 22) 
      {
        ESP_LOGI("LunarCore", "Setting TX power to %d dBm", power);
        RadioConfig config = core->radio.config;
        config.tx_power = power;
        if (sx1262_configure(&core->radio, &config) == 0) uart_write_bytes(uart, "OK\r\n", 4);
        else uart_write_bytes(uart, "ERROR\r\n", 7);
      } 
      else uart_write_bytes(uart, "ERROR: Power must be -9 to +22\r\n", 33);
    } 
    else uart_write_bytes(uart, "ERROR: Invalid power\r\n", 22);
  }
  else if (memcmp(cmd_upper, "AT+RESET", 8) == 0) 
  {
    if (sx1262_init(&core->radio) == 0) uart_write_bytes(uart, "OK\r\n", 4);
    else uart_write_bytes(uart, "ERROR\r\n", 7);
  }
  else if (memcmp(cmd_upper, "AT+RX", 5) == 0) 
  {
    if (sx1262_start_rx(&core->radio, 0) == 0) 
    {
      core->rx_active = true;
      uart_write_bytes(uart, "OK\r\n", 4);
    } 
    else uart_write_bytes(uart, "ERROR\r\n", 7);
  }
  else if (memcmp(cmd_upper, "AT+RSSI", 7) == 0) 
  {
    int16_t rssi;
    if (sx1262_get_rssi(&core->radio, &rssi) == 0) 
    {
      uart_write_bytes(uart, "RSSI: ", 6);
      write_i16(uart, rssi);
      uart_write_bytes(uart, " dBm\r\n", 6);
      uart_write_bytes(uart, "OK\r\n", 4);
    } 
    else uart_write_bytes(uart, "ERROR\r\n", 7);
  }
  else if (memcmp(cmd_upper, "AT", 2) == 0 && len == 2) uart_write_bytes(uart, "OK\r\n", 4);
  else if (memcmp(cmd_upper, "AT+HELP", 7) == 0 || memcmp(cmd_upper, "AT?", 3) == 0) 
  {
    uart_write_bytes(uart, "Available commands:\r\n", 21);
    uart_write_bytes(uart, "  AT          - Test\r\n", 22);
    uart_write_bytes(uart, "  ATI         - Version info\r\n", 30);
    uart_write_bytes(uart, "  AT+STATUS   - System status\r\n", 31);
    uart_write_bytes(uart, "  AT+NODEID   - Node ID\r\n", 25);
    uart_write_bytes(uart, "  AT+MAC      - MAC address\r\n", 29);
    uart_write_bytes(uart, "  AT+FREQ=Hz  - Set frequency\r\n", 31);
    uart_write_bytes(uart, "  AT+SF=n     - Set spreading factor\r\n", 38);
    uart_write_bytes(uart, "  AT+TXPOWER=n - Set TX power\r\n", 32);
    uart_write_bytes(uart, "  AT+RX       - Start RX mode\r\n", 31);
    uart_write_bytes(uart, "  AT+RSSI     - Get RSSI\r\n", 26);
    uart_write_bytes(uart, "  AT+RESET    - Reset radio\r\n", 29);
    uart_write_bytes(uart, "OK\r\n", 4);
  }
  else uart_write_bytes(uart, "ERROR: Unknown command\r\n", 24);
}