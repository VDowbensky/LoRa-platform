/**
 * meshtastic_task.h — FreeRTOS integration for meshtastic-lite on T-Display-P4.
 *
 * Hardware notes (from t_display_p4_config.h):
 *   SPI1: SCLK=GPIO2, MOSI=GPIO3, MISO=GPIO4  (dedicated to SX1262, NOT shared with display)
 *   SX1262_CS   = GPIO 24
 *   SX1262_BUSY = GPIO 6
 *   SX1262_DIO1 = XL9535 IO17 (I2C GPIO expander — no direct HW interrupt)
 *   SX1262_RST  = XL9535 IO16 (assumed, also via expander)
 *
 * Because DIO1 is behind the XL9535 I2C expander, we cannot use hardware
 * interrupts. Instead, we poll the SX1262's IRQ status register via SPI
 * using RadioLib's getIrqFlags(). This is the same approach used by the
 * Meshtastic firmware's startReceiveDutyCycleAuto() path.
 *
 * Architecture:
 *   - meshTaskRx: FreeRTOS task that polls for incoming packets
 *   - meshTaskTx: optional task that drains a TX queue with CSMA/CA
 *   - Both tasks communicate with the main ADS-B app via FreeRTOS queues
 *
 * This is an EXAMPLE / SKELETON. Adapt to your RadioLib and XL9535 driver.
 *
 * Part of meshtastic-lite.
 */
#pragma once

// ── You would include your actual project headers here ──
// #include "driver/spi_master.h"
// #include "driver/gpio.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "RadioLib.h"      // jgromes/RadioLib
// #include "xl9535_driver.h" // your XL9535 I2C GPIO expander driver

#include "meshtastic.h"


// ─── Message Types for Inter-task Communication ────────────────────────────────

/**
 * A decoded Meshtastic message passed to the main app via queue.
 */
typedef struct MeshInboundMsg 
{
  uint32_t    from;
  uint32_t    to;
  uint32_t    id;
  MeshPortNum portnum;
  int8_t      channel_idx;    // -1 for PKI DM
  bool        is_pki;
  float       rssi;
  float       snr;
  uint8_t     payload[240];
  size_t      payload_len;
}MeshInboundMsg_t;

/**
 * An outbound message queued for TX.
 */
typedef struct MeshOutboundMsg 
{
  uint32_t    to;             // MESH_ADDR_BROADCAST or specific node
  MeshPortNum portnum;
  int8_t      channel_idx;    // -1 = PKI DM to `to`
  bool        want_ack;
  uint8_t     payload[200];
  size_t      payload_len;
}MeshOutboundMsg_t;

// ─── Global State ──────────────────────────────────────────────────────────────


// ─── Initialization ────────────────────────────────────────────────────────────

/**
 * Initialize the Meshtastic subsystem.
 * Call from app_main() AFTER the XL9535 I2C driver is initialized.
 *
 * Pseudocode — adapt to your specific RadioLib / ESP-IDF setup.
 */
void meshSystemInit(/* your SPI handle, XL9535 handle, etc. */);

// ─── RX Task (polling) ─────────────────────────────────────────────────────────

/**
 * FreeRTOS task that polls the SX1262 for received packets.
 *
 * Since DIO1 is behind the XL9535 I2C expander, we can't use a GPIO ISR.
 * Instead we poll at ~50ms intervals. At SF11/BW250, a minimum packet is
 * ~230ms on-air, so 50ms polling gives us plenty of margin.
 *
 * Alternative: poll the XL9535 DIO1 pin via I2C, which avoids SPI traffic
 * when idle but adds I2C latency. Choose based on your bus utilization.
 */
void meshRxTask(void *pvParameters);
// ─── TX Task (CSMA/CA) ────────────────────────────────────────────────────────

/**
 * FreeRTOS task that drains the TX queue with CSMA/CA backoff.
 */
void meshTxTask(void *pvParameters);

// ─── Public API for Main App ───────────────────────────────────────────────────

/**
 * Send a text message on the default channel (broadcast).
 * Non-blocking — queues the message for the TX task.
 */
bool meshSendText(const char *text);

/**
 * Send a PKI-encrypted direct message to a specific node.
 */
bool meshSendDm(uint32_t to_node, const char *text);

/**
 * Poll for received Meshtastic messages from your main loop / UI task.
 * Returns true if a message was available.
 */
bool meshPollRx(MeshInboundMsg *msg);
