# MeshLoRaLite_CLI

Firmware for LoRa mesh nodes based on **STM32F1** + **SX1262** (RadioLib) with a serial command-line interface (CLI), support for unicast/broadcast messages, intelligent forwarding, and persistent Flash configuration. <br>
Written entirely in the Arduino IDE and tested on **DX-SMART DX-PJ26-V1.1** + **DX-LR30**.<br>
To upload the binary to the STM32, see the following repository: <a href="https://github.com/aayes89/STM32F1-serie">STM32F1-serie</a>

This project is geared towards low-cost, long-range, and low-power LoRa networks for applications such as distributed monitoring, off-grid communication, or experimentation with decentralized mesh networks.

***<h3>New: Added Android Client App</h3>***

## Main Features

- **Mesh topology** based on controlled flooding with TTL and duplicate suppression
- **Unicast** (nodeID) and **broadcast** (0xFFFF) messages
- Logical channel configuration (`chan`) persisted in Flash (Allows segmenting mesh networks on the same physical frequency.)
- Duplicate detection and removal (cache of last seen messages)
- **Periodic beacon (30s)** for neighbor discovery (adjustable)
- Full CLI interface via serial port (115200 by default, configurable)
- Commands: `send`, `broadcast`, `set`, `get`, `save`, `load`, `status`, `reboot`, etc.

- Persistent configuration on the last Flash page (frequency, SF, BW, CR, power, TTL, beacon, debug)
- Persistent debug level control (`set debug on/off`)
- Use of **RadioLib** for DX-LR30 (SX1262) management
- Generation of a **unique NodeID** based on the STM32 chip UID + CRC16
- Random jitter in sends and resends to reduce collisions
- ACKs and delivery confirmations
- List of neighbors detected via beacons

## Hardware Requirements

- MCU: **STM32F103** (Blue Pill or similar)
- LoRa Module: **SX1262** (DX-LR30, RA-02, E22, Heltec, etc. with standard pins)
- Recommended Connections:

| STM32 Pin | SX1262 Pin | Function |

|-------------|----------------|------------------|
| PA4 | NSS | ChipSelect |
| PA5 | SCK | SPI Clock |
| PA7 | MOSI | SPI Data In |
| PA6 | MISO | SPI Data Out |
| PA3 | RESET | Reset |
| PA2 | BUSY | Busy/IRQ |
| PC15 | DIO1 | IRQ RX/TX |
| PB11 | LED (optional) | Status indicator |

- Optional: TXEN/RXEN pins if your module requires them (currently unused, Dio2 as an RF switch)

## Quick Installation and Use

1. Clone the repository

``bash

git clone https://github.com/aayes89/MeshLoRaLite.git

2. Open the project in the Arduino IDE

3. Connect the node via USB and upload the firmware

4. Open the serial monitor at 115200 baud (or your configured baud rate)

## Available Commands:

- help → displays the command menu

- get [all|radio|mesh] → gets specific module information

- status → displays node information

- send <NODEID> <message> → sends a unicast message to node 0xXXXX

- broadcast <message> → sends a message to all nodes

- set freq 915000000 → Change frequency (Hz)
- set sf 9 → Spreading factor 7-12
- set bw 250 → Bandwidth in kHz
- set cr <4 to 8> → Encoding rate
- set ttl <0 to 10 (max)> → Time to live
- set beacon <ms> → Beacon transmission interval. Example: 30000
- set power <-9 to 22> → dBm power
- set debug <on|off|1|0> → detailed logs
- set chan <0 to 255> → modifies the transmission channel
- set baud <115200> → baud rate
- nodes → returns information about listed nearby nodes
- nodes clear → clears the nearby nodes table
- save → saves configuration to Flash
- load → loads configuration from Flash
- reboot → restarts the node

* NodeID is entered in hexadecimal format without 0x (e.g., 6019, 4CA9, etc.) — it matches the value displayed at startup.

* Persistent configuration. All parameters are saved on the last page of Flash (1 KB reserved). At startup, it is loaded automatically. If it fails, it uses default values.

## Development

### Logical Channel:
* Each packet has a `chan` field in its mesh header.

* The node only processes messages whose `chan` matches the currently configured channel.

* This allows multiple mesh networks to coexist on the same physical frequency, avoiding unwanted collisions.

* The logical channel does not encrypt or isolate traffic; it only filters it.

### Acknowledgments and Retries
* Unicast packets wait for an ACK from the destination.

* If it is not received within approximately 3 seconds, the packet is retried up to two times.

* The original content is saved for successful retransmission (not just the header).

### Neighbor Table
* Nodes detected via beacons are registered with RSSI/SNR and time.

* The table is automatically cleared if no beacons are received for approximately 90 seconds.

## Debugging
Enables/disables detailed logs:
- set debug on
- set debug off
- save

* RSSI and SNR are automatically displayed in RX logs if debug is enabled.

* Logs conditioned on the debug variable (volatile boolean synchronized with cfg.debug)

## Flash Persistence

Saved:
- Radio parameters
- TTL, beacon, debug, logical channel

Not saved:
- Neighbor table
- Seen packet cache
- Retry queue

## Internal Architecture (Summary)

- Continuous RX radio with interrupts (DIO1)
- Non-blocking TX + explicit return to RX
- Seen packet cache (src + id)
- Unicast retry queue
- Forwarding conditioned by TTL, destination, and logical channel

## Memory Considerations and Limitations

STM32F103C8T6 (~20 KB RAM):
- Fixed and limited table and buffer sizes (payload ~64 bytes)
- Optimized for small/medium-sized networks
- Not suitable for hundreds of nodes Simultaneous
- Flooding only (no smart routing yet)
- Pending: Dynamic support for serial baud rate changes without a full reset
- Pending: Integration with apps/web (WebSerial, Bluetooth, etc.)

## Logic Diagram
```
TX → Radio → Air → RX ISR → handlePkt()

↓
Filter (ver/chan/seen)

↓
Process / Forward / ACK
```

## License
MIT License

## Contributions
**Welcome!**<br> Open an issue or pull request if you want to add ACKs (adaptive backoff, metrics, etc.), simple encryption, better collision handling, topology visualization, etc.<br>
Thanks for trying MeshLoRaLite!