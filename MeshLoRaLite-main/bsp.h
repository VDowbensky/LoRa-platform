#ifndef _BSP_H_
#define _BSP_H_

/* ================= FLASH ================= */
#define FLASH_BASE_ADDR 0x08000000
#define FLASH_SIZE_KB (*((uint16_t*)0x1FFFF7E0))
#define FLASH_CFG_ADDR (FLASH_BASE_ADDR + FLASH_SIZE_KB * 1024 - 2048)

/* ================= CONFIG ================= */
#define MAX_PKT 220
#define NODE_BCAST 0xFFFF
#define SEEN_CACHE 32
#define MESH_VER 1
#define UART_SPEED 115200
#define NODE_TABLE_SIZE 16     // maximum number of nodes
#define NODE_TIMEOUT_MS 90000  // 90s before updating node table
#define MAX_PENDING 4

#endif
