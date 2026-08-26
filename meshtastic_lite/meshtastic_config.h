#ifndef __MESHTASTIC_CONFIG_H__
#define __MESHTASTIC_CONFIG_H__


#include <stdint.h>
#include <stddef.h>

// ─── Radio Constants ───────────────────────────────────────────────────────────

#define  MESH_SYNC_WORD        0x2B    // RadioLibInterface.h:84
#define  MESH_PREAMBLE_LENGTH  16      // RadioInterface.h:98
#define  MESH_HOP_MAX          7       // MeshTypes.h:38
#define  MESH_HOP_RELIABLE     3       // MeshTypes.h:41
#define  MESH_MAX_PAYLOAD      255     // RadioInterface.h:20
#define  MESH_HEADER_LEN       16      // RadioInterface.h:21

// CSMA/CA parameters (RadioInterface.h:95-103)
#define  MESH_CW_MIN           3
#define  MESH_CW_MAX           8
#define  MESH_NUM_SYM_CAD      2

// ─── Modem Presets ─────────────────────────────────────────────────────────────

typedef enum
{
  MODEM_LONG_FAST = 0,     // SF11, BW250, CR4/5  (default)
  MODEM_LONG_SLOW,         // SF12, BW125, CR4/8
  MODEM_LONG_MODERATE,     // SF11, BW125, CR4/8
  MODEM_LONG_TURBO,        // SF11, BW500, CR4/8
  MODEM_MEDIUM_FAST,       // SF9,  BW250, CR4/5
  MODEM_MEDIUM_SLOW,       // SF10, BW250, CR4/5
  MODEM_SHORT_FAST,        // SF7,  BW250, CR4/5
  MODEM_SHORT_SLOW,        // SF8,  BW250, CR4/5
  MODEM_SHORT_TURBO,       // SF7,  BW500, CR4/5
  MODEM_PRESET_COUNT
}MeshModemPreset_t;

typedef struct  
{
  float    bw_khz;         // bandwidth in kHz
  uint8_t  sf;             // spreading factor 7-12
  uint8_t  cr;             // coding rate denominator (5 = 4/5, 8 = 4/8)
}ModemParams_t;


// ─── Region Definitions (not in use anymore) ────────────────────────────────────────────────────────

typedef enum
{
  REGION_US = 0,
  REGION_EU_433,
  REGION_EU_868,
  REGION_CN,
  REGION_JP,
  REGION_ANZ,
  REGION_ANZ_433,
  REGION_KR,
  REGION_TW,
  REGION_IN,
  REGION_NZ_865,
  REGION_TH,
  REGION_RU,
  REGION_UNSET,   // Falls back to US band plan
  REGION_COUNT
}MeshRegion_t;

typedef struct
{
  MeshRegion region;
  float      freq_start;    // MHz
  float      freq_end;      // MHz
  uint8_t    duty_cycle;    // percent (100 = no limit)
  uint8_t    power_limit;   // dBm (0 = use default 17)
  const char *name;
}RegionDef_t;

// ─── Device Roles (subset we support) ──────────────────────────────────────────

typedef enum
{
  ROLE_CLIENT      = 0,   // Normal node: TX + RX, rebroadcast with SNR-weighted delay
  ROLE_CLIENT_MUTE = 1,   // RX only, no rebroadcast, no TX
  ROLE_ROUTER_LATE = 2,   // TX + RX, rebroadcast with LONG delay (after routers)
}MeshRole_t;


ModemParams_t meshPresetParams(MeshModemPreset_t preset);
const char* meshPresetName(MeshModemPreset_t preset); 
const RegionDef* meshGetRegion(MeshRegion_t r); //unused

#endif


