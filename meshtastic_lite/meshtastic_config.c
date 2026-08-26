#include "meshtastic_config.h"

static const RegionDef MESH_REGIONS[] = 
{
  {REGION_US,      902.0f,  928.0f,  100, 30, "US"    },
  {REGION_EU_433,  433.0f,  434.0f,  10,  10, "EU_433"},
  {REGION_EU_868,  869.4f,  869.65f, 10,  27, "EU_868"},
  {REGION_CN,      470.0f,  510.0f,  100, 19, "CN"    },
  {REGION_JP,      920.5f,  923.5f,  100, 13, "JP"    },
  {REGION_ANZ,     915.0f,  928.0f,  100, 30, "ANZ"   },
  {REGION_ANZ_433, 433.05f, 434.79f, 100, 14, "ANZ433"},
  {REGION_KR,      920.0f,  923.0f,  100, 23, "KR"    },
  {REGION_TW,      920.0f,  925.0f,  100, 27, "TW"    },
  {REGION_IN,      865.0f,  867.0f,  100, 30, "IN"    },
  {REGION_NZ_865,  864.0f,  868.0f,  100, 36, "NZ_865"},
  {REGION_TH,      920.0f,  925.0f,  100, 16, "TH"    },
  {REGION_RU,      868.7f,  869.2f,  100, 20, "RU"    },
  {REGION_UNSET,   902.0f,  928.0f,  100, 30, "UNSET" },
};

/* void meshPresetParams(MeshModemPreset_t preset,ModemParams_t *params)
{
  switch(preset)  
  {
    case MODEM_LONG_FAST:
    default:
    params->bw_khz = 250.0;
    params->sf = 11;
    params->cr = 5;
    break;
    
    case MODEM_LONG_SLOW:
    params->bw_khz = 125.0;
    params->sf = 12;
    params->cr = 8;
    break;
    
    case MODEM_LONG_MODERATE:
    params->bw_khz = 125.0;
    params->sf = 11;
    params->cr = 8;
    break;
    
    case MODEM_LONG_TURBO:
    params->bw_khz = 500.0;
    params->sf = 11;
    params->cr = 8;
    break;
    
    case MODEM_MEDIUM_FAST:
    params->bw_khz = 250.0;
    params->sf = 9;
    params->cr = 5;
    break;
    
    case MODEM_MEDIUM_SLOW:
    params->bw_khz = 250.0;
    params->sf = 10;
    params->cr = 5;
    break;
    
    case MODEM_SHORT_FAST:
    params->bw_khz = 250.0;
    params->sf = 7;
    params->cr = 5;
    break;
    
    case MODEM_SHORT_SLOW:
    params->bw_khz = 250.0;
    params->sf = 8;
    params->cr = 5;
    break;
    
    case MODEM_SHORT_TURBO:
    params->bw_khz = 500.0;
    params->sf = 7;
    params->cr = 5;
    break;
  }
} */

ModemParams_t meshPresetParams(MeshModemPreset_t preset)
{
  switch(preset)  
  {
    case MODEM_SHORT_TURBO:
    default:
    return {500.0f,7,5};
    
    case MODEM_SHORT_FAST:    
    return {250.0f,7,5};
    
    case MODEM_SHORT_SLOW:    
    return {250.0f,8,5};
    
    case MODEM_MEDIUM_FAST:   
    return {250.0f,9,5};
    
    case MODEM_MEDIUM_SLOW:   
    return {250.0f,10,5};
    
    case MODEM_LONG_TURBO:    
    return {500.0f,11,8};
    
    case MODEM_LONG_MODERATE: 
    return {125.0f,11,8};
    
    case MODEM_LONG_SLOW:     
    return {125.0f,12,8};
  }
}

const char* meshPresetName(MeshModemPreset_t preset)
{
  switch (preset) 
  {
    case MODEM_SHORT_TURBO:   
    default:
    return "ShortTurbo";
    
    case MODEM_SHORT_FAST:    
    return "ShortFast";
    
    case MODEM_SHORT_SLOW:    
    return "ShortSlow";
    
    case MODEM_MEDIUM_FAST:   
    return "MediumFast";
    
    case MODEM_MEDIUM_SLOW:   
    return "MediumSlow";
    
    case MODEM_LONG_TURBO:    
    return "LongTurbo";
    
    case MODEM_LONG_MODERATE: 
    return "LongMod";
    
    case MODEM_LONG_SLOW:     
    return "LongSlow";
  }
}

const RegionDef_t* meshGetRegion(MeshRegion_t r) //unused
{
  for (size_t i = 0; i < sizeof(MESH_REGIONS) / sizeof(MESH_REGIONS[0]); i++) 
  {
    if (MESH_REGIONS[i].region == r) return &MESH_REGIONS[i];
  }
  return &MESH_REGIONS[sizeof(MESH_REGIONS) / sizeof(MESH_REGIONS[0]) - 1]; // UNSET
}