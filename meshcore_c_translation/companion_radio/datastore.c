#include <Arduino.h>
#include "DataStore.h"
#include <string.h>
#include <stdio.h>

#if defined(EXTRAFS) || defined(QSPIFLASH)
  #define MAX_BLOBRECS 100
#else
  #define MAX_BLOBRECS 20
#endif


  #if defined(QSPIFLASH)
    #include <CustomLFS_QSPIFlash.h>
  #elif defined(EXTRAFS)
    #include <CustomLFS.h>
  #else 
    #include <InternalFileSystem.h>
  #endif


void data_store_init(data_store_t* self, filesystem_t* fs, rtc_clock_t* clock) {
  self->_fs = fs;
  self->_fs_extra = NULL;
  self->_clock = clock;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  identity_store_init(&self->identity_store, fs, "");
#elif defined(RP2040_PLATFORM)
  identity_store_init(&self->identity_store, fs, "/identity");
#else
  identity_store_init(&self->identity_store, fs, "/identity");
#endif
}

#if defined(EXTRAFS) || defined(QSPIFLASH)
void data_store_init_extra(data_store_t* self, filesystem_t* fs, filesystem_t* fs_extra, rtc_clock_t* clock) {
  self->_fs = fs;
  self->_fs_extra = fs_extra;
  self->_clock = clock;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  identity_store_init(&self->identity_store, fs, "");
#elif defined(RP2040_PLATFORM)
  identity_store_init(&self->identity_store, fs, "/identity");
#else
  identity_store_init(&self->identity_store, fs, "/identity");
#endif
}
#endif

static file_t open_write(filesystem_t* fs, const char* filename) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  filesystem_remove(fs, filename);
  return filesystem_open(fs, filename, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  return filesystem_open(fs, filename, "w");
#else
  return filesystem_open(fs, filename, "w", true);
#endif
}

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  static uint32_t _contacts_channels_total_blocks = 0;
#endif

void data_store_begin(data_store_t* self) {
#if defined(RP2040_PLATFORM)
  identity_store_begin(&self->identity_store);
#endif

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  _contacts_channels_total_blocks = filesystem_get_fs(data_store_get_contacts_channels_fs(self))->cfg->block_count;
  data_store_check_adv_blob_file(self);
  #if defined(EXTRAFS) || defined(QSPIFLASH)
  data_store_migrate_to_secondary_fs(self);
  #endif
#else
  // init 'blob store' support
  filesystem_mkdir(self->_fs, "/bl");
#endif
}

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
int _count_lfs_block(void *p, lfs_block_t block){
      if (block > _contacts_channels_total_blocks) {
        MESH_DEBUG_PRINTLN("ERROR: Block %d exceeds filesystem bounds - CORRUPTION DETECTED!", block);
        return LFS_ERR_CORRUPT;  // return error to abort lfs_traverse() gracefully
    }
  lfs_size_t *size = (lfs_size_t*) p;
  *size += 1;
    return 0;
}

lfs_ssize_t _get_lfs_used_block_count(filesystem_t* fs) {
  lfs_size_t size = 0;
  int err = lfs_traverse(filesystem_get_fs(fs), _count_lfs_block, &size);
  if (err) {
    MESH_DEBUG_PRINTLN("ERROR: lfs_traverse() error: %d", err);
    return 0;
  }
  return size;
}
#endif

uint32_t data_store_get_storage_used_kb(const data_store_t* self) {
#if defined(ESP32)
  return SPIFFS.usedBytes() / 1024;
#elif defined(RP2040_PLATFORM)
  FSInfo info;
  info.usedBytes = 0;
  filesystem_info(self->_fs, &info);
  return info.usedBytes / 1024;
#elif defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  const lfs_config* config = filesystem_get_fs(data_store_get_contacts_channels_fs(self))->cfg;
  int used_block_count = _get_lfs_used_block_count(data_store_get_contacts_channels_fs(self));
  int used_bytes = config->block_size * used_block_count;
  return used_bytes / 1024;
#else
  return 0;
#endif
}

uint32_t data_store_get_storage_total_kb(const data_store_t* self) {
#if defined(ESP32)
  return SPIFFS.totalBytes() / 1024;
#elif defined(RP2040_PLATFORM)
  FSInfo info;
  info.totalBytes = 0;
  filesystem_info(self->_fs, &info);
  return info.totalBytes / 1024;
#elif defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  const lfs_config* config = filesystem_get_fs(data_store_get_contacts_channels_fs(self))->cfg;
  int total_bytes = config->block_size * config->block_count;
  return total_bytes / 1024;
#else
  return 0;
#endif
}

file_t data_store_open_read_default(data_store_t* self, const char* filename) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return filesystem_open(self->_fs, filename, FILE_O_READ);
#elif defined(RP2040_PLATFORM)
  return filesystem_open(self->_fs, filename, "r");
#else
  return filesystem_open(self->_fs, filename, "r", false);
#endif
}

file_t data_store_open_read(data_store_t* self, filesystem_t* fs, const char* filename) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return filesystem_open(fs, filename, FILE_O_READ);
#elif defined(RP2040_PLATFORM)
  return filesystem_open(fs, filename, "r");
#else
  return filesystem_open(fs, filename, "r", false);
#endif
}

bool data_store_remove_file_default(data_store_t* self, const char* filename) {
  return filesystem_remove(self->_fs, filename);
}

bool data_store_remove_file(data_store_t* self, filesystem_t* fs, const char* filename) {
  return filesystem_remove(fs, filename);
}

bool data_store_format_file_system(data_store_t* self) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  if (self->_fs_extra == NULL) {
    return filesystem_format(self->_fs);
  } else {
    return filesystem_format(self->_fs) && filesystem_format(self->_fs_extra);
  }
#elif defined(RP2040_PLATFORM)
  return LittleFS.format();
#elif defined(ESP32)
  bool fs_success = ((fs_spiffs_t *)self->_fs)->format();
  esp_err_t nvs_err = nvs_flash_erase(); // no need to reinit, will be done by reboot
  return fs_success && (nvs_err == ESP_OK);
#else
  #error "need to implement format()"
#endif
}

bool data_store_load_main_identity(data_store_t* self, local_identity_t *identity) {
  return identity_store_load(&self->identity_store, "_main", identity);
}

bool data_store_save_main_identity(data_store_t* self, const local_identity_t *identity) {
  return identity_store_save(&self->identity_store, "_main", identity);
}

void data_store_load_prefs(data_store_t* self, node_prefs_t* prefs) {
  if (filesystem_exists(self->_fs, "/prefs.json")) {
    file_t file = data_store_open_read(self, self->_fs, "/prefs.json");
    if (file_is_open(&file)) {
      node_prefs_load_serial(prefs, &file);   // new Serial prefs
      file_close(&file);
    }
  } else if (filesystem_exists(self->_fs, "/new_prefs")) {
    data_store_load_prefs_int(self, "/new_prefs", prefs);
    if (data_store_save_prefs(self, prefs) ) {                // save to new format
      //filesystem_remove(self->_fs, "/new_prefs"); // remove old
    }
  }
}

void data_store_load_prefs_int(data_store_t* self, const char *filename, node_prefs_t* _prefs) {
  file_t file = data_store_open_read(self, self->_fs, filename);
  if (file_is_open(&file)) {
    uint8_t pad[8];

    file_read(&file, (uint8_t *)&_prefs->airtime_factor, sizeof(float));                           // 0
    file_read(&file, (uint8_t *)_prefs->node_name, sizeof(_prefs->node_name));                      // 4
    file_read(&file, pad, 4);                                                                     // 36
    file_read(&file, (uint8_t *)&_prefs->node_lat, sizeof(_prefs->node_lat));                       // 40
    file_read(&file, (uint8_t *)&_prefs->node_lon, sizeof(_prefs->node_lon));                       // 48
    file_read(&file, (uint8_t *)&_prefs->freq, sizeof(_prefs->freq));                               // 56
    file_read(&file, (uint8_t *)&_prefs->sf, sizeof(_prefs->sf));                                   // 60
    file_read(&file, (uint8_t *)&_prefs->cr, sizeof(_prefs->cr));                                   // 61
    file_read(&file, (uint8_t *)&_prefs->_client_repeat, sizeof(_prefs->_client_repeat));             // 62
    file_read(&file, (uint8_t *)&_prefs->manual_add_contacts, sizeof(_prefs->manual_add_contacts)); // 63
    file_read(&file, (uint8_t *)&_prefs->bw, sizeof(_prefs->bw));                                   // 64
    file_read(&file, (uint8_t *)&_prefs->tx_power_dbm, sizeof(_prefs->tx_power_dbm));               // 68
    file_read(&file, (uint8_t *)&_prefs->telemetry_mode_base, sizeof(_prefs->telemetry_mode_base)); // 69
    file_read(&file, (uint8_t *)&_prefs->telemetry_mode_loc, sizeof(_prefs->telemetry_mode_loc));   // 70
    file_read(&file, (uint8_t *)&_prefs->telemetry_mode_env, sizeof(_prefs->telemetry_mode_env));   // 71
    file_read(&file, (uint8_t *)&_prefs->rx_delay_base, sizeof(_prefs->rx_delay_base));             // 72
    file_read(&file, (uint8_t *)&_prefs->advert_loc_policy, sizeof(_prefs->advert_loc_policy));     // 76
    file_read(&file, (uint8_t *)&_prefs->multi_acks, sizeof(_prefs->multi_acks));                   // 77
    file_read(&file, (uint8_t *)&_prefs->path_hash_mode, sizeof(_prefs->path_hash_mode));           // 78
    file_read(&file, pad, 1);                                                                     // 79
    file_read(&file, (uint8_t *)&_prefs->ble_pin, sizeof(_prefs->ble_pin));                         // 80
    file_read(&file, (uint8_t *)&_prefs->buzzer_quiet, sizeof(_prefs->buzzer_quiet));               // 84
    file_read(&file, (uint8_t *)&_prefs->gps_enabled, sizeof(_prefs->gps_enabled));                 // 85
    file_read(&file, (uint8_t *)&_prefs->gps_interval, sizeof(_prefs->gps_interval));               // 86
    file_read(&file, (uint8_t *)&_prefs->autoadd_config, sizeof(_prefs->autoadd_config));           // 87
    file_read(&file, (uint8_t *)&_prefs->autoadd_max_hops, sizeof(_prefs->autoadd_max_hops));       // 88
    file_read(&file, (uint8_t *)&_prefs->rx_boosted_gain, sizeof(_prefs->rx_boosted_gain));         // 89
    file_read(&file, (uint8_t *)_prefs->default_scope_name, sizeof(_prefs->default_scope_name));    // 90
    file_read(&file, (uint8_t *)_prefs->default_scope_key, sizeof(_prefs->default_scope_key));     // 121

    // migrate old fields
    node_prefs_set_repeat_en(_prefs, _prefs->_client_repeat != 0);

    file_close(&file);
  }
}

bool data_store_save_prefs(data_store_t* self, node_prefs_t* _prefs) {
  file_t file = open_write(self->_fs, "/prefs.json");
  if (file_is_open(&file)) {
    bool success = node_prefs_save_serial(_prefs, &file);
    file_close(&file);
    return success;
  }
  return false;
}

void data_store_load_contacts(data_store_t* self, data_store_host_t* host) {
file_t file = data_store_open_read(self, data_store_get_contacts_channels_fs(self), "/contacts3");
    if (file_is_open(&file)) {
      bool full = false;
      while (!full) {
        contact_info_t c;
        uint8_t pub_key[32];
        uint8_t unused;

        bool success = (file_read(&file, pub_key, 32) == 32);
        success = success && (file_read(&file, (uint8_t *)&c.name, 32) == 32);
        success = success && (file_read(&file, &c.type, 1) == 1);
        success = success && (file_read(&file, &c.flags, 1) == 1);
        success = success && (file_read(&file, &unused, 1) == 1);
        success = success && (file_read(&file, (uint8_t *)&c.sync_since, 4) == 4); // was 'reserved'
        success = success && (file_read(&file, (uint8_t *)&c.out_path_len, 1) == 1);
        success = success && (file_read(&file, (uint8_t *)&c.last_advert_timestamp, 4) == 4);
        success = success && (file_read(&file, c.out_path, 64) == 64);
        success = success && (file_read(&file, (uint8_t *)&c.lastmod, 4) == 4);
        success = success && (file_read(&file, (uint8_t *)&c.gps_lat, 4) == 4);
        success = success && (file_read(&file, (uint8_t *)&c.gps_lon, 4) == 4);

        if (!success) break; // EOF

        identity_init_from_pub_key(&c.id, pub_key);
        if (!data_store_host_on_contact_loaded(host, &c)) full = true;
      }
      file_close(&file);
    }
}

void data_store_save_contacts(data_store_t* self, data_store_host_t* host, bool (*filter)(const contact_info_t* c)) {
  file_t file = open_write(data_store_get_contacts_channels_fs(self), "/contacts3");
  if (file_is_open(&file)) {
    uint32_t idx = 0;
    contact_info_t c;
    uint8_t unused = 0;

    while (data_store_host_get_contact_for_save(host, idx, &c)) {
      if (filter && !filter(&c)) {
        idx++;  // advance to next contact
        continue;
      }
      bool success = (file_write(&file, c.id.pub_key, 32) == 32);
      success = success && (file_write(&file, (uint8_t *)&c.name, 32) == 32);
      success = success && (file_write(&file, &c.type, 1) == 1);
      success = success && (file_write(&file, &c.flags, 1) == 1);
      success = success && (file_write(&file, &unused, 1) == 1);
      success = success && (file_write(&file, (uint8_t *)&c.sync_since, 4) == 4);
      success = success && (file_write(&file, (uint8_t *)&c.out_path_len, 1) == 1);
      success = success && (file_write(&file, (uint8_t *)&c.last_advert_timestamp, 4) == 4);
      success = success && (file_write(&file, c.out_path, 64) == 64);
      success = success && (file_write(&file, (uint8_t *)&c.lastmod, 4) == 4);
      success = success && (file_write(&file, (uint8_t *)&c.gps_lat, 4) == 4);
      success = success && (file_write(&file, (uint8_t *)&c.gps_lon, 4) == 4);

      if (!success) break; // write failed

      idx++;  // advance to next contact
    }
    file_close(&file);
  }
}

void data_store_load_channels(data_store_t* self, data_store_host_t* host) {
    file_t file = data_store_open_read(self, data_store_get_contacts_channels_fs(self), "/channels2");
    if (file_is_open(&file)) {
      bool full = false;
      uint8_t channel_idx = 0;
      while (!full) {
        channel_details_t ch;
        uint8_t unused[4];

        bool success = (file_read(&file, unused, 4) == 4);
        success = success && (file_read(&file, (uint8_t *)ch.name, 32) == 32);
        success = success && (file_read(&file, (uint8_t *)ch.channel.secret, 32) == 32);

        if (!success) break; // EOF

        if (data_store_host_on_channel_loaded(host, channel_idx, &ch)) {
          channel_idx++;
        } else {
          full = true;
        }
      }
      file_close(&file);
    }
}

void data_store_save_channels(data_store_t* self, data_store_host_t* host) {
  file_t file = open_write(data_store_get_contacts_channels_fs(self), "/channels2");
  if (file_is_open(&file)) {
    uint8_t channel_idx = 0;
    channel_details_t ch;
    uint8_t unused[4];
    memset(unused, 0, 4);

    while (data_store_host_get_channel_for_save(host, channel_idx, &ch)) {
      bool success = (file_write(&file, unused, 4) == 4);
      success = success && (file_write(&file, (uint8_t *)ch.name, 32) == 32);
      success = success && (file_write(&file, (uint8_t *)ch.channel.secret, 32) == 32);

      if (!success) break; // write failed
      channel_idx++;
    }
    file_close(&file);
  }
}

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)

#define MAX_ADVERT_PKT_LEN   (2 + 32 + PUB_KEY_SIZE + 4 + SIGNATURE_SIZE + MAX_ADVERT_DATA_SIZE)

typedef struct {
  uint32_t timestamp;
  uint8_t  key[7];
  uint8_t  len;
  uint8_t  data[MAX_ADVERT_PKT_LEN];
} blob_rec_t;

void data_store_check_adv_blob_file(data_store_t* self) {
  if (!filesystem_exists(data_store_get_contacts_channels_fs(self), "/adv_blobs")) {
    file_t file = open_write(data_store_get_contacts_channels_fs(self), "/adv_blobs");
    if (file_is_open(&file)) {
      blob_rec_t zeroes;
      memset(&zeroes, 0, sizeof(zeroes));
      for (int i = 0; i < MAX_BLOBRECS; i++) {     // pre-allocate to fixed size
        file_write(&file, (uint8_t *) &zeroes, sizeof(zeroes));
      }
      file_close(&file);
    }
  }
}

void data_store_migrate_to_secondary_fs(data_store_t* self) {
  // migrate old adv_blobs, contacts3 and channels2 files to secondary FS if they don't already exist
  if (!filesystem_exists(self->_fs_extra, "/adv_blobs")) {
    if (filesystem_exists(self->_fs, "/adv_blobs")) {
    file_t old_adv_blobs = data_store_open_read(self, self->_fs, "/adv_blobs");
    file_t new_adv_blobs = open_write(self->_fs_extra, "/adv_blobs");

    if (file_is_open(&old_adv_blobs) && file_is_open(&new_adv_blobs)) {
      blob_rec_t rec;
      size_t count = 0;

      // Copy 20 BlobRecs from old to new
      while (count < 20 && file_read(&old_adv_blobs, (uint8_t *)&rec, sizeof(rec)) == sizeof(rec)) {
        file_seek(&new_adv_blobs, count * sizeof(blob_rec_t));
        file_write(&new_adv_blobs, (uint8_t *)&rec, sizeof(rec));
        count++;
      }
    }
    if (file_is_open(&old_adv_blobs)) file_close(&old_adv_blobs);
    if (file_is_open(&new_adv_blobs)) file_close(&new_adv_blobs);
    filesystem_remove(self->_fs, "/adv_blobs");
    }
  }
  if (!filesystem_exists(self->_fs_extra, "/contacts3")) {
    if (filesystem_exists(self->_fs, "/contacts3")) {
      file_t old_file = data_store_open_read(self, self->_fs, "/contacts3");
      file_t new_file = open_write(self->_fs_extra, "/contacts3");

      if (file_is_open(&old_file) && file_is_open(&new_file)) {
        uint8_t buf[64];
        int n;
        while ((n = file_read(&old_file, buf, sizeof(buf))) > 0) {
          file_write(&new_file, buf, n);
        }
      }
      if (file_is_open(&old_file)) file_close(&old_file);
      if (file_is_open(&new_file)) file_close(&new_file);
      filesystem_remove(self->_fs, "/contacts3");
    }
  }
  if (!filesystem_exists(self->_fs_extra, "/channels2")) {
    if (filesystem_exists(self->_fs, "/channels2")) {
      file_t old_file = data_store_open_read(self, self->_fs, "/channels2");
      file_t new_file = open_write(self->_fs_extra, "/channels2");

      if (file_is_open(&old_file) && file_is_open(&new_file)) {
        uint8_t buf[64];
        int n;
        while ((n = file_read(&old_file, buf, sizeof(buf))) > 0) {
          file_write(&new_file, buf, n);
        }
      }
      if (file_is_open(&old_file)) file_close(&old_file);
      if (file_is_open(&new_file)) file_close(&new_file);
      filesystem_remove(self->_fs, "/channels2");
    }
  }
  // cleanup nodes which have been testing the extra fs, copy _main.id and new_prefs back to primary
  if (filesystem_exists(self->_fs_extra, "/_main.id")) {
      if (filesystem_exists(self->_fs, "/_main.id")) {filesystem_remove(self->_fs, "/_main.id");}
      file_t old_file = data_store_open_read(self, self->_fs_extra, "/_main.id");
      file_t new_file = open_write(self->_fs, "/_main.id");

      if (file_is_open(&old_file) && file_is_open(&new_file)) {
        uint8_t buf[64];
        int n;
        while ((n = file_read(&old_file, buf, sizeof(buf))) > 0) {
          file_write(&new_file, buf, n);
        }
      }
      if (file_is_open(&old_file)) file_close(&old_file);
      if (file_is_open(&new_file)) file_close(&new_file);
      filesystem_remove(self->_fs_extra, "/_main.id");
  }
  if (filesystem_exists(self->_fs_extra, "/new_prefs")) {
    if (filesystem_exists(self->_fs, "/new_prefs")) {filesystem_remove(self->_fs, "/new_prefs");}
      file_t old_file = data_store_open_read(self, self->_fs_extra, "/new_prefs");
      file_t new_file = open_write(self->_fs, "/new_prefs");

      if (file_is_open(&old_file) && file_is_open(&new_file)) {
        uint8_t buf[64];
        int n;
        while ((n = file_read(&old_file, buf, sizeof(buf))) > 0) {
          file_write(&new_file, buf, n);
        }
      }
      if (file_is_open(&old_file)) file_close(&old_file);
      if (file_is_open(&new_file)) file_close(&new_file);
      filesystem_remove(self->_fs_extra, "/new_prefs");
  }
  // remove files from where they should not be anymore
  if (filesystem_exists(self->_fs, "/adv_blobs")) {
    filesystem_remove(self->_fs, "/adv_blobs");
  }
  if (filesystem_exists(self->_fs, "/contacts3")) {
    filesystem_remove(self->_fs, "/contacts3");
  }
  if (filesystem_exists(self->_fs, "/channels2")) {
    filesystem_remove(self->_fs, "/channels2");
  }
  if (filesystem_exists(self->_fs_extra, "/_main.id")) {
    filesystem_remove(self->_fs_extra, "/_main.id");
  }
  if (filesystem_exists(self->_fs_extra, "/new_prefs")) {
    filesystem_remove(self->_fs_extra, "/new_prefs");
  }
}

uint8_t data_store_get_blob_by_key(data_store_t* self, const uint8_t key[], int key_len, uint8_t dest_buf[]) {
  file_t file = data_store_open_read(self, data_store_get_contacts_channels_fs(self), "/adv_blobs");
  uint8_t len = 0;  // 0 = not found
  if (file_is_open(&file)) {
    blob_rec_t tmp;
    while (file_read(&file, (uint8_t *) &tmp, sizeof(tmp)) == sizeof(tmp)) {
      if (memcmp(key, tmp.key, sizeof(tmp.key)) == 0) {  // only match by 7 byte prefix
        len = tmp.len;
        memcpy(dest_buf, tmp.data, len);
        break;
      }
    }
    file_close(&file);
  }
  return len;
}

bool data_store_put_blob_by_key(data_store_t* self, const uint8_t key[], int key_len, const uint8_t src_buf[], uint8_t len) {
  if (len < PUB_KEY_SIZE+4+SIGNATURE_SIZE || len > MAX_ADVERT_PKT_LEN) return false;
  data_store_check_adv_blob_file(self);
  file_t file = filesystem_open(data_store_get_contacts_channels_fs(self), "/adv_blobs", FILE_O_WRITE);
  if (file_is_open(&file)) {
    uint32_t pos = 0, found_pos = 0;
    uint32_t min_timestamp = 0xFFFFFFFF;

    // search for matching key OR evict by oldest timestamp
    blob_rec_t tmp;
    file_seek(&file, 0);
    while (file_read(&file, (uint8_t *) &tmp, sizeof(tmp)) == sizeof(tmp)) {
      if (memcmp(key, tmp.key, sizeof(tmp.key)) == 0) {  // only match by 7 byte prefix
        found_pos = pos;
        break;
      }
      if (tmp.timestamp < min_timestamp) {
        min_timestamp = tmp.timestamp;
        found_pos = pos;
      }

      pos += sizeof(tmp);
    }

    memcpy(tmp.key, key, sizeof(tmp.key));  // just record 7 byte prefix of key
    memcpy(tmp.data, src_buf, len);
    tmp.len = len;
    tmp.timestamp = rtc_clock_get_current_time(self->_clock);

    file_seek(&file, found_pos);
    file_write(&file, (uint8_t *) &tmp, sizeof(tmp));

    file_close(&file);
    return true;
  }
  return false; // error
}
bool data_store_delete_blob_by_key(data_store_t* self, const uint8_t key[], int key_len) {
  return true; // this is just a stub on NRF52/STM32 platforms
}
#else
static inline void make_blob_path(const uint8_t key[], int key_len, char* path, size_t path_size) {
  char fname[18];
  if (key_len > 8) key_len = 8; // just use first 8 bytes (prefix)
  mesh_utils_to_hex(fname, key, key_len);
  sprintf(path, "/bl/%s", fname);
}

uint8_t data_store_get_blob_by_key(data_store_t* self, const uint8_t key[], int key_len, uint8_t dest_buf[]) {
  char path[64];
  make_blob_path(key, key_len, path, sizeof(path));

  if (filesystem_exists(self->_fs, path)) {
    file_t f = data_store_open_read(self, self->_fs, path);
    if (file_is_open(&f)) {
      int len = file_read(&f, dest_buf, 255); // currently MAX 255 byte blob len supported!!
      file_close(&f);
      return len;
    }
  }
  return 0; // not found
}

bool data_store_put_blob_by_key(data_store_t* self, const uint8_t key[], int key_len, const uint8_t src_buf[], uint8_t len) {
  char path[64];
  make_blob_path(key, key_len, path, sizeof(path));

  file_t f = open_write(self->_fs, path);
  if (file_is_open(&f)) {
    int n = file_write(&f, src_buf, len);
    file_close(&f);
    if (n == len) return true; // success!

    filesystem_remove(self->_fs, path); // blob was only partially written!
  }
  return false; // error
}

bool data_store_delete_blob_by_key(data_store_t* self, const uint8_t key[], int key_len) {
  char path[64];
  make_blob_path(key, key_len, path, sizeof(path));

  filesystem_remove(self->_fs, path);
  
  return true; // return true even if file did not exist
}
#endif

filesystem_t* data_store_get_contacts_channels_fs(const data_store_t* self) {
#if defined(EXTRAFS) || defined(QSPIFLASH)
  if (self->_fs_extra) return self->_fs_extra;
#endif
  return self->_fs;
}
