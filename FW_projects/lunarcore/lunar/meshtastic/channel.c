#include "channel.h"

// Default ModemPreset
ModemPreset modem_preset_default(void) 
{
  return MODEM_PRESET_LONG_FAST;
}

// Get lora_params for a ModemPreset
LoraParams modem_preset_lora_params(ModemPreset preset) 
{
  LoraParams params;
  switch (preset) 
  {
    case MODEM_PRESET_LONG_SLOW:
    params.spreading_factor = 12;
    params.bandwidth = 125000;
    params.coding_rate = 8;
    break;
        
    case MODEM_PRESET_LONG_FAST:
    params.spreading_factor = 11;
    params.bandwidth = 125000;
    params.coding_rate = 8;
    break;
        
    case MODEM_PRESET_LONG_MODERATE:
    params.spreading_factor = 11;
    params.bandwidth = 125000;
    params.coding_rate = 5;
    break;
        
    case MODEM_PRESET_VERY_LONG_SLOW:
    params.spreading_factor = 12;
    params.bandwidth = 125000;
    params.coding_rate = 8;
    break;
        
    case MODEM_PRESET_MEDIUM_SLOW:
    params.spreading_factor = 10;
    params.bandwidth = 250000;
    params.coding_rate = 5;
    break;
        
    case MODEM_PRESET_MEDIUM_FAST:
    params.spreading_factor = 9;
    params.bandwidth = 250000;
    params.coding_rate = 5;
    break;
        
    case MODEM_PRESET_SHORT_SLOW:
    params.spreading_factor = 8;
    params.bandwidth = 250000;
    params.coding_rate = 5;
    break;
        
    case MODEM_PRESET_SHORT_FAST:
    params.spreading_factor = 7;
    params.bandwidth = 250000;
    params.coding_rate = 5;
    break;
        
    case MODEM_PRESET_SHORT_TURBO:
    params.spreading_factor = 7;
    params.bandwidth = 500000;
    params.coding_rate = 5;
    break;
        
    default:
    params.spreading_factor = 11;
    params.bandwidth = 125000;
    params.coding_rate = 8;
    break;
  }
  return params;
}

// Calculate airtime in milliseconds
uint32_t modem_preset_airtime_ms(ModemPreset preset, size_t payload_bytes) 
{
  LoraParams params = modem_preset_lora_params(preset);
  float sf = (float)params.spreading_factor;
  float bw = (float)params.bandwidth;
  float cr = (float)params.coding_rate;
  float t_sym = (powf(2.0f, sf)) / bw * 1000.0f;
  float t_preamble = (8.0f + 4.25f) * t_sym;
  float pl = (float)payload_bytes;
  float de = (sf >= 11.0f) ? 1.0f : 0.0f;
  float h = 0.0f;
  float crc = 1.0f;
  float numerator = 8.0f * pl - 4.0f * sf + 28.0f + 16.0f * crc - 20.0f * h;
  float denominator = 4.0f * (sf - 2.0f * de);
  float ceiling_val = ceilf(numerator / denominator);
  if (ceiling_val < 0.0f) ceiling_val = 0.0f;
  float n_payload = 8.0f + ceiling_val * (cr + 4.0f);
  float t_payload = n_payload * t_sym;
  return (uint32_t)(t_preamble + t_payload);
}

// Drop/cleanup for ChannelKey
void channel_key_drop(ChannelKey* key) 
{
  if (key->type == CHANNEL_KEY_AES128) secure_zero(key->data.aes128, KEY_SIZE_128);
  else if (key->type == CHANNEL_KEY_AES256) secure_zero(key->data.aes256, KEY_SIZE_256);
}

// Create ChannelKey from bytes
ChannelKey channel_key_from_bytes(const uint8_t* key, size_t len) 
{
  ChannelKey ck;
  if (len == 0) ck.type = CHANNEL_KEY_NONE;
  else if (len <= 16) 
  {
    ck.type = CHANNEL_KEY_AES128;
    memset(ck.data.aes128, 0, KEY_SIZE_128);
    memcpy(ck.data.aes128, key, len);
  } 
  else 
  {
    ck.type = CHANNEL_KEY_AES256;
    memset(ck.data.aes256, 0, KEY_SIZE_256);
    size_t copy_len = (len < KEY_SIZE_256) ? len : KEY_SIZE_256;
    memcpy(ck.data.aes256, key, copy_len);
  }
  return ck;
}

// Default key
ChannelKey channel_key_default(void) 
{
  ChannelKey ck;
  ck.type = CHANNEL_KEY_AES128;
  memcpy(ck.data.aes128, mesh_kdf_default_key(), KEY_SIZE_128);
  return ck;
}

// Create ChannelKey from channel name
ChannelKey channel_key_from_channel_name(const char* name) 
{
  if (name == NULL || name[0] == '\0') return channel_key_default();
  ChannelKey ck;
  ck.type = CHANNEL_KEY_AES256;
  mesh_kdf_derive_channel_key(name, ck.data.aes256);
  return ck;
}

// Check if key is encrypted
bool channel_key_is_encrypted(const ChannelKey* key) 
{
  return key->type != CHANNEL_KEY_NONE;
}

// Get key as bytes
const uint8_t* channel_key_as_bytes(const ChannelKey* key, size_t* out_len) 
{
  switch (key->type) 
  {
    case CHANNEL_KEY_NONE:
    *out_len = 0;
    return NULL;
        
    case CHANNEL_KEY_AES128:
    *out_len = KEY_SIZE_128;
    return key->data.aes128;
        
    case CHANNEL_KEY_AES256:
    *out_len = KEY_SIZE_256;
    return key->data.aes256;
        
    default:
    *out_len = 0;
    return NULL;
  }
}



// Vec operations for VecChannelName
void vec_channel_name_clear(VecChannelName* vec) 
{
  vec->len = 0;
}

bool vec_channel_name_extend(VecChannelName* vec, const uint8_t* data, size_t len) 
{
  if (vec->len + len > MAX_CHANNEL_NAME) return false;
  memcpy(&vec->data[vec->len], data, len);
  vec->len += len;
  return true;
}

// Vec operations for Vec256
bool vec256_from_slice(Vec256* vec, const uint8_t* data, size_t len) 
{
  if (len > MAX_VEC_SIZE) return false;
  memcpy(vec->data, data, len);
  vec->len = len;
  return true;
}

bool vec256_extend(Vec256* vec, const uint8_t* data, size_t len)
{
  if (vec->len + len > MAX_VEC_SIZE) return false;
  memcpy(&vec->data[vec->len], data, len);
  vec->len += len;
  return true;
}



// Create new Channel
Channel channel_new(uint8_t index) 
{
  Channel ch;
  ch.index = index;
  ch.name.len = 0;
  ch.key = channel_key_default();
  ch.modem_preset = modem_preset_default();
  ch.uplink_enabled = false;
  ch.downlink_enabled = false;
  ch.position_precision = 0;
  return ch;
}

// Create primary Channel
Channel channel_primary(void) 
{
  Channel ch = channel_new(0);
  const char* primary_name = "Primary";
  vec_channel_name_extend(&ch.name, (const uint8_t*)primary_name, strlen(primary_name));
  return ch;
}

// Set channel name
void channel_set_name(Channel* ch, const char* name) 
{
  vec_channel_name_clear(&ch->name);
  size_t len = strlen(name);
  if (len > MAX_CHANNEL_NAME) len = MAX_CHANNEL_NAME;
  vec_channel_name_extend(&ch->name, (const uint8_t*)name, len);
  // Drop old key before replacing
  channel_key_drop(&ch->key);
  ch->key = channel_key_from_channel_name(name);
}

// Set channel key
void channel_set_key(Channel* ch, const uint8_t* key, size_t len) 
{
  // Drop old key before replacing
  channel_key_drop(&ch->key);
  ch->key = channel_key_from_bytes(key, len);
}

// Encrypt data
bool channel_encrypt(const Channel* ch, uint32_t packet_id, uint32_t sender, 
                                   const uint8_t* plaintext, size_t plaintext_len,
                                   Vec256* ciphertext) 
{
  if (!channel_key_is_encrypted(&ch->key)) return vec256_from_slice(ciphertext, plaintext, plaintext_len);// No encryption, just copy plaintext
  uint8_t nonce[NONCE_SIZE];
  mesh_kdf_derive_nonce(packet_id, sender, nonce);
  // Copy plaintext to ciphertext
  if (!vec256_from_slice(ciphertext, plaintext, plaintext_len)) return false;
  switch (ch->key.type) 
  {
    case CHANNEL_KEY_AES128: 
    {
      Aes128 cipher;
      aes128_new(&cipher, ch->key.data.aes128);
      uint8_t nonce_block[16];
      memcpy(nonce_block, nonce, 16);
      aes128_encrypt_ctr(&cipher, nonce_block, ciphertext->data, ciphertext->len);
      break;
    }
    
    case CHANNEL_KEY_AES256: 
    {
      Aes256 cipher;
      aes256_new(&cipher, ch->key.data.aes256);
      uint8_t nonce_block[16];
      memcpy(nonce_block, nonce, 16);
      aes256_encrypt_ctr(&cipher, nonce_block, ciphertext->data, ciphertext->len);
      break;
    }
    
    case CHANNEL_KEY_NONE:
    break;
  }
  return true;
}

// Decrypt data
bool channel_decrypt(const Channel* ch, uint32_t packet_id, uint32_t sender,
                                   const uint8_t* ciphertext_data, size_t ciphertext_len,
                                   Vec256* plaintext) 
{
  if (!channel_key_is_encrypted(&ch->key)) return vec256_from_slice(plaintext, ciphertext_data, ciphertext_len);// No encryption, just copy ciphertext
  uint8_t nonce[NONCE_SIZE];
  mesh_kdf_derive_nonce(packet_id, sender, nonce);
  // Copy ciphertext to plaintext
  if (!vec256_from_slice(plaintext, ciphertext_data, ciphertext_len)) return false;
  switch (ch->key.type) 
  {
    case CHANNEL_KEY_AES128: 
    {
      Aes128 cipher;
      aes128_new(&cipher, ch->key.data.aes128);
      uint8_t nonce_block[16];
      memcpy(nonce_block, nonce, 16);
      aes128_decrypt_ctr(&cipher, nonce_block, plaintext->data, plaintext->len);
      break;
    }
    
    case CHANNEL_KEY_AES256: 
    {
      Aes256 cipher;
      aes256_new(&cipher, ch->key.data.aes256);
      uint8_t nonce_block[16];
      memcpy(nonce_block, nonce, 16);
      aes256_decrypt_ctr(&cipher, nonce_block, plaintext->data, plaintext->len);
      break;
    }
    
    case CHANNEL_KEY_NONE:
    break;
  }
  return true;
}

// Calculate channel hash
uint8_t channel_hash(const Channel* ch) 
{
  size_t key_len;
  const uint8_t* key_bytes = channel_key_as_bytes(&ch->key, &key_len);
  if (key_len == 0) return 0;
  uint8_t h = 0;
  for (size_t i = 0; i < key_len; i++) h ^= key_bytes[i];
  return h;
}

// Get channel name as string
const char* channel_name_str(const Channel* ch) 
{
  // Note: This assumes the name is valid UTF-8
  // In production, you'd want to ensure null-termination
  static char buf[MAX_CHANNEL_NAME + 1];
  if (ch->name.len > 0) 
  {
    memcpy(buf, ch->name.data, ch->name.len);
    buf[ch->name.len] = '\0';
    return buf;
  }
  return "";
}

// Default Channel
Channel channel_default(void) 
{
  return channel_primary();
}

// Cleanup Channel
void channel_drop(Channel* ch) 
{
  channel_key_drop(&ch->key);
}

// Create new ChannelSet
ChannelSet channel_set_new(void) 
{
  ChannelSet cs;
  for (int i = 0; i < MAX_CHANNELS; i++) cs.channels[i] = NULL;
  // Initialize primary channel
  cs.channels[0] = (Channel*)malloc(sizeof(Channel));
  *cs.channels[0] = channel_primary();
  return cs;
}

// Get channel by index
const Channel* channel_set_get(const ChannelSet* cs, uint8_t index) 
{
  if (index >= MAX_CHANNELS) return NULL;
  return cs->channels[index];
}

// Get mutable channel by index
Channel* channel_set_get_mut(ChannelSet* cs, uint8_t index) 
{
  if (index >= MAX_CHANNELS) return NULL;
  return cs->channels[index];
}

// Set channel at index
void channel_set_set(ChannelSet* cs, uint8_t index, Channel channel) 
{
  if (index >= MAX_CHANNELS) return;
  // Free existing channel if present
  if (cs->channels[index] != NULL) 
  {
    channel_drop(cs->channels[index]);
    free(cs->channels[index]);
  }
  // Allocate and set new channel
  cs->channels[index] = (Channel*)malloc(sizeof(Channel));
  *cs->channels[index] = channel;
}

// Get primary channel
const Channel* channel_set_primary(const ChannelSet* cs) 
{
  return cs->channels[0];
}

// Get mutable primary channel
Channel* channel_set_primary_mut(ChannelSet* cs) 
{
  return cs->channels[0];
}

// Get or initialize primary channel
Channel* channel_set_primary_or_init(ChannelSet* cs) 
{
  if (cs->channels[0] == NULL) 
  {
    cs->channels[0] = (Channel*)malloc(sizeof(Channel));
    *cs->channels[0] = channel_primary();
  }
  return cs->channels[0];
}

// Find channel by hash
const Channel* channel_set_find_by_hash(const ChannelSet* cs, uint8_t hash) 
{
  for (int i = 0; i < MAX_CHANNELS; i++) 
  {
    if (cs->channels[i] != NULL) 
    {
      if (channel_hash(cs->channels[i]) == hash) return cs->channels[i];
    }
  }
  return NULL;
}

// Count channels
size_t channel_set_count(const ChannelSet* cs) 
{
  size_t count = 0;
  for (int i = 0; i < MAX_CHANNELS; i++) 
  {
    if (cs->channels[i] != NULL) count++;
  }
  return count;
}

// Default ChannelSet
ChannelSet channel_set_default(void) 
{
  return channel_set_new();
}

// Cleanup ChannelSet
void channel_set_drop(ChannelSet* cs) 
{
  for (int i = 0; i < MAX_CHANNELS; i++) 
  {
    if (cs->channels[i] != NULL) 
    {
      channel_drop(cs->channels[i]);
      free(cs->channels[i]);
      cs->channels[i] = NULL;
    }
  }
}


