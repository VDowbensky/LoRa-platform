// Helper function: Initialize HeaplessVec128
static void heapless_vec128_init(HeaplessVec128* vec) 
{
  vec->len = 0;
  memset(vec->data, 0, MAX_DID_LENGTH);
}

// Helper function: Initialize HeaplessVec64
static void heapless_vec64_init(HeaplessVec64* vec) 
{
  vec->len = 0;
  memset(vec->data, 0, MAX_NAME_LENGTH);
}

// Helper function: Initialize HeaplessVec256
static void heapless_vec256_init(HeaplessVec256* vec) 
{
  vec->len = 0;
  memset(vec->data, 0, 256);
}

// Helper function: Initialize HeaplessVec512
static void heapless_vec512_init(HeaplessVec512* vec) 
{
  vec->len = 0;
  memset(vec->data, 0, 512);
}

// Helper function: Extend HeaplessVec128 from slice
static ContactHelloError heapless_vec128_extend_from_slice(HeaplessVec128* vec, const uint8_t* data, size_t len) 
{
  if (vec->len + len > MAX_DID_LENGTH) return CONTACT_HELLO_ERROR_DID_TOO_LONG;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return CONTACT_HELLO_ERROR_NONE;
}

// Helper function: Extend HeaplessVec64 from slice
static ContactHelloError heapless_vec64_extend_from_slice(HeaplessVec64* vec, const uint8_t* data, size_t len) 
{
  if (vec->len + len > MAX_NAME_LENGTH) return CONTACT_HELLO_ERROR_NAME_TOO_LONG;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return CONTACT_HELLO_ERROR_NONE;
}

// Helper function: Extend HeaplessVec256 from slice
static void heapless_vec256_extend_from_slice(HeaplessVec256* vec, const uint8_t* data, size_t len) 
{
  if (vec->len + len <= 256) 
  {
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
  }
}

// Helper function: Extend HeaplessVec512 from slice
static void heapless_vec512_extend_from_slice(HeaplessVec512* vec, const uint8_t* data, size_t len) 
{
  if (vec->len + len <= 512) 
  {
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
  }
}

// Helper function: Push byte to HeaplessVec256
static void heapless_vec256_push(HeaplessVec256* vec, uint8_t byte) 
{
  if (vec->len < 256) 
  {
    vec->data[vec->len] = byte;
    vec->len++;
  }
}

// Helper function: Push byte to HeaplessVec512
static void heapless_vec512_push(HeaplessVec512* vec, uint8_t byte) 
{
  if (vec->len < 512) 
  {
    vec->data[vec->len] = byte;
    vec->len++;
  }
}

// Helper function: Clear HeaplessVec64
static void heapless_vec64_clear(HeaplessVec64* vec) 
{
  vec->len = 0;
}

static inline bool heapless_vec_u8_256_init(HeaplessVec_u8_256 *vec) 
{
  vec->len = 0;
  vec->capacity = 256;
  memset(vec->data, 0, 256);
  return true;
}

static inline bool heapless_vec_u8_256_push(HeaplessVec_u8_256 *vec, uint8_t val) 
{
  if (vec->len >= vec->capacity) return false;
  vec->data[vec->len] = val;
  vec->len++;
  return true;
}

static inline bool heapless_vec_u8_256_extend_from_slice(HeaplessVec_u8_256 *vec, const uint8_t *data, size_t len) 
{
  if (vec->len + len > vec->capacity) return false;
  memcpy(&vec->data[vec->len], data, len);
  vec->len += len;
  return true;
}

static inline uint8_t* heapless_vec_u8_256_as_slice(HeaplessVec_u8_256 *vec) 
{
  return vec->data;
}

static inline size_t heapless_vec_u8_256_len(const HeaplessVec_u8_256 *vec) 
{
  return vec->len;
}

static inline bool heapless_vec_u8_320_init(HeaplessVec_u8_320 *vec) 
{
  vec->len = 0;
  vec->capacity = 320;
  memset(vec->data, 0, 320);
  return true;
}

static inline bool heapless_vec_u8_320_extend_from_slice(HeaplessVec_u8_320 *vec, const uint8_t *data, size_t len) 
{
  if (vec->len + len > vec->capacity) return false;
  memcpy(&vec->data[vec->len], data, len);
  vec->len += len;
  return true;