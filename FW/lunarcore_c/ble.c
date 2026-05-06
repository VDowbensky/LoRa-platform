#include "ble.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdatomic.h>

// Constants
#define MAX_CONNECTIONS 3
#define MAX_MTU 512
#define MAX_ADV_DATA 31
#define MAX_SCAN_RSP 31
#define TX_QUEUE_DEPTH 8
#define RX_BUFFER_SIZE 512
#define EVENT_QUEUE_SIZE 16
#define FROM_RADIO_QUEUE_SIZE 16

// NUS Service UUIDs
static const uint8_t NUS_SERVICE_UUID[16] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
};

static const uint8_t NUS_RX_UUID[16] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
};

static const uint8_t NUS_TX_UUID[16] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
};

// Meshtastic Service UUIDs
static const uint8_t MESHTASTIC_SERVICE_UUID[16] = {
    0xfd, 0xea, 0x73, 0xe2, 0xca, 0x5d, 0xa8, 0x9f,
    0x1f, 0x46, 0xa8, 0x15, 0x18, 0xb2, 0xa1, 0x6b
};

static const uint8_t MESHTASTIC_FROM_RADIO_UUID[16] = {
    0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0x78, 0xb8,
    0xed, 0x11, 0x93, 0x49, 0x9e, 0xe6, 0x55, 0x2c
};

static const uint8_t MESHTASTIC_TO_RADIO_UUID[16] = {
    0xe7, 0x01, 0x44, 0x12, 0x66, 0x78, 0xdd, 0xa1,
    0xad, 0x4d, 0x9e, 0x12, 0xd2, 0x76, 0x5c, 0xf7
};

static const uint8_t MESHTASTIC_FROM_NUM_UUID[16] = {
    0x53, 0x44, 0xe3, 0x47, 0x75, 0xaa, 0x70, 0xa6,
    0x66, 0x4f, 0x00, 0xa8, 0x8c, 0xa1, 0x9d, 0xed
};

// Connection State Enum
typedef enum {
    CONNECTION_STATE_DISCONNECTED,
    CONNECTION_STATE_CONNECTED,
    CONNECTION_STATE_SUBSCRIBED,
    CONNECTION_STATE_ENCRYPTED
} ConnectionState;

// Service Type Enum
typedef enum {
    SERVICE_TYPE_NUS,
    SERVICE_TYPE_MESHTASTIC,
    SERVICE_TYPE_UNKNOWN
} ServiceType;

// BLE Error Enum
typedef enum {
    BLE_ERROR_OK = 0,
    BLE_ERROR_NOT_INITIALIZED,
    BLE_ERROR_ALREADY_INITIALIZED,
    BLE_ERROR_MAX_CONNECTIONS,
    BLE_ERROR_NOT_CONNECTED,
    BLE_ERROR_NOT_SUBSCRIBED,
    BLE_ERROR_MTU_EXCEEDED,
    BLE_ERROR_QUEUE_FULL,
    BLE_ERROR_INVALID_HANDLE,
    BLE_ERROR_STACK_ERROR,
    BLE_ERROR_TIMEOUT,
    BLE_ERROR_INVALID_PARAMETER
} BleError;

// Variable-length array structure for data storage
typedef struct {
    uint8_t data[MAX_MTU];
    size_t len;
} DataVec;

typedef struct {
    uint8_t data[RX_BUFFER_SIZE];
    size_t len;
} RxDataVec;

// Deque structure for DataVec
typedef struct {
    DataVec items[TX_QUEUE_DEPTH];
    size_t head;
    size_t tail;
    size_t count;
} TxQueue;

typedef struct {
    DataVec items[FROM_RADIO_QUEUE_SIZE];
    size_t head;
    size_t tail;
    size_t count;
} FromRadioQueue;

// BLE Event Types
typedef enum {
    BLE_EVENT_CONNECTED,
    BLE_EVENT_DISCONNECTED,
    BLE_EVENT_MTU_EXCHANGE,
    BLE_EVENT_SUBSCRIBED,
    BLE_EVENT_UNSUBSCRIBED,
    BLE_EVENT_DATA_RECEIVED,
    BLE_EVENT_TX_COMPLETE,
    BLE_EVENT_ENCRYPTION_CHANGED
} BleEventType;

// BLE Event Structure (tagged union)
typedef struct {
    BleEventType type;
    union {
        struct {
            uint16_t conn_handle;
        } connected;
        struct {
            uint16_t conn_handle;
            uint8_t reason;
        } disconnected;
        struct {
            uint16_t conn_handle;
            uint16_t mtu;
        } mtu_exchange;
        struct {
            uint16_t conn_handle;
            ServiceType service;
        } subscribed;
        struct {
            uint16_t conn_handle;
            ServiceType service;
        } unsubscribed;
        struct {
            uint16_t conn_handle;
            ServiceType service;
            RxDataVec data;
        } data_received;
        struct {
            uint16_t conn_handle;
        } tx_complete;
        struct {
            uint16_t conn_handle;
            bool encrypted;
        } encryption_changed;
    } data;
} BleEvent;

// Event queue
typedef struct {
    BleEvent items[EVENT_QUEUE_SIZE];
    size_t head;
    size_t tail;
    size_t count;
} EventQueue;

// BLE Data Packet Structure
typedef struct {
    uint16_t conn_handle;
    ServiceType service;
    RxDataVec data;
} BleDataPacket;

// BLE Connection Structure
typedef struct {
    uint16_t handle;
    ConnectionState state;
    ServiceType service;
    uint16_t mtu;
    TxQueue tx_queue;
    RxDataVec rx_buffer;
    bool nus_tx_notify;
    bool mesh_from_radio_notify;
    bool mesh_from_num_notify;
    bool encrypted;
    uint8_t peer_addr[6];
    uint16_t conn_interval;
    uint16_t conn_latency;
    uint16_t supervision_timeout;
    bool valid; // Indicates if this slot is occupied
} BleConnection;

// BLE State Structure
typedef struct {
    BleConnection connections[MAX_CONNECTIONS];
    uint8_t device_name[32];
    size_t device_name_len;
    uint16_t nus_rx_handle;
    uint16_t nus_tx_handle;
    uint16_t mesh_to_radio_handle;
    uint16_t mesh_from_radio_handle;
    uint16_t mesh_from_num_handle;
    FromRadioQueue from_radio_queue;
} BleState;

// BLE Manager Structure
typedef struct {
    bool initialized;
} BleManager;

// Global state variables
static BleState *g_ble_state = NULL;
static EventQueue g_event_queue = {0};
static atomic_bool g_advertising = ATOMIC_VAR_INIT(false);
static atomic_uint_least16_t g_connection_count = ATOMIC_VAR_INIT(0);

// Mutex for critical sections (platform-specific implementation)
// For demonstration, using a simple flag - in production, use proper mutex
static volatile bool g_state_lock = false;
static volatile bool g_event_lock = false;

// Critical section helpers (simplified - use proper mutex in production)
static inline void lock_state(void) {
    while (__atomic_test_and_set(&g_state_lock, __ATOMIC_ACQUIRE)) {
        // Spin wait
    }
}

static inline void unlock_state(void) {
    __atomic_clear(&g_state_lock, __ATOMIC_RELEASE);
}

static inline void lock_event(void) {
    while (__atomic_test_and_set(&g_event_lock, __ATOMIC_ACQUIRE)) {
        // Spin wait
    }
}

static inline void unlock_event(void) {
    __atomic_clear(&g_event_lock, __ATOMIC_RELEASE);
}

// TxQueue operations
static void tx_queue_init(TxQueue *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

static bool tx_queue_push_back(TxQueue *queue, const DataVec *item) {
    if (queue->count >= TX_QUEUE_DEPTH) {
        return false;
    }
    queue->items[queue->tail] = *item;
    queue->tail = (queue->tail + 1) % TX_QUEUE_DEPTH;
    queue->count++;
    return true;
}

static bool tx_queue_pop_front(TxQueue *queue, DataVec *item) {
    if (queue->count == 0) {
        return false;
    }
    *item = queue->items[queue->head];
    queue->head = (queue->head + 1) % TX_QUEUE_DEPTH;
    queue->count--;
    return true;
}

// FromRadioQueue operations
static void from_radio_queue_init(FromRadioQueue *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

static bool from_radio_queue_push_back(FromRadioQueue *queue, const DataVec *item) {
    if (queue->count >= FROM_RADIO_QUEUE_SIZE) {
        return false;
    }
    queue->items[queue->tail] = *item;
    queue->tail = (queue->tail + 1) % FROM_RADIO_QUEUE_SIZE;
    queue->count++;
    return true;
}

static bool from_radio_queue_pop_front(FromRadioQueue *queue, DataVec *item) {
    if (queue->count == 0) {
        return false;
    }
    *item = queue->items[queue->head];
    queue->head = (queue->head + 1) % FROM_RADIO_QUEUE_SIZE;
    queue->count--;
    return true;
}

static bool from_radio_queue_is_empty(const FromRadioQueue *queue) {
    return queue->count == 0;
}

// EventQueue operations
static void event_queue_init(EventQueue *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

static bool event_queue_push_back(EventQueue *queue, const BleEvent *item) {
    if (queue->count >= EVENT_QUEUE_SIZE) {
        return false;
    }
    queue->items[queue->tail] = *item;
    queue->tail = (queue->tail + 1) % EVENT_QUEUE_SIZE;
    queue->count++;
    return true;
}

static bool event_queue_pop_front(EventQueue *queue, BleEvent *item) {
    if (queue->count == 0) {
        return false;
    }
    *item = queue->items[queue->head];
    queue->head = (queue->head + 1) % EVENT_QUEUE_SIZE;
    queue->count--;
    return true;
}

// BleConnection functions
static void ble_connection_init(BleConnection *conn, uint16_t handle, const uint8_t peer_addr[6]) {
    conn->handle = handle;
    conn->state = CONNECTION_STATE_CONNECTED;
    conn->service = SERVICE_TYPE_UNKNOWN;
    conn->mtu = 23;
    tx_queue_init(&conn->tx_queue);
    conn->rx_buffer.len = 0;
    conn->nus_tx_notify = false;
    conn->mesh_from_radio_notify = false;
    conn->mesh_from_num_notify = false;
    conn->encrypted = false;
    memcpy(conn->peer_addr, peer_addr, 6);
    conn->conn_interval = 0;
    conn->conn_latency = 0;
    conn->supervision_timeout = 0;
    conn->valid = true;
}

static bool ble_connection_is_subscribed(const BleConnection *conn) {
    switch (conn->service) {
        case SERVICE_TYPE_NUS:
            return conn->nus_tx_notify;
        case SERVICE_TYPE_MESHTASTIC:
            return conn->mesh_from_radio_notify;
        case SERVICE_TYPE_UNKNOWN:
        default:
            return false;
    }
}

static size_t ble_connection_max_payload(const BleConnection *conn) {
    size_t mtu_size = (size_t)conn->mtu;
    return (mtu_size > 3) ? (mtu_size - 3) : 0;
}

static BleError ble_connection_queue_tx(BleConnection *conn, const uint8_t *data, size_t data_len) {
    if (data_len > ble_connection_max_payload(conn)) {
        return BLE_ERROR_MTU_EXCEEDED;
    }
    
    DataVec vec;
    if (data_len > MAX_MTU) {
        return BLE_ERROR_MTU_EXCEEDED;
    }
    memcpy(vec.data, data, data_len);
    vec.len = data_len;
    
    if (!tx_queue_push_back(&conn->tx_queue, &vec)) {
        return BLE_ERROR_QUEUE_FULL;
    }
    
    return BLE_ERROR_OK;
}

// BleState functions
static void ble_state_init(BleState *state, const char *name) {
    // Initialize connections array
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        state->connections[i].valid = false;
    }
    
    // Copy device name
    size_t name_len = strlen(name);
    if (name_len > 32) {
        name_len = 32;
    }
    memcpy(state->device_name, name, name_len);
    state->device_name_len = name_len;
    
    // Initialize handles
    state->nus_rx_handle = 0;
    state->nus_tx_handle = 0;
    state->mesh_to_radio_handle = 0;
    state->mesh_from_radio_handle = 0;
    state->mesh_from_num_handle = 0;
    
    // Initialize from_radio_queue
    from_radio_queue_init(&state->from_radio_queue);
}

static BleConnection *ble_state_find_connection(BleState *state, uint16_t handle) {
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        if (state->connections[i].valid && state->connections[i].handle == handle) {
            return &state->connections[i];
        }
    }
    return NULL;
}

static BleError ble_state_add_connection(BleState *state, uint16_t handle, const uint8_t peer_addr[6]) {
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        if (!state->connections[i].valid) {
            ble_connection_init(&state->connections[i], handle, peer_addr);
            atomic_fetch_add(&g_connection_count, 1);
            return BLE_ERROR_OK;
        }
    }
    return BLE_ERROR_MAX_CONNECTIONS;
}

static void ble_state_remove_connection(BleState *state, uint16_t handle) {
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        if (state->connections[i].valid && state->connections[i].handle == handle) {
            state->connections[i].valid = false;
            atomic_fetch_sub(&g_connection_count, 1);
            return;
        }
    }
}

// Platform-specific structures and functions (xtensa)
#ifdef __XTENSA__

// NimBLE structures (simplified - actual structures depend on NimBLE headers)
typedef struct {
    uint8_t conn_mode;
    uint8_t disc_mode;
    uint16_t itvl_min;
    uint16_t itvl_max;
    uint8_t channel_map;
    uint8_t filter_policy;
    uint8_t high_duty_cycle;
} ble_gap_adv_params;

typedef struct {
    uint16_t itvl_min;
    uint16_t itvl_max;
    uint16_t latency;
    uint16_t supervision_timeout;
    uint16_t min_ce_len;
    uint16_t max_ce_len;
} ble_gap_upd_params;

typedef struct ble_gap_event ble_gap_event;
typedef struct os_mbuf os_mbuf;

// BLE constants
#define BLE_GAP_CONN_MODE_UND 0
#define BLE_GAP_DISC_MODE_GEN 1
#define BLE_OWN_ADDR_PUBLIC 0
#define BLE_HS_FOREVER (-1)
#define BLE_HS_EALREADY 30
#define BLE_GAP_EVENT_CONNECT 0
#define BLE_GAP_EVENT_DISCONNECT 1
#define BLE_ERR_REM_USER_CONN_TERM 0x13

// GATT structures
#define CHR_FLAG_WRITE 0x08
#define CHR_FLAG_WRITE_NO_RSP 0x04
#define CHR_FLAG_NOTIFY 0x10
#define CHR_FLAG_READ 0x02

typedef struct {
    const uint8_t *uuid;
    uint16_t flags;
    int (*callback)(uint16_t conn_handle, uint16_t attr_handle, void *arg);
} GattCharacteristic;

typedef struct {
    const uint8_t *uuid;
    const GattCharacteristic *characteristics;
    size_t char_count;
} GattService;

// External NimBLE functions
extern int nimble_port_init(void);
extern int nimble_port_freertos_init(void (*task)(void *));
extern void nimble_host_task(void *param);
extern int ble_gatts_count_cfg(const GattService *svcs);
extern int ble_gatts_add_svcs(const GattService *svcs);
extern int ble_gap_adv_start(uint8_t own_addr_type, const uint8_t *direct_addr, 
                             int32_t duration_ms, const ble_gap_adv_params *params,
                             int (*cb)(ble_gap_event *, void *), void *arg);
extern int ble_gap_adv_stop(void);
extern int ble_gatts_notify_custom(uint16_t conn_handle, uint16_t chr_val_handle, os_mbuf *om);
extern os_mbuf *ble_hs_mbuf_att_pkt(void);
extern int os_mbuf_append(os_mbuf *om, const uint8_t *data, uint16_t len);
extern int os_mbuf_free_chain(os_mbuf *om);
extern int ble_gap_terminate(uint16_t conn_handle, uint8_t reason);
extern int ble_gap_update_params(uint16_t conn_handle, const ble_gap_upd_params *params);

// Global NimBLE configuration (simplified)
struct {
    void (*sync_cb)(void);
    void (*reset_cb)(int reason);
} ble_hs_cfg;

// Forward declarations
static int gap_event_callback(ble_gap_event *event, void *arg);
static int nus_rx_callback(uint16_t conn_handle, uint16_t attr_handle, void *arg);
static int mesh_to_radio_callback(uint16_t conn_handle, uint16_t attr_handle, void *arg);
static int mesh_from_radio_callback(uint16_t conn_handle, uint16_t attr_handle, void *arg);
static void on_sync(void);
static void on_reset(int reason);

#endif // __XTENSA__

// BleManager functions
static BleManager ble_manager_new(void) {
    BleManager manager;
    manager.initialized = false;
    return manager;
}

static BleError ble_manager_init(BleManager *manager, const char *device_name) {
    if (manager->initialized) {
        return BLE_ERROR_ALREADY_INITIALIZED;
    }
    
    // Initialize global state
    lock_state();
    if (g_ble_state == NULL) {
        g_ble_state = (BleState *)malloc(sizeof(BleState));
        if (g_ble_state == NULL) {
            unlock_state();
            return BLE_ERROR_STACK_ERROR;
        }
        ble_state_init(g_ble_state, device_name);
    }
    unlock_state();
    
    // Initialize event queue
    lock_event();
    event_queue_init(&g_event_queue);
    unlock_event();
    
#ifdef __XTENSA__
    // Initialize NimBLE
    int rc = nimble_port_init();
    if (rc != 0) {
        return BLE_ERROR_STACK_ERROR;
    }
    
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    
    rc = nimble_port_freertos_init(nimble_host_task);
    if (rc != 0) {
        return BLE_ERROR_STACK_ERROR;
    }
    
    // Register services
    static GattCharacteristic nus_chars[] = {
        {NUS_RX_UUID, CHR_FLAG_WRITE | CHR_FLAG_WRITE_NO_RSP, nus_rx_callback},
        {NUS_TX_UUID, CHR_FLAG_NOTIFY, NULL}
    };
    
    static GattCharacteristic mesh_chars[] = {
        {MESHTASTIC_TO_RADIO_UUID, CHR_FLAG_WRITE | CHR_FLAG_WRITE_NO_RSP, mesh_to_radio_callback},
        {MESHTASTIC_FROM_RADIO_UUID, CHR_FLAG_READ | CHR_FLAG_NOTIFY, mesh_from_radio_callback},
        {MESHTASTIC_FROM_NUM_UUID, CHR_FLAG_NOTIFY, NULL}
    };
    
    static GattService services[] = {
        {NUS_SERVICE_UUID, nus_chars, 2},
        {MESHTASTIC_SERVICE_UUID, mesh_chars, 3}
    };
    
    rc = ble_gatts_count_cfg(services);
    if (rc != 0) {
        return BLE_ERROR_STACK_ERROR;
    }
    
    rc = ble_gatts_add_svcs(services);
    if (rc != 0) {
        return BLE_ERROR_STACK_ERROR;
    }
#endif
    
    manager->initialized = true;
    return BLE_ERROR_OK;
}

static BleError ble_manager_start_advertising(BleManager *manager) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
#ifdef __XTENSA__
    ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 160,
        .itvl_max = 320,
        .channel_map = 0x07,
        .filter_policy = 0,
        .high_duty_cycle = 0
    };
    
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv_params, gap_event_callback, NULL);
    if (rc != 0) {
        return BLE_ERROR_STACK_ERROR;
    }
#endif
    
    atomic_store(&g_advertising, true);
    return BLE_ERROR_OK;
}

static BleError ble_manager_stop_advertising(BleManager *manager) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
#ifdef __XTENSA__
    int rc = ble_gap_adv_stop();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        return BLE_ERROR_STACK_ERROR;
    }
#endif
    
    atomic_store(&g_advertising, false);
    return BLE_ERROR_OK;
}

static bool ble_manager_is_advertising(const BleManager *manager) {
    return atomic_load(&g_advertising);
}

static uint16_t ble_manager_connection_count(const BleManager *manager) {
    return atomic_load(&g_connection_count);
}

#ifdef __XTENSA__
static BleError ble_manager_send_notification(const BleManager *manager, uint16_t conn_handle,
                                              uint16_t char_handle, const uint8_t *data, size_t data_len) {
    os_mbuf *om = ble_hs_mbuf_att_pkt();
    if (om == NULL) {
        return BLE_ERROR_STACK_ERROR;
    }
    
    int rc = os_mbuf_append(om, data, (uint16_t)data_len);
    if (rc != 0) {
        os_mbuf_free_chain(om);
        return BLE_ERROR_STACK_ERROR;
    }
    
    rc = ble_gatts_notify_custom(conn_handle, char_handle, om);
    if (rc != 0) {
        return BLE_ERROR_STACK_ERROR;
    }
    
    return BLE_ERROR_OK;
}
#endif

static BleError ble_manager_send(const BleManager *manager, uint16_t conn_handle,
                                 const uint8_t *data, size_t data_len) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    BleError result;
    uint16_t char_handle = 0;
    
    lock_state();
    if (g_ble_state == NULL) {
        unlock_state();
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    BleConnection *conn = ble_state_find_connection(g_ble_state, conn_handle);
    if (conn == NULL) {
        unlock_state();
        return BLE_ERROR_NOT_CONNECTED;
    }
    
    if (!ble_connection_is_subscribed(conn)) {
        unlock_state();
        return BLE_ERROR_NOT_SUBSCRIBED;
    }
    
    if (data_len > ble_connection_max_payload(conn)) {
        unlock_state();
        return BLE_ERROR_MTU_EXCEEDED;
    }
    
    result = ble_connection_queue_tx(conn, data, data_len);
    if (result != BLE_ERROR_OK) {
        unlock_state();
        return result;
    }
    
    switch (conn->service) {
        case SERVICE_TYPE_NUS:
            char_handle = g_ble_state->nus_tx_handle;
            break;
        case SERVICE_TYPE_MESHTASTIC:
            char_handle = g_ble_state->mesh_from_radio_handle;
            break;
        case SERVICE_TYPE_UNKNOWN:
        default:
            unlock_state();
            return BLE_ERROR_INVALID_HANDLE;
    }
    unlock_state();
    
#ifdef __XTENSA__
    result = ble_manager_send_notification(manager, conn_handle, char_handle, data, data_len);
    if (result != BLE_ERROR_OK) {
        return result;
    }
#endif
    
    return BLE_ERROR_OK;
}

static BleError ble_manager_broadcast(const BleManager *manager, ServiceType service,
                                      const uint8_t *data, size_t data_len, size_t *sent_count) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    size_t sent = 0;
    
    lock_state();
    if (g_ble_state == NULL) {
        unlock_state();
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        BleConnection *conn = &g_ble_state->connections[i];
        if (conn->valid && conn->service == service && ble_connection_is_subscribed(conn)) {
            if (data_len <= ble_connection_max_payload(conn)) {
                if (ble_connection_queue_tx(conn, data, data_len) == BLE_ERROR_OK) {
                    sent++;
                }
            }
        }
    }
    unlock_state();
    
    if (sent_count != NULL) {
        *sent_count = sent;
    }
    
    return BLE_ERROR_OK;
}

static BleError ble_manager_notify_from_num(const BleManager *manager, uint32_t counter, size_t *sent_count) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    // Convert counter to little-endian bytes
    uint8_t counter_bytes[4];
    counter_bytes[0] = (uint8_t)(counter & 0xFF);
    counter_bytes[1] = (uint8_t)((counter >> 8) & 0xFF);
    counter_bytes[2] = (uint8_t)((counter >> 16) & 0xFF);
    counter_bytes[3] = (uint8_t)((counter >> 24) & 0xFF);
    
    size_t sent = 0;
    
    lock_state();
    if (g_ble_state == NULL) {
        unlock_state();
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    uint16_t from_num_handle = g_ble_state->mesh_from_num_handle;
    
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        BleConnection *conn = &g_ble_state->connections[i];
        if (conn->valid && conn->mesh_from_num_notify) {
#ifdef __XTENSA__
            if (ble_manager_send_notification(manager, conn->handle, from_num_handle,
                                            counter_bytes, 4) == BLE_ERROR_OK) {
                sent++;
            }
#else
            sent++;
#endif
        }
    }
    unlock_state();
    
    if (sent_count != NULL) {
        *sent_count = sent;
    }
    
    return BLE_ERROR_OK;
}

static BleError ble_manager_queue_from_radio(const BleManager *manager, const uint8_t *data, size_t data_len) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    if (data_len > MAX_MTU) {
        return BLE_ERROR_MTU_EXCEEDED;
    }
    
    lock_state();
    if (g_ble_state == NULL) {
        unlock_state();
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    DataVec vec;
    memcpy(vec.data, data, data_len);
    vec.len = data_len;
    
    bool success = from_radio_queue_push_back(&g_ble_state->from_radio_queue, &vec);
    unlock_state();
    
    return success ? BLE_ERROR_OK : BLE_ERROR_QUEUE_FULL;
}

static bool ble_manager_dequeue_from_radio(const BleManager *manager, DataVec *out_data) {
    if (!manager->initialized) {
        return false;
    }
    
    lock_state();
    if (g_ble_state == NULL) {
        unlock_state();
        return false;
    }
    
    bool success = from_radio_queue_pop_front(&g_ble_state->from_radio_queue, out_data);
    unlock_state();
    
    return success;
}

static bool ble_manager_has_from_radio_data(const BleManager *manager) {
    if (!manager->initialized) {
        return false;
    }
    
    lock_state();
    if (g_ble_state == NULL) {
        unlock_state();
        return false;
    }
    
    bool has_data = !from_radio_queue_is_empty(&g_ble_state->from_radio_queue);
    unlock_state();
    
    return has_data;
}

static bool ble_manager_poll_event(const BleManager *manager, BleEvent *event) {
    lock_event();
    bool success = event_queue_pop_front(&g_event_queue, event);
    unlock_event();
    
    return success;
}

static bool ble_manager_read(const BleManager *manager, RxDataVec *data) {
    BleDataPacket packet;
    if (ble_manager_read_with_service(manager, &packet)) {
        *data = packet.data;
        return true;
    }
    return false;
}

static bool ble_manager_read_with_service(const BleManager *manager, BleDataPacket *packet) {
    if (!manager->initialized) {
        return false;
    }
    
    lock_event();
    
    // Find first data received event
    size_t found_idx = EVENT_QUEUE_SIZE;
    for (size_t i = 0; i < g_event_queue.count; i++) {
        size_t idx = (g_event_queue.head + i) % EVENT_QUEUE_SIZE;
        if (g_event_queue.items[idx].type == BLE_EVENT_DATA_RECEIVED) {
            found_idx = idx;
            break;
        }
    }
    
    if (found_idx == EVENT_QUEUE_SIZE) {
        unlock_event();
        return false;
    }
    
    // Extract the data received event
    BleEvent data_event = g_event_queue.items[found_idx];
    
    // Remove it from the queue by shifting
    EventQueue temp;
    event_queue_init(&temp);
    
    bool found = false;
    BleEvent ev;
    while (event_queue_pop_front(&g_event_queue, &ev)) {
        if (!found && ev.type == BLE_EVENT_DATA_RECEIVED) {
            found = true;
            continue; // Skip this event
        }
        event_queue_push_back(&temp, &ev);
    }
    
    // Copy back
    while (event_queue_pop_front(&temp, &ev)) {
        event_queue_push_back(&g_event_queue, &ev);
    }
    
    unlock_event();
    
    // Fill output packet
    packet->conn_handle = data_event.data.data_received.conn_handle;
    packet->service = data_event.data.data_received.service;
    packet->data = data_event.data.data_received.data;
    
    return true;
}

static BleError ble_manager_process_tx(const BleManager *manager) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    lock_state();
    if (g_ble_state == NULL) {
        unlock_state();
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
    for (size_t i = 0; i < MAX_CONNECTIONS; i++) {
        BleConnection *conn = &g_ble_state->connections[i];
        if (conn->valid && ble_connection_is_subscribed(conn)) {
            DataVec data;
            while (tx_queue_pop_front(&conn->tx_queue, &data)) {
                uint16_t char_handle;
                switch (conn->service) {
                    case SERVICE_TYPE_NUS:
                        char_handle = g_ble_state->nus_tx_handle;
                        break;
                    case SERVICE_TYPE_MESHTASTIC:
                        char_handle = g_ble_state->mesh_from_radio_handle;
                        break;
                    case SERVICE_TYPE_UNKNOWN:
                    default:
                        continue;
                }
                
#ifdef __XTENSA__
                // Send notification (ignore errors)
                ble_manager_send_notification(manager, conn->handle, char_handle, data.data, data.len);
#endif
            }
        }
    }
    unlock_state();
    
    return BLE_ERROR_OK;
}

static BleError ble_manager_disconnect(const BleManager *manager, uint16_t conn_handle) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
#ifdef __XTENSA__
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0) {
        return BLE_ERROR_STACK_ERROR;
    }
#endif
    
    return BLE_ERROR_OK;
}

static BleError ble_manager_update_conn_params(const BleManager *manager, uint16_t conn_handle,
                                               uint16_t min_interval, uint16_t max_interval,
                                               uint16_t latency, uint16_t timeout) {
    if (!manager->initialized) {
        return BLE_ERROR_NOT_INITIALIZED;
    }
    
#ifdef __XTENSA__
    ble_gap_upd_params params = {
        .itvl_min = min_interval,
        .itvl_max = max_interval,
        .latency = latency,
        .supervision_timeout = timeout,
        .min_ce_len = 0,
        .max_ce_len = 0
    };
    
    int rc = ble_gap_update_params(conn_handle, &params);
    if (rc != 0) {
        return BLE_ERROR_STACK_ERROR;
    }
#endif
    
    return BLE_ERROR_OK;
}

// Platform-specific callback implementations
#ifdef __XTENSA__

static void on_sync(void) {
    // BLE stack synchronized
}

static void on_reset(int reason) {
    // BLE stack reset
}

static int gap_event_callback(ble_gap_event *event, void *arg) {
    // This is a simplified version - actual implementation depends on ble_gap_event structure
    // The original Rust code was incomplete, so this provides the framework
    
    // Handle connection event
    // Handle disconnection event
    // etc.
    
    return 0;
}

static int nus_rx_callback(uint16_t conn_handle, uint16_t attr_handle, void *arg) {
    // Handle NUS RX characteristic write
    return 0;
}

static int mesh_to_radio_callback(uint16_t conn_handle, uint16_t attr_handle, void *arg) {
    // Handle Meshtastic TO_RADIO characteristic write
    return 0;
}

static int mesh_from_radio_callback(uint16_t conn_handle, uint16_t attr_handle, void *arg) {
    // Handle Meshtastic FROM_RADIO characteristic read
    return 0;
}

#endif // __XTENSA__
