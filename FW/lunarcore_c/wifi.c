#include "wifi.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Forward declarations for crypto functions (assuming these are implemented elsewhere) */
typedef struct ChaCha20 ChaCha20;
typedef struct Sha256 Sha256;

ChaCha20* chacha20_new(const uint8_t key[32], const uint8_t nonce[12]);
void chacha20_encrypt(ChaCha20* cipher, uint8_t* data, size_t len);
void chacha20_free(ChaCha20* cipher);

Sha256* sha256_new(void);
void sha256_update(Sha256* hasher, const uint8_t* data, size_t len);
void sha256_finalize(Sha256* hasher, uint8_t output[32]);
void sha256_free(Sha256* hasher);

void secure_zero(uint8_t* data, size_t len);

/* Constants */
#define DEFAULT_TCP_PORT 4000
#define MAX_TCP_CLIENTS 4
#define TCP_RX_BUFFER_SIZE 512
#define TCP_TX_BUFFER_SIZE 512
#define WIFI_CONNECT_TIMEOUT_SEC 30
#define DEFAULT_AP_SSID "LunarCore"
#define DEFAULT_AP_PASSWORD "lunarpunk"
#define MAX_SSID_LEN 32
#define MAX_PASSWORD_LEN 64
#define AUTH_CHALLENGE_SIZE 32
#define AUTH_RESPONSE_SIZE 32
#define SESSION_KEY_SIZE 32
#define SESSION_NONCE_SIZE 12
#define AUTH_TIMEOUT_SEC 10
#define MAX_AUTH_FAILURES 3
#define AUTH_LOCKOUT_SEC 300
#define CONN_RATE_LIMIT 10

/* Enums */
typedef enum {
    WIFI_MODE_OFF,
    WIFI_MODE_AP,
    WIFI_MODE_STA,
    WIFI_MODE_APSTA
} WifiMode;

typedef enum {
    AUTH_STATE_NONE,
    AUTH_STATE_CHALLENGE_SENT,
    AUTH_STATE_AUTHENTICATED,
    AUTH_STATE_FAILED
} AuthState;

typedef enum {
    TCP_CLIENT_STATE_DISCONNECTED,
    TCP_CLIENT_STATE_AUTHENTICATING,
    TCP_CLIENT_STATE_DETECTING,
    TCP_CLIENT_STATE_CONNECTED
} TcpClientState;

typedef enum {
    WIFI_ERROR_NOT_INITIALIZED,
    WIFI_ERROR_ALREADY_INITIALIZED,
    WIFI_ERROR_CONFIG_ERROR,
    WIFI_ERROR_CONNECTION_FAILED,
    WIFI_ERROR_TIMEOUT,
    WIFI_ERROR_SOCKET_ERROR,
    WIFI_ERROR_NO_CLIENT_SLOTS,
    WIFI_ERROR_BUFFER_FULL,
    WIFI_ERROR_AUTHENTICATION_FAILED,
    WIFI_ERROR_NO_PSK_CONFIGURED,
    WIFI_ERROR_NOT_AUTHENTICATED,
    WIFI_ERROR_RATE_LIMITED
} WifiError;

/* Struct definitions */
typedef struct {
    char data[MAX_SSID_LEN];
    size_t len;
} String32;

typedef struct {
    char data[MAX_PASSWORD_LEN];
    size_t len;
} String64;

typedef struct {
    char data[32];
    size_t len;
} String32B;

typedef struct {
    uint8_t data[TCP_RX_BUFFER_SIZE];
    size_t len;
    size_t capacity;
} VecRx;

typedef struct {
    uint8_t data[TCP_TX_BUFFER_SIZE];
    size_t len;
    size_t capacity;
} VecTx;

typedef struct {
    bool require_auth;
    uint8_t psk[32];
    bool encrypt_traffic;
    bool rate_limiting;
    bool allow_external;
} SecurityConfig;

typedef struct {
    WifiMode mode;
    String32 ap_ssid;
    String64 ap_password;
    uint8_t ap_channel;
    String32 sta_ssid;
    String64 sta_password;
    uint16_t tcp_port;
    bool mdns_enabled;
    String32B mdns_hostname;
    SecurityConfig security;
} WifiConfig;

typedef struct {
    uint8_t octets[4];
} Ipv4Addr;

typedef struct {
    Ipv4Addr ip;
    uint16_t port;
} SocketAddrV4;

typedef struct {
    WifiMode mode;
    bool sta_connected;
    bool sta_ip_set;
    Ipv4Addr sta_ip;
    bool ap_active;
    bool ap_ip_set;
    Ipv4Addr ap_ip;
    uint8_t ap_client_count;
    uint8_t tcp_client_count;
    int8_t rssi;
} NetworkStatus;

typedef struct {
    uint8_t key[SESSION_KEY_SIZE];
    uint8_t nonce[SESSION_NONCE_SIZE];
    uint32_t counter;
    bool enabled;
} SessionCrypto;

typedef struct {
    int32_t fd;
    SocketAddrV4 addr;
    TcpClientState state;
    AuthState auth_state;
    uint8_t auth_challenge[AUTH_CHALLENGE_SIZE];
    uint8_t auth_failures;
    uint32_t auth_started;
    SessionCrypto session;
    uint8_t protocol;
    VecRx rx_buffer;
    VecTx tx_buffer;
    uint32_t last_activity;
} TcpClient;

typedef struct {
    WifiConfig config;
    NetworkStatus status;
    int32_t server_fd;
    TcpClient clients[MAX_TCP_CLIENTS];
    bool initialized;
} WifiManager;

typedef struct {
    String32 ssid;
    int8_t rssi;
    uint8_t channel;
    wifi_auth_mode_t auth_mode;
} NetworkInfo;

typedef struct {
    NetworkInfo data[16];
    size_t len;
    size_t capacity;
} NetworkInfoVec;

/* Helper function implementations */

/* String helper functions */
static void string32_init(String32* s) {
    memset(s->data, 0, MAX_SSID_LEN);
    s->len = 0;
}

static void string32_push_str(String32* s, const char* str) {
    size_t str_len = strlen(str);
    size_t copy_len = (str_len < MAX_SSID_LEN - s->len) ? str_len : (MAX_SSID_LEN - s->len);
    memcpy(s->data + s->len, str, copy_len);
    s->len += copy_len;
}

static const uint8_t* string32_as_bytes(const String32* s) {
    return (const uint8_t*)s->data;
}

static void string64_init(String64* s) {
    memset(s->data, 0, MAX_PASSWORD_LEN);
    s->len = 0;
}

static void string64_push_str(String64* s, const char* str) {
    size_t str_len = strlen(str);
    size_t copy_len = (str_len < MAX_PASSWORD_LEN - s->len) ? str_len : (MAX_PASSWORD_LEN - s->len);
    memcpy(s->data + s->len, str, copy_len);
    s->len += copy_len;
}

static const uint8_t* string64_as_bytes(const String64* s) {
    return (const uint8_t*)s->data;
}

static void string32b_init(String32B* s) {
    memset(s->data, 0, 32);
    s->len = 0;
}

static void string32b_push_str(String32B* s, const char* str) {
    size_t str_len = strlen(str);
    size_t copy_len = (str_len < 32 - s->len) ? str_len : (32 - s->len);
    memcpy(s->data + s->len, str, copy_len);
    s->len += copy_len;
}

/* Vec helper functions */
static void vec_rx_init(VecRx* v) {
    memset(v->data, 0, TCP_RX_BUFFER_SIZE);
    v->len = 0;
    v->capacity = TCP_RX_BUFFER_SIZE;
}

static bool vec_rx_push(VecRx* v, uint8_t byte) {
    if (v->len >= v->capacity) {
        return false;
    }
    v->data[v->len++] = byte;
    return true;
}

static void vec_rx_clear(VecRx* v) {
    v->len = 0;
}

static void vec_tx_init(VecTx* v) {
    memset(v->data, 0, TCP_TX_BUFFER_SIZE);
    v->len = 0;
    v->capacity = TCP_TX_BUFFER_SIZE;
}

static void vec_tx_clear(VecTx* v) {
    v->len = 0;
}

/* Ipv4Addr functions */
static Ipv4Addr ipv4_addr_new(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    Ipv4Addr addr;
    addr.octets[0] = a;
    addr.octets[1] = b;
    addr.octets[2] = c;
    addr.octets[3] = d;
    return addr;
}

/* SocketAddrV4 functions */
static SocketAddrV4 socket_addr_v4_new(Ipv4Addr ip, uint16_t port) {
    SocketAddrV4 addr;
    addr.ip = ip;
    addr.port = port;
    return addr;
}

/* SecurityConfig functions */
static void security_config_drop(SecurityConfig* config) {
    /* Securely zero the PSK */
    secure_zero(config->psk, 32);
}

static SecurityConfig security_config_default(void) {
    SecurityConfig config;
    config.require_auth = true;
    memset(config.psk, 0, 32);
    config.encrypt_traffic = true;
    config.rate_limiting = true;
    config.allow_external = false;
    return config;
}

static SecurityConfig security_config_with_psk(const uint8_t psk[32]) {
    SecurityConfig config;
    config.require_auth = true;
    memcpy(config.psk, psk, 32);
    config.encrypt_traffic = true;
    config.rate_limiting = true;
    config.allow_external = false;
    return config;
}

static SecurityConfig security_config_insecure(void) {
    SecurityConfig config;
    config.require_auth = false;
    memset(config.psk, 0, 32);
    config.encrypt_traffic = false;
    config.rate_limiting = false;
    config.allow_external = true;
    return config;
}

static bool security_config_is_psk_set(const SecurityConfig* config) {
    for (int i = 0; i < 32; i++) {
        if (config->psk[i] != 0) {
            return true;
        }
    }
    return false;
}

/* WifiConfig functions */
static WifiConfig wifi_config_default(void) {
    WifiConfig config;
    config.mode = WIFI_MODE_OFF;
    
    string32_init(&config.ap_ssid);
    string32_push_str(&config.ap_ssid, DEFAULT_AP_SSID);
    
    string64_init(&config.ap_password);
    string64_push_str(&config.ap_password, DEFAULT_AP_PASSWORD);
    
    config.ap_channel = 1;
    
    string32_init(&config.sta_ssid);
    string64_init(&config.sta_password);
    
    config.tcp_port = DEFAULT_TCP_PORT;
    config.mdns_enabled = true;
    
    string32b_init(&config.mdns_hostname);
    string32b_push_str(&config.mdns_hostname, "lunarcore");
    
    config.security = security_config_default();
    
    return config;
}

/* NetworkStatus functions */
static NetworkStatus network_status_default(void) {
    NetworkStatus status;
    status.mode = WIFI_MODE_OFF;
    status.sta_connected = false;
    status.sta_ip_set = false;
    status.sta_ip = ipv4_addr_new(0, 0, 0, 0);
    status.ap_active = false;
    status.ap_ip_set = false;
    status.ap_ip = ipv4_addr_new(0, 0, 0, 0);
    status.ap_client_count = 0;
    status.tcp_client_count = 0;
    status.rssi = 0;
    return status;
}

/* HMAC-SHA256 implementation */
static void hmac_sha256(const uint8_t key[32], const uint8_t* message, size_t message_len, uint8_t output[32]) {
    const uint8_t IPAD = 0x36;
    const uint8_t OPAD = 0x5c;
    const size_t BLOCK_SIZE = 64;
    
    uint8_t k_pad[BLOCK_SIZE];
    memset(k_pad, 0, BLOCK_SIZE);
    memcpy(k_pad, key, 32);
    
    /* Inner hash */
    Sha256* inner_hasher = sha256_new();
    uint8_t inner_key[BLOCK_SIZE];
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
        inner_key[i] = k_pad[i] ^ IPAD;
    }
    sha256_update(inner_hasher, inner_key, BLOCK_SIZE);
    sha256_update(inner_hasher, message, message_len);
    uint8_t inner_hash[32];
    sha256_finalize(inner_hasher, inner_hash);
    sha256_free(inner_hasher);
    
    /* Outer hash */
    Sha256* outer_hasher = sha256_new();
    uint8_t outer_key[BLOCK_SIZE];
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
        outer_key[i] = k_pad[i] ^ OPAD;
    }
    sha256_update(outer_hasher, outer_key, BLOCK_SIZE);
    sha256_update(outer_hasher, inner_hash, 32);
    sha256_finalize(outer_hasher, output);
    sha256_free(outer_hasher);
}

/* Constant-time comparison for 32-byte arrays */
__attribute__((noinline))
static bool ct_eq_32(const uint8_t a[32], const uint8_t b[32]) {
    uint8_t diff = 0;
    for (int i = 0; i < 32; i++) {
        diff |= a[i] ^ b[i];
    }
    return diff == 0;
}

/* SessionCrypto functions */
static SessionCrypto session_crypto_new(void) {
    SessionCrypto session;
    memset(session.key, 0, SESSION_KEY_SIZE);
    memset(session.nonce, 0, SESSION_NONCE_SIZE);
    session.counter = 0;
    session.enabled = false;
    return session;
}

static void session_crypto_init(SessionCrypto* session, const uint8_t key[SESSION_KEY_SIZE], const uint8_t nonce[SESSION_NONCE_SIZE]) {
    memcpy(session->key, key, SESSION_KEY_SIZE);
    memcpy(session->nonce, nonce, SESSION_NONCE_SIZE);
    session->counter = 0;
    session->enabled = true;
}

static void session_crypto_encrypt(SessionCrypto* session, uint8_t* data, size_t len) {
    if (!session->enabled) {
        return;
    }
    
    /* Update nonce with counter */
    uint8_t counter_bytes[4];
    counter_bytes[0] = (session->counter) & 0xFF;
    counter_bytes[1] = (session->counter >> 8) & 0xFF;
    counter_bytes[2] = (session->counter >> 16) & 0xFF;
    counter_bytes[3] = (session->counter >> 24) & 0xFF;
    memcpy(session->nonce, counter_bytes, 4);
    
    /* Encrypt */
    ChaCha20* cipher = chacha20_new(session->key, session->nonce);
    chacha20_encrypt(cipher, data, len);
    chacha20_free(cipher);
    
    /* Increment counter */
    session->counter = session->counter + 1; /* wrapping add */
}

static void session_crypto_decrypt(SessionCrypto* session, uint8_t* data, size_t len) {
    /* ChaCha20 encryption and decryption are the same operation */
    session_crypto_encrypt(session, data, len);
}

static void session_crypto_clear(SessionCrypto* session) {
    /* Securely zero the key and nonce using volatile writes */
    for (size_t i = 0; i < SESSION_KEY_SIZE; i++) {
        volatile uint8_t* ptr = &session->key[i];
        *ptr = 0;
    }
    for (size_t i = 0; i < SESSION_NONCE_SIZE; i++) {
        volatile uint8_t* ptr = &session->nonce[i];
        *ptr = 0;
    }
    session->counter = 0;
    session->enabled = false;
}

static void session_crypto_drop(SessionCrypto* session) {
    /* Clean up when dropping */
    session_crypto_clear(session);
}

/* TcpClient functions */
static TcpClient tcp_client_empty(void) {
    TcpClient client;
    client.fd = -1;
    client.addr = socket_addr_v4_new(ipv4_addr_new(0, 0, 0, 0), 0);
    client.state = TCP_CLIENT_STATE_DISCONNECTED;
    client.auth_state = AUTH_STATE_NONE;
    memset(client.auth_challenge, 0, AUTH_CHALLENGE_SIZE);
    client.auth_failures = 0;
    client.auth_started = 0;
    client.session = session_crypto_new();
    client.protocol = 0;
    vec_rx_init(&client.rx_buffer);
    vec_tx_init(&client.tx_buffer);
    client.last_activity = 0;
    return client;
}

static bool tcp_client_is_available(const TcpClient* client) {
    return client->state == TCP_CLIENT_STATE_DISCONNECTED;
}

static bool tcp_client_is_authenticated(const TcpClient* client) {
    return client->auth_state == AUTH_STATE_AUTHENTICATED || client->auth_state == AUTH_STATE_NONE;
}

static void tcp_client_reset(TcpClient* client) {
    client->fd = -1;
    client->state = TCP_CLIENT_STATE_DISCONNECTED;
    client->auth_state = AUTH_STATE_NONE;
    memset(client->auth_challenge, 0, AUTH_CHALLENGE_SIZE);
    client->auth_failures = 0;
    client->auth_started = 0;
    session_crypto_clear(&client->session);
    client->protocol = 0;
    vec_rx_clear(&client->rx_buffer);
    vec_tx_clear(&client->tx_buffer);
}

/* WifiManager functions */
static WifiManager wifi_manager_new(void) {
    WifiManager manager;
    manager.config = wifi_config_default();
    manager.status = network_status_default();
    manager.server_fd = -1;
    
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        manager.clients[i] = tcp_client_empty();
    }
    
    manager.initialized = false;
    return manager;
}

static void wifi_manager_set_psk(WifiManager* manager, const uint8_t psk[32]) {
    memcpy(manager->config.security.psk, psk, 32);
}

static bool wifi_manager_is_auth_configured(const WifiManager* manager) {
    return !manager->config.security.require_auth || security_config_is_psk_set(&manager->config.security);
}

static int wifi_manager_init(WifiManager* manager, WifiConfig config) {
    if (manager->initialized) {
        return WIFI_ERROR_ALREADY_INITIALIZED;
    }
    
    manager->config = config;
    
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    /* Initialize TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    
    /* Create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    /* Create network interfaces based on mode */
    switch (manager->config.mode) {
        case WIFI_MODE_OFF:
            /* No interfaces needed */
            break;
        case WIFI_MODE_AP:
            esp_netif_create_default_wifi_ap();
            break;
        case WIFI_MODE_STA:
            esp_netif_create_default_wifi_sta();
            break;
        case WIFI_MODE_APSTA:
            esp_netif_create_default_wifi_ap();
            esp_netif_create_default_wifi_sta();
            break;
    }
    
    /* Initialize WiFi */
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    
    /* Set WiFi storage to RAM */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    manager->initialized = true;
    return 0; /* Success */
}

static int wifi_manager_configure_ap(const WifiManager* manager) {
    wifi_config_t ap_config = {0};
    
    /* Copy SSID */
    size_t ssid_len = manager->config.ap_ssid.len < 32 ? manager->config.ap_ssid.len : 32;
    memcpy(ap_config.ap.ssid, manager->config.ap_ssid.data, ssid_len);
    ap_config.ap.ssid_len = ssid_len;
    
    /* Copy password */
    size_t pass_len = manager->config.ap_password.len < 64 ? manager->config.ap_password.len : 64;
    memcpy(ap_config.ap.password, manager->config.ap_password.data, pass_len);
    
    /* Set channel and max connections */
    ap_config.ap.channel = manager->config.ap_channel;
    ap_config.ap.max_connection = MAX_TCP_CLIENTS;
    
    /* Set authentication mode */
    if (pass_len > 0) {
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    /* Apply configuration */
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (ret != ESP_OK) {
        return WIFI_ERROR_CONFIG_ERROR;
    }
    
    return 0; /* Success */
}

static int wifi_manager_configure_sta(const WifiManager* manager) {
    wifi_config_t sta_config = {0};
    
    /* Copy SSID */
    size_t ssid_len = manager->config.sta_ssid.len < 32 ? manager->config.sta_ssid.len : 32;
    memcpy(sta_config.sta.ssid, manager->config.sta_ssid.data, ssid_len);
    
    /* Copy password */
    size_t pass_len = manager->config.sta_password.len < 64 ? manager->config.sta_password.len : 64;
    memcpy(sta_config.sta.password, manager->config.sta_password.data, pass_len);
    
    /* Apply configuration */
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret != ESP_OK) {
        return WIFI_ERROR_CONFIG_ERROR;
    }
    
    return 0; /* Success */
}

static int wifi_manager_start(WifiManager* manager) {
    if (!manager->initialized) {
        return WIFI_ERROR_NOT_INITIALIZED;
    }
    
    switch (manager->config.mode) {
        case WIFI_MODE_OFF:
            esp_wifi_stop();
            break;
            
        case WIFI_MODE_AP: {
            int ret = wifi_manager_configure_ap(manager);
            if (ret != 0) return ret;
            
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
            ESP_ERROR_CHECK(esp_wifi_start());
            
            manager->status.ap_active = true;
            manager->status.ap_ip_set = true;
            manager->status.ap_ip = ipv4_addr_new(192, 168, 4, 1);
            break;
        }
        
        case WIFI_MODE_STA: {
            int ret = wifi_manager_configure_sta(manager);
            if (ret != 0) return ret;
            
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_start());
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        }
        
        case WIFI_MODE_APSTA: {
            int ret_ap = wifi_manager_configure_ap(manager);
            if (ret_ap != 0) return ret_ap;
            
            int ret_sta = wifi_manager_configure_sta(manager);
            if (ret_sta != 0) return ret_sta;
            
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
            ESP_ERROR_CHECK(esp_wifi_start());
            ESP_ERROR_CHECK(esp_wifi_connect());
            
            manager->status.ap_active = true;
            manager->status.ap_ip_set = true;
            manager->status.ap_ip = ipv4_addr_new(192, 168, 4, 1);
            break;
        }
    }
    
    manager->status.mode = manager->config.mode;
    return 0; /* Success */
}

static int wifi_manager_start_tcp_server(WifiManager* manager) {
    if (!manager->initialized) {
        return WIFI_ERROR_NOT_INITIALIZED;
    }
    
    /* Create socket */
    int fd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        return WIFI_ERROR_SOCKET_ERROR;
    }
    
    /* Set socket options */
    int opt = 1;
    lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    /* Bind socket */
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(manager->config.tcp_port);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    int ret = lwip_bind(fd, (struct sockaddr*)&addr, sizeof(addr));
    if (ret < 0) {
        lwip_close(fd);
        return WIFI_ERROR_SOCKET_ERROR;
    }
    
    /* Listen */
    ret = lwip_listen(fd, MAX_TCP_CLIENTS);
    if (ret < 0) {
        lwip_close(fd);
        return WIFI_ERROR_SOCKET_ERROR;
    }
    
    /* Set non-blocking */
    int flags = lwip_fcntl(fd, F_GETFL, 0);
    lwip_fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    
    manager->server_fd = fd;
    return 0; /* Success */
}

static void wifi_manager_stop_tcp_server(WifiManager* manager) {
    if (manager->server_fd >= 0) {
        lwip_close(manager->server_fd);
        manager->server_fd = -1;
    }
    
    /* Close all client connections */
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (manager->clients[i].fd >= 0) {
            lwip_close(manager->clients[i].fd);
            tcp_client_reset(&manager->clients[i]);
        }
    }
}

static void wifi_manager_accept_connections(WifiManager* manager) {
    struct sockaddr_in client_addr = {0};
    socklen_t addr_len = sizeof(client_addr);
    
    int client_fd = lwip_accept(manager->server_fd, (struct sockaddr*)&client_addr, &addr_len);
    
    if (client_fd >= 0) {
        /* Find available client slot */
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (tcp_client_is_available(&manager->clients[i])) {
                /* Set non-blocking */
                int flags = lwip_fcntl(client_fd, F_GETFL, 0);
                lwip_fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
                
                /* Store client info */
                manager->clients[i].fd = client_fd;
                manager->clients[i].state = TCP_CLIENT_STATE_DETECTING;
                
                /* Extract IP and port */
                uint32_t ip_addr = client_addr.sin_addr.s_addr;
                uint8_t* ip_bytes = (uint8_t*)&ip_addr;
                Ipv4Addr ip = ipv4_addr_new(ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);
                uint16_t port = ntohs(client_addr.sin_port);
                manager->clients[i].addr = socket_addr_v4_new(ip, port);
                
                manager->status.tcp_client_count++;
                return;
            }
        }
        
        /* No available slots, close connection */
        lwip_close(client_fd);
    }
}

static bool wifi_manager_read_client(WifiManager* manager, size_t idx, VecRx* out_data) {
    TcpClient* client = &manager->clients[idx];
    if (client->fd < 0) {
        return false;
    }
    
    uint8_t buf[TCP_RX_BUFFER_SIZE];
    ssize_t n = lwip_recv(client->fd, buf, TCP_RX_BUFFER_SIZE, 0);
    
    if (n > 0) {
        vec_rx_init(out_data);
        for (ssize_t i = 0; i < n; i++) {
            vec_rx_push(out_data, buf[i]);
        }
        return true;
    } else if (n == 0) {
        /* Connection closed */
        wifi_manager_disconnect_client(manager, idx);
    }
    /* n < 0 means no data available (non-blocking) or error */
    
    return false;
}

static bool wifi_manager_poll(WifiManager* manager, size_t* out_idx, VecRx* out_data) {
    if (manager->server_fd < 0) {
        return false;
    }
    
    /* Accept new connections */
    wifi_manager_accept_connections(manager);
    
    /* Check all clients for data */
    for (size_t i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (manager->clients[i].fd >= 0) {
            if (wifi_manager_read_client(manager, i, out_data)) {
                *out_idx = i;
                return true;
            }
        }
    }
    
    return false;
}

static int wifi_manager_send_to_client(WifiManager* manager, size_t idx, const uint8_t* data, size_t len, size_t* out_sent) {
    if (idx >= MAX_TCP_CLIENTS) {
        return WIFI_ERROR_SOCKET_ERROR;
    }
    
    TcpClient* client = &manager->clients[idx];
    if (client->fd < 0) {
        return WIFI_ERROR_SOCKET_ERROR;
    }
    
    ssize_t n = lwip_send(client->fd, data, len, 0);
    
    if (n < 0) {
        wifi_manager_disconnect_client(manager, idx);
        return WIFI_ERROR_SOCKET_ERROR;
    }
    
    *out_sent = (size_t)n;
    return 0; /* Success */
}

static void wifi_manager_broadcast(WifiManager* manager, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (manager->clients[i].fd >= 0) {
            size_t sent;
            wifi_manager_send_to_client(manager, i, data, len, &sent);
        }
    }
}

static void wifi_manager_disconnect_client(WifiManager* manager, size_t idx) {
    if (idx >= MAX_TCP_CLIENTS) {
        return;
    }
    
    TcpClient* client = &manager->clients[idx];
    if (client->fd >= 0) {
        lwip_close(client->fd);
        tcp_client_reset(client);
        if (manager->status.tcp_client_count > 0) {
            manager->status.tcp_client_count--;
        }
    }
}

static const TcpClient* wifi_manager_get_client(const WifiManager* manager, size_t idx) {
    if (idx < MAX_TCP_CLIENTS && manager->clients[idx].fd >= 0) {
        return &manager->clients[idx];
    }
    return NULL;
}

static TcpClient* wifi_manager_get_client_mut(WifiManager* manager, size_t idx) {
    if (idx < MAX_TCP_CLIENTS && manager->clients[idx].fd >= 0) {
        return &manager->clients[idx];
    }
    return NULL;
}

static void wifi_manager_stop(WifiManager* manager) {
    wifi_manager_stop_tcp_server(manager);
    
    esp_wifi_stop();
    
    manager->status.mode = WIFI_MODE_OFF;
    manager->status.sta_connected = false;
    manager->status.ap_active = false;
}

static const NetworkStatus* wifi_manager_status(const WifiManager* manager) {
    return &manager->status;
}

static void wifi_manager_update_status(WifiManager* manager) {
    /* Update station info */
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        manager->status.sta_connected = true;
        manager->status.rssi = ap_info.rssi;
    } else {
        manager->status.sta_connected = false;
    }
    
    /* Additional status updates can be added here */
}

static int wifi_manager_scan(WifiManager* manager, NetworkInfoVec* out_networks) {
    if (!manager->initialized) {
        return WIFI_ERROR_NOT_INITIALIZED;
    }
    
    /* Initialize output vector */
    out_networks->len = 0;
    out_networks->capacity = 16;
    
    /* Start scan */
    wifi_scan_config_t scan_config = {0};
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        return WIFI_ERROR_CONFIG_ERROR;
    }
    
    /* Get number of APs found */
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    /* Limit to capacity */
    uint16_t count = ap_count < 16 ? ap_count : 16;
    
    /* Get AP records */
    wifi_ap_record_t ap_records[16];
    uint16_t actual_count = count;
    esp_wifi_scan_get_ap_records(&actual_count, ap_records);
    
    /* Convert to NetworkInfo */
    for (uint16_t i = 0; i < actual_count; i++) {
        NetworkInfo info;
        string32_init(&info.ssid);
        
        /* Copy SSID */
        for (int j = 0; j < 33; j++) {
            if (ap_records[i].ssid[j] == 0) {
                break;
            }
            if (info.ssid.len < MAX_SSID_LEN) {
                info.ssid.data[info.ssid.len++] = ap_records[i].ssid[j];
            }
        }
        
        info.rssi = ap_records[i].rssi;
        info.channel = ap_records[i].primary;
        info.auth_mode = ap_records[i].authmode;
        
        out_networks->data[out_networks->len++] = info;
    }
    
    return 0; /* Success */
}
