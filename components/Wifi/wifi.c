#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "nvs_flash.h"
#include "../CUI/defines.h"
#include "cui.h"
#include "wifi.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>

// Logging tag for Wi-Fi-related messages
static const char *TAG = "wifi";

// Global variables
static bool is_enabled = true;
static char rx_buffer[BUFFER_SIZE][DATA_SIZE];
static uint16_t write_idx = 0;
static uint16_t read_idx = 0;
static esp_netif_t *sta_netif = NULL;
static esp_netif_t *ap_netif = NULL;
static int sta_server_sock = -1;
static int sta_client_sock = -1;
static int ap_server_sock = -1;
static int ap_client_sock = -1;
static bool logged_empty_warning = false; 
static bool is_sta_enabled = false;
static bool is_ap_enabled = false;


// Helper function to log Wi-Fi buffer status
static void log_wifi_buffer_status(const uint8_t *data, size_t len, uint16_t read_idx, uint16_t write_idx, bool is_full, bool is_empty) {
    // Step 1: Allocate buffer for data string
    char data_str[DATA_SIZE * 3 + 1];
    // Step 2: Check if data is printable
    bool is_printable = true;
    for (size_t i = 0; i < len; i++) {
        if (!isprint(data[i]) && !isspace(data[i])) {
            is_printable = false;
            break;
        }
    }
    // Step 3: Format data as string or hex
    if (is_printable && len > 0) {
        snprintf(data_str, sizeof(data_str), "%.*s", (int)len, (const char *)data);
    } else if (len == 0) {
        strcpy(data_str, "(empty)");
    } else {
        size_t pos = 0;
        for (size_t i = 0; i < len && pos < sizeof(data_str) - 3; i++) {
            pos += snprintf(data_str + pos, sizeof(data_str) - pos, "%02X ", data[i]);
        }
        if (pos > 0) data_str[pos - 1] = '\0';
    }
    // Step 4: Determine buffer status
    const char *status = is_full ? "full" : (is_empty ? "empty" : "normal");
    // Step 5: Log buffer details
    ESP_LOGI(TAG, "Wi-Fi buffer: data=%s, read_idx=%u, write_idx=%u, status=%s", data_str, read_idx, write_idx, status);
}

// TCP server task for STA mode
static void wifi_sta_tcp_server_task(void *param) {
    // Step 1: Initialize server and client address structures
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    int ret;
    // Step 2: Create TCP socket
    sta_server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sta_server_sock < 0) {
        ESP_LOGE(TAG, "STA: Failed to create server socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }
    // Step 3: Set socket option to reuse address
    int opt = 1;
    setsockopt(sta_server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    // Step 4: Configure server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(WIFI_STA_TCP_PORT);
    // Step 5: Bind socket to address
    ret = bind(sta_server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        ESP_LOGE(TAG, "STA: Failed to bind server socket to port %d: %d", WIFI_STA_TCP_PORT, errno);
        close(sta_server_sock);
        vTaskDelete(NULL);
        return;
    }
    // Step 6: Listen for incoming connections
    ret = listen(sta_server_sock, 1);
    if (ret < 0) {
        ESP_LOGE(TAG, "STA: Failed to listen on server socket: %d", errno);
        close(sta_server_sock);
        vTaskDelete(NULL);
        return;
    }
    // Step 7: Log server listening status
    ESP_LOGI(TAG, "STA TCP server listening on port %d", WIFI_STA_TCP_PORT);
    while (1) {
        // Step 8: Accept client connection
        client_len = sizeof(client_addr); // Reset client_len for each accept call
        sta_client_sock = accept(sta_server_sock, (struct sockaddr *)&client_addr, &client_len);
        if (sta_client_sock < 0) {
            ESP_LOGE(TAG, "STA: Failed to accept client connection: %d", errno);
            continue;
        }
        // Step 9: Log client connection
        ESP_LOGI(TAG, "STA: Client connected from " IPSTR, IP2STR((ip4_addr_t *)&client_addr.sin_addr));
        // Step 10: Handle client communication
        while (1) {
            // Step 11: Receive data from client
            uint8_t buf[DATA_SIZE];
            int len = recv(sta_client_sock, buf, sizeof(buf) - 1, 0);
            if (len <= 0) {
                ESP_LOGI(TAG, "STA: Client disconnected or error: %d", errno);
                break;
            }
            // Step 12: Null-terminate received data
            buf[len] = '\0';
            // Step 13: Store data in receive buffer
            wifi_buffer_write(buf, len);
            // Step 14: Prepare response
            char response[DATA_SIZE];
            int resp_len = snprintf(response, sizeof(response), "ESP: STA RESPOND: %.*s", len, buf);
            if (resp_len >= sizeof(response)) {
                ESP_LOGW(TAG, "STA: Response truncated to fit buffer size");
                resp_len = sizeof(response) - 1;
                response[resp_len] = '\0';
            }
            // Step 15: Send response to client
            wifi_send_data((const uint8_t *)response, resp_len);
        }
        // Step 16: Close client socket
        if (sta_client_sock >= 0) {
            close(sta_client_sock);
            sta_client_sock = -1;
        }
    }
    // Step 17: Close server socket (unreachable)
    if (sta_server_sock >= 0) {
        close(sta_server_sock);
        sta_server_sock = -1;
    }
    vTaskDelete(NULL);
}

// TCP server task for AP mode
static void wifi_ap_tcp_server_task(void *param) {
    // Step 1: Initialize server and client address structures
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    int ret;
    // Step 2: Create TCP socket for AP
    ap_server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (ap_server_sock < 0) {
        ESP_LOGE(TAG, "AP: Failed to create server socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }
    // Step 3: Set socket option to reuse address
    int opt = 1;
    setsockopt(ap_server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    // Step 4: Configure server address for AP
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(WIFI_AP_IP_ADDR);
    server_addr.sin_port = htons(WIFI_AP_TCP_PORT);
    // Step 5: Bind socket to address
    ret = bind(ap_server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        ESP_LOGE(TAG, "AP: Failed to bind server socket to port %d: %d", WIFI_AP_TCP_PORT, errno);
        close(ap_server_sock);
        vTaskDelete(NULL);
        return;
    }
    // Step 6: Listen for incoming connections
    ret = listen(ap_server_sock, 1);
    if (ret < 0) {
        ESP_LOGE(TAG, "AP: Failed to listen on server socket: %d", errno);
        close(ap_server_sock);
        vTaskDelete(NULL);
        return;
    }
    // Step 7: Log server listening status
    ESP_LOGI(TAG, "AP TCP server listening on %s:%d", WIFI_AP_IP_ADDR, WIFI_AP_TCP_PORT);
    while (1) {
        // Step 8: Accept client connection
        client_len = sizeof(client_addr); // Reset client_len for each accept call
        ap_client_sock = accept(ap_server_sock, (struct sockaddr *)&client_addr, &client_len);
        if (ap_client_sock < 0) {
            ESP_LOGE(TAG, "AP: Failed to accept client connection: %d", errno);
            continue;
        }
        // Step 9: Log client connection
        ESP_LOGI(TAG, "AP: Client connected from " IPSTR, IP2STR((ip4_addr_t *)&client_addr.sin_addr));
        // Step 10: Handle client communication
        while (1) {
            // Step 11: Receive data from client
            uint8_t buf[DATA_SIZE];
            int len = recv(ap_client_sock, buf, sizeof(buf) - 1, 0);
            if (len <= 0) {
                ESP_LOGI(TAG, "AP: Client disconnected or error: %d", errno);
                break;
            }
            // Step 12: Null-terminate received data
            buf[len] = '\0';
            // Step 13: Store data in receive buffer
            wifi_buffer_write(buf, len);
            // Step 14: Prepare response
            char response[DATA_SIZE];
            int resp_len = snprintf(response, sizeof(response), "ESP: AP RESPOND: %.*s", len, buf);
            if (resp_len >= sizeof(response)) {
                ESP_LOGW(TAG, "AP: Response truncated to fit buffer size");
                resp_len = sizeof(response) - 1;
                response[resp_len] = '\0';
            }
            // Step 15: Send response to client
            wifi_send_data((const uint8_t *)response, resp_len);
        }
        // Step 16: Close client socket
        if (ap_client_sock >= 0) {
            close(ap_client_sock);
            ap_client_sock = -1;
        }
    }
    // Step 17: Close server socket (unreachable)
    if (ap_server_sock >= 0) {
        close(ap_server_sock);
        ap_server_sock = -1;
    }
    vTaskDelete(NULL);
}

bool wifi_buffer_full(void) {
    // Step 1: Check if next write index overlaps read index
    bool full = ((write_idx + 1) % BUFFER_SIZE) == read_idx;
    // Step 2: Log interface, status, and indices
    ESP_LOGI(TAG, "Interface: Wifi, status: %s, read_idx=%u, write_idx=%u",
             full ? "full" : "not full", read_idx, write_idx);
    // Step 3: Return full status
    return full;
}

bool wifi_buffer_empty(void) {
    // Step 1: Check if write and read indices are equal
    bool empty = write_idx == read_idx;
    // Step 2: Return empty status
    return empty;
}

esp_err_t wifi_buffer_write(const uint8_t *data, size_t len) {
    // Step 1: Check if either Wi-Fi interface is enabled
    if (!is_enabled && !is_sta_enabled && !is_ap_enabled) {
        ESP_LOGW(TAG, "Cannot write data: Both Wi-Fi interfaces are disabled");
        return ESP_FAIL;
    }
    // Step 2: Check if buffer is full
    if (wifi_buffer_full()) {
        ESP_LOGE(TAG, "Wi-Fi buffer full, cannot write %d bytes", len);
        return ESP_FAIL;
    }
    // Step 3: Copy data to buffer, filtering non-printable characters
    char *dst = rx_buffer[write_idx];
    size_t j = 0;
    for (size_t i = 0; i < len && j < DATA_SIZE - 1; i++) {
        if (data[i] != '\n' && data[i] != '\r' && (isprint(data[i]) || isspace(data[i]))) {
            dst[j++] = data[i];
        }
    }
    dst[j] = '\0';
    size_t copy_len = j;
    // Step 4: Update write index
    write_idx = (write_idx + 1) % BUFFER_SIZE;
    // Step 5: Log received data
    ESP_LOGI(TAG, "Received %d bytes via Wi-Fi: %.*s", copy_len, copy_len, (const char *)dst);
    // Step 6: Log buffer status
    log_wifi_buffer_status((const uint8_t *)dst, copy_len, read_idx, write_idx, wifi_buffer_full(), wifi_buffer_empty());
    // Step 7: Return success
    return ESP_OK;
}

esp_err_t wifi_buffer_read(uint8_t *data, size_t *len) {
    // Step 1: Check if buffer is empty
    if (wifi_buffer_empty()) {
        if (!logged_empty_warning) {
            ESP_LOGW(TAG, "Wi-Fi buffer empty, cannot read");
            logged_empty_warning = true;
        }
        return ESP_FAIL;
    }
    // Step 2: Calculate length to copy
    size_t copy_len = strlen(rx_buffer[read_idx]);
    copy_len = copy_len < *len ? copy_len : *len - 1;
    // Step 3: Copy data from buffer
    memcpy(data, rx_buffer[read_idx], copy_len);
    data[copy_len] = '\0';
    // Step 4: Log buffer status
    log_wifi_buffer_status(data, copy_len, read_idx, write_idx, wifi_buffer_full(), wifi_buffer_empty());
    // Step 5: Update read index
    read_idx = (read_idx + 1) % BUFFER_SIZE;
    // Step 6: Log read data
    ESP_LOGI(TAG, "Read %d bytes from Wi-Fi buffer: %.*s", copy_len, copy_len, (const char *)data);
    // Step 7: Return success
    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    static int retry_count = 0;
    const int MAX_RETRIES = 5;
    if (event_base != WIFI_EVENT) return;
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "Wi-Fi STA started");
            esp_wifi_connect();
            retry_count = 0;
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Wi-Fi STA connected to AP");
            ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
            ESP_LOGI(TAG, "Wi-Fi power save mode re-disabled after STA connection");
            wifi_sta_enable(); // Enable STA mode
            retry_count = 0;
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Wi-Fi STA disconnected from AP");
            if (retry_count < MAX_RETRIES) {
                ESP_LOGI(TAG, "Retrying STA connection (attempt %d/%d)...", retry_count + 1, MAX_RETRIES);
                esp_wifi_connect();
                retry_count++;
            } else {
                ESP_LOGE(TAG, "Max STA retry attempts reached, disabling STA");
                wifi_sta_disable(); // Disable only STA mode
            }
            break;
        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "Wi-Fi AP started");
            wifi_ap_enable(); // Enable AP mode
            xTaskCreate(wifi_ap_tcp_server_task, "wifi_ap_tcp_server", 4096, NULL, 5, NULL);
            ESP_LOGI(TAG, "AP TCP server task created");
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            {
                wifi_event_ap_staconnected_t *conn_event = (wifi_event_ap_staconnected_t *)event_data;
                ESP_LOGI(TAG, "Station connected to AP, MAC: " MACSTR, MAC2STR(conn_event->mac));
            }
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            {
                wifi_event_ap_stadisconnected_t *disconn_event = (wifi_event_ap_stadisconnected_t *)event_data;
                ESP_LOGI(TAG, "Station disconnected from AP, MAC: " MACSTR, MAC2STR(disconn_event->mac));
            }
            break;
        default:
            break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    // Step 1: Check if event is an IP event
    if (event_base != IP_EVENT) return;
    // Step 2: Handle IP events
    if (event_id == IP_EVENT_STA_GOT_IP) {
        // Step 3: Extract STA IP information
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        // Step 4: Log STA IP
        ESP_LOGI(TAG, "Got STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // Step 5: Start STA TCP server
        xTaskCreate(wifi_sta_tcp_server_task, "wifi_sta_tcp_server", 4096, NULL, 5, NULL);
        ESP_LOGI(TAG, "STA TCP server task created");
    } else if (event_id == IP_EVENT_AP_STAIPASSIGNED) {
        // Step 3: Extract AP client IP information
        ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t *)event_data;
        // Step 4: Log AP client IP
        ESP_LOGI(TAG, "Assigned IP to AP client: " IPSTR, IP2STR(&event->ip));
    }
}

bool wifi_is_enabled(void) {
    return is_enabled && (is_sta_enabled || is_ap_enabled);
}

static uint8_t select_best_channel(void) {
    // Step 1: Ensure Wi-Fi is started
    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_STATE) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi for scanning: 0x%x", ret);
        return 6; // Fallback to channel 6
    }
    ESP_LOGI(TAG, "Wi-Fi started for channel scanning");

    // Step 2: Configure and start scan
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0, // Scan all channels
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 120,
        .scan_time.active.max = 150
    };
    ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi scan: 0x%x", ret);
        return 6; // Fallback to channel 6
    }

    // Step 3: Get scan results
    uint16_t ap_count = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(ap_count * sizeof(wifi_ap_record_t));
    if (!ap_list) {
        ESP_LOGE(TAG, "Failed to allocate memory for AP scan");
        return 6; // Fallback to channel 6
    }
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));

    // Step 4: Count APs per channel
    int channel_usage[14] = {0}; // Channels 1-13
    for (uint16_t i = 0; i < ap_count; i++) {
        if (ap_list[i].primary >= 1 && ap_list[i].primary <= 13) {
            channel_usage[ap_list[i].primary]++;
        }
    }
    free(ap_list);

    // Step 5: Select least congested channel (prefer 1, 6, 11)
    uint8_t best_channel = 6; // Default to 6
    int min_usage = channel_usage[6];
    for (int i = 1; i <= 13; i += 5) { // Check 1, 6, 11
        if (channel_usage[i] < min_usage) {
            min_usage = channel_usage[i];
            best_channel = i;
        }
    }
    ESP_LOGI(TAG, "Selected AP channel: %d", best_channel);

    // Step 6: Stop Wi-Fi to allow reconfiguration
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_LOGI(TAG, "Wi-Fi stopped after scanning");

    // Step 7: Return selected channel
    return best_channel;
}

void wifi_init(void) {
    is_sta_enabled = false;
    is_ap_enabled = false;
    // Step 1: Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition issue detected, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Step 2: Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "Network interface initialized");

    // Step 3: Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Event loop created");

    // Step 4: Create Wi-Fi STA and AP network interfaces
    sta_netif = esp_netif_create_default_wifi_sta();
    ap_netif = esp_netif_create_default_wifi_ap();
    if (sta_netif == NULL || ap_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create Wi-Fi STA or AP netif");
        return;
    }
    ESP_LOGI(TAG, "Wi-Fi STA and AP netifs created");

    // Step 5: Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_LOGI(TAG, "Wi-Fi driver initialized");

    // Step 6: Set Wi-Fi mode to AP + STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_LOGI(TAG, "Wi-Fi mode set to AP+STA");

    // Step 7: Configure Wi-Fi settings for STA
    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASSWORD,
            .threshold.authmode = WIFI_SECURITY_DEFAULT,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -90
        }
    };

    // Step 8: Adjust STA security settings
    switch (WIFI_SECURITY_DEFAULT) {
        case WIFI_SECURITY_OPEN:
            sta_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
            sta_config.sta.password[0] = '\0';
            ESP_LOGI(TAG, "STA security set to OPEN");
            break;
        case WIFI_SECURITY_WPA:
            sta_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
            ESP_LOGI(TAG, "STA security set to WPA-Personal");
            break;
        case WIFI_SECURITY_WPA2:
            sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
            ESP_LOGI(TAG, "STA security set to WPA2-Personal");
            break;
        case WIFI_SECURITY_WPA3:
            sta_config.sta.threshold.authmode = WIFI_AUTH_WPA3_PSK;
            ESP_LOGI(TAG, "STA security set to WPA3-Personal");
            break;
        default:
            ESP_LOGE(TAG, "Invalid STA security type: %d", WIFI_SECURITY_DEFAULT);
            return;
    }

    // Step 9: Select the best channel for AP
    uint8_t best_channel = select_best_channel();

    // Step 10: Configure Wi-Fi settings for AP
    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .password = WIFI_AP_PASSWORD,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = best_channel,
            .authmode = WIFI_SECURITY_DEFAULT,
            .max_connection = 2,
            .beacon_interval = 100,
            .pairwise_cipher = WIFI_CIPHER_TYPE_CCMP,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
            .ftm_responder = false
        }
    };

    // Step 11: Adjust AP security settings
    if (ap_config.ap.authmode == WIFI_AUTH_OPEN) {
        ap_config.ap.password[0] = '\0';
        ESP_LOGI(TAG, "AP security set to OPEN");
    } else if (strlen(WIFI_AP_PASSWORD) < 8) {
        ESP_LOGE(TAG, "AP password too short (<8 chars) for %d", ap_config.ap.authmode);
        return;
    }

    // Step 12: Apply Wi-Fi configurations
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_LOGI(TAG, "Wi-Fi STA config set (SSID: %s)", sta_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_LOGI(TAG, "Wi-Fi AP config set (SSID: %s, Channel: %d)", ap_config.ap.ssid, ap_config.ap.channel);

    // Step 13: Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler, NULL, NULL));
    ESP_LOGI(TAG, "Event handlers registered");

    // Step 14: Configure IP settings for STA
    if (WIFI_DHCP_ENABLE) {
        ESP_LOGI(TAG, "Enabling DHCP client for STA");
        ESP_ERROR_CHECK(esp_netif_dhcpc_start(sta_netif));
    } else {
        ESP_LOGI(TAG, "Disabling DHCP client and setting static IP for STA");
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));
        esp_netif_ip_info_t ip_info;
        ip4_addr_t temp_ip;
        if (ip4addr_aton(WIFI_STA_IP_ADDR, &temp_ip) != 1) {
            ESP_LOGE(TAG, "Invalid STA IP address: %s", WIFI_STA_IP_ADDR);
            return;
        }
        ip_info.ip.addr = temp_ip.addr;
        if (ip4addr_aton(WIFI_STA_GATEWAY, &temp_ip) != 1) {
            ESP_LOGE(TAG, "Invalid STA gateway address: %s", WIFI_STA_GATEWAY);
            return;
        }
        ip_info.gw.addr = temp_ip.addr;
        if (ip4addr_aton(WIFI_STA_SUBNET_MASK, &temp_ip) != 1) {
            ESP_LOGE(TAG, "Invalid STA netmask: %s", WIFI_STA_SUBNET_MASK);
            return;
        }
        ip_info.netmask.addr = temp_ip.addr;
        ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));
        ESP_LOGI(TAG, "Static STA IP configured as %s", WIFI_STA_IP_ADDR);
    }

    // Step 15: Configure IP settings for AP
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    esp_netif_ip_info_t ap_ip_info;
    ip4_addr_t temp_ip;
    if (ip4addr_aton(WIFI_AP_IP_ADDR, &temp_ip) != 1) {
        ESP_LOGE(TAG, "Invalid AP IP address: %s", WIFI_AP_IP_ADDR);
        return;
    }
    ap_ip_info.ip.addr = temp_ip.addr;
    if (ip4addr_aton(WIFI_AP_GATEWAY, &temp_ip) != 1) {
        ESP_LOGE(TAG, "Invalid AP gateway address: %s", WIFI_AP_GATEWAY);
        return;
    }
    ap_ip_info.gw.addr = temp_ip.addr;
    if (ip4addr_aton(WIFI_AP_SUBNET_MASK, &temp_ip) != 1) {
        ESP_LOGE(TAG, "Invalid AP netmask: %s", WIFI_AP_SUBNET_MASK);
        return;
    }
    ap_ip_info.netmask.addr = temp_ip.addr;
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ap_ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));
    ESP_LOGI(TAG, "AP IP configured as %s", WIFI_AP_IP_ADDR);

    // Step 16: Disable power save mode for better stability
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi power save disabled");

    // Step 17: Start Wi-Fi driver
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi started");
}

void wifi_reset(void) {
    ESP_LOGI(TAG, "Resetting Wi-Fi interface...");
    // Close STA sockets
    if (sta_client_sock >= 0) {
        close(sta_client_sock);
        sta_client_sock = -1;
    }
    if (sta_server_sock >= 0) {
        close(sta_server_sock);
        sta_server_sock = -1;
    }
    // Close AP sockets
    if (ap_client_sock >= 0) {
        close(ap_client_sock);
        ap_client_sock = -1;
    }
    if (ap_server_sock >= 0) {
        close(ap_server_sock);
        ap_server_sock = -1;
    }
    // Reset enable flags
    is_sta_enabled = false;
    is_ap_enabled = false;
    // Disconnect and stop Wi-Fi
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();
    // Reinitialize Wi-Fi
    wifi_init();
    ESP_LOGI(TAG, "Wi-Fi interface reset");
}

void wifi_enable(void) {
    // Step 1: Set enable flag
    is_enabled = true;
    // Step 2: Log enable status
    ESP_LOGI(TAG, "Wi-Fi interface enabled");
}

void wifi_disable(void) {
    // Step 1: Clear enable flag
    is_enabled = false;
    // Step 2: Log disable status
    ESP_LOGI(TAG, "Wi-Fi interface disabled");
}

void wifi_sta_enable(void) {
    is_sta_enabled = true;
    ESP_LOGI(TAG, "Wi-Fi STA interface enabled");
}

void wifi_sta_disable(void) {
    is_sta_enabled = false;
    ESP_LOGI(TAG, "Wi-Fi STA interface disabled");
}

void wifi_ap_enable(void) {
    is_ap_enabled = true;
    ESP_LOGI(TAG, "Wi-Fi AP interface enabled");
}

void wifi_ap_disable(void) {
    is_ap_enabled = false;
    ESP_LOGI(TAG, "Wi-Fi AP interface disabled");
}

void wifi_send_data(const uint8_t *data, size_t len) {
    // Step 1: Check if both interfaces are disabled
    if (!is_enabled && !is_sta_enabled && !is_ap_enabled) {
        ESP_LOGW(TAG, "Cannot send data: Both Wi-Fi interfaces are disabled");
        return;
    }
    // Step 2: Send to STA client if connected
    if (is_sta_enabled && sta_client_sock >= 0) {
        int sent = send(sta_client_sock, data, len, 0);
        if (sent < 0) {
            ESP_LOGE(TAG, "STA: Failed to send data: %d", errno);
            close(sta_client_sock);
            sta_client_sock = -1;
        } else {
            ESP_LOGI(TAG, "STA: Sent %d bytes: %.*s", sent, sent, (const char *)data);
        }
    }
    // Step 3: Send to AP client if connected
    if (is_ap_enabled && ap_client_sock >= 0) {
        int sent = send(ap_client_sock, data, len, 0);
        if (sent < 0) {
            ESP_LOGE(TAG, "AP: Failed to send data: %d", errno);
            close(ap_client_sock);
            ap_client_sock = -1;
        } else {
            ESP_LOGI(TAG, "AP: Sent %d bytes: %.*s", sent, sent, (const char *)data);
        }
    }
    // Step 4: Warn if no clients are connected
    if ((is_sta_enabled && sta_client_sock < 0) && (is_ap_enabled && ap_client_sock < 0)) {
        ESP_LOGW(TAG, "Cannot send data: No clients connected for enabled interfaces");
    }
}