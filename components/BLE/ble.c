#include <ctype.h>
#include "ble.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "../CUI/cui.h"
#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include <string.h>
#include "esp_gatt_common_api.h"

// Logging tag for BLE
static const char* TAG = "BLE";

// Circular receive buffer
static char rx_buffer[BUFFER_SIZE][DATA_SIZE];

// Write and read indices
static uint16_t write_idx = 0;
static uint16_t read_idx = 0;

// BLE connection status
static bool is_connected = false;
static bool is_enabled = false;
static bool notifications_enabled = false; // Track if notifications are enabled
static bool logged_empty_warning = false; // Track empty buffer warning

// GATT interface and handles
static esp_gatt_if_t gatts_if = ESP_GATT_IF_NONE;
static uint16_t service_handle = 0;
static uint16_t char_handle = 0;
static uint16_t descr_handle = 0;
static uint16_t conn_id = 0;

#define SERVICE_UUID 0xFFF0  // BLE service UUID
#define CHAR_UUID    0xFFF1  // BLE characteristic UUID

// Advertising parameters (global to restart advertising on disconnect)
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x00A0,  // 100 ms
    .adv_int_max = 0x0140,  // 200 ms
    .adv_type = ADV_TYPE_IND,  // General discoverable, connectable
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatt_if, esp_ble_gatts_cb_param_t* param) {
    // Handle GATT server events
    // Step 1: Process GATT event based on type
    switch (event) {
        case ESP_GATTS_REG_EVT:
            // Step 2: Handle GATT server registration
            if (param->reg.status == ESP_GATT_OK) {
                // Step 3: Store GATT interface and create service
                gatts_if = gatt_if;
                esp_gatt_srvc_id_t service_id = {
                    .id = {
                        .uuid = {
                            .len = ESP_UUID_LEN_16,
                            .uuid.uuid16 = SERVICE_UUID,
                        },
                        .inst_id = 0,
                    },
                    .is_primary = true,
                };
                esp_ble_gatts_create_service(gatt_if, &service_id, 4);
            } else {
                ESP_LOGE(TAG, "GATT server registration failed, status: %d", param->reg.status);
            }
            break;
        case ESP_GATTS_CREATE_EVT:
            // Step 2: Handle service creation
            if (param->create.status == ESP_GATT_OK) {
                // Step 3: Store service handle and add characteristic
                service_handle = param->create.service_handle;
                static esp_gatt_char_prop_t char_prop = ESP_GATT_CHAR_PROP_BIT_READ | 
                                                        ESP_GATT_CHAR_PROP_BIT_WRITE | 
                                                        ESP_GATT_CHAR_PROP_BIT_NOTIFY;
                esp_ble_gatts_add_char(service_handle,
                                       &(esp_bt_uuid_t){.len = ESP_UUID_LEN_16, .uuid.uuid16 = CHAR_UUID},
                                       ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                       char_prop,
                                       NULL, NULL);
            } else {
                ESP_LOGE(TAG, "Service creation failed, status: %d", param->create.status);
            }
            break;
        case ESP_GATTS_ADD_CHAR_EVT:
            // Step 2: Handle characteristic addition
            if (param->add_char.status == ESP_GATT_OK) {
                // Step 3: Store characteristic handle and add descriptor
                char_handle = param->add_char.attr_handle;
                esp_ble_gatts_add_char_descr(service_handle,
                                             &(esp_bt_uuid_t){.len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
                                             ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                             NULL, NULL);
            } else {
                ESP_LOGE(TAG, "Characteristic addition failed, status: %d", param->add_char.status);
            }
            break;
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            // Step 2: Handle descriptor addition
            if (param->add_char_descr.status == ESP_GATT_OK) {
                // Step 3: Store descriptor handle and start service
                descr_handle = param->add_char_descr.attr_handle;
                ESP_LOGI(TAG, "Descriptor added successfully, handle: %d", descr_handle);
                esp_ble_gatts_start_service(service_handle);
            } else {
                ESP_LOGE(TAG, "Descriptor addition failed, status: %d", param->add_char_descr.status);
            }
            break;
        case ESP_GATTS_START_EVT:
            // Step 2: Handle service start
            if (param->start.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Service start failed, status: %d", param->start.status);
            }
            break;
        case ESP_GATTS_CONNECT_EVT:
            // Step 2: Handle BLE connection
            is_connected = true;
            conn_id = param->connect.conn_id;
            ESP_LOGI(TAG, "BLE connected, conn_id: %d", conn_id);
            // Step 3: Update connection parameters
            esp_ble_conn_update_params_t conn_params = {
                .min_int = 0x30,    // 37.5 ms (min interval)
                .max_int = 0x50,    // 62.5 ms (max interval)
                .latency = 0,       // Slave latency
                .timeout = 400,     // Supervision timeout (4 seconds)
            };
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            // Step 2: Handle BLE disconnection
            is_connected = false;
            notifications_enabled = false; 
            ESP_LOGI(TAG, "BLE disconnected");
            // Step 3: Restart advertising
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_WRITE_EVT:
            // Step 2: Handle write event
            if (param->write.handle == char_handle && param->write.len <= DATA_SIZE) {
                // Step 3: Log received data
                ESP_LOGI(TAG, "Received %d bytes: %.*s", param->write.len, 
                         param->write.len, (const char*)param->write.value);
                // Step 4: Write data to BLE buffer
                if (ble_buffer_write(param->write.value, param->write.len) == ESP_OK) {
                    // Step 5: Echo data if notifications enabled
                    if (notifications_enabled) {
                        ble_send_data(param->write.value, param->write.len);
                    } else {
                        ESP_LOGW(TAG, "Notifications not enabled by client, cannot echo data");
                        // Step 6: Send response if required
                        if (!param->write.is_prep && param->write.need_rsp) {
                            esp_err_t ret = esp_ble_gatts_send_response(gatt_if, param->write.conn_id, 
                                                                        param->write.trans_id, ESP_GATT_OK, NULL);
                            if (ret != ESP_OK) {
                                ESP_LOGE(TAG, "Failed to send basic write response: %s", esp_err_to_name(ret));
                            }
                        }
                    }
                }
            } else if (param->write.handle == descr_handle && param->write.len == 2) {
                // Step 3: Handle descriptor write
                ESP_LOGI(TAG, "Descriptor write: handle=%d, value=0x%02x%02x", 
                         param->write.handle, param->write.value[1], param->write.value[0]);
                // Step 4: Update notifications status
                if (param->write.value[0] == 0x01 && param->write.value[1] == 0x00) {
                    notifications_enabled = true;
                    ESP_LOGI(TAG, "Client enabled notifications");
                } else if (param->write.value[0] == 0x00 && param->write.value[1] == 0x00) {
                    notifications_enabled = false;
                    ESP_LOGI(TAG, "Client disabled notifications");
                } else {
                    ESP_LOGW(TAG, "Unexpected descriptor value, notifications_enabled=%d", notifications_enabled);
                }
                // Step 5: Send response if required
                if (param->write.need_rsp) {
                    esp_err_t ret = esp_ble_gatts_send_response(gatt_if, param->write.conn_id, 
                                                                param->write.trans_id, ESP_GATT_OK, NULL);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to send descriptor write response: %s", esp_err_to_name(ret));
                    }
                }
            }
            break;
        case ESP_GATTS_READ_EVT:
            // Step 2: Handle read event
            if (param->read.handle == char_handle) {
                // Step 3: Prepare read response
                static uint8_t read_data[] = "ESP32 Read Response";
                esp_gatt_rsp_t rsp = {0};
                rsp.attr_value.len = sizeof(read_data) - 1; // Exclude null terminator
                memcpy(rsp.attr_value.value, read_data, rsp.attr_value.len);
                rsp.attr_value.handle = char_handle;
                // Step 4: Send response
                esp_err_t ret = esp_ble_gatts_send_response(gatt_if, param->read.conn_id, 
                                                            param->read.trans_id, ESP_GATT_OK, &rsp);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to send read response: %s", esp_err_to_name(ret));
                } else {
                    ESP_LOGI(TAG, "Handled read request, sent %d bytes", rsp.attr_value.len);
                }
            }
            break;
        default:
            break;
    }
}

bool ble_buffer_full(void) {
    // Check if BLE buffer is full
    // Step 1: Check if next write index equals read index
    bool full = ((write_idx + 1) % BUFFER_SIZE) == read_idx;
    // Step 2: Log interface, status, and indices
    ESP_LOGI(TAG, "Interface: BLE, status: %s, read_idx=%u, write_idx=%u",
             full ? "full" : "not full", read_idx, write_idx);
    // Step 3: Return full status
    return full;
}

bool ble_buffer_empty(void) {
    // Check if BLE buffer is empty
    // Step 1: Check if write and read indices are equal
    bool empty = write_idx == read_idx;
    // Step 2: Log interface, status, and indices
    // ESP_LOGI(TAG, "Interface: BLE, status: %s, read_idx=%u, write_idx=%u",
    //          empty ? "empty" : "not empty", read_idx, write_idx);
    // Step 3: Return empty status
    return empty;
}

esp_err_t ble_buffer_write(const uint8_t* data, size_t len) {
    // Write data to BLE buffer
    // Step 1: Check connection and enable status
    if (!is_connected || !is_enabled) {
        ESP_LOGW(TAG, "Cannot write to buffer: BLE not connected or not enabled");
        return ESP_ERR_INVALID_STATE;
    }
    // Step 2: Check if buffer is full
    if (ble_buffer_full()) {
        ESP_LOGE(TAG, "BLE buffer full, cannot write %d bytes", len);
        return ESP_FAIL;
    }
    // Step 3: Copy valid data (excluding newlines)
    size_t copy_len = len < DATA_SIZE - 1 ? len : DATA_SIZE - 1;
    char *dst = rx_buffer[write_idx];
    size_t j = 0;
    for (size_t i = 0; i < copy_len && j < DATA_SIZE - 1; i++) {
        if (data[i] != '\n' && data[i] != '\r' && (isprint(data[i]) || isspace(data[i]))) {
            dst[j++] = data[i];
        }
    }
    dst[j] = '\0';
    copy_len = j;
    // Step 4: Update write index
    write_idx = (write_idx + 1) % BUFFER_SIZE;
    // Step 5: Log write operation and buffer status
    const char* status = ble_buffer_full() ? "full" : (ble_buffer_empty() ? "empty" : "normal");
    ESP_LOGI(TAG, "BLE buffer write: read_idx=%d, write_idx=%d, data=%s, status=%s",
             read_idx, write_idx, dst, status);
    // Step 6: Return success
    return ESP_OK;
}

esp_err_t ble_buffer_read(uint8_t* data, size_t* len) {
    // Read data from BLE buffer
    // Step 1: Check if buffer is empty
    if (ble_buffer_empty()) {
        if (!logged_empty_warning) {
            ESP_LOGW(TAG, "BLE buffer empty, cannot read");
            logged_empty_warning = true;
        }
        return ESP_FAIL;
    }
    // Step 2: Copy data from buffer
    size_t copy_len = strlen(rx_buffer[read_idx]);
    copy_len = copy_len < *len ? copy_len : *len - 1;
    memcpy(data, rx_buffer[read_idx], copy_len);
    data[copy_len] = '\0';

    // Step 3: Update read index
    read_idx = (read_idx + 1) % BUFFER_SIZE;

    // Step 4: Log read operation and buffer status
    const char* status = ble_buffer_full() ? "full" : (ble_buffer_empty() ? "empty" : "normal");
    ESP_LOGI(TAG, "BLE buffer read: read_idx=%d, write_idx=%d, data=%s, status=%s",
        read_idx, write_idx, (char*)data, status);
    // Step 5: Return success
    return ESP_OK;
}

bool ble_is_enabled(void) {
    // Step 1: Return enable status
    return is_enabled;
}

void ble_init(void) {
    // Initialize BLE module
    esp_err_t ret;
    // Step 1: Release Classic Bluetooth memory
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller memory release failed: %s", esp_err_to_name(ret));
        return;
    }
    // Step 2: Configure BLE controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return;
    }
    // Step 3: Enable BLE controller
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    // Step 4: Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }
    // Step 5: Enable Bluedroid stack
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }
    // Step 6: Register GATT callback
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATT callback register failed: %s", esp_err_to_name(ret));
        return;
    }
    // Step 7: Register GATT application
    ret = esp_ble_gatts_app_register(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATT app register failed: %s", esp_err_to_name(ret));
        return;
    }
    // Step 8: Set device name
    ret = esp_ble_gap_set_device_name("ESP32-BLE");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(ret));
        return;
    }
    // Step 9: Configure advertising data
    static esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = false,  
        .include_txpower = false,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0, 
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config adv data failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Advertising data configured successfully");
    // Step 10: Configure scan response data
    static esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp = true,
        .include_name = true,  
        .include_txpower = false,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = 0,
    };
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config scan response data failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Scan response data configured successfully");
    // Step 11: Start advertising
    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "BLE advertising started");
}

void ble_reset(void) {
    // Reset BLE module
    // Step 1: Disable Bluedroid
    esp_bluedroid_disable();
    // Step 2: Deinitialize Bluedroid
    esp_bluedroid_deinit();
    // Step 3: Disable controller
    esp_bt_controller_disable();
    // Step 4: Deinitialize controller
    esp_bt_controller_deinit();
    // Step 5: Reinitialize BLE
    ble_init();
}

void ble_enable(void) {
    // Enable BLE interface
    // Step 1: Set enable flag
    is_enabled = true;
    // Step 2: Log enable status
    ESP_LOGI(TAG, "BLE enabled");
}

void ble_disable(void) {
    // Disable BLE interface
    // Step 1: Clear enable flag
    is_enabled = false;
    // Step 2: Log disable status
    ESP_LOGI(TAG, "BLE disabled");
}

void ble_send_data(const uint8_t* data, size_t len) {
    // Send data over BLE
    // Step 1: Check connection and enable status
    if (is_connected && is_enabled) {
        // Step 2: Check if notifications are enabled
        if (notifications_enabled) {
            // Step 3: Send data via GATT indication
            esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, char_handle, len, (uint8_t*)data, false);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Sent %d bytes via BLE: %.*s", len, len, (const char*)data);
            } else {
                ESP_LOGE(TAG, "Failed to send data: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "Cannot send data: Notifications not enabled by client");
        }
    } else {
        ESP_LOGW(TAG, "Cannot send data: BLE not connected or not enabled");
    }
}