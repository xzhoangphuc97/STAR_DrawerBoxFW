#include <ctype.h>
#include "cui.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <ctype.h>
#include <esp_random.h>
#include <stdlib.h>
#include <time.h>

// Logging tag for CUI
static const char *TAG = "CUI";

// ASB-Commands
static const char *ASB_STATUS_SHORT[] = {"2F", "0C", "01", "00", "00", "00", "00", "00", "00", "00", "00", "00", "02", "03", "04"}; // For USB/BLE
static const char *ASB_STATUS_LONG[] = {"2F", "8C", "01", "00", "00", "00", "00", "00", "00", "00", "00", "00", "02", "03", "04", "00", "00"}; // For LAN/Wifi

// Common buffer and indices
static uint8_t common_buffer[COMMON_BUFFER_SIZE][DATA_SIZE];
static uint16_t common_write_idx = 0;
static uint16_t common_read_idx = 0;
static bool interface_enabled[IF_CNT] = {true, true, true, true};
static bool interface_has_permission = false;
static interface_t current_interface = IF_LAN;
static TickType_t last_data_time = 0;
static bool interfaces_in_round[IF_CNT] = {false};
static TaskHandle_t lan_monitor_task = NULL;
static TaskHandle_t interface_round_task = NULL;
static TaskHandle_t arbitrate_task = NULL;

static const char* interface_to_string(interface_t interface) {
    // Convert interface enum to string
    // Step 1: Return string based on interface type
    switch (interface) {
        case IF_LAN: return "LAN";
        case IF_WIFI: return "Wi-Fi";
        case IF_BLE: return "BLE";
        case IF_USB: return "USB";
        default: return "Unknown";
    }
}

uint8_t ByteStr2Hex(const char *str) {
    // Convert hex string to byte
    // Step 1: Convert string to long using base 16
    char *endptr;
    long value = strtol(str, &endptr, 16);
    // Step 2: Validate conversion
    if (endptr == str + 2 && *endptr == '\0') {
        return (uint8_t)value;
    }
    // Step 3: Log error and return 0
    ESP_LOGE(TAG, "Invalid hex string: %s", str);
    return 0;
}

char *ByteHex2Str(uint8_t byte) {
    // Convert byte to hex string
    // Step 1: Allocate memory for hex string
    char *str = (char *)malloc(3 * sizeof(char));
    // Step 2: Check allocation success
    if (!str) {
        ESP_LOGE(TAG, "Failed to allocate memory for ByteHex2Str");
        return NULL;
    }
    // Step 3: Format byte as hex
    snprintf(str, 3, "%02X", byte);
    // Step 4: Return hex string
    return str;
}

void cui_init(void) {
    // Initialize CUI module and tasks
    // Step 1: Log initialization start
    ESP_LOGI(TAG, "Initializing CUI...");
    // Step 2: Reset buffer indices
    common_write_idx = 0;
    common_read_idx = 0;
    // Step 3: Clear common buffer
    memset(common_buffer, 0, sizeof(common_buffer));
    // Step 4: Grant initial permission to LAN
    interface_has_permission = true;
    last_data_time = xTaskGetTickCount();
    // Step 5: Log initial permission
    ESP_LOGI(TAG, "Interface %s granted permission to access common buffer", interface_to_string(IF_LAN));
    // Step 6: Create monitoring and arbitration tasks
    xTaskCreate(cui_monitor_ethernet_wifi, "lan_monitor", 4096, NULL, 5, &lan_monitor_task);
    xTaskCreate(cui_monitor_interfaces_in_round, "interface_round", 4096, NULL, 5, &interface_round_task);
    xTaskCreate(cui_arbitrate_interfaces, "arbitrate", 4096, NULL, 5, &arbitrate_task);
    // Step 7: Log initialization complete
    ESP_LOGI(TAG, "CUI initialized");
}

void cui_monitor_ethernet_wifi(void *pvParameters) {
    // Monitor Ethernet/Wi-Fi connection states
    // Step 1: Initialize last LAN state
    bool last_lan_state = false;
    // Step 2: Monitor loop
    while (1) {
        // Step 3: Check Ethernet connection
        bool lan_state = ethernet_connected();
        // Step 4: Handle state change
        if (lan_state != last_lan_state) {
            // Step 5: Log connection status
            ESP_LOGI(TAG, "Interface LAN %s", lan_state ? "connected" : "disconnected");
            // Step 6: Toggle Wi-Fi based on LAN state
            if (lan_state) {
                wifi_disable();
                interface_enabled[IF_WIFI] = false;
                ESP_LOGI(TAG, "Interface Wi-Fi disabled");
            } else {
                wifi_enable();
                interface_enabled[IF_WIFI] = true;
                ESP_LOGI(TAG, "Interface Wi-Fi enabled");
            }
            // Step 7: Update last LAN state
            last_lan_state = lan_state;
        }
        // Step 8: Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void cui_remove_multiple_spaces(char *str, char *result, size_t max_len) {
    // Replace multiple spaces with single space
    // Step 1: Initialize pointers and state
    char *dst = result;
    char *src = str;
    bool in_space = false;
    size_t pos = 0;
    // Step 2: Process input string
    while (*src && pos < max_len - 1) {
        // Step 3: Handle spaces
        if (isspace((unsigned char)*src)) {
            if (!in_space) {
                *dst++ = ' ';
                pos++;
                in_space = true;
            }
            src++;
        } else {
            // Step 4: Copy non-space characters
            *dst++ = *src++;
            pos++;
            in_space = false;
        }
    }
    // Step 5: Null-terminate result
    *dst = '\0';
    // Step 6: Trim trailing spaces
    while (dst > result && isspace((unsigned char)*(dst - 1))) {
        *--dst = '\0';
    }
}

void cui_string_to_array(const char *str, char **arr, size_t *arr_len, size_t max_arr_size) {
    // Split string into array by spaces
    // Step 1: Initialize array length
    *arr_len = 0;
    // Step 2: Create string copy
    char *copy = strdup(str);
    if (!copy) {
        ESP_LOGE(TAG, "Failed to allocate memory for string copy");
        return;
    }
    // Step 3: Tokenize string
    char *token = strtok(copy, " ");
    // Step 4: Process tokens
    while (token != NULL && *arr_len < max_arr_size) {
        // Step 5: Allocate memory for token
        arr[*arr_len] = strdup(token);
        if (!arr[*arr_len]) {
            ESP_LOGE(TAG, "Failed to allocate memory for token");
            // Step 6: Clean up on failure
            while (*arr_len > 0) {
                free(arr[--(*arr_len)]);
            }
            free(copy);
            return;
        }
        // Step 7: Increment array length
        (*arr_len)++;
        // Step 8: Get next token
        token = strtok(NULL, " ");
    }
    // Step 9: Free string copy
    free(copy);
}

void cui_array_to_string(const char **arr, size_t arr_len, char *str, size_t max_len) {
    // Join string array into single string
    // Step 1: Initialize output position
    size_t pos = 0;
    // Step 2: Process array elements
    for (size_t i = 0; i < arr_len && pos < max_len - 1; i++) {
        // Step 3: Get string length
        size_t len = strlen(arr[i]);
        // Step 4: Check available space
        if (pos + len + 1 >= max_len) break;
        // Step 5: Copy string
        memcpy(str + pos, arr[i], len);
        pos += len;
        // Step 6: Add space
        if (i < arr_len - 1) {
            str[pos++] = ' ';
        }
    }
    // Step 7: Null-terminate output
    str[pos] = '\0';
}

// bool cui_check_esc_ack_soh(const uint8_t *data, size_t len) {
//     // Check for ESC, ACK, or SOH in data
//     // Step 1: Process data to remove spaces
//     char processed_data[DATA_SIZE];
//     cui_remove_multiple_spaces((char *)data, processed_data, DATA_SIZE);
//     // Step 2: Check for ESC_ACK_SOH
//     return strstr(processed_data, ESC_ACK_SOH) != NULL;
// }

bool cui_check_esc_ack_soh(const uint8_t *data, size_t len) {
    // Check if processed data exactly matches ESC_ACK_SOH or ESC_ACK_SOH_USBC_HEX
    // Step 1: Process data to remove spaces
    char processed_data[DATA_SIZE];
    cui_remove_multiple_spaces((char *)data, processed_data, DATA_SIZE);
    // Step 2: Check for exact match with ESC_ACK_SOH or ESC_ACK_SOH_USBC_HEX
    return (strcmp(processed_data, ESC_ACK_SOH) == 0);
}

void cui_generate_ble_asb_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, char *output, size_t max_len) {
    // Generate BLE ASB status string with spaces
    // Step 1: Initialize output position
    size_t pos = 0;
    // Step 2: Build status string with spaces
    for (size_t i = 0; i < 15 && pos < max_len - 4; i++) {
        // Step 3: Convert ASB status to hex
        uint8_t byte = ByteStr2Hex(ASB_STATUS_SHORT[i]);
        char *hex = ByteHex2Str(byte);
        size_t hex_len = strlen(hex);
        // Step 4: Copy hex and add space
        memcpy(output + pos, hex, hex_len);
        pos += hex_len;
        output[pos++] = ' ';
        free(hex);
    }
    // Step 5: Check buffer full
    if (pos >= max_len) {
        output[max_len - 1] = '\0';
        return;
    }
    // Step 6: Insert status bytes at spaced positions
    char *hex;
    hex = ByteHex2Str(s1); memcpy(output + 6, hex, 2); free(hex);  // 2 bytes before 
    hex = ByteHex2Str(s2); memcpy(output + 36, hex, 2); free(hex); // 13 bytes before 
    hex = ByteHex2Str(s3); memcpy(output + 39, hex, 2); free(hex); // 14 bytes before 
    hex = ByteHex2Str(s4); memcpy(output + 42, hex, 2); free(hex); // 15 bytes before 
    // Step 7: Remove trailing space
    if (pos > 0 && output[pos - 1] == ' ') {
        pos--;
    }
    // Step 8: Null-terminate output
    output[pos] = '\0';
}

void cui_generate_usb_asb_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, char *output, size_t max_len) {
    // Generate USB ASB status string with spaces
    // Step 1: Initialize output position
    size_t pos = 0;
    // Step 2: Build status string with spaces
    for (size_t i = 0; i < 15 && pos < max_len - 4; i++) {
        // Step 3: Convert ASB status to hex
        uint8_t byte = ByteStr2Hex(ASB_STATUS_SHORT[i]);
        char *hex = ByteHex2Str(byte);
        size_t hex_len = strlen(hex);
        // Step 4: Copy hex and add space
        memcpy(output + pos, hex, hex_len);
        pos += hex_len;
        output[pos++] = ' ';
        free(hex);
    }
    // Step 5: Check buffer full
    if (pos >= max_len) {
        output[max_len - 1] = '\0';
        return;
    }
    // Step 6: Insert status bytes at spaced positions
    char *hex;
    hex = ByteHex2Str(s1); memcpy(output + 6, hex, 2); free(hex);  // 2 bytes before 
    hex = ByteHex2Str(s2); memcpy(output + 36, hex, 2); free(hex); // 13 bytes before 
    hex = ByteHex2Str(s3); memcpy(output + 39, hex, 2); free(hex); // 14 bytes before 
    hex = ByteHex2Str(s4); memcpy(output + 42, hex, 2); free(hex); // 15 bytes before 
    // Step 7: Remove trailing space
    if (pos > 0 && output[pos - 1] == ' ') {
        pos--;
    }
    // Step 8: Null-terminate output
    output[pos] = '\0';
}

void cui_generate_ethernet_asb_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, char *output, size_t max_len) {
    // Generate Ethernet ASB status string with spaces
    // Step 1: Initialize output position
    size_t pos = 0;
    // Step 2: Build status string with spaces
    for (size_t i = 0; i < 17 && pos < max_len - 4; i++) {
        // Step 3: Convert ASB status to hex
        uint8_t byte = ByteStr2Hex(ASB_STATUS_LONG[i]);
        char *hex = ByteHex2Str(byte);
        size_t hex_len = strlen(hex);
        // Step 4: Copy hex and add space
        memcpy(output + pos, hex, hex_len);
        pos += hex_len;
        output[pos++] = ' ';
        free(hex);
    }
    // Step 5: Check buffer full
    if (pos >= max_len) {
        output[max_len - 1] = '\0';
        return;
    }
    // Step 6: Insert status bytes at spaced positions
    char *hex;
    hex = ByteHex2Str(s1); memcpy(output + 6, hex, 2); free(hex);  // 2 bytes before 
    hex = ByteHex2Str(s2); memcpy(output + 36, hex, 2); free(hex); // 13 bytes before 
    hex = ByteHex2Str(s3); memcpy(output + 39, hex, 2); free(hex); // 14 bytes before 
    hex = ByteHex2Str(s4); memcpy(output + 42, hex, 2); free(hex); // 15 bytes before 
    // Step 7: Remove trailing space
    if (pos > 0 && output[pos - 1] == ' ') {
        pos--;
    }
    // Step 8: Null-terminate output
    output[pos] = '\0';
}

void cui_generate_wifi_asb_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, char *output, size_t max_len) {
    // Generate Wi-Fi ASB status string with spaces
    // Step 1: Initialize output position
    size_t pos = 0;
    // Step 2: Build status string with spaces
    for (size_t i = 0; i < 17 && pos < max_len - 4; i++) {
        // Step 3: Convert ASB status to hex
        uint8_t byte = ByteStr2Hex(ASB_STATUS_LONG[i]);
        char *hex = ByteHex2Str(byte);
        size_t hex_len = strlen(hex);
        // Step 4: Copy hex and add space
        memcpy(output + pos, hex, hex_len);
        pos += hex_len;
        output[pos++] = ' ';
        free(hex);
    }
    // Step 5: Check buffer full
    if (pos >= max_len) {
        output[max_len - 1] = '\0';
        return;
    }
    // Step 6: Insert status bytes at spaced positions
    char *hex;
    hex = ByteHex2Str(s1); memcpy(output + 6, hex, 2); free(hex);  // 2 bytes before 
    hex = ByteHex2Str(s2); memcpy(output + 36, hex, 2); free(hex); // 13 bytes before 
    hex = ByteHex2Str(s3); memcpy(output + 39, hex, 2); free(hex); // 14 bytes before 
    hex = ByteHex2Str(s4); memcpy(output + 42, hex, 2); free(hex); // 15 bytes before 
    // Step 7: Remove trailing space
    if (pos > 0 && output[pos - 1] == ' ') {
        pos--;
    }
    // Step 8: Null-terminate output
    output[pos] = '\0';
}

uint8_t cui_generate_status_byte(void) {
    // Generate random status byte
    // Step 1: Generate and return random byte
    return (uint8_t)(esp_random() % 256);
}

bool common_buffer_is_empty(void) {
    // Check if common buffer is empty
    // Step 1: Compare write and read indices
    bool is_empty = common_write_idx == common_read_idx;
    // Step 2: Log interface, status, and indices
    // ESP_LOGI(TAG, "Interface: CUI, status: %s, read_idx=%u, write_idx=%u",
    //          is_empty ? "empty" : "not empty", common_read_idx, common_write_idx);
    // Step 3: Return empty status
    return is_empty;
}

bool common_buffer_is_full(void) {
    // Check if common buffer is full
    // Step 1: Check if next write index equals read index
    bool is_full = ((common_write_idx + 1) % COMMON_BUFFER_SIZE) == common_read_idx;
    // Step 2: Log interface, status, and indices
    ESP_LOGI(TAG, "Interface: CUI, status: %s, read_idx=%u, write_idx=%u",
             is_full ? "full" : "not full", common_read_idx, common_write_idx);
    // Step 3: Return full status
    return is_full;
}

esp_err_t interface_buffer_read(interface_t interface, uint8_t *data, size_t *len) {
    // Read data from interface buffer
    // Step 1: Verify interface permission
    if (interface != current_interface || !interface_has_permission) {
        return ESP_ERR_NOT_FOUND;
    }
    // Step 2: Read from interface buffer
    esp_err_t ret;
    size_t orig_len = *len;
    switch (interface) {
        case IF_LAN:
            ret = ethernet_buffer_read(data, len);
            break;
        case IF_WIFI:
            ret = wifi_buffer_read(data, len);
            break;
        case IF_BLE:
            ret = ble_buffer_read(data, len);
            break;
        case IF_USB:
            ret = usb_cdc_buffer_read(data, len);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    // Step 3: Process read data
    if (ret == ESP_OK && *len > 0) {
        // Step 4: Filter printable characters
        size_t valid_len = 0;
        for (size_t i = 0; i < *len && i < orig_len - 1; i++) {
            if ((isprint(data[i]) || data[i] == ' ') && data[i] != '\0') {
                data[valid_len++] = data[i];
            } else {
                break;
            }
        }
        // Step 5: Null-terminate and update length
        data[valid_len] = '\0';
        *len = valid_len;
        // Step 6: Handle empty valid data
        if (valid_len == 0) {
            ESP_LOGW(TAG, "No valid printable characters from %s buffer", interface_to_string(interface));
            return ESP_FAIL;
        }
        // Step 7: Log successful read
        ESP_LOGI(TAG, "Read %d valid bytes from %s buffer: %s", valid_len, interface_to_string(interface), (char *)data);
    }
    // Step 8: Return result
    return ret;
}

esp_err_t common_buffer_write(const uint8_t *data, size_t len) {
    // Write data to common buffer
    // Step 1: Check if buffer is full
    if (common_buffer_is_full()) {
        ESP_LOGE(TAG, "Common buffer full, cannot write %d bytes", len);
        return ESP_FAIL;
    }
    // Step 2: Validate input data
    size_t valid_len = 0;
    for (size_t i = 0; i < len && i < DATA_SIZE - 1; i++) {
        if ((isprint(data[i]) || data[i] == ' ') && data[i] != '\0') {
            valid_len++;
        } else {
            break;
        }
    }
    // Step 3: Handle empty valid data
    if (valid_len == 0) {
        ESP_LOGW(TAG, "No valid printable characters to write to common buffer");
        return ESP_OK;
    }
    // Step 4: Write to buffer
    char *dst = (char *)common_buffer[common_write_idx];
    memset(dst, 0, DATA_SIZE);
    memcpy(dst, data, valid_len);
    dst[valid_len] = '\0';
    // Step 5: Log write operation
    ESP_LOGI(TAG, "Wrote %d bytes to common buffer, write_idx=%u, read_idx=%u, data=%s", valid_len, common_write_idx, common_read_idx, dst);
    // Step 6: Update write index
    common_write_idx = (common_write_idx + 1) % COMMON_BUFFER_SIZE;
    // Step 7: Update last data time
    last_data_time = xTaskGetTickCount();
    // Step 8: Return success
    return ESP_OK;
}

esp_err_t common_buffer_read(uint8_t *data, size_t *len) {
    // Read data from common buffer
    // Step 1: Check if buffer is empty
    if (common_buffer_is_empty()) {
        ESP_LOGD(TAG, "Common buffer empty, nothing to read");
        return ESP_FAIL;
    }
    // Step 2: Calculate copy length
    size_t copy_len = strlen((char *)common_buffer[common_read_idx]);
    copy_len = copy_len < *len ? copy_len : *len - 1;
    // Step 3: Copy data
    strncpy((char *)data, (char *)common_buffer[common_read_idx], copy_len);
    data[copy_len] = '\0';
    // Step 4: Log read operation
    ESP_LOGI(TAG, "Read %d bytes from common buffer, write_idx=%u, read_idx=%u, data=%s", copy_len, common_write_idx, common_read_idx, (char *)data);
    // Step 5: Update read index
    common_read_idx = (common_read_idx + 1) % COMMON_BUFFER_SIZE;
    // Step 6: Return success
    return ESP_OK;
}

void cui_monitor_interfaces_in_round(void *pvParameters) {
    // Monitor interfaces for round-robin
    // Step 1: Monitor loop
    while (1) {
        // Step 2: Check LAN status
        interfaces_in_round[IF_LAN] = interface_enabled[IF_LAN] && ethernet_connected() && !ethernet_buffer_empty();
        // Step 3: Check Wi-Fi status
        interfaces_in_round[IF_WIFI] = interface_enabled[IF_WIFI] && wifi_is_enabled() && !wifi_buffer_empty();
        // Step 4: Check BLE status
        interfaces_in_round[IF_BLE] = interface_enabled[IF_BLE] && ble_is_enabled() && !ble_buffer_empty();
        // Step 5: Check USB status
        interfaces_in_round[IF_USB] = interface_enabled[IF_USB] && usb_cdc_is_connected() && usb_cdc_is_enabled() && !usb_cdc_buffer_empty();
        // Step 6: Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void cui_arbitrate_interfaces(void *pvParameters) {
    // Arbitrate interface access
    // Step 1: Define timeout
    const TickType_t timeout_ticks = pdMS_TO_TICKS(3000);
    bool logged_empty_warning = false;
    // Step 2: Arbitration loop
    while (1) {
        // Step 3: Get current time
        TickType_t current_time = xTaskGetTickCount();
        // Step 4: Check timeout and empty buffer
        if (common_buffer_is_empty() && (current_time - last_data_time) >= timeout_ticks) {
            // Step 5: Cycle interfaces
            interface_t start_interface = current_interface;
            do {
                current_interface = (current_interface + 1) % IF_CNT;
                // Step 6: Grant permission
                if (interfaces_in_round[current_interface] && interface_enabled[current_interface]) {
                    interface_has_permission = true;
                    time_t now = time(NULL);
                    char time_str[32];
                    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
                    ESP_LOGI(TAG, "Interface %s granted permission at %s", interface_to_string(current_interface), time_str);
                    logged_empty_warning = false;
                    break;
                }
            } while (current_interface != start_interface);
            // Step 7: Update last data time
            last_data_time = current_time;
        }
        // Step 8: Process interface data
        if (interface_has_permission && interfaces_in_round[current_interface]) {
            uint8_t data[DATA_SIZE];
            size_t len = DATA_SIZE - 1;
            // Step 9: Read and write data
            while (true) {
                esp_err_t ret = interface_buffer_read(current_interface, data, &len);
                if (ret == ESP_OK) {
                    common_buffer_write(data, len);
                    len = DATA_SIZE - 1;
                } else {
                    if (!logged_empty_warning) {
                        ESP_LOGW(TAG, "Interface %s buffer empty, cannot read", interface_to_string(current_interface));
                        logged_empty_warning = true;
                    }
                    break;
                }
            }
        }
        // Step 10: Delay for 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

interface_t cui_get_current_interface(void) {
    // Get current active interface
    // Step 1: Return current interface
    return current_interface;
}

void delspace(const char *input, char *output, size_t max_len) {
    // Remove all spaces from input string
    // Step 1: Initialize pointers and position
    char *dst = output;
    const char *src = input;
    size_t pos = 0;
    // Step 2: Process input string
    while (*src && pos < max_len - 1) {
        // Step 3: Copy non-space characters
        if (!isspace((unsigned char)*src)) {
            *dst++ = *src;
            pos++;
        }
        src++;
    }
    // Step 4: Null-terminate output
    *dst = '\0';
}

// bool is_hex(const char *str) {
//     // Check if string contains only hex values
//     // Step 1: Process each character
//     for (size_t i = 0; str[i]; i++) {
//         // Step 2: Skip spaces
//         if (isspace((unsigned char)str[i])) {
//             continue;
//         }
//         // Step 3: Check if character is valid hex
//         if (!isxdigit((unsigned char)str[i])) {
//             ESP_LOGW(TAG, "Invalid hex character: %c", str[i]);
//             return false;
//         }
//     }
//     // Step 4: Return true if all characters are valid
//     return true;
// }

// bool is_bin(const char *str) {
//     // Check if string contains only bit values
//     // Step 1: Process each character
//     for (size_t i = 0; str[i]; i++) {
//         // Step 2: Skip spaces
//         if (isspace((unsigned char)str[i])) {
//             continue;
//         }
//         // Step 3: Check if character is 0 or 1
//         if (str[i] != '0' && str[i] != '1') {
//             ESP_LOGW(TAG, "Invalid bit character: %c", str[i]);
//             return false;
//         }
//     }
//     // Step 4: Return true if all characters are valid
//     return true;
// }

void hex2bin(const char *hex, char *bit, size_t max_len) {
    // Convert hex string to bit string
    // Step 1: Initialize output position
    size_t pos = 0;
    
    // Step 2: Process each hex character
    for (size_t i = 0; hex[i] && pos < max_len - 5; i++) {
        // Step 3: Skip spaces
        if (isspace((unsigned char)hex[i])) {
            continue;
        }
        
        // Step 4: Convert hex character to uppercase
        char c = toupper((unsigned char)hex[i]);
        
        // Step 5: Convert to 4-bit binary using switch-case
        const char *bits;
        switch(c) {
            case '0': bits = "0000"; break;
            case '1': bits = "0001"; break;
            case '2': bits = "0010"; break;
            case '3': bits = "0011"; break;
            case '4': bits = "0100"; break;
            case '5': bits = "0101"; break;
            case '6': bits = "0110"; break;
            case '7': bits = "0111"; break;
            case '8': bits = "1000"; break;
            case '9': bits = "1001"; break;
            case 'A': bits = "1010"; break;
            case 'B': bits = "1011"; break;
            case 'C': bits = "1100"; break;
            case 'D': bits = "1101"; break;
            case 'E': bits = "1110"; break;
            case 'F': bits = "1111"; break;
            default:  bits = "0000";  // Invalid hex treated as 0
        }
        
        // Step 6: Copy the 4 bits to output
        for (int j = 0; j < 4 && pos < max_len - 1; j++) {
            bit[pos++] = bits[j];
        }
    }
    
    // Step 7: Null-terminate output
    bit[pos] = '\0';
}

// void bin2hex(const char *bit, char *hex, size_t max_len) {
//     // Convert bit string to hex string
//     // Step 1: Initialize output position and bit counter
//     size_t hex_pos = 0;
//     size_t bit_count = 0;
//     uint8_t value = 0;
//     // Step 2: Process each bit
//     for (size_t i = 0; bit[i] && hex_pos < max_len - 3; i++) {
//         // Step 3: Skip spaces
//         if (isspace((unsigned char)bit[i])) {
//             continue;
//         }
//         // Step 4: Accumulate bits
//         value = (value << 1) | (bit[i] == '1' ? 1 : 0);
//         bit_count++;
//         // Step 5: Convert to hex every 4 bits
//         if (bit_count == 4) {
//             char *hex_str = ByteHex2Str(value);
//             if (hex_str) {
//                 memcpy(hex + hex_pos, hex_str, 2);
//                 hex_pos += 2;
//                 free(hex_str);
//             }
//             value = 0;
//             bit_count = 0;
//         }
//     }
//     // Step 6: Handle remaining bits
//     if (bit_count > 0 && hex_pos < max_len - 3) {
//         value <<= (4 - bit_count);
//         char *hex_str = ByteHex2Str(value);
//         if (hex_str) {
//             memcpy(hex + hex_pos, hex_str, 2);
//             hex_pos += 2;
//             free(hex_str);
//         }
//     }
//     // Step 7: Null-terminate output
//     hex[hex_pos] = '\0';
// }

// // Function to convert hex string to raw byte array
// static size_t convertdata(const char *asb_status, uint8_t *out, size_t max_len) {
//     // Step 1: Copy asb_status to a temporary buffer for tokenization
//     char temp[DATA_SIZE];
//     strncpy(temp, asb_status, DATA_SIZE - 1);
//     temp[DATA_SIZE - 1] = '\0';

//     // Step 2: Initialize output length
//     size_t out_len = 0;

//     // Step 3: Tokenize the hex string by spaces
//     char *token = strtok(temp, " ");
//     while (token != NULL && out_len < max_len) {
//         // Step 4: Convert hex token to byte
//         unsigned int value;
//         if (sscanf(token, "%x", &value) == 1 && value <= 0xFF) {
//             out[out_len++] = (uint8_t)value;
//         } else {
//             ESP_LOGW(TAG, "Invalid hex token: %s", token);
//         }
//         token = strtok(NULL, " ");
//     }

//     // Step 5: Log the converted data
//     if (out_len > 0) {
//         char log_buffer[128] = "Converted hex to raw data: ";
//         size_t log_pos = strlen(log_buffer);
//         for (size_t i = 0; i < out_len && log_pos < sizeof(log_buffer) - 5; i++) {
//             log_pos += snprintf(log_buffer + log_pos, sizeof(log_buffer) - log_pos, "0x%02X ", out[i]);
//         }
//         ESP_LOGI(TAG, "%s (%u bytes)", log_buffer, out_len);
//     } else {
//         ESP_LOGW(TAG, "No valid hex data converted from asb_status: %s", asb_status);
//     }

//     // Step 6: Return the number of bytes converted
//     return out_len;
// }

// Function to send ASB status based on interface
void send_asb(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4)
{
    char asb_status[DATA_SIZE] = {0};
    // char sendata[DATA_SIZE] = {0};

    if (ethernet_connected())
    {
        cui_generate_ethernet_asb_status(s1, s2, s3, s4, asb_status, DATA_SIZE);
        // hex2bit(asb_status, sendata, DATA_SIZE);
        ESP_LOGI(TAG, "[ASB Before] Sent Ethernet ASB status(HEX): %s", asb_status);
        // ESP_LOGI(TAG, "[ASB Before] Sent Ethernet ASB status(BIT): %s", sendata);
        ethernet_send_data((uint8_t *)asb_status, strlen(asb_status));
        ESP_LOGI(TAG, "Sent Ethernet ASB: %s", asb_status);
    }

    if (wifi_is_enabled())
    {
        cui_generate_wifi_asb_status(s1, s2, s3, s4, asb_status, DATA_SIZE);
        // hex2bit(asb_status, sendata, DATA_SIZE);
        ESP_LOGI(TAG, "[ASB Before] Sent WIFI ASB status(HEX): %s", asb_status);
        // ESP_LOGI(TAG, "[ASB Before] Sent WIFI ASB status(BIT): %s", sendata);
        wifi_send_data((uint8_t *)asb_status, strlen(asb_status));
        ESP_LOGI(TAG, "Sent WiFi ASB: %s", asb_status);
    }

    if (ble_is_enabled())
    {
        cui_generate_ble_asb_status(s1, s2, s3, s4, asb_status, DATA_SIZE);
        // hex2bit(asb_status, sendata, DATA_SIZE);
        ESP_LOGI(TAG, "[ASB Before] Sent BLE ASB status(HEX): %s", asb_status);
        // ESP_LOGI(TAG, "[ASB Before] Sent BLE ASB status(BIT): %s", sendata);
        ble_send_data((uint8_t *)asb_status, strlen(asb_status));
        ESP_LOGI(TAG, "Sent BLE ASB: %s", asb_status);
    }

    if (usb_cdc_is_connected())
    {
        cui_generate_usb_asb_status(s1, s2, s3, s4, asb_status, DATA_SIZE);
        // hex2bit(asb_status, sendata, DATA_SIZE);
        ESP_LOGI(TAG, "[ASB Before] Sent USB ASB status(HEX): %s", asb_status);
        // ESP_LOGI(TAG, "[ASB Before] Sent USB ASB status(BIT): %s", sendata);
        usb_cdc_send_data((uint8_t *)asb_status, strlen(asb_status));
        ESP_LOGI(TAG, "Sent USB ASB: %s", asb_status);
    }
}
