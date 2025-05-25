#ifndef CUI_H
#define CUI_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "defines.h"
#include "ethernet.h"
#include "wifi.h"
#include "usb_cdc.h"
#include "ble.h"

// Define the number of interfaces
#define IF_CNT 4
// #define ESC_ACK_SOH "000110110000011000000001"
#define ESC_ACK_SOH "1B 06 01"

// Enum for interface types, representing the four communication interfaces
typedef enum {
    IF_LAN = 0,  // Ethernet LAN interface
    IF_WIFI,     // Wi-Fi interface
    IF_BLE,      // Bluetooth Low Energy interface
    IF_USB       // USB CDC interface
} interface_t;

// Function prototypes for CUI operations
uint8_t ByteStr2Hex(const char *str); // Convert hex string to byte
char *ByteHex2Str(uint8_t byte); // Convert byte to hex string
void cui_init(void); // Initialize CUI module
void cui_monitor_ethernet_wifi(void *pvParameters); // Monitor Ethernet and Wi-Fi interfaces
void cui_remove_multiple_spaces(char *str, char *result, size_t max_len); // Replace multiple spaces with single space
void cui_string_to_array(const char *str, char **arr, size_t *arr_len, size_t max_arr_size); // Split string into array
void cui_array_to_string(const char **arr, size_t arr_len, char *str, size_t max_len); // Join array into string
bool cui_check_esc_ack_soh(const uint8_t *data, size_t len); // Check for ESC, ACK, or SOH in data
bool cui_check_bit_esc_ack_soh(const uint8_t *data, size_t len); // Check for ESC, ACK, or SOH in bit data
void cui_generate_ble_asb_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, char *output, size_t max_len); // Generate BLE status string
void cui_generate_usb_asb_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, char *output, size_t max_len); // Generate USB status string
void cui_generate_ethernet_asb_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, char *output, size_t max_len); // Generate Ethernet status string
void cui_generate_wifi_asb_status(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, char *output, size_t max_len); // Generate Wi-Fi status string
uint8_t cui_generate_status_byte(void); // Generate status byte
bool common_buffer_is_empty(void); // Check if common buffer is empty
bool common_buffer_is_full(void); // Check if common buffer is full
esp_err_t interface_buffer_read(interface_t interface, uint8_t *data, size_t *len); // Read from interface buffer
esp_err_t common_buffer_write(const uint8_t *data, size_t len); // Write to common buffer
esp_err_t common_buffer_read(uint8_t *data, size_t *len); // Read from common buffer
void cui_monitor_interfaces_in_round(void *pvParameters); // Monitor interfaces in round-robin
void cui_arbitrate_interfaces(void *pvParameters); // Arbitrate between interfaces
interface_t cui_get_current_interface(void); // Get current active interface

void delspace(const char *input, char *output, size_t max_len); // Remove all spaces from input string
// bool is_hex(const char *str); // Check if string contains only hex values
// bool is_bin(const char *str); // Check if string contains only bit values
void hex2bin(const char *hex, char *bit, size_t max_len); // Convert hex string to bit string
// void bin2hex(const char *bit, char *hex, size_t max_len); // Convert bit string to hex string

void send_asb(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4); // Send ASB status over interfaces

#endif // CUI_H