#ifndef BLE_H
#define BLE_H

#include "esp_err.h"
#include "esp_mac.h"
#include <stdbool.h>  

// BLE operations
void ble_init(void);                       // Initialize BLE
void ble_reset(void);                      // Reset BLE module
void ble_enable(void);                     // Enable BLE
void ble_disable(void);                    // Disable BLE
void ble_send_data(const uint8_t* data, size_t len); // Send data over BLE

// Check if BLE is enabled
bool ble_is_enabled(void);

// Buffer operations
bool ble_buffer_full(void);                // Check if BLE buffer is full
bool ble_buffer_empty(void);               // Check if BLE buffer is empty
esp_err_t ble_buffer_write(const uint8_t* data, size_t len); // Write to BLE buffer
esp_err_t ble_buffer_read(uint8_t* data, size_t* len);       // Read from BLE buffer

#endif