#ifndef WIFI_H
#define WIFI_H

#include "../CUI/defines.h"

#include <stdint.h>
#include "esp_err.h"

// Wifi operations
void wifi_init(void);                      // Initialize Wifi
void wifi_reset(void);                     // Reset Wifi
void wifi_enable(void);                    // Enable Wifi (global)
void wifi_disable(void);                   // Disable Wifi (global)
void wifi_sta_enable(void);                // Enable Wifi STA mode
void wifi_sta_disable(void);               // Disable Wifi STA mode
void wifi_ap_enable(void);                 // Enable Wifi AP mode
void wifi_ap_disable(void);                // Disable Wifi AP mode
void wifi_send_data(const uint8_t* data, size_t len); // Send data over Wifi

// Check if Wifi is enabled
bool wifi_is_enabled(void);

// Buffer operations
bool wifi_buffer_full(void);               // Check if Wifi buffer is full
bool wifi_buffer_empty(void);              // Check if Wifi buffer is empty
esp_err_t wifi_buffer_write(const uint8_t* data, size_t len); // Write to Wifi buffer
esp_err_t wifi_buffer_read(uint8_t* data, size_t* len);       // Read from Wifi buffer

#endif