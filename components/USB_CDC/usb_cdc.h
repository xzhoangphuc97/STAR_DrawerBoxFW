#ifndef USB_CDC_H
#define USB_CDC_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

// Buffer sizes (consistent with Main1.c)
#define BUFFER_SIZE 64
#define DATA_SIZE 256

void usb_cdc_init(void);
void usb_cdc_enable(void);
void usb_cdc_disable(void);

// Send data over USB CDC, returns ESP_OK on success or an error code on failure
esp_err_t usb_cdc_send_data(const uint8_t *data, size_t len);

// Check if USB CDC is connected
bool usb_cdc_is_connected(void);

// Check if USB CDC is enabled
bool usb_cdc_is_enabled(void);

// Buffer management functions
bool usb_cdc_buffer_full(void);
bool usb_cdc_buffer_empty(void);
esp_err_t usb_cdc_buffer_write(const uint8_t *data, size_t len);
esp_err_t usb_cdc_buffer_read(uint8_t *data, size_t *len);

#endif // USB_CDC_H