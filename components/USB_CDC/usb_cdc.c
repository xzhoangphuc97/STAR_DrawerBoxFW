#include <ctype.h>
#include "usb_cdc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "tusb.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "cui.h"
#include "driver/gpio.h"

// Logging tag for USB CDC
static const char *TAG = "USB_CDC";

// GPIO15 monitoring
#define GPIO15_NUM 15
static bool gpio15_state = false;

// Circular receive buffer
static char rx_buffer[BUFFER_SIZE][DATA_SIZE];

// Write and read indices
static uint16_t write_idx = 0;
static uint16_t read_idx = 0;

// Current line buffer for processing received data
static char current_line[DATA_SIZE];
static size_t line_pos = 0;

// USB CDC connection and enable status
static bool is_connected = false;
static bool is_enabled = false;
static bool logged_empty_warning = false; // Track empty buffer warning

// TinyUSB CDC ACM callbacks
static void cdc_rx_callback(int itf, cdcacm_event_t *event);
static void cdc_line_state_callback(int itf, cdcacm_event_t *event);

// USB Device Descriptor
static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,  // Class specified in interface descriptor
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x303A,
    .idProduct = 0x4001,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static void log_usb_cdc_buffer_status(const uint8_t *data, size_t len, uint16_t read_idx, uint16_t write_idx, bool is_full, bool is_empty) {
    // Log USB CDC buffer status
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
        if (pos > 0) data_str[pos - 1] = '\0'; // Remove trailing space
    }
    // Step 4: Determine buffer status
    const char *status = is_full ? "full" : (is_empty ? "empty" : "normal");
    // Step 5: Log buffer details
    ESP_LOGI(TAG, "USB_CDC buffer: data=%s, read_idx=%u, write_idx=%u, status=%s", data_str, read_idx, write_idx, status);
}

static void gpio15_init(void) {
    // Initialize GPIO15 monitoring
    // Step 1: Configure GPIO15 as input with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO15_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    // Step 2: Read initial GPIO15 state
    gpio15_state = gpio_get_level(GPIO15_NUM);
    // Step 3: Log initial state
    ESP_LOGI(TAG, "GPIO15 initialized, current state: %d", gpio15_state);
}

bool usb_cdc_get_gpio15_state(void) {
    // Get current GPIO15 state
    // Step 1: Read GPIO15 level
    bool state = gpio_get_level(GPIO15_NUM);
    // Step 2: Return current state
    return state;
}

// Complete string descriptor array with proper formatting
static const char *string_desc_arr[] = {
    // Index 0: Language ID (must be first)
    (const char[]){0x09, 0x04},  // US English
    // Index 1: Manufacturer
    "Espressif",
    // Index 2: Product
    "ESP32-S3 USB CDC",
    // Index 3: Serial Number
    "1234567890",
    // Index 4: Configuration
    "CDC Configuration",
    // Index 5: Interface
    "CDC Interface",
    NULL
};

static void usb_task(void *param) {
    // Handle TinyUSB events with GPIO15 monitoring
    // Step 1: Initialize last GPIO check time
    uint32_t last_gpio_check = 0;
    while (1) {
        // Step 2: Process TinyUSB events
        tud_task();
        // Step 3: Check GPIO15 state every 5 seconds
        uint32_t now = xTaskGetTickCount();
        if (now - last_gpio_check > pdMS_TO_TICKS(5000)) {
            // Step 4: Update last check time
            last_gpio_check = now;
            // Step 5: Read current GPIO15 state
            bool current_state = gpio_get_level(GPIO15_NUM);
            // Step 6: Log state change if occurred
            if (current_state != gpio15_state) {
                gpio15_state = current_state;
                ESP_LOGW(TAG, "GPIO15 state changed to: %d", gpio15_state);
            }
            // Step 7: Log USB and GPIO status
            ESP_LOGI(TAG, "USB state: connected=%d, enabled=%d, tud_ready=%d, GPIO15=%d",
                     tud_cdc_n_connected(TINYUSB_CDC_ACM_0), is_enabled, tud_ready(), gpio15_state);
        }
        // Step 8: Delay for next iteration
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool usb_cdc_buffer_full(void) {
    // Check if USB CDC buffer is full
    // Step 1: Check if next write index overlaps read index
    bool full = ((write_idx + 1) % BUFFER_SIZE) == read_idx;
    // Step 2: Log interface, status, and indices
    ESP_LOGI(TAG, "Interface: USB_CDC, status: %s, read_idx=%u, write_idx=%u",
             full ? "full" : "not full", read_idx, write_idx);
    // Step 3: Return full status
    return full;
}

bool usb_cdc_buffer_empty(void) {
    // Check if USB CDC buffer is empty
    // Step 1: Check if write and read indices are equal
    bool empty = write_idx == read_idx;
    // Step 2: Log interface, status, and indices
    // ESP_LOGI(TAG, "Interface: USB_CDC, status: %s, read_idx=%u, write_idx=%u",
    //          empty ? "empty" : "not empty", read_idx, write_idx);
    // Step 3: Return empty status
    return empty;
}

esp_err_t usb_cdc_buffer_write(const uint8_t *data, size_t len) {
    // Write to USB CDC buffer
    // Step 1: Check connection and enable status
    if (!is_connected || !is_enabled) {
        ESP_LOGW(TAG, "Cannot write to buffer: USB CDC not connected or not enabled");
        return ESP_ERR_INVALID_STATE;
    }
    // Step 2: Handle buffer overflow by dropping oldest data
    if (usb_cdc_buffer_full()) {
        ESP_LOGW(TAG, "Buffer overflow! Dropping oldest data");
        read_idx = (read_idx + 1) % BUFFER_SIZE;
    }
    // Step 3: Copy only printable characters to buffer, excluding newlines
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
    // Step 5: Log buffer status
    log_usb_cdc_buffer_status((const uint8_t *)dst, copy_len, read_idx, write_idx, usb_cdc_buffer_full(), usb_cdc_buffer_empty());
    // Step 6: Return success
    return ESP_OK;
}

esp_err_t usb_cdc_buffer_read(uint8_t *data, size_t *len) {
    // Read from USB CDC buffer
    // Step 1: Check if buffer is empty
    if (usb_cdc_buffer_empty()) {
        if (!logged_empty_warning) {
            ESP_LOGW(TAG, "USB CDC buffer empty, cannot read");
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
    // Step 4: Update read index
    read_idx = (read_idx + 1) % BUFFER_SIZE;
    // Step 5: Log buffer status
    log_usb_cdc_buffer_status(data, copy_len, read_idx, write_idx, usb_cdc_buffer_full(), usb_cdc_buffer_empty());
    // Step 6: Reset empty warning flag on successful read
    logged_empty_warning = false;
    // Step 7: Return success
    return ESP_OK;
}

void usb_cdc_init(void) {
    ESP_LOGI(TAG, "Initializing USB CDC...");
    
    // Initialize GPIO15 monitoring
    gpio15_init();

    // Configure TinyUSB with device descriptor
    // tinyusb_config_t tusb_cfg = {
    //     .device_descriptor = &device_descriptor,
    //     .string_descriptor = string_desc_arr,
    //     .external_phy = false
    // };

    tinyusb_config_t tusb_cfg = {
        .descriptor = &device_descriptor,
        .string_descriptor = string_desc_arr,
        .external_phy = false,
        .configuration_descriptor = NULL,  // Add this if needed
    };

    // Install TinyUSB driver
    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TinyUSB: %s", esp_err_to_name(ret));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGE(TAG, "TinyUSB initialization failed, GPIO15=%d", gpio_get_level(GPIO15_NUM));
        }
    }

    // Delay for USB enumeration
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Configure CDC ACM with callbacks
    // tinyusb_config_cdcacm_t acm_cfg = {
    //     .usb_dev = TINYUSB_USBDEV_0,
    //     .cdc_port = TINYUSB_CDC_ACM_0,
    //     .rx_unread_buf_sz = DATA_SIZE,
    //     .callback_rx = &cdc_rx_callback,
    //     .callback_rx_wanted_char = NULL,
    //     .callback_line_state_changed = &cdc_line_state_callback,
    //     .callback_line_coding_changed = NULL
    // };

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = DATA_SIZE,
        .callback_rx = &cdc_rx_callback,
        .callback_line_state_changed = &cdc_line_state_callback,
        .callback_line_coding_changed = NULL
    };

    // Initialize CDC ACM
    ret = tusb_cdc_acm_init(&acm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize USB CDC ACM: %s, GPIO15=%d",
                 esp_err_to_name(ret), gpio_get_level(GPIO15_NUM));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGE(TAG, "USB CDC ACM initialization failed");
        }
    }

    ESP_LOGI(TAG, "USB CDC initialized with GPIO15=%d", gpio_get_level(GPIO15_NUM));
    xTaskCreate(usb_task, "usb_task", 4096, NULL, 5, NULL);
}

void usb_cdc_enable(void) {
    // Enable USB CDC
    // Step 1: Set enable flag
    is_enabled = true;
    // Step 2: Log enable status
    ESP_LOGI(TAG, "USB CDC enabled");
}

void usb_cdc_disable(void) {
    // Disable USB CDC
    // Step 1: Clear enable flag
    is_enabled = false;
    // Step 2: Log disable status
    ESP_LOGI(TAG, "USB CDC disabled");
}

esp_err_t usb_cdc_send_data(const uint8_t *data, size_t len) {
    // Send data over USB CDC
    // Step 1: Check connection and enable status
    if (!is_connected || !is_enabled) {
        ESP_LOGW(TAG, "Cannot write to buffer: USB CDC not connected or not enabled");
        return ESP_ERR_INVALID_STATE;
    }
    // Step 2: Check CDC connection
    if (!tud_cdc_n_connected(TINYUSB_CDC_ACM_0)) {
        ESP_LOGW(TAG, "Cannot send data: USB CDC not connected");
        return ESP_ERR_NOT_SUPPORTED;
    }
    // Step 3: Check TinyUSB readiness
    if (!tud_ready()) {
        ESP_LOGW(TAG, "TinyUSB not ready, cannot send data");
        return ESP_FAIL;
    }
    // Step 4: Write data to CDC buffer
    size_t written = tud_cdc_n_write(TINYUSB_CDC_ACM_0, data, len);
    if (written == 0 || written < len) {
        ESP_LOGW(TAG, "Failed to write all data: wrote %d of %d bytes", written, len);
        return ESP_FAIL;
    }
    // Step 5: Flush CDC buffer
    if (!tud_cdc_n_write_flush(TINYUSB_CDC_ACM_0)) {
        ESP_LOGW(TAG, "Flush failed");
        return ESP_FAIL;
    }
    // Step 6: Log send operation
    ESP_LOGI(TAG, "Sent %d bytes via USB CDC: %.*s", len, len, (const char *)data);
    // Step 7: Return success
    return ESP_OK;
}

bool usb_cdc_is_connected(void) {
    // Check if USB CDC is connected
    // Step 1: Check connection status and CDC connection
    bool connected = is_connected && tud_cdc_n_connected(TINYUSB_CDC_ACM_0);
    // Step 2: Return connection status
    return connected;
}

bool usb_cdc_is_enabled(void) {
    // Check if USB CDC is enabled
    // Step 1: Return enable status
    return is_enabled;
}

static void cdc_rx_callback(int itf, cdcacm_event_t *event) {
    // Handle received data over USB CDC
    // Step 1: Initialize receive buffer
    uint8_t buf[DATA_SIZE];
    size_t rx_size = 0;
    // Step 2: Read received data
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size);
    if (ret == ESP_OK && rx_size > 0) {
        // Step 3: Process each received byte
        for (size_t i = 0; i < rx_size; i++) {
            // Step 4: Store printable characters only
            if (line_pos < DATA_SIZE - 1 && (isprint(buf[i]) || isspace(buf[i]))) {
                current_line[line_pos++] = buf[i];
            }
            // Step 5: Check for newline (CR or LF)
            if (buf[i] == '\n' || buf[i] == '\r') {
                if (line_pos > 0) {
                    // Step 6: Null-terminate line
                    current_line[line_pos] = '\0';
                    // Step 7: Write to circular buffer
                    usb_cdc_buffer_write((const uint8_t *)current_line, line_pos);
                    // Step 8: Log received data
                    ESP_LOGI(TAG, "Received: %s", current_line);
                    // Step 9: Echo data if connected and enabled
                    if (tud_cdc_n_connected(TINYUSB_CDC_ACM_0) && is_enabled) {
                        char echo_buf[DATA_SIZE + 8];
                        size_t max_input_len = DATA_SIZE - 8;
                        current_line[max_input_len] = '\0';
                        snprintf(echo_buf, sizeof(echo_buf), "Echo: %s\n", current_line);
                        usb_cdc_send_data((const uint8_t *)echo_buf, strlen(echo_buf));
                    }
                    // Step 10: Reset line position
                    line_pos = 0;
                }
            }
        }
    } else {
        // Step 3: Log read failure
        ESP_LOGW(TAG, "Failed to read data: %s", esp_err_to_name(ret));
    }
}

static void cdc_line_state_callback(int itf, cdcacm_event_t *event) {
    // Handle USB line state changes
    // Step 1: Store previous connection state
    bool prev_connected = is_connected;
    // Step 2: Update connection state
    is_connected = tud_cdc_n_connected(TINYUSB_CDC_ACM_0);
    // Step 3: Log connection change
    if (prev_connected != is_connected) {
        ESP_LOGI(TAG, "USB %s", is_connected ? "connected" : "disconnected");
        if (!is_connected) {
            ESP_LOGI(TAG, "DTR: %d, RTS: %d",
                     event->line_state_changed_data.dtr,
                     event->line_state_changed_data.rts);
            // Step 4: Reset line position on disconnect
            line_pos = 0;
        }
    }
    // Step 5: Handle DTR signal for terminal readiness
    if (event->line_state_changed_data.dtr) {
        tud_cdc_n_write_flush(TINYUSB_CDC_ACM_0);
    }
}