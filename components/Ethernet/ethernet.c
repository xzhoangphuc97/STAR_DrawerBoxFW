#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "../CUI/defines.h"
#include "ethernet.h"

static const char *TAG = "ethernet";

static spi_device_handle_t spi_handle = NULL;
static SemaphoreHandle_t spi_mutex = NULL;
static bool is_enabled = false;
static bool is_connected = false;
static char rx_buffer[BUFFER_SIZE][DATA_SIZE];
static uint16_t write_idx = 0;
static uint16_t read_idx = 0;
static bool last_connection_state = false; // Track last connection state
static bool logged_empty_warning = false; // Track empty buffer warning

static void log_ethernet_buffer_status(const uint8_t *data, size_t len, uint16_t read_idx, uint16_t write_idx, bool is_full, bool is_empty) {
    // Log Ethernet buffer status
    // Step 1: Check if data is printable
    char data_str[DATA_SIZE * 3 + 1];
    bool is_printable = true;
    for (size_t i = 0; i < len; i++) {
        if (!isprint(data[i]) && !isspace(data[i])) {
            is_printable = false;
            break;
        }
    }
    // Step 2: Format data as string (printable or hex)
    if (is_printable) {
        snprintf(data_str, sizeof(data_str), "%.*s", (int)len, (const char *)data);
    } else {
        size_t pos = 0;
        for (size_t i = 0; i < len && pos < sizeof(data_str) - 3; i++) {
            pos += snprintf(data_str + pos, sizeof(data_str) - pos, "%02X ", data[i]);
        }
        if (pos > 0) data_str[pos - 1] = '\0';
    }
    // Step 3: Log buffer status with data, indices, and status
    const char *status = is_full ? "full" : (is_empty ? "empty" : "normal");
    ESP_LOGI(TAG, "Ethernet buffer: data=%s, read_idx=%u, write_idx=%u, status=%s", data_str, read_idx, write_idx, status);
}

static esp_err_t parse_ip_addr(const char *ip_str, uint8_t *ip_array) {
    // Parse IP address string to array
    // Step 1: Parse IP string into four integers
    int a, b, c, d;
    if (sscanf(ip_str, "%d.%d.%d.%d", &a, &b, &c, &d) != 4) {
        // Step 2: Validate parsing result
        ESP_LOGE(TAG, "Invalid IP format: %s", ip_str);
        return ESP_FAIL;
    }
    // Step 3: Store integers in array
    ip_array[0] = a;
    ip_array[1] = b;
    ip_array[2] = c;
    ip_array[3] = d;
    // Step 4: Return success
    return ESP_OK;
}

static esp_err_t parse_mac_addr(const char *mac_str, uint8_t *mac_array) {
    // Parse MAC address string to array
    // Step 1: Parse MAC string into six integers
    int a, b, c, d, e, f;
    if (sscanf(mac_str, "%x:%x:%x:%x:%x:%x", &a, &b, &c, &d, &e, &f) != 6) {
        // Step 2: Validate parsing result
        ESP_LOGE(TAG, "Invalid MAC format: %s", mac_str);
        return ESP_FAIL;
    }
    // Step 3: Store integers in array
    mac_array[0] = a;
    mac_array[1] = b;
    mac_array[2] = c;
    mac_array[3] = d;
    mac_array[4] = e;
    mac_array[5] = f;
    // Step 4: Return success
    return ESP_OK;
}

static esp_err_t w5100s_write_reg(uint16_t addr, uint8_t *data, size_t len) {
    // Write to W5100S register
    // Step 1: Acquire SPI mutex
    if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire SPI mutex");
        return ESP_ERR_TIMEOUT;
    }
    // Step 2: Configure SPI transaction
    spi_transaction_t t = {
        .flags = 0,
        .cmd = SPI_WRITE,
        .addr = addr,
        .length = len * 8,
        .tx_buffer = data,
        .rx_buffer = NULL
    };
    // Step 3: Transmit data
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    // Step 4: Release mutex
    xSemaphoreGive(spi_mutex);
    // Step 5: Log and return result
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t w5100s_read_reg(uint16_t addr, uint8_t *data, size_t len) {
    // Read from W5100S register
    // Step 1: Acquire SPI mutex
    if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire SPI mutex");
        return ESP_ERR_TIMEOUT;
    }
    // Step 2: Configure SPI transaction
    spi_transaction_t t = {
        .flags = 0,
        .cmd = SPI_READ,
        .addr = addr,
        .length = len * 8,
        .tx_buffer = NULL,
        .rx_buffer = data
    };
    // Step 3: Transmit and receive data
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    // Step 4: Release mutex
    xSemaphoreGive(spi_mutex);
    // Step 5: Log and return result
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t spi_init(void) {
    // Initialize SPI interface
    // Init for SPI_CS
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << SPI_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cs_conf));
    // Init for RXD
    gpio_config_t rx_conf = {
        .pin_bit_mask = (1ULL << RXD),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rx_conf));

    // Init for TXD
    gpio_config_t tx_conf = {
        .pin_bit_mask = (1ULL << TXD),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&tx_conf));
    // Init for SCLK
    gpio_config_t sck_conf = {
        .pin_bit_mask = (1ULL << SCLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&sck_conf));

    // Init for RESET
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_conf));

    // Step 1: Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = RXD,
        .mosi_io_num = TXD,
        .sclk_io_num = SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    // Step 2: Configure SPI device
    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .address_bits = 16,
        .dummy_bits = 0,
        .mode = 0,
        .clock_speed_hz = 10 * 1000 * 1000,
        .spics_io_num = SPI_CS,
        .queue_size = 7,
    };

    // Step 3: Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    // Step 4: Add SPI device
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));
    // Step 5: Log and return success
    ESP_LOGI(TAG, "SPI initialized");
    return ESP_OK;
}

static esp_err_t reset_init(void) {
    // Step 1: Perform reset sequence (high-low-high)
    ESP_ERROR_CHECK(gpio_set_level(RESET, 1));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(RESET, 0));
    esp_rom_delay_us(10);
    ESP_ERROR_CHECK(gpio_set_level(RESET, 1));
    vTaskDelay(pdMS_TO_TICKS(100));
    // Step 2: Log and return success
    ESP_LOGI(TAG, "Hardware reset complete");
    return ESP_OK;
}

static esp_err_t w5100s_init(void) {
    // Initialize W5100S chip
    // Step 1: Log initialization start
    esp_err_t ret;
    uint8_t data;
    uint8_t ip_array[4];
    uint8_t subnet_array[4];
    uint8_t mac_array[6];
    ESP_LOGI(TAG, "Initializing W5100S...");
    // Step 2: Perform hardware reset
    ret = reset_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reset failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // Step 3: Verify chip version
    ret = w5100s_read_reg(W5100S_VERR, &data, 1);
    if (ret != ESP_OK || data != 0x51) {
        ESP_LOGE(TAG, "Invalid chip version 0x%02X, expected 0x51", data);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "W5100S Version: 0x%02X", data);
    // Step 4: Unlock network registers
    data = 0x3A;
    ret = w5100s_write_reg(W5100S_NETLCKR, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unlock NETLCKR: %s", esp_err_to_name(ret));
        return ret;
    }
    // Step 5: Set MAC address
    ret = parse_mac_addr(ETH_MAC_ADDR, mac_array);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = w5100s_write_reg(W5100S_SHAR0, mac_array, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MAC: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Setting MAC: %s", ETH_MAC_ADDR);
    // Step 6: Set IP address
    ret = parse_ip_addr(ETH_IP_ADDR, ip_array);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = w5100s_write_reg(W5100S_SIPR0, ip_array, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set IP: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Setting IP: %s", ETH_IP_ADDR);
    // Step 7: Set subnet mask
    ret = parse_ip_addr(ETH_SUBNET_MASK, subnet_array);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = w5100s_write_reg(W5100S_SUBR0, subnet_array, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Subnet: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Setting Subnet: %s", ETH_SUBNET_MASK);
    // Step 8: Set port number
    data = (ETH_PORT >> 8) & 0xFF;
    ret = w5100s_write_reg(W5100S_S0_PORT0, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Port high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    data = ETH_PORT & 0xFF;
    ret = w5100s_write_reg(W5100S_S0_PORT0 + 1, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Port low byte: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Setting Port: %u", ETH_PORT);
    // Step 9: Lock network registers
    data = 0xC5;
    ret = w5100s_write_reg(W5100S_NETLCKR, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to lock NETLCKR: %s", esp_err_to_name(ret));
        return ret;
    }
    // Step 10: Return result
    return ESP_OK;
}

static void link_status_task(void *pvParameters) {
    // Monitor Ethernet link status
    // Step 1: Initialize previous status
    uint8_t prev_status = 0xFF;
    uint8_t data;
    // Step 2: Enter infinite loop
    while (1) {
        // Step 3: Read PHY status register
        if (w5100s_read_reg(W5100S_PHYSR0, &data, 1) == ESP_OK) {
            // Step 4: Log status change if detected
            uint8_t link = data & 0x01;
            if (link != prev_status) {
                ESP_LOGI(TAG, "Ethernet %s", link ? "Connected" : "Disconnected");
                prev_status = link;
            }
        } else {
            ESP_LOGW(TAG, "Failed to read PHY status");
        }
        // Step 5: Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

bool ethernet_buffer_full(void) {
    // Check if Ethernet buffer is full
    // Step 1: Check if next write index equals read index
    bool is_full = ((write_idx + 1) % BUFFER_SIZE) == read_idx;
    // Step 2: Log interface, status, and indices
    ESP_LOGI(TAG, "Interface: Ethernet, status: %s, read_idx=%u, write_idx=%u",
             is_full ? "full" : "not full", read_idx, write_idx);
    // Step 3: Return full status
    return is_full;
}

bool ethernet_buffer_empty(void) {
    // Check if Ethernet buffer is empty
    // Step 1: Check if write and read indices are equal
    bool is_empty = write_idx == read_idx;
    // Step 2: Log interface, status, and indices
    // ESP_LOGI(TAG, "Interface: Ethernet, status: %s, read_idx=%u, write_idx=%u",
    //          is_empty ? "empty" : "not empty", read_idx, write_idx);
    // Step 3: Return empty status
    return is_empty;
}

esp_err_t ethernet_buffer_write(const uint8_t *data, size_t len) {
    // Write data to Ethernet buffer
    // Step 1: Check if Ethernet is enabled
    if (!is_enabled) {
        ESP_LOGW(TAG, "Cannot send data: Ethernet not enabled");
        return ESP_FAIL;
    }
    // Step 2: Check if Ethernet is connected
    if (!ethernet_connected()) {
        ESP_LOGW(TAG, "Cannot send data: Ethernet not connected");
        return ESP_FAIL;
    }
    // Step 3: Check if buffer is full
    if (ethernet_buffer_full()) {
        ESP_LOGE(TAG, "Ethernet buffer full, cannot write %d bytes", len);
        return ESP_FAIL;
    }
    // Step 4: Filter and copy valid data
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
    // Step 5: Log write operation and buffer status
    ESP_LOGI(TAG, "Wrote %d bytes to buffer: %.*s", copy_len, copy_len, (char *)dst);
    log_ethernet_buffer_status((const uint8_t *)dst, copy_len, read_idx, write_idx, ethernet_buffer_full(), ethernet_buffer_empty());
    // Step 6: Update write index
    write_idx = (write_idx + 1) % BUFFER_SIZE;
    // Step 7: Return success
    return ESP_OK;
}

esp_err_t ethernet_buffer_read(uint8_t *data, size_t *len) {
    // Read data from Ethernet buffer
    // Step 1: Check if buffer is empty
    if (ethernet_buffer_empty()) {
        if (!logged_empty_warning) {
            ESP_LOGW(TAG, "Ethernet buffer empty, cannot read");
            logged_empty_warning = true;
        }
        return ESP_FAIL;
    }
    // Step 2: Copy data from buffer
    size_t buffer_len = strlen(rx_buffer[read_idx]);
    *len = buffer_len < *len ? buffer_len : *len;
    memcpy(data, rx_buffer[read_idx], *len);
    
    // Step 3: Update read index
    read_idx = (read_idx + 1) % BUFFER_SIZE;

    // Step 4: Log read operation and buffer status
    ESP_LOGI(TAG, "Read %d bytes from buffer: %.*s", *len, *len, (char *)data);
    log_ethernet_buffer_status(data, *len, read_idx, write_idx, ethernet_buffer_full(), ethernet_buffer_empty());
    // Step 5: Return success
    return ESP_OK;
}

static void tcp_server_task(void *pvParameters) {
    // TCP server task for Ethernet communication
    // Step 1: Log task start
    uint8_t data;
    esp_err_t ret;
    ESP_LOGI(TAG, "TCP server task started; %d buffers available", BUFFER_SIZE);
    // Step 2: Configure socket for TCP mode
    data = 0x01;
    ret = w5100s_write_reg(W5100S_S0_MR, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TCP mode: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    // Step 3: Open socket
    data = W5100S_S0_CR_OPEN;
    ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open socket: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = w5100s_read_reg(W5100S_S0_SR, &data, 1);
    if (ret != ESP_OK || data != W5100S_SOCK_INIT) {
        ESP_LOGE(TAG, "Socket not initialized, status: 0x%02X", data);
        goto cleanup;
    }
    // Step 4: Set socket to listen
    data = W5100S_S0_CR_LISTEN;
    ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to listen: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = w5100s_read_reg(W5100S_S0_SR, &data, 1);
    if (ret != ESP_OK || data != W5100S_SOCK_LISTEN) {
        ESP_LOGE(TAG, "Socket not listening, status: 0x%02X", data);
        goto cleanup;
    }
    ESP_LOGI(TAG, "TCP server listening on %s:%d", ETH_IP_ADDR, ETH_PORT);
    // Step 5: Enter main loop
    while (1) {
        // Step 6: Check socket status
        ret = w5100s_read_reg(W5100S_S0_SR, &data, 1);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read socket status: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (data == W5100S_SOCK_ESTABLISHED) {
            // Step 7: Handle established connection
            uint16_t rx_size;
            ret = w5100s_read_reg(W5100S_S0_RX_RSR0, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read RX size: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            rx_size = data << 8;
            ret = w5100s_read_reg(W5100S_S0_RX_RSR0 + 1, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read RX size low byte: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            rx_size |= data;
            if (rx_size > 0) {
                if (rx_size > DATA_SIZE - 1) rx_size = DATA_SIZE - 1;
                uint16_t rx_rd;
                ret = w5100s_read_reg(W5100S_S0_RX_RD0, &data, 1);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to read RX pointer: %s", esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
                rx_rd = data << 8;
                ret = w5100s_read_reg(W5100S_S0_RX_RD0 + 1, &data, 1);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to read RX pointer low byte: %s", esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
                rx_rd |= data;
                uint8_t temp_buffer[DATA_SIZE];
                ret = w5100s_read_reg(0x6000 + (rx_rd & 0x07FF), temp_buffer, rx_size);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to read RX buffer: %s", esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
                temp_buffer[rx_size] = '\0';
                ret = ethernet_buffer_write(temp_buffer, rx_size);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to write to Ethernet buffer: %s", esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
                ethernet_send_data(temp_buffer, rx_size);
                rx_rd += rx_size;
                data = (rx_rd >> 8) & 0xFF;
                ret = w5100s_write_reg(W5100S_S0_RX_RD0, &data, 1);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to write RX pointer: %s", esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
                data = rx_rd & 0xFF;
                ret = w5100s_write_reg(W5100S_S0_RX_RD0 + 1, &data, 1);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to write RX pointer low byte: %s", esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
                data = W5100S_S0_CR_RECV;
                ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to issue RECV command: %s", esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
            }
        } else if (data == W5100S_SOCK_CLOSE_WAIT) {
            // Step 8: Handle close-wait state
            ESP_LOGI(TAG, "Socket in CLOSE_WAIT, closing...");
            data = W5100S_S0_CR_CLOSE;
            ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to close socket: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            data = W5100S_S0_CR_OPEN;
            ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to re-open socket: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            ret = w5100s_read_reg(W5100S_S0_SR, &data, 1);
            if (ret != ESP_OK || data != W5100S_S0_CR_OPEN) {
                ESP_LOGW(TAG, "Socket not re-initialized, status: 0x%02X", data);
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            data = W5100S_S0_CR_LISTEN;
            ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to re-listen: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            ESP_LOGI(TAG, "Socket re-opened and listening");
        } else if (data == W5100S_SOCK_CLOSED) {
            // Step 9: Handle closed state
            ESP_LOGW(TAG, "Socket signed unexpectedly, reinitializing...");
            data = W5100S_S0_CR_CLOSE;
            ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to close socket: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            data = 0x01;
            ret = w5100s_write_reg(W5100S_S0_MR, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set TCP mode: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            data = W5100S_S0_CR_OPEN;
            ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to open socket: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            ret = w5100s_read_reg(W5100S_S0_SR, &data, 1);
            if (ret != ESP_OK || data != W5100S_SOCK_INIT) {
                ESP_LOGW(TAG, "Socket not initialized, status: 0x%02X", data);
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            data = W5100S_S0_CR_LISTEN;
            ret = w5100s_write_reg(W5100S_S0_CR, &data, 1);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to listen: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            ESP_LOGI(TAG, "Socket reinitialized and listening");
        }
        // Step 10: Delay for 10ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Step 11: Clean up on failure
cleanup:
    data = W5100S_S0_CR_CLOSE;
    w5100s_write_reg(W5100S_S0_CR, &data, 1);
    ESP_LOGE(TAG, "TCP server task terminated");
    vTaskDelete(NULL);
}

void ethernet_init(void) {
    // Initialize Ethernet module
    // Step 1: Create SPI mutex
    spi_mutex = xSemaphoreCreateMutex();
    if (spi_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        return;
    }
    // Step 2: Initialize SPI interface
    if (spi_init() != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed");
        return;
    }
    // Step 3: Initialize W5100S chip
    if (w5100s_init() != ESP_OK) {
        ESP_LOGE(TAG, "W5100S initialization failed");
        return;
    }
    // Step 4: Create TCP server and link status tasks
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(link_status_task, "link_status", 4096, NULL, 4, NULL);
}

void ethernet_reset(void) {
    // Reset Ethernet module
    // Step 1: Log reset start
    ESP_LOGI(TAG, "Resetting W5100S...");
    // Step 2: Perform hardware reset
    reset_init();
}

void ethernet_enable(void) {
    // Enable Ethernet module
    // Step 1: Set enabled flag
    is_enabled = true;
    // Step 2: Log enable status
    ESP_LOGI(TAG, "Ethernet enabled");
}

void ethernet_disable(void) {
    // Disable Ethernet module
    // Step 1: Clear enabled flag
    is_enabled = false;
    // Step 2: Log disable status
    ESP_LOGI(TAG, "Ethernet disabled");
}

bool ethernet_connected(void) {
    // Check Ethernet connection status
    // Step 1: Read PHY status register
    uint8_t data;
    if (w5100s_read_reg(W5100S_PHYSR0, &data, 1) == ESP_OK) {
        // Step 2: Update and log connection state
        bool current_state = (data & 0x01) == 0x01;
        if (current_state != last_connection_state) {
            ESP_LOGI(TAG, "Ethernet connection status: %s", current_state ? "Connected" : "Disconnected");
            last_connection_state = current_state;
        }
        is_connected = current_state;
    } else {
        // Step 3: Handle read failure
        ESP_LOGW(TAG, "Failed to read PHY status");
        is_connected = false;
    }
    // Step 4: Return connection status
    return is_connected;
}

void ethernet_send_data(const uint8_t *data, size_t len) {
    // Send data over Ethernet
    // Step 1: Check if Ethernet is enabled
    if (!is_enabled) {
        ESP_LOGW(TAG, "Cannot send data: Ethernet not enabled");
        return;
    }
    // Step 2: Check if Ethernet is connected
    if (!ethernet_connected()) {
        ESP_LOGW(TAG, "Cannot send data: Ethernet not connected");
        return;
    }
    // Step 3: Verify socket status
    uint8_t status;
    esp_err_t ret = w5100s_read_reg(W5100S_S0_SR, &status, 1);
    if (ret != ESP_OK || status != W5100S_SOCK_ESTABLISHED) {
        ESP_LOGW(TAG, "Cannot send data: Socket not established (status: 0x%02X)", status);
        return;
    }
    // Step 4: Check TX buffer space
    uint16_t tx_free;
    uint8_t temp_data;
    ret = w5100s_read_reg(W5100S_S0_TX_FSR0, &temp_data, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read TX free size high byte: %s", esp_err_to_name(ret));
        return;
    }
    tx_free = temp_data << 8;
    ret = w5100s_read_reg(W5100S_S0_TX_FSR0 + 1, &temp_data, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read TX free size low byte: %s", esp_err_to_name(ret));
        return;
    }
    tx_free |= temp_data;
    if (tx_free < len) {
        ESP_LOGW(TAG, "Insufficient TX buffer space: %d bytes available, %d needed", tx_free, len);
        return;
    }
    // Step 5: Get TX write pointer
    uint16_t tx_wr;
    ret = w5100s_read_reg(W5100S_S0_TX_WR0, &temp_data, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read TX pointer high byte: %s", esp_err_to_name(ret));
        return;
    }
    tx_wr = temp_data << 8;
    ret = w5100s_read_reg(W5100S_S0_TX_WR0 + 1, &temp_data, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read TX pointer low byte: %s", esp_err_to_name(ret));
        return;
    }
    tx_wr |= temp_data;
    // Step 6: Write data to TX buffer
    ret = w5100s_write_reg(0x4000 + (tx_wr & 0x07FF), (uint8_t *)data, len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write TX buffer: %s", esp_err_to_name(ret));
        return;
    }
    // Step 7: Update TX write pointer
    tx_wr += len;
    temp_data = (tx_wr >> 8) & 0xFF;
    ret = w5100s_write_reg(W5100S_S0_TX_WR0, &temp_data, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write TX pointer high byte: %s", esp_err_to_name(ret));
        return;
    }
    temp_data = tx_wr & 0xFF;
    ret = w5100s_write_reg(W5100S_S0_TX_WR0 + 1, &temp_data, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write TX pointer low byte: %s", esp_err_to_name(ret));
        return;
    }
    // Step 8: Issue send command
    temp_data = W5100S_S0_CR_SEND;
    ret = w5100s_write_reg(W5100S_S0_CR, &temp_data, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to issue SEND command: %s", esp_err_to_name(ret));
        return;
    }
    // Step 9: Log sent data
    ESP_LOGI(TAG, "Sent %d bytes: %.*s", len, len, (char *)data);
}