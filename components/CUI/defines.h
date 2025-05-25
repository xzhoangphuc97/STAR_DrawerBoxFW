#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>

// Buffer sizes for interface buffers
#define BUFFER_SIZE 64
#define DATA_SIZE 256
// Common buffer sizes
#define COMMON_BUFFER_SIZE 64
#define COMMON_DATA_SIZE 256

// Ethernet SPI pins for W5100S
#define SPI_CS GPIO_NUM_10
#define RXD GPIO_NUM_11   // MISO
#define SCLK GPIO_NUM_12  
#define TXD GPIO_NUM_13   // MOSI
#define INIT GPIO_NUM_14  // Input
#define RESET GPIO_NUM_15 // Output

// Hard-coded Ethernet settings
#define ETH_IP_ADDR "192.168.1.120"
#define ETH_SUBNET_MASK "255.255.255.0"
#define ETH_MAC_ADDR "00:08:DC:1D:43:0B"
#define ETH_PORT 12346
#define ETH_DHCP_ENABLE false

// Hard-coded Network configuration for Wi-Fi interface
#define WIFI_STA_SSID "HTBT"                 // STA mode(access point), Wi-Fi SSID
#define WIFI_STA_PASSWORD "98765abcde"       // STA mode(access point), Wi-Fi password (for WPA/WPA2/WPA3)
#define WIFI_STA_IP_ADDR "192.168.11.175"     // STA mode(access point), Static IP address for Wi-Fi
#define WIFI_STA_SUBNET_MASK "255.255.255.0" // STA mode(access point), Subnet mask for Wi-Fi
#define WIFI_STA_GATEWAY "192.168.1.1"       // STA mode(access point), Gateway address for Wi-Fi
#define WIFI_STA_TCP_PORT 12345
#define WIFI_DHCP_ENABLE false           // Enable (true, for DHCP) or disable (false, for STA) DHCP, this is Infrastructure mode

#define WIFI_AP_SSID "ESP_AP"                
#define WIFI_AP_PASSWORD "12345678"   
#define WIFI_AP_IP_ADDR "192.168.4.1"     
#define WIFI_AP_SUBNET_MASK "255.255.255.0" 
#define WIFI_AP_GATEWAY "192.168.4.1"       
#define WIFI_AP_TCP_PORT  12346

// Wi-Fi security types (values match wifi_auth_mode_t in esp_wifi_types.h)
#define WIFI_SECURITY_OPEN 0                     // Open (no security)
#define WIFI_SECURITY_WPA 1                      // WPA-Personal
#define WIFI_SECURITY_WPA2 2                     // WPA2-Personal
#define WIFI_SECURITY_WPA3 4                     // WPA3-Personal
#define WIFI_SECURITY_DEFAULT WIFI_SECURITY_WPA2 // Default security type

#endif