#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>
//------------------------------------------------------------------------------controller: user need to define here
#define VOLTAGE_THRESHOLD 23.5 // threshold run drawer
#define RealTimeASB 1 // 1 is auto realtime send, 0 is stop 
#define ONCountSW1 10  // number of consecutive series
#define OFFCountSW1 10 // number of consecutive series
#define ONCountSW2 10  // number of consecutive series
#define OFFCountSW2 10 // number of consecutive series

#define ONCountConnSensor1 10  // number of consecutive series
#define OFFCountConnSensor1 10 // number of consecutive series
#define ONCountConnSensor2 10  // number of consecutive series
#define OFFCountConnSensor2 10 // number of consecutive series
#define ONCountConnSensor3 10  // number of consecutive series
#define OFFCountConnSensor3 10 // number of consecutive series
#define ONCountConnSensor4 10  // number of consecutive series
#define OFFCountConnSensor4 10 // number of consecutive series

#define SW1Polarity 1 // 1 is HIGH, 0 is LOW
#define SW2Polarity 1 // 1 is HIGH, 0 is LOW

#define Drawer1Polarity 1 // 1 is HIGH, 0 is LOW
#define Drawer2Polarity 1 // 1 is HIGH, 0 is LOW
#define Drawer3Polarity 1 // 1 is HIGH, 0 is LOW
#define Drawer4Polarity 1 // 1 is HIGH, 0 is LOW

#define TaskDelayTime 500
//------------------------------------------------------------------------------controller: user end to define here
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
#define ETH_DHCP_ENABLE false   // Not Support Yes. Not able use the lib(https://github.com/Wiznet/ioLibrary_Driver) for implement because it has a lot of erorrs, So I implemented the DHCP algorithm using SPI and access W5100s Registers but not success yet.

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


// Wi-Fi country code
/* Able to specify the following strings
    "01"(world safe mode) 
    "AT","AU","BE","BG","BR","CA","CH","CN","CY","CZ","DE",
    "DK","EE","ES","FI","FR","GB","GR","HK","HR","HU","IE",
    "IN","IS","IT","JP","KR","LI","LT","LU","LV","MT","MX",
    "NL","NO","NZ","PL","PT","RO","SE","SI","SK","TW","US"
*/
#define WIFI_COUNTRY_CODE   "US"
#define WIFI_ieee80211d     true        // 接続したAPの国別コードに従うかどうか

// Wi-Fi max_tx_power
#define WIFI_MAX_TX_POWER   (74)        // 18.5 [dBm] x 4

#endif