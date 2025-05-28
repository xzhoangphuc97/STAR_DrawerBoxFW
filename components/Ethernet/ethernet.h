#ifndef ETHERNET_H
#define ETHERNET_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include "../defines.h"

// W5100S Register Addresses and Command Values
#define W5100S_SIPR0     0x000F  // Source IP Address
#define W5100S_VERR      0x0080  // Version Register
#define W5100S_PHYSR0    0x003C  // PHY Status Register
#define W5100S_S0_PORT0  0x0404  // Socket 0 Source Port
#define W5100S_S0_MR     0x0400  // Socket 0 Mode Register
#define W5100S_S0_CR     0x0401  // Socket 0 Command Register
#define W5100S_S0_SR     0x0403  // Socket 0 Status Register
#define W5100S_S0_RX_RSR0 0x0426 // Socket 0 RX Received Size
#define W5100S_S0_RX_RD0 0x0428  // Socket 0 RX Read Pointer
#define W5100S_S0_TX_FSR0 0x0420 // Socket 0 TX Free Size
#define W5100S_S0_TX_WR0 0x0424  // Socket 0 TX Write Pointer
#define W5100S_SHAR0     0x0009  // Source Hardware Address (MAC)
#define W5100S_SUBR0     0x0005  // Subnet Mask
#define W5100S_NETLCKR   0x0071  // Network Lock Register

// W5100S Commands
#define W5100S_S0_CR_OPEN   0x01
#define W5100S_S0_CR_LISTEN 0x02
#define W5100S_S0_CR_SEND   0x20
#define W5100S_S0_CR_RECV   0x40
#define W5100S_S0_CR_CLOSE  0x10

// W5100S Status
#define W5100S_SOCK_INIT        0x13
#define W5100S_SOCK_LISTEN      0x14
#define W5100S_SOCK_ESTABLISHED 0x17
#define W5100S_SOCK_CLOSE_WAIT  0x1C
#define W5100S_SOCK_CLOSED      0x00

// SPI Control Phase Commands
#define SPI_WRITE 0xF0
#define SPI_READ  0x0F

void ethernet_init(void);
void ethernet_enable(void);
void ethernet_disable(void);
void ethernet_reset(void);
void ethernet_send_data(const uint8_t *data, size_t len);
bool ethernet_connected(void);
bool ethernet_buffer_full(void);
bool ethernet_buffer_empty(void);
esp_err_t ethernet_buffer_write(const uint8_t *data, size_t len);
esp_err_t ethernet_buffer_read(uint8_t *data, size_t *len);

#endif // ETHERNET_H