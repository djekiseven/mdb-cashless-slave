#ifndef MDB_PROTOCOL_H
#define MDB_PROTOCOL_H

#include "driver/gpio.h"

// MDB Protocol timing constants (in microseconds)
#define MDB_BIT_TIME_US        104   // 9600 bps
#define MDB_RESPONSE_TIMEOUT_US 5000000 // 5s timeout

// Function prototypes
void mdb_protocol_init(gpio_num_t rx_pin, gpio_num_t tx_pin, gpio_num_t led_pin);
uint16_t mdb_read_9(uint8_t *checksum);
void mdb_write_9(uint16_t nth9);
void mdb_write_payload(uint8_t *mdb_payload, uint8_t length);
void mdb_set_led(bool state);

#endif // MDB_PROTOCOL_H
