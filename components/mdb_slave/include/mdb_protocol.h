/*
 * mdb_protocol.h - Basic MDB protocol operations
 */

#ifndef MDB_PROTOCOL_H
#define MDB_PROTOCOL_H

#include <stdbool.h>
#include "driver/gpio.h"

/**
 * @brief Initialize MDB protocol hardware
 * 
 * @param rx_pin GPIO pin for receiving data from MDB
 * @param tx_pin GPIO pin for transmitting data to MDB
 * @param led_pin GPIO pin for LED indicator
 */
void mdb_protocol_init(gpio_num_t rx_pin, gpio_num_t tx_pin, gpio_num_t led_pin);

/**
 * @brief Read 9 bits from MDB bus
 * 
 * @param checksum Pointer to checksum variable (will be updated if not NULL)
 * @return uint16_t The 9-bit value read from the bus
 */
uint16_t mdb_read_9(uint8_t *checksum);

/**
 * @brief Write 9 bits to MDB bus
 * 
 * @param nth9 The 9-bit value to write
 */
void mdb_write_9(uint16_t nth9);

/**
 * @brief Write a payload to MDB bus with checksum
 * 
 * @param mdb_payload Pointer to payload buffer
 * @param length Length of payload
 */
void mdb_write_payload(uint8_t *mdb_payload, uint8_t length);

/**
 * @brief Set MDB LED state
 * 
 * @param state LED state (true = on, false = off)
 */
void mdb_set_led(bool state);

#endif /* MDB_PROTOCOL_H */
