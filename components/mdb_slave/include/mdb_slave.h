/*
 * mdb_slave.h - MDB Cashless Device Protocol Implementation
 *
 * Based on MDB/ICP Version 4.2 specification
 */

#ifndef MDB_SLAVE_H
#define MDB_SLAVE_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

// MDB communication constants
#define ACK_DATA    0x00  // ACK data byte
#define RET_DATA    0xAA  // RET data byte (only VMC can send)
#define NAK_DATA    0xFF  // NAK data byte

// Для ответов периферии mode bit должен быть установлен
#define ACK     (ACK_DATA | BIT_MODE_SET)  // ACK с mode bit
#define RET     RET_DATA                    // RET только от VMC
#define NAK     (NAK_DATA | BIT_MODE_SET)  // NAK с mode bit

// Bit masks for MDB operations
#define BIT_MODE_SET     0b100000000
#define BIT_ADD_SET      0b011111000
#define BIT_CMD_SET      0b000000111

// Timeout for idle state (90 seconds in microseconds)
#define TIMEOUT_IDLE_US  (90 * 1000000ULL)

// Defining MDB commands
enum MDB_COMMAND {
    RESET = 0x00,
    SETUP = 0x01,
    POLL = 0x02,
    VEND = 0x03,
    READER = 0x04,
    EXPANSION = 0x07
};

// Defining MDB setup flow
enum MDB_SETUP_FLOW {
    CONFIG_DATA = 0x00,
    MAX_MIN_PRICES = 0x01
};

// Defining MDB vending flow
enum MDB_VEND_FLOW {
    VEND_REQUEST = 0x00,
    VEND_CANCEL = 0x01,
    VEND_SUCCESS = 0x02,
    VEND_FAILURE = 0x03,
    SESSION_COMPLETE = 0x04,
    CASH_SALE = 0x05
};

// Defining MDB reader flow
enum MDB_READER_FLOW {
    READER_DISABLE = 0x00,
    READER_ENABLE = 0x01,
    READER_CANCEL = 0x02
};

// Defining MDB expansion flow
enum MDB_EXPANSION_FLOW {
    REQUEST_ID = 0x00
};

// Defining machine states
typedef enum MACHINE_STATE {
    INACTIVE_STATE,
    DISABLED_STATE,
    ENABLED_STATE,
    IDLE_STATE,
    VEND_STATE
} machine_state_t;

// Structure for MDB session message
typedef struct {
    uint16_t sequential;
    uint16_t fundsAvailable;
    uint16_t itemPrice;
    uint16_t itemNumber;
} mdb_session_msg_t;

/**
 * @brief Initialize MDB slave interface
 * 
 * @param rx_pin GPIO pin for receiving data from MDB
 * @param tx_pin GPIO pin for transmitting data to MDB
 * @param led_pin GPIO pin for LED indicator
 */
void mdb_slave_init(gpio_num_t rx_pin, gpio_num_t tx_pin, gpio_num_t led_pin);

/**
 * @brief Start a new cashless session with specified funds
 * 
 * @param funds_available Amount of funds available for the session
 * @return true if session was successfully queued, false otherwise
 */
bool mdb_start_session(uint16_t funds_available);

/**
 * @brief Cancel current cashless session
 */
void mdb_cancel_session(void);

/**
 * @brief Get current machine state
 * 
 * @return Current machine state
 */
machine_state_t mdb_get_state(void);

/**
 * @brief Register callback for vend request events
 * 
 * @param callback Function to be called when vend is requested
 */
void mdb_register_vend_callback(void (*callback)(uint16_t item_number, uint16_t item_price));

/**
 * @brief Approve vend request
 */
void mdb_approve_vend(void);

/**
 * @brief Deny vend request
 */
void mdb_deny_vend(void);

#endif /* MDB_SLAVE_H */
