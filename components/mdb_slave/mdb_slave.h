#ifndef MDB_SLAVE_H
#define MDB_SLAVE_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

// MDB Commands
#define RESET           0x00
#define JUST_RESET      0x00
#define CONFIG_DATA     0x01

// MDB Protocol Constants
#define ACK_DATA        0x00
#define RET_DATA        0x55
#define NAK_DATA        0xFF

#define ACK             0x100
#define RET             0x155
#define NAK             0x1FF

// Bit masks
#define BIT_MODE_SET    0x100
#define BIT_ADD_SET     0xF8
#define BIT_CMD_SET     0x07

typedef enum {
    INACTIVE_STATE,   // Начальное состояние после RESET
    DISABLED_STATE,   // Устройство отключено
    ENABLED_STATE,    // Устройство включено и готово к работе
    IDLE_STATE,      // Устройство в режиме ожидания
    VEND_STATE       // Идет процесс продажи
} machine_state_t;

// Session message structure
typedef struct {
    uint8_t sequential;
    uint16_t fundsAvailable;
    uint16_t itemPrice;
    uint16_t itemNumber;
} mdb_session_msg_t;

// Function prototypes
void mdb_slave_init(gpio_num_t rx_pin, gpio_num_t tx_pin, gpio_num_t led_pin);
bool mdb_start_session(uint16_t funds_available);
void mdb_cancel_session(void);
machine_state_t mdb_get_state(void);
void mdb_register_vend_callback(void (*callback)(uint16_t item_number, uint16_t item_price));
void mdb_approve_vend(void);
void mdb_deny_vend(void);

#endif // MDB_SLAVE_H
