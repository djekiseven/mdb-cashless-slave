/*
 * mdb_protocol.c - Basic MDB protocol operations
 */

#include <rom/ets_sys.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdb_slave.h"
#include "mdb_protocol.h"

static const char *TAG = "mdb_protocol";

// GPIO pins for MDB communication
static gpio_num_t pin_mdb_rx;
static gpio_num_t pin_mdb_tx;
static gpio_num_t pin_mdb_led;

void mdb_protocol_init(gpio_num_t rx_pin, gpio_num_t tx_pin, gpio_num_t led_pin)
{
    pin_mdb_rx = rx_pin;
    pin_mdb_tx = tx_pin;
    pin_mdb_led = led_pin;
    
    // Configure GPIO pins
    gpio_reset_pin(pin_mdb_rx);
    gpio_reset_pin(pin_mdb_tx);
    gpio_reset_pin(pin_mdb_led);
    
    gpio_set_direction(pin_mdb_rx, GPIO_MODE_INPUT);
    gpio_set_direction(pin_mdb_tx, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_mdb_led, GPIO_MODE_OUTPUT);
    
    // Set initial state (idle = 0 for MDB)
    gpio_set_level(pin_mdb_tx, 0);
    gpio_set_level(pin_mdb_led, 0);
    
    ESP_LOGI(TAG, "MDB protocol initialized on pins RX:%d, TX:%d, LED:%d", rx_pin, tx_pin, led_pin);
}

uint16_t mdb_read_9(uint8_t *checksum)
{
    uint16_t coming_read = 0;
    int64_t start_time = esp_timer_get_time();
    const int64_t timeout_us = 1000000; // 1 second timeout

    // Wait for start bit (transition from idle 0 to 1)
    while (gpio_get_level(pin_mdb_rx) == 0) {
        if (esp_timer_get_time() - start_time > timeout_us) {
            ESP_LOGW(TAG, "MDB read timeout waiting for start bit");
            return 0xFFFF; // Return error code
        }
        vTaskDelay(1); // Yield to other tasks
    }

    // Wait for half bit time to sample in the middle of the start bit
    ets_delay_us(52); // Half of 104us

    // Verify we're still in the start bit (should still be 1)
    if (gpio_get_level(pin_mdb_rx) == 0) {
        ESP_LOGW(TAG, "MDB false start bit detected");
        return 0xFFFF;
    }

    // Wait for the rest of the start bit
    ets_delay_us(52);

    // Read 9 bits (8 data + 1 mode) - LSB first
    for (uint8_t x = 0; x < 9; x++) {
        // Sample in the middle of the bit
        ets_delay_us(52);
        coming_read |= (gpio_get_level(pin_mdb_rx) << x);
        ets_delay_us(52);
    }

    if (checksum)
        *checksum += coming_read;

    return coming_read;
}

void mdb_write_9(uint16_t nth9)
{
    // Start bit (1)
    gpio_set_level(pin_mdb_tx, 1);
    ets_delay_us(104);

    // Write 9 bits (8 data + 1 mode) - LSB first
    for (uint8_t x = 0; x < 9; x++) {
        gpio_set_level(pin_mdb_tx, (nth9 >> x) & 1);
        ets_delay_us(104); // 9600bps timing
    }

    // Return to idle state (0)
    gpio_set_level(pin_mdb_tx, 0);
    ets_delay_us(104); // Extra stop bit time for safety
}

void mdb_write_payload(uint8_t *mdb_payload, uint8_t length)
{
    uint8_t checksum = 0;

    // Calculate checksum
    for (int x = 0; x < length; x++) {
        checksum += mdb_payload[x];
        mdb_write_9(mdb_payload[x]);
    }

    // Send checksum with mode bit set
    mdb_write_9(BIT_MODE_SET | checksum);
}

void mdb_set_led(bool state)
{
    gpio_set_level(pin_mdb_led, state);
}
