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
    // Reset and configure pins
    gpio_reset_pin(pin_mdb_rx);
    gpio_reset_pin(pin_mdb_tx);
    gpio_reset_pin(pin_mdb_led);
    
    // Configure pin directions
    gpio_set_direction(pin_mdb_rx, GPIO_MODE_INPUT);
    gpio_set_direction(pin_mdb_tx, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_mdb_led, GPIO_MODE_OUTPUT);
    
    // Enable pull-up for RX pin to ensure it reads 1 when no signal (inactive state)
    gpio_set_pull_mode(pin_mdb_rx, GPIO_PULLUP_ONLY);
    
    // Check initial pin states
    ESP_LOGI(TAG, "Initial pin states - RX:%d, TX:%d, LED:%d", 
             gpio_get_level(pin_mdb_rx),
             gpio_get_level(pin_mdb_tx),
             gpio_get_level(pin_mdb_led));
    
    // Set initial state
    gpio_set_level(pin_mdb_tx, 1);
    gpio_set_level(pin_mdb_led, 0);
    
    ESP_LOGI(TAG, "MDB protocol initialized on pins RX:%d, TX:%d, LED:%d", rx_pin, tx_pin, led_pin);
}

uint16_t mdb_read_9(uint8_t *checksum)
{
    uint16_t coming_read = 0;
    int64_t start_time = esp_timer_get_time();
    const int64_t timeout_us = 1000000; // 1 second timeout

    // Wait until the RX signal is 0 with timeout
    int rx_level;
    while ((rx_level = gpio_get_level(pin_mdb_rx))) {
        if (esp_timer_get_time() - start_time > timeout_us) {
            ESP_LOGW(TAG, "MDB read timeout waiting for RX=0, current level: %d", rx_level);
            return 0xFFFF; // Return error code
        }
        ESP_LOGD(TAG, "RX pin level: %d", rx_level);
        vTaskDelay(1); // Yield to other tasks
    }
    ESP_LOGD(TAG, "RX pin went low, level: %d", rx_level);

    ets_delay_us(156); // Delay between bits

    for (uint8_t x = 0; x < 9 /*9bits*/; x++) {
        coming_read |= (gpio_get_level(pin_mdb_rx) << x);
        ets_delay_us(104); // 9600bps timing
    }

    if (checksum)
        *checksum += coming_read;

    return coming_read;
}

void mdb_write_9(uint16_t nth9)
{
    gpio_set_level(pin_mdb_tx, 0); // Start transmission
    ets_delay_us(104);

    for (uint8_t x = 0; x < 9 /*9bits*/; x++) {
        gpio_set_level(pin_mdb_tx, (nth9 >> x) & 1);
        ets_delay_us(104); // 9600bps timing
    }

    gpio_set_level(pin_mdb_tx, 1); // End transmission
    ets_delay_us(104);
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
