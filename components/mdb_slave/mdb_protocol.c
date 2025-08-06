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
    
    // Enable pull-up for RX pin (idle is physical 1)
    gpio_set_pull_mode(pin_mdb_rx, GPIO_PULLUP_ONLY);
    
    // Check initial pin states
    ESP_LOGI(TAG, "Initial pin states - RX:%d, TX:%d, LED:%d", 
             gpio_get_level(pin_mdb_rx),
             gpio_get_level(pin_mdb_tx),
             gpio_get_level(pin_mdb_led));
    
    // Set initial state - TX idle high (logical 1 = physical 1)
    gpio_set_level(pin_mdb_tx, 1);
    gpio_set_level(pin_mdb_led, 0);
    
    ESP_LOGI(TAG, "MDB protocol initialized on pins RX:%d, TX:%d, LED:%d", rx_pin, tx_pin, led_pin);
}

uint16_t mdb_read_9(uint8_t *checksum)
{
    uint16_t coming_read = 0;
    int64_t start_time = esp_timer_get_time();
    const int64_t timeout_us = 1000000; // 1 second timeout

    // Ищем start bit (переход 1->0)
    // Wait for falling edge (start bit) with timeout
    int prev_level = gpio_get_level(pin_mdb_rx);
    int curr_level = prev_level; // Initialize with current pin state
    sample_count = 0; // Переиспользуем существующую переменную
    bool edge_found = false;

    while (!edge_found && (esp_timer_get_time() - start_time <= timeout_us)) {
        curr_level = gpio_get_level(pin_mdb_rx);
        if (prev_level == 1 && curr_level == 0) {
            edge_found = true;
            ESP_LOGD(TAG, "Found start bit after %d samples", sample_count);
            break;
        }
        prev_level = curr_level;
        sample_count++;

        if (sample_count % 10000 == 0) {
            ESP_LOGD(TAG, "Waiting for rising edge, level: %d, samples: %d", curr_level, sample_count);
        }
        ets_delay_us(1);
    }

    if (!edge_found) {
        ESP_LOGW(TAG, "Timeout waiting for falling edge, last level: %d, samples: %d", curr_level, sample_count);
        return 0xFFFF;
    }

    // Wait for half of the bit time to sample in the middle of the start bit
    ets_delay_us(52); // Half of 104us (9600bps timing)

    ESP_LOGD(TAG, "Reading 9 bits after start bit");
    for (uint8_t x = 0; x < 9 /*9bits*/; x++) {
        int pin_level = gpio_get_level(pin_mdb_rx);
        int bit_value = pin_level;  // Не инвертируем - физический уровень = логическому
        coming_read |= (bit_value << x);
        ESP_LOGD(TAG, "Bit %d: pin=%d, value=%d", x, pin_level, bit_value);
        ets_delay_us(104); // 9600bps timing
    }
    ESP_LOGI(TAG, "Read complete: 0x%03X", coming_read);

    if (checksum)
        *checksum += coming_read;

    return coming_read;
}

void mdb_write_9(uint16_t nth9)
{
    ESP_LOGI(TAG, "Writing 9-bit value: 0x%03X", nth9);

    gpio_set_level(pin_mdb_tx, 0); // Start bit (logical 0 = physical 0)
    ets_delay_us(104);

    for (uint8_t x = 0; x < 9 /*9bits*/; x++) {
        int bit = (nth9 >> x) & 1;
        gpio_set_level(pin_mdb_tx, bit); // Не инвертируем - физический уровень = логическому
        ESP_LOGD(TAG, "TX bit %d: %d", x, bit);
        ets_delay_us(104); // 9600bps timing
    }

    gpio_set_level(pin_mdb_tx, 1); // Stop bit (logical 1 = physical 1)
    ets_delay_us(104);

    ESP_LOGI(TAG, "Finished writing 9-bit value");
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
