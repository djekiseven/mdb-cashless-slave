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
    ESP_LOGI(TAG, "Waiting for start bit...");

    // Ждем idle (физическая 1)
    while (!gpio_get_level(pin_mdb_rx)) {
        ets_delay_us(10);
    }

    // Ждем start bit (переход 1->0)
    while (gpio_get_level(pin_mdb_rx)) {
        ets_delay_us(10);
    }
    ESP_LOGI(TAG, "Start bit detected");

    ets_delay_us(156); // Delay between bits

    // Читаем 9 бит
    ESP_LOGI(TAG, "Reading bits:");
    for (uint8_t x = 0; x < 9; x++) {
        int pin_level = gpio_get_level(pin_mdb_rx);
        int bit_value = !pin_level;  // Инвертируем: физический 0 -> логическая 1
        coming_read |= (bit_value << x);  // LSB first
        ESP_LOGI(TAG, "  Bit %d: Physical:%d Logical:%d (after inversion)", x, pin_level, bit_value);
        ets_delay_us(104); // 9600bps timing
    }
    ESP_LOGI(TAG, "Final value: 0x%03X", coming_read);

    if (checksum)
        *checksum += coming_read;

    return coming_read;
}

void mdb_write_9(uint16_t nth9)
{
    ESP_LOGI(TAG, "Writing value: 0x%03X", nth9);
    ESP_LOGI(TAG, "Start bit (physical 0)");
    gpio_set_level(pin_mdb_tx, 0); // Start bit
    ets_delay_us(104);

    // Отправляем 9 бит
    ESP_LOGI(TAG, "Writing bits:");
    for (uint8_t x = 0; x < 9; x++) {
        int bit = (nth9 >> x) & 1;
        int physical_level = !bit;  // Инвертируем: логическая 1 -> физический 0
        ESP_LOGI(TAG, "  Bit %d: Logical:%d Physical:%d (after inversion)", x, bit, physical_level);
        gpio_set_level(pin_mdb_tx, physical_level);
        ets_delay_us(104); // 9600bps timing
    }

    ESP_LOGI(TAG, "Stop bit (physical 1)");
    gpio_set_level(pin_mdb_tx, 1); // Stop bit
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
