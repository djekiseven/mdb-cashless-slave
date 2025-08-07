/*
 * mdb_protocol.c - Basic MDB protocol operations
 */

#include <rom/ets_sys.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdb_slave.h"
#include "mdb_protocol.h"
#include "driver/gpio.h"

// Макросы для работы с GPIO для MDB
#define UART_GPIO_SET(pin, level) gpio_set_level(pin, !(level))
#define UART_GPIO_GET(pin) (!gpio_get_level(pin))

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
    
    // Initialize GPIO pins
    gpio_reset_pin(pin_mdb_rx);
    gpio_reset_pin(pin_mdb_tx);
    gpio_reset_pin(pin_mdb_led);
    
    gpio_set_direction(pin_mdb_rx, GPIO_MODE_INPUT);
    gpio_set_direction(pin_mdb_tx, GPIO_MODE_OUTPUT_OD);  // Open-drain output
    gpio_set_direction(pin_mdb_led, GPIO_MODE_OUTPUT);

    // Configure pull-up/down
    gpio_set_pull_mode(pin_mdb_rx, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(pin_mdb_tx, GPIO_FLOATING);       // No internal pull-up
    gpio_set_pull_mode(pin_mdb_led, GPIO_FLOATING);

    // Set initial pin states
    UART_GPIO_SET(pin_mdb_tx, 1);  // High = floating (pulled up externally)
    UART_GPIO_SET(pin_mdb_led, 0);
    
    // Проверка начального состояния пинов
    int tx_level = UART_GPIO_GET(pin_mdb_tx);
    ESP_LOGI(TAG, "MDB protocol initialized on pins RX:%d, TX:%d, LED:%d", rx_pin, tx_pin, led_pin);
    ESP_LOGI(TAG, "TX pin level: %d", tx_level);
}

uint16_t mdb_read_9(uint8_t *checksum)
{
    uint16_t coming_read = 0;
    ESP_LOGI(TAG, "Waiting for start bit...");

    // Ждем idle 
    while (!UART_GPIO_GET(pin_mdb_rx)) {
        ets_delay_us(10);
    }

    // Ждем start bit 
    while (UART_GPIO_GET(pin_mdb_rx)) {
        ets_delay_us(10);
    }
    ESP_LOGI(TAG, "Start bit detected");

    ets_delay_us(156); // Delay between bits

    // Читаем 9 бит
    ESP_LOGI(TAG, "Reading bits:");
    uint16_t mode_bit = 0;
    uint8_t data_byte = 0;

    // Сначала читаем 8 бит данных (MSB first)
    for (uint8_t x = 0; x < 8; x++) {
        int bit_value = UART_GPIO_GET(pin_mdb_rx);
        data_byte |= (bit_value << (7 - x));  // MSB first
        ESP_LOGI(TAG, "  Data Bit %d: %d", 7-x, bit_value);
        ets_delay_us(104);
    }

    // Затем читаем mode bit
    mode_bit = UART_GPIO_GET(pin_mdb_rx); 
    ESP_LOGI(TAG, "  Mode Bit: %d", mode_bit);
    ets_delay_us(104);

    // Собираем итоговое значение: mode bit в 9-м бите
    coming_read = data_byte | (mode_bit << 8);
    ESP_LOGI(TAG, "Final value: 0x%03X (Data: 0x%02X, Mode: %d)", coming_read, data_byte, mode_bit);

    if (checksum)
        *checksum += coming_read;

    return coming_read;
}

void mdb_write_9(uint16_t nth9)
{
    uint8_t data_byte = nth9 & 0xFF;
    uint8_t mode_bit = (nth9 >> 8) & 1;

    ESP_LOGW(TAG, "Writing value: 0x%03X (Data: 0x%02X [0b%d%d%d%d%d%d%d%d], Mode: %d)",
             nth9, data_byte,
             (data_byte >> 7) & 1,
             (data_byte >> 6) & 1,
             (data_byte >> 5) & 1,
             (data_byte >> 4) & 1,
             (data_byte >> 3) & 1,
             (data_byte >> 2) & 1,
             (data_byte >> 1) & 1,
             data_byte & 1,
             mode_bit);



    // Start bit 
    UART_GPIO_SET(pin_mdb_tx, 0); 
    ets_delay_us(104);

    // Отправляем 8 бит данных (MSB first)
    for (uint8_t x = 0; x < 8; x++) {
        int bit_value = (data_byte >> (7 - x)) & 1;  // MSB first
        UART_GPIO_SET(pin_mdb_tx, bit_value);
        ets_delay_us(104);
    }

    // Mode bit
    UART_GPIO_SET(pin_mdb_tx, mode_bit);
    ets_delay_us(104);

    // Stop bit
    UART_GPIO_SET(pin_mdb_tx, 1);
    ets_delay_us(104);

    // Возврат в idle
    UART_GPIO_SET(pin_mdb_tx, 0);
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
    UART_GPIO_SET(pin_mdb_led, state);
}
