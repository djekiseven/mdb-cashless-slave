/*
 * mdb_protocol.c - Basic MDB protocol operations
 */

#include <rom/ets_sys.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdb_slave.h"
#include "mdb_protocol.h"
#include "driver/gpio.h"

// Макросы для работы с GPIO для MDB
#define UART_GPIO_SET(pin, level) gpio_set_level(pin, level)
#define UART_GPIO_GET(pin) gpio_get_level(pin)

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
        
    gpio_set_direction(pin_mdb_rx, GPIO_MODE_INPUT);
    gpio_set_direction(pin_mdb_tx, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_mdb_led, GPIO_MODE_OUTPUT);

    gpio_set_pull_mode(pin_mdb_rx, GPIO_PULLUP_ONLY);
    UART_GPIO_SET(pin_mdb_tx, 0);  // TX idle = 1 (low level for inverted UART)
        
    ESP_LOGI(TAG, "MDB protocol initialized on pins RX:%d, TX:%d, LED:%d", rx_pin, tx_pin, led_pin);
}

uint16_t mdb_read_9(uint8_t *checksum)
{
    uint16_t coming_read = 0;
    
    // Время последнего изменения состояния линии
    int64_t last_change_time = esp_timer_get_time();
    int last_state = UART_GPIO_GET(pin_mdb_rx);
    uint8_t bits_read = 0;
    uint16_t temp_data = 0;
    
    while (1) {
        int current_state = UART_GPIO_GET(pin_mdb_rx);
        int64_t current_time = esp_timer_get_time();
        
        // Проверяем на сброс шины
        if (current_state == last_state) {
            int64_t stable_duration = current_time - last_change_time;
            if (stable_duration >= 100000) {
                ESP_LOGW(TAG, "Bus RESET detected! Line held %s for %lld us", 
                        last_state ? "HIGH" : "LOW", stable_duration);
                return BUS_RESET;
            }
        } else {
            // Состояние изменилось
            last_change_time = current_time;
            last_state = current_state;
            
            // Если обнаружен start bit (0)
            if (current_state == 0 && bits_read == 0) {
                break;  // Выходим для чтения данных
            }
        }
        
        vTaskDelay(1); // Даем другим задачам шанс выполниться
    }

    ets_delay_us(156); // Delay between bits

    // Читаем 9 бит
    for (uint8_t x = 0; x < 9; x++) {
        int bit_value = !UART_GPIO_GET(pin_mdb_rx);  // Инвертируем биты из-за физической инверсии UART
        coming_read |= (bit_value << x);
        ESP_LOGI(TAG, "  Bit %d: %d", x, bit_value);
        ets_delay_us(104); // 9600bps timing
    }

    ESP_LOGI(TAG, "Final value: 0x%03X (Data: 0x%02X, Mode: %d)", 
             coming_read, coming_read & 0xFF, (coming_read >> 8) & 1);

    return coming_read;
}

void mdb_write_9(uint16_t nth9)
{
    ESP_LOGI(TAG, "Writing value: 0x%03X (Data: 0x%02X, Mode: %d)",
             nth9, nth9 & 0xFF, (nth9 >> 8) & 1);
    ESP_LOGW(TAG, "Writing bits:");
    UART_GPIO_SET(pin_mdb_tx, 0); // Start bit = 0
    ESP_LOGW(TAG, "  Start bit: 0");
    ets_delay_us(104);
    for (uint8_t x = 0; x < 9; x++) {
        bool bit = !((nth9 >> x) & 1);
        UART_GPIO_SET(pin_mdb_tx, bit);
        ESP_LOGW(TAG, "  Bit %d: %d", x, bit);
        ets_delay_us(104); // 9600bps timing
    }

    UART_GPIO_SET(pin_mdb_tx, 1); // Stop bit = 1
    ESP_LOGW(TAG, "  Stop bit: 1");
    ESP_LOGW(TAG, "  Stop bit: 1");
    ets_delay_us(104);
}

void mdb_write_payload(uint8_t *mdb_payload, uint8_t length)
{
    if (!mdb_payload || length == 0) {
        ESP_LOGE(TAG, "Invalid payload or length");
        return;
    }
    
    uint8_t checksum = 0;
    
    // Сначала отправляем все данные без задержек на логирование
    for (int i = 0; i < length; i++) {
        checksum += mdb_payload[i];
        mdb_write_9(mdb_payload[i]);
        ets_delay_us(MDB_BIT_TIME_US);
    }

    // Send checksum with mode bit set (CHK* ACK*)
    mdb_write_9(BIT_MODE_SET | checksum);
    
    // Логируем после отправки
    ESP_LOGI(TAG, "Wrote payload of length %d:", length);
    for (int i = 0; i < length; i++) {
        ESP_LOGI(TAG, "[WRITE] Byte[%d]: 0x%02X", i, mdb_payload[i]);
    }
    ESP_LOGI(TAG, "[WRITE] Checksum: 0x%02X", checksum);
}

void mdb_set_led(bool state)
{
    UART_GPIO_SET(pin_mdb_led, state);
}
