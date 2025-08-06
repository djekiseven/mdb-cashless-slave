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
    
    // Настройка GPIO пинов
    // Сброс и конфигурация пинов
    gpio_reset_pin(pin_mdb_rx);
    gpio_reset_pin(pin_mdb_tx);
    gpio_reset_pin(pin_mdb_led);
    
    // Настройка направления пинов
    gpio_config_t rx_conf = {
        .pin_bit_mask = (1ULL << pin_mdb_rx),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    gpio_config_t tx_conf = {
        .pin_bit_mask = (1ULL << pin_mdb_tx),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << pin_mdb_led),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    gpio_config(&rx_conf);
    gpio_config(&tx_conf);
    gpio_config(&led_conf);
    
    // Проверка начального состояния пинов
    ESP_LOGI(TAG, "Initial pin states - RX:%d, TX:%d, LED:%d", 
             gpio_get_level(pin_mdb_rx),
             gpio_get_level(pin_mdb_tx),
             gpio_get_level(pin_mdb_led));
    
    // Установка начального состояния - TX idle high (physical 1)
    gpio_set_level(pin_mdb_tx, 0);
    ets_delay_us(100);
    gpio_set_level(pin_mdb_tx, 1);
    gpio_set_level(pin_mdb_led, 0);
    
    // Проверяем что TX действительно в high
    int tx_level = gpio_get_level(pin_mdb_tx);
    ESP_LOGI(TAG, "MDB protocol initialized on pins RX:%d, TX:%d, LED:%d", rx_pin, tx_pin, led_pin);
    ESP_LOGI(TAG, "TX pin set to idle (physical 1), current level: %d", tx_level);
    
    // Если TX не в high, пробуем еще раз
    if (tx_level == 0) {
        ESP_LOGW(TAG, "TX pin not in idle state, retrying...");
        gpio_set_level(pin_mdb_tx, 0);
        ets_delay_us(100);
        gpio_set_level(pin_mdb_tx, 1);
        tx_level = gpio_get_level(pin_mdb_tx);
        ESP_LOGI(TAG, "TX pin level after retry: %d", tx_level);
    }
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
    uint16_t mode_bit = 0;
    uint8_t data_byte = 0;

    // Сначала читаем 8 бит данных (MSB first)
    for (uint8_t x = 0; x < 8; x++) {
        int pin_level = gpio_get_level(pin_mdb_rx);
        int bit_value = !pin_level;  // Инвертируем: физический 0 -> логическая 1
        data_byte |= (bit_value << (7 - x));  // MSB first
        ESP_LOGI(TAG, "  Data Bit %d: Physical:%d Logical:%d (after inversion)", 7-x, pin_level, bit_value);
        ets_delay_us(104);
    }

    // Затем читаем mode bit
    int pin_level = gpio_get_level(pin_mdb_rx);
    mode_bit = !pin_level;  // Инвертируем
    ESP_LOGI(TAG, "  Mode Bit: Physical:%d Logical:%d (after inversion)", pin_level, mode_bit);
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

    ESP_LOGI(TAG, "Writing value: 0x%03X (Data: 0x%02X [0b%d%d%d%d%d%d%d%d], Mode: %d)",
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

    // Перед началом передачи убедимся что мы в idle (physical 1)
    ESP_LOGI(TAG, "Ensuring TX line is in idle state (physical 1)");
    
    // Пробуем переключить TX линию в high несколько раз
    for (int i = 0; i < 3; i++) {
        gpio_set_level(pin_mdb_tx, 0);
        ets_delay_us(100);
        gpio_set_level(pin_mdb_tx, 1);
        ets_delay_us(100);
        
        int tx_level = gpio_get_level(pin_mdb_tx);
        ESP_LOGI(TAG, "TX pin level (attempt %d): %d", i+1, tx_level);
        
        if (tx_level == 1) {
            break;
        }
        
        if (i == 2) {
            ESP_LOGE(TAG, "Failed to set TX line to idle state after 3 attempts!");
        }
    }
    
    ets_delay_us(104);

    // Start bit (физический 0)
    ESP_LOGI(TAG, "Start bit (physical 0)");
    gpio_set_level(pin_mdb_tx, 0);
    ets_delay_us(104);

    // Отправляем 8 бит данных (MSB first)
    ESP_LOGI(TAG, "Writing data bits:");
    for (uint8_t x = 0; x < 8; x++) {
        int bit_value = (data_byte >> (7 - x)) & 1;  // MSB first
        int pin_level = !bit_value;  // Инвертируем: логическая 1 -> физический 0
        
        // Пробуем установить бит несколько раз
        for (int i = 0; i < 3; i++) {
            gpio_set_level(pin_mdb_tx, pin_level);
            ets_delay_us(10);
            int actual_level = gpio_get_level(pin_mdb_tx);
            
            if (actual_level == pin_level) {
                break;
            }
            
            if (i == 2) {
                ESP_LOGW(TAG, "  Failed to set bit %d to physical level %d!", 7-x, pin_level);
            }
        }
        
        ESP_LOGI(TAG, "  Data Bit %d: Logical:%d Physical:%d (after inversion)", 7-x, bit_value, gpio_get_level(pin_mdb_tx));
        ets_delay_us(94);  // 104 - 10 = 94 (учитываем задержку при установке)
    }

    // Отправляем mode bit
    int physical_level = !mode_bit;  // Инвертируем: логическая 1 -> физический 0
    
    // Пробуем установить mode bit несколько раз
    for (int i = 0; i < 3; i++) {
        gpio_set_level(pin_mdb_tx, physical_level);
        ets_delay_us(10);
        int actual_level = gpio_get_level(pin_mdb_tx);
        
        if (actual_level == physical_level) {
            break;
        }
        
        if (i == 2) {
            ESP_LOGW(TAG, "  Failed to set mode bit to physical level %d!", physical_level);
        }
    }
    
    ESP_LOGI(TAG, "  Mode Bit: Logical:%d Physical:%d (after inversion)", mode_bit, gpio_get_level(pin_mdb_tx));
    ets_delay_us(94);

    // Stop bit (физическая 1) и возврат в idle
    ESP_LOGI(TAG, "Stop bit and return to idle (physical 1)");
    
    // Пробуем переключить TX линию в high несколько раз
    for (int i = 0; i < 3; i++) {
        gpio_set_level(pin_mdb_tx, 0);
        ets_delay_us(100);
        gpio_set_level(pin_mdb_tx, 1);
        ets_delay_us(100);
        
        int final_tx_level = gpio_get_level(pin_mdb_tx);
        ESP_LOGI(TAG, "TX pin level (attempt %d): %d", i+1, final_tx_level);
        
        if (final_tx_level == 1) {
            break;
        }
        
        if (i == 2) {
            ESP_LOGE(TAG, "Failed to set TX line to idle state after 3 attempts!");
        }
    }
    
    ets_delay_us(208);  // Двойная задержка для надежности
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
