#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "mdb_slave.h"

static const char *TAG = "mdb-cashless";

// MDB GPIO pin definitions
#define MDB_RX_PIN      CONFIG_MDB_RX_PIN
#define MDB_TX_PIN      CONFIG_MDB_TX_PIN
#define MDB_LED_PIN     CONFIG_MDB_LED_PIN

// Funds available for testing (in MDB currency units)
#define TEST_FUNDS_AVAILABLE 500

// Callback function for vend requests
static void vend_request_handler(uint16_t item_number, uint16_t item_price)
{
    ESP_LOGI(TAG, "Vend request received: Item #%d, Price: %d", item_number, item_price);
    
    // Автоматически одобряем запрос на продажу
    ESP_LOGI(TAG, "Автоматически одобряем запрос на продажу");
    mdb_approve_vend();
}

// Task to monitor MDB state and log it
static void mdb_monitor_task(void *arg)
{
    machine_state_t last_state = INACTIVE_STATE;
    
    while (1) {
        machine_state_t current_state = mdb_get_state();
        
        // Log state changes
        if (current_state != last_state) {
            const char *state_names[] = {
                "INACTIVE", "DISABLED", "ENABLED", "IDLE", "VEND"
            };
            
            ESP_LOGI(TAG, "MDB состояние изменилось: %s -> %s", 
                     state_names[last_state], state_names[current_state]);
            
            last_state = current_state;
            
            // Если перешли в состояние ENABLED, начинаем сессию с тестовыми средствами
            if (current_state == ENABLED_STATE) {
                ESP_LOGI(TAG, "Начинаем сессию с доступными средствами: %d", TEST_FUNDS_AVAILABLE);
                mdb_start_session(TEST_FUNDS_AVAILABLE);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Инициализация MDB Cashless Slave");
    
    // Initialize MDB slave
    mdb_slave_init(MDB_RX_PIN, MDB_TX_PIN, MDB_LED_PIN);
    
    // Register vend request callback
    mdb_register_vend_callback(vend_request_handler);
    
    // Create monitor task
    xTaskCreate(mdb_monitor_task, "mdb_monitor", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "MDB Cashless Slave запущен и готов к работе");
}
