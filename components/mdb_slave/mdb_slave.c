/*
 * mdb_slave.c - MDB Cashless Device Implementation
 */

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mdb_protocol.h"
#include "mdb_slave.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

static const char *TAG = "mdb_slave";

// State names for logging
static const char *state_names[] = {
    "INACTIVE_STATE",
    "DISABLED_STATE",
    "ENABLED_STATE",
    "IDLE_STATE",
    "VEND_STATE"
};

// Machine state
static machine_state_t machine_state = INACTIVE_STATE;

// Function to log state transitions
static void log_state_transition(machine_state_t old_state, machine_state_t new_state) {
    ESP_LOGI(TAG, "State transition: %s -> %s", 
             state_names[old_state], state_names[new_state]);
}

// Control flags for MDB flows
static bool session_begin_todo = false;
static bool session_cancel_todo = false;
static bool session_end_todo = false;
static bool vend_approved_todo = false;
static bool vend_denied_todo = false;
static bool cashless_reset_todo = false;
static bool outsequence_todo = false;

// Timestamp for state tracking
static int64_t state_start_time_us = 0;

// Current session data
static mdb_session_msg_t current_session = {0};

// Queue for session messages
static QueueHandle_t mdb_session_queue = NULL;

// Callback for vend requests
static void (*vend_request_callback)(uint16_t item_number, uint16_t item_price) = NULL;

// Прототип функции mdb_cashless_loop
void mdb_cashless_loop(void *pvParameters);

void mdb_slave_init(gpio_num_t rx_pin, gpio_num_t tx_pin, gpio_num_t led_pin)
{
    // Initialize protocol layer
    mdb_protocol_init(rx_pin, tx_pin, led_pin);
    
    // Create session queue
    mdb_session_queue = xQueueCreate(5, sizeof(mdb_session_msg_t));
    if (mdb_session_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create MDB session queue");
        return;
    }
    
    // Create MDB task
    BaseType_t result = xTaskCreate(
        mdb_cashless_loop,
        "mdb_cashless",
        4096,
        NULL,
        5,
        NULL
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MDB cashless task");
        return;
    }
    
    ESP_LOGI(TAG, "MDB slave initialized");
}

bool mdb_start_session(uint16_t funds_available)
{
    if (machine_state != ENABLED_STATE) {
        ESP_LOGW(TAG, "Cannot start session: machine not in ENABLED state (current: %d)", machine_state);
        return false;
    }
    
    mdb_session_msg_t session = {
        .sequential = 1,
        .fundsAvailable = funds_available,
        .itemPrice = 0,
        .itemNumber = 0
    };
    
    if (xQueueSend(mdb_session_queue, &session, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue session start");
        return false;
    }
    
    ESP_LOGI(TAG, "Session start queued with funds: %d", funds_available);
    return true;
}

void mdb_cancel_session(void)
{
    if (machine_state == IDLE_STATE || machine_state == VEND_STATE) {
        session_cancel_todo = true;
        ESP_LOGI(TAG, "Session cancel requested");
    } else {
        ESP_LOGW(TAG, "Cannot cancel session: no active session");
    }
}

machine_state_t mdb_get_state(void)
{
    return machine_state;
}

void mdb_register_vend_callback(void (*callback)(uint16_t item_number, uint16_t item_price))
{
    vend_request_callback = callback;
}

void mdb_approve_vend(void)
{
    if (machine_state == VEND_STATE) {
        vend_approved_todo = true;
        ESP_LOGI(TAG, "Vend approved for item: %d, price: %d", 
                 current_session.itemNumber, current_session.itemPrice);
    } else {
        ESP_LOGW(TAG, "Cannot approve vend: no vend in progress");
    }
}

void mdb_deny_vend(void)
{
    if (machine_state == VEND_STATE) {
        vend_denied_todo = true;
        ESP_LOGI(TAG, "Vend denied for item: %d, price: %d", 
                 current_session.itemNumber, current_session.itemPrice);
    } else {
        ESP_LOGW(TAG, "Cannot deny vend: no vend in progress");
    }
}

void mdb_cashless_loop(void *pvParameters)
{
    // Payload buffer and available transmission flag
    uint8_t mdb_payload[32];
    uint8_t available_tx = 0;

    for (;;) {
        // Checksum calculation
        uint8_t checksum = 0x00;

        // Read from MDB and check if the mode bit is set
        uint16_t coming_read = mdb_read_9(&checksum);

        uint8_t command = coming_read & BIT_CMD_SET;  // Команда в битах 2-0
        const char* cmd_name;
        switch(command) {
            case RESET: cmd_name = "RESET"; break;
            case SETUP: cmd_name = "SETUP"; break;
            case POLL: cmd_name = "POLL"; break;
            case VEND: cmd_name = "VEND"; break;
            case READER: cmd_name = "READER"; break;
            case REVALUE: cmd_name = "REVALUE"; break;
            case EXPANSION: cmd_name = "EXPANSION"; break;
            default: cmd_name = "UNKNOWN"; break;
        }

        ESP_LOGI(TAG, "Received: 0x%04X (Command:%s/0x%02X) Bits:[%c%c%c%c%c|%c%c%c%c]",
                 coming_read,
                 cmd_name,
                 command,
                 (coming_read & (1 << 8)) ? '1' : '0',
                 (coming_read & (1 << 7)) ? '1' : '0',
                 (coming_read & (1 << 6)) ? '1' : '0',
                 (coming_read & (1 << 5)) ? '1' : '0',
                 (coming_read & (1 << 4)) ? '1' : '0',
                 (coming_read & (1 << 3)) ? '1' : '0',
                 (coming_read & (1 << 2)) ? '1' : '0',
                 (coming_read & (1 << 1)) ? '1' : '0',
                 (coming_read & (1 << 0)) ? '1' : '0');

        if (coming_read & BIT_MODE_SET) {
            uint8_t data_byte = coming_read & 0xFF;  // Только данные без mode bit

            if (data_byte == ACK_DATA) {
                ESP_LOGI(TAG, "Received ACK");
            } else if (data_byte == RET_DATA) {
                ESP_LOGI(TAG, "Received RET");
            } else if (data_byte == NAK_DATA) {
                ESP_LOGI(TAG, "Received NAK in state %s", state_names[machine_state]);
                // В состоянии INACTIVE после NAK отправляем ACK
                if (machine_state == INACTIVE_STATE) {
                    mdb_write_9(ACK);
                    ESP_LOGI(TAG, "Sent ACK in response to NAK in INACTIVE state");
            }
        } else if ((coming_read & BIT_ADD_SET) == 0x10) {  // Адрес Cashless #1 = 00010xxxB (10H)
                ESP_LOGI(TAG, "Address match: masked=0x%02X, expected=0x10", coming_read & BIT_ADD_SET);
                // Отправляем ACK на каждую команду для нас
                ESP_LOGI(TAG, "[RESPONSE] Sending ACK for command 0x%02X", coming_read & BIT_CMD_SET);
                mdb_write_9(ACK);
                ESP_LOGI(TAG, "[RESPONSE] ACK sent successfully");

                // Reset transmission availability
                available_tx = 0;
                
                // Command decoding based on incoming data
                switch (coming_read & BIT_CMD_SET) {
                    case RESET: {
                        mdb_read_9(NULL);  // Read checksum byte
                        ESP_LOGI(TAG, "MDB: RESET command received");

                        if (machine_state == VEND_STATE) {
                            ESP_LOGI(TAG, "Reset during VEND_STATE, treating as VEND_SUCCESS");
                        }

                        machine_state_t old_state = machine_state;
                        machine_state = INACTIVE_STATE;
                        log_state_transition(old_state, machine_state);
                        cashless_reset_todo = true;
                        
                        // Отправляем JUST RESET после получения RESET
                        mdb_payload[0] = JUST_RESET;
                        mdb_payload[1] = CONFIG_DATA;
                        ESP_LOGI(TAG, "[RESPONSE] Sending JUST_RESET response: [0x%02X, 0x%02X]", mdb_payload[0], mdb_payload[1]);
                        mdb_write_payload(mdb_payload, 2);
                        ESP_LOGI(TAG, "[RESPONSE] JUST_RESET response sent successfully");
                        break;
                    }
                    
                    case SETUP: {
                        switch (mdb_read_9(&checksum)) {
                            case CONFIG_DATA: {
                                uint8_t vmcFeatureLevel = mdb_read_9(&checksum);
                                uint8_t vmcColumnsOnDisplay = mdb_read_9(&checksum);
                                uint8_t vmcRowsOnDisplay = mdb_read_9(&checksum);
                                uint8_t vmcDisplayInfo = mdb_read_9(&checksum);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)vmcDisplayInfo;
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                
                                ESP_LOGI(TAG, "MDB: SETUP CONFIG_DATA received (VMC Level: %d, Display: %dx%d)",
                                         vmcFeatureLevel, vmcColumnsOnDisplay, vmcRowsOnDisplay);

                                machine_state = DISABLED_STATE;

                                // Prepare response with reader configuration
                                mdb_payload[0] = 0x01;        // Reader Config Data
                                mdb_payload[1] = 1;           // Reader Feature Level
                                mdb_payload[2] = 0xff;        // Country Code High
                                ESP_LOGI(TAG, "[RESPONSE] Preparing SETUP response: Config=0x%02X, Level=0x%02X, CountryHigh=0x%02X",
                                         mdb_payload[0], mdb_payload[1], mdb_payload[2]);
                                mdb_payload[3] = 0xff;        // Country Code Low
                                mdb_payload[4] = 1;           // Scale Factor
                                mdb_payload[5] = 2;           // Decimal Places
                                mdb_payload[6] = 3;           // Maximum Response Time (3s)
                                mdb_payload[7] = 0b00001001;  // Miscellaneous Options
                                available_tx = 8;
                                break;
                            }
                            
                            case MAX_MIN_PRICES: {
                                uint16_t maxPrice = (mdb_read_9(&checksum) << 8) | mdb_read_9(&checksum);
                                uint16_t minPrice = (mdb_read_9(&checksum) << 8) | mdb_read_9(&checksum);
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                
                                ESP_LOGI(TAG, "MDB: SETUP MAX_MIN_PRICES received (Max: %d, Min: %d)",
                                         maxPrice, minPrice);
                                break;
                            }
                        }
                        break;
                    }
                    
                    case POLL: {
                        uint8_t checksum_ = mdb_read_9(NULL);
                        // Отмечаем переменную как намеренно неиспользуемую
                        (void)checksum_;
                        
                        ESP_LOGI(TAG, "Received POLL command, available_tx=%d", available_tx);
                        
                        ESP_LOGD(TAG, "MDB: POLL command received");

                        if (outsequence_todo) {
                            // Command out of sequence
                            outsequence_todo = false;
                            mdb_payload[0] = 0x0b;
                            available_tx = 1;
                            ESP_LOGW(TAG, "MDB: Responding with OUT-OF-SEQUENCE");

                        } else if (cashless_reset_todo) {
                            // Just reset
                            cashless_reset_todo = false;
                            mdb_payload[0] = 0x00;
                            available_tx = 1;
                            ESP_LOGI(TAG, "MDB: Responding with JUST-RESET");

                        } else if (vend_approved_todo) {
                            // Vend approved
                            vend_approved_todo = false;
                            uint16_t vendAmount = current_session.itemPrice;
                            mdb_payload[0] = 0x05;
                            mdb_payload[1] = vendAmount >> 8;
                            mdb_payload[2] = vendAmount;
                            available_tx = 3;
                            ESP_LOGI(TAG, "MDB: Responding with VEND-APPROVED (Amount: %d)", vendAmount);

                        } else if (vend_denied_todo) {
                            // Vend denied
                            vend_denied_todo = false;
                            mdb_payload[0] = 0x06;
                            available_tx = 1;
                            machine_state = IDLE_STATE;
                            ESP_LOGI(TAG, "MDB: Responding with VEND-DENIED");

                        } else if (session_end_todo) {
                            // End session
                            session_end_todo = false;
                            mdb_payload[0] = 0x07;
                            available_tx = 1;
                            machine_state = ENABLED_STATE;
                            ESP_LOGI(TAG, "MDB: Responding with SESSION-COMPLETE");

                        } else if (machine_state == ENABLED_STATE && 
                                  xQueueReceive(mdb_session_queue, &current_session, 0)) {
                            // Begin session
                            session_begin_todo = false;
                            machine_state = IDLE_STATE;
                            state_start_time_us = esp_timer_get_time();

                            uint16_t fundsAvailable = current_session.fundsAvailable;
                            mdb_payload[0] = 0x03;
                            mdb_payload[1] = fundsAvailable >> 8;
                            mdb_payload[2] = fundsAvailable;
                            available_tx = 3;
                            ESP_LOGI(TAG, "MDB: Responding with BEGIN-SESSION (Funds: %d)", fundsAvailable);

                        } else if (session_cancel_todo) {
                            // Cancel session
                            session_cancel_todo = false;
                            mdb_payload[0] = 0x04;
                            available_tx = 1;
                            ESP_LOGI(TAG, "MDB: Responding with SESSION-CANCEL");

                        } else {
                            // Check for session timeout
                            if (machine_state >= IDLE_STATE) {
                                int64_t now = esp_timer_get_time();
                                int64_t elapsed = now - state_start_time_us;

                                if (elapsed > TIMEOUT_IDLE_US) {
                                    session_cancel_todo = true;
                                    ESP_LOGW(TAG, "MDB: Session timeout, will cancel");
                                }
                            }
                        }
                        break;
                    }
                    
                    case VEND: {
                        switch (mdb_read_9(&checksum)) {
                            case VEND_REQUEST: {
                                uint16_t itemPrice = (mdb_read_9(&checksum) << 8) | mdb_read_9(&checksum);
                                uint16_t itemNumber = (mdb_read_9(&checksum) << 8) | mdb_read_9(&checksum);
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                
                                ESP_LOGI(TAG, "MDB: VEND_REQUEST received (Item: %d, Price: %d)",
                                         itemNumber, itemPrice);

                                machine_state = VEND_STATE;
                                current_session.itemNumber = itemNumber;
                                current_session.itemPrice = itemPrice;
                                
                                // Call the callback if registered
                                if (vend_request_callback != NULL) {
                                    vend_request_callback(itemNumber, itemPrice);
                                }
                                
                                // Auto-approve if price is within available funds
                                if (itemPrice <= current_session.fundsAvailable) {
                                    vend_approved_todo = true;
                                    ESP_LOGI(TAG, "Auto-approving vend (price within funds)");
                                }
                                break;
                            }
                            
                            case VEND_CANCEL: {
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                ESP_LOGI(TAG, "MDB: VEND_CANCEL received");
                                vend_denied_todo = true;
                                break;
                            }
                            
                            case VEND_SUCCESS: {
                                uint16_t itemNumber = (mdb_read_9(&checksum) << 8) | mdb_read_9(&checksum);
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                
                                ESP_LOGI(TAG, "MDB: VEND_SUCCESS received (Item: %d)", itemNumber);
                                machine_state = IDLE_STATE;
                                current_session.itemNumber = itemNumber;
                                break;
                            }
                            
                            case VEND_FAILURE: {
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                ESP_LOGI(TAG, "MDB: VEND_FAILURE received");
                                machine_state = IDLE_STATE;
                                break;
                            }
                            
                            case SESSION_COMPLETE: {
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                ESP_LOGI(TAG, "MDB: SESSION_COMPLETE received");
                                session_end_todo = true;
                                break;
                            }
                            
                            case CASH_SALE: {
                                uint16_t itemPrice = (mdb_read_9(&checksum) << 8) | mdb_read_9(&checksum);
                                uint16_t itemNumber = (mdb_read_9(&checksum) << 8) | mdb_read_9(&checksum);
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                
                                ESP_LOGI(TAG, "MDB: CASH_SALE received (Item: %d, Price: %d)",
                                         itemNumber, itemPrice);
                                break;
                            }
                        }
                        break;
                    }
                    
                    case READER: {
                        switch (mdb_read_9(&checksum)) {
                            case READER_DISABLE: {
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                ESP_LOGI(TAG, "MDB: READER_DISABLE received");
                                machine_state = DISABLED_STATE;
                                break;
                            }
                            
                            case READER_ENABLE: {
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                ESP_LOGI(TAG, "MDB: READER_ENABLE received");
                                machine_state = ENABLED_STATE;
                                break;
                            }
                            
                            case READER_CANCEL: {
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                ESP_LOGI(TAG, "MDB: READER_CANCEL received");
                                mdb_payload[0] = 0x08; // Canceled
                                available_tx = 1;
                                break;
                            }
                        }
                        break;
                    }
                    
                    case EXPANSION: {
                        switch (mdb_read_9(&checksum)) {
                            case REQUEST_ID: {
                                uint8_t manufacturer_code[3];
                                uint8_t serial_number[12];
                                uint8_t model_number[12];
                                uint8_t software_version[2];
                                
                                // Read manufacturer code
                                for (int i = 0; i < 3; i++) {
                                    manufacturer_code[i] = mdb_read_9(&checksum);
                                }
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)manufacturer_code;
                                
                                // Read serial number
                                for (int i = 0; i < 12; i++) {
                                    serial_number[i] = mdb_read_9(&checksum);
                                }
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)serial_number;
                                
                                // Read model number
                                for (int i = 0; i < 12; i++) {
                                    model_number[i] = mdb_read_9(&checksum);
                                }
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)model_number;
                                
                                // Read software version
                                for (int i = 0; i < 2; i++) {
                                    software_version[i] = mdb_read_9(&checksum);
                                }
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)software_version;
                                
                                uint8_t checksum_ = mdb_read_9(NULL);
                                // Отмечаем переменную как намеренно неиспользуемую
                                (void)checksum_;
                                
                                ESP_LOGI(TAG, "MDB: REQUEST_ID received");
                                
                                // Prepare response
                                mdb_payload[0] = 0x09; // PERIPHERAL_ID
                                mdb_payload[1] = 'M';  // Manufacturer code
                                mdb_payload[2] = 'D';
                                mdb_payload[3] = 'B';
                                
                                // Serial number (12 bytes)
                                memcpy(&mdb_payload[4], "CASHLESS0001", 12);
                                
                                // Model number (12 bytes)
                                memcpy(&mdb_payload[16], "ESP32-SLAVE ", 12);
                                
                                // Software version (2 bytes)
                                mdb_payload[28] = 0x01;
                                mdb_payload[29] = 0x00;
                                // Send ACK if no activity
                        if (!available_tx) {
                            ESP_LOGI(TAG, "Sending ACK (0x00) in response to POLL");
                            mdb_write_9(ACK);
                        }    }
                        }
                        break;
                    }
                }

                // Transmit the prepared payload via MDB
                if (available_tx > 0) {
                    mdb_write_payload(mdb_payload, available_tx);
                }
            } else {
                // Not addressed to us
                mdb_set_led(false);
            }
        }
        
        // Small delay to prevent CPU hogging
        vTaskDelay(1);
    }
}
