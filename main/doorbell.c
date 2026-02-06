#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "ESP_LOG.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"

#include "esp_spp_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"
#include "esp_pm.h"

/************************************************************************ Bluetooth ***************************************************************************/

enum{
    DOORBELL_SVC,
    DOORBELL_IDX_CHAR,
    DOORBELL_IDX_VAL,
    DOORBELL_IDX_CFG,
    DOORBELL_IDX_NB,
};

/******************************** PROFILE DESCRIPTION *************************************/
#define GATTS_TAG           "Doorbell"  //Just for logging where info came from, i.e. with ////ESP_LOGI
#define PROFILE_NUM         1           //Number of profiles: profile is a collection of servers
#define PROFILE_IDX         0           //Index of the profile to be used: 0 since there is no other profiles
#define APP_ID              0x56        //Just a random number, used to identify the application
#define DEVICE_NAME         "Doorbell"    //The Device Name Characteristics in GAP
#define SERVICE_INST_ID	    0           //Service Instance ID: 0 since there is only one instance of the service

/*********************** HANDLE TABLE ********************************************/
static uint16_t doorbell_handle_table[DOORBELL_IDX_NB]; //Handles for the attributes in the SPP service

/*********************************** ADVERTISING DATA ****************************************************/
static const uint8_t advertise_data[17] = {
    // Length: 0x02 - 2 bytes, Type: 0x01 - Flags, Value: 0x06 - LE General Discoverable Mode, BR/EDR Not Supported
    0x02,0x01,0x06,
    // Length: 0x03 - 3 bytes, Type: 0x03 - Complete List of 16-bit Service Class UUIDs, Value: 0xF0,0xAB - SPP Service UUID
    0x03,0x03,0xF0,0xAB,
    // Length: 0x09 - 15 bytes, Type: 0x09 - Complete Local Name, Value: 'D', 'O', 'O', 'R', 'B', 'E', 'L', 'L'
    0x09,0x09, 'D', 'O', 'O', 'R', 'B', 'E', 'L', 'L'
};

/******************************* ADVERTISING PARAMATERS ****************************************/
static esp_ble_adv_params_t advertising_params = {
    //How often the advertising packet is sent, in units of 0.625 ms
    //Increased from 20-40ms to 800-1600ms to reduce power consumption when not connected
    .adv_int_min        = 0x320, //0x320 * 0.625ms = 800ms
    .adv_int_max        = 0x640, //0x640 * 0.625ms = 1600ms
    .adv_type           = ADV_TYPE_IND, //Indicate that the device is connectable and scannable
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC, //Use the public address
    .channel_map        = ADV_CHNL_ALL, //Advertise on all 3 channels
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, //Allow both scan and connection requests from any device
};

/********************************************* SPP Services *******************************************************/
static const uint16_t doorbell_uuid = 0xABF0;

/**************************************** Charactify characteristic ******************************************/
#define NOTIFY_UUID       0xABF2 //UUID for Notify characteristic


/**************************************** Packet limitations *******************************************/
#define MTU_SIZE          (512) //MTU (Max Transmission Unit) size set to 512 bytes
#define SPP_DATA_MAX_LEN  (512) 

static uint16_t connection_id = 0xffff;
static esp_gatt_if_t gatts_interface = 0xff; //Server ID

/***************************** CLIENT SIDE *******************************/
static bool enable_notify = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,}; //MAC Address of the connected device

/**
 * Holds all the imporatnt information about the gatts profile
 */
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb; //Callback function for the profile
    uint16_t gatts_if; //Interface ID for the profile
    uint16_t app_id; //Application ID for the profile
    uint16_t conn_id; //Connection ID for the profile
    uint16_t service_handle; //Handle of the service
    esp_gatt_srvc_id_t service_id; //Service ID
    uint16_t char_handle; //Handle of the characteristic
    esp_bt_uuid_t char_uuid; //UUID of the characteristic
    esp_gatt_perm_t perm; //Permissions for the characteristic
    esp_gatt_char_prop_t property; //Properties of the characteristic
    uint16_t descr_handle; //Handle of the descriptor
    esp_bt_uuid_t descr_uuid; //UUID of the descriptor
};

//Just a handler for any [gatts_if] that takes and event and parameters
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/******************************* GATTS PROFILE INITIALIZATION *****************************************/
/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst doorbell_profile_tab[PROFILE_NUM] = {
    [PROFILE_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .app_id = APP_ID,
        .conn_id = 0xffff,
        .service_handle = 0,
        .char_handle = 0,
    },
};

/*
 ****************************************************************************************
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t)) //Each characteristic declaration is 1 byte long
static const uint16_t primary_doorbell_uuid = ESP_GATT_UUID_PRI_SERVICE; //UUID for a primary service
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE; //UUID for a characteristic declaration
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG; //UUID for a client characteristic configuration descriptor

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY; //Characteristic will be readable and can notify

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = NOTIFY_UUID;
static uint8_t  spp_data_notify_val[1] = {0x00};
static uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t doorbell_db[DOORBELL_IDX_NB] =
{
    //SPP -  Service Declaration
    [DOORBELL_SVC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_doorbell_uuid, ESP_GATT_PERM_READ,
        sizeof(doorbell_uuid), sizeof(doorbell_uuid), (uint8_t *)&doorbell_uuid}},

    //SPP -  data notify characteristic Declaration
    [DOORBELL_IDX_CHAR] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [DOORBELL_IDX_VAL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
        SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [DOORBELL_IDX_CFG] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&advertising_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //Check if advertising started successfully
        if(param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            //ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        //ESP_LOGI(GATTS_TAG, "Advertising started successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if(param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            //ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            break;
        }
        //ESP_LOGI(GATTS_TAG, "Advertising stopped successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         //ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                //    param->update_conn_params.status,
                //    param->update_conn_params.conn_int,
                //    param->update_conn_params.latency,
                //    param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    //uint8_t res = 0xff;

    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    //ESP_LOGI(GATTS_TAG, "GATT server regiser, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
        	esp_ble_gap_set_device_name(DEVICE_NAME);
        	esp_ble_gatts_create_attr_tab(doorbell_db, gatts_if, DOORBELL_IDX_NB, SERVICE_INST_ID);
             
        	//esp_ble_gap_config_adv_data_raw((uint8_t *)advertise_data, sizeof(advertise_data));
       	break;
        case ESP_GATTS_CONNECT_EVT:
            //ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                 //param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
    	    connection_id = p_data->connect.conn_id;
    	    gatts_interface = gatts_if;
    	    is_connected = true;

    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
            
            // Stop advertising to save power while connected
            esp_ble_gap_stop_advertising();
            
            // Update connection parameters for power efficiency
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = 0x10;    // 20ms (16 * 1.25ms)
            conn_params.max_int = 0x20;    // 40ms (32 * 1.25ms)
            conn_params.latency = 50;      // Allow 50 connection events to be skipped
            conn_params.timeout = 400;     // 4000ms timeout (400 * 10ms)
            esp_ble_gap_update_conn_params(&conn_params);
            
    	    esp_ble_gap_start_advertising(&advertising_params);
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ////ESP_LOGI(GATTS_TAG, "The number handle %x",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK) {
    	        //////ESP_LOGE(GATTS_TAG, "Create attribute table failed, error code 0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != DOORBELL_IDX_NB) {
    	        //////ESP_LOGE(GATTS_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, DOORBELL_IDX_NB);
    	    }
    	    else {
    	        memcpy(doorbell_handle_table, param->add_attr_tab.handles, sizeof(doorbell_handle_table));
                esp_err_t sret = esp_ble_gatts_start_service(doorbell_handle_table[DOORBELL_SVC]);
                if (sret != ESP_OK) {
                    //ESP_LOGE(GATTS_TAG, "start_service failed: %d", sret);
                } else {
                    vTaskDelay(150 / portTICK_PERIOD_MS);

                    // Configure and start advertising AFTER service started
                    esp_ble_gap_config_adv_data_raw((uint8_t *)advertise_data, sizeof(advertise_data));
                    // Note: gap_event_handler will start advertising on ADV_DATA_RAW_SET_COMPLETE_EVT
                }
    	    }
    	    break;
    	}
        case ESP_GATTS_WRITE_EVT: {
            if (param->write.handle == doorbell_handle_table[DOORBELL_IDX_CFG]) {
                if (param->write.len == 2) {
                    uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                    if (descr_value == 0x0001) {
                        enable_notify = true;
                        //ESP_LOGI(GATTS_TAG, "Notifications ENABLED");
                    } else if (descr_value == 0x0000) {
                        enable_notify = false;
                        //ESP_LOGI(GATTS_TAG, "Notifications DISABLED");
                    }
                }
            }
            break;
        }
    	default:
    	    break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            doorbell_profile_tab[PROFILE_IDX].gatts_if = gatts_if;
        } else {
            //ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == doorbell_profile_tab[idx].gatts_if) {
                if (doorbell_profile_tab[idx].gatts_cb) {
                    doorbell_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


/************************************************************** Doorbell **************************************************************************/
// GPIO pins used for button
#define GPIO_BUTTON GPIO_NUM_16
#define GPIO_LED    GPIO_NUM_17

TaskHandle_t Click_handler = NULL;
QueueHandle_t myQueue = NULL;

uint8_t curr_state = 0;

void CLICK_Task() {
    while(1) {
        if(xQueueReceive(myQueue, &curr_state, portMAX_DELAY)) {
            if (curr_state == 1) {
                
                temp = 1;
                esp_ble_gatts_set_attr_value(
                    doorbell_handle_table[DOORBELL_IDX_VAL],
                    sizeof(curr_state),
                    &curr_state
                    );
                    
                    esp_ble_gatts_send_indicate(
                    gatts_interface,
                    connection_id,
                    doorbell_handle_table[DOORBELL_IDX_VAL],
                    sizeof(curr_state),
                    &curr_state,
                    false
                    );
            }
            // temp = 0;
            // esp_ble_gatts_set_attr_value(
            //     doorbell_handle_table[DOORBELL_IDX_VAL],
            //     sizeof(temp),
            //     &temp
            //     );
                
            // esp_ble_gatts_send_indicate(
            //     gatts_interface,
            //     connection_id,
            //     doorbell_handle_table[DOORBELL_IDX_VAL],
            //     sizeof(temp),
            //     &temp,
            //     false
            // );
        }
        curr_state = 0;
        vTaskDelay(50 / portTICK_PERIOD_MS); // debounce-ish

        //gpio_set_level(GPIO_LED, led_state);
    }
}

/**
 * Watches for button presses and sends the button state to the CLICK_Task via a queue
 */
static void IRAM_ATTR button_isr_handler(void* arg) {
    curr_state = 1;
    xQueueSendFromISR(myQueue, &curr_state, NULL);
}

void app_main(void)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { //Cheak if NVS partition was truncated and needs to be erased 
        ESP_ERROR_CHECK(nvs_flash_erase()); //Erase NVS partition or crash the if nvs_flash_erase fails
        ret = nvs_flash_init(); //attempt to init again
    }
    ESP_ERROR_CHECK( ret ); //Crashes if not initialized properly

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        //ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        //ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    //ESP_LOGI(GATTS_TAG, "%s init bluetooth", __func__);

    ret = esp_bluedroid_init();
    if (ret) {
        //ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        //ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // Set BLE transmit power to -4dBm (low power mode)
    // Reduced from P20 (max) to N4 for 5-10x power savings
    esp_err_t power_ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N3);
    if (power_ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to set default TX power, error %d", ret);
        ESP_LOGE(GATTS_TAG, "Error string: %s", esp_err_to_name(ret));
    }
    
    // Also set advertising TX power to low
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N3);
    power_ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N3);
    if (power_ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to set advertising TX power, error %d", ret);
        ESP_LOGE(GATTS_TAG, "Error string: %s", esp_err_to_name(ret));
    }
    // esp_err_t power_ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P20); // max for advertising
    // if (power_ret != ESP_OK) {
    //     ESP_LOGE(GATTS_TAG, "Failed to set advertising TX power, error %d", power_ret);
    //     ESP_LOGE(GATTS_TAG, "Error string: %s", esp_err_to_name(power_ret));
    // }
    // esp_power_level_t level;
    // level = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);

    // if (level != ESP_PWR_LVL_INVALID) {
    //     ESP_LOGI(GATTS_TAG, "BLE TX Power for connection handle %u = %d (index, not dBm)\n", connection_id, level);
    // } else {
    //     ESP_LOGE(GATTS_TAG, "Failed to read TX power, error code");
    // }
    // power_ret = ESP_FAIL;
    // for (int i = 0; i < 5 && power_ret != ESP_OK; i++) {
    //     vTaskDelay(200 / portTICK_PERIOD_MS);
    //     power_ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P20);
    // }
    // if (power_ret != ESP_OK) {
    //     ESP_LOGE(GATTS_TAG, "Failed to set connection TX power, error %d", power_ret);
    //     ESP_LOGE(GATTS_TAG, "Error string: %s", esp_err_to_name(power_ret));
    // }

    // level = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_CONN_HDL0 + connection_id);

    // if (level != ESP_PWR_LVL_INVALID) {
    //     ESP_LOGI(GATTS_TAG, "BLE TX Power for connection handle %u = %d (index, not dBm)\n", connection_id, level);
    // } else {
    //     ESP_LOGE(GATTS_TAG, "Failed to read TX power, error code");
    // }

    // Configure Dynamic Frequency Scaling for power efficiency
    // Reduces CPU frequency when idle, allowing deeper sleep states
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 240,    // Max CPU frequency: 240MHz
        .min_freq_mhz = 80,     // Min CPU frequency: 80MHz (reduces when idle)
        .light_sleep_enable = true,  // Enable light sleep mode
    };
    esp_pm_configure(&pm_config);

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(MTU_SIZE);
        if (local_mtu_ret){
        //////ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    //Initialize the button 
    esp_rom_gpio_pad_select_gpio(GPIO_BUTTON);
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);

    // esp_rom_gpio_pad_select_gpio(GPIO_LED);
    // gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    gpio_pullup_en(GPIO_BUTTON);
    gpio_pulldown_dis(GPIO_BUTTON);

    gpio_set_intr_type(GPIO_BUTTON, GPIO_INTR_POSEDGE); //Interrupt of falling edge

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_BUTTON, button_isr_handler, NULL);
    gpio_intr_enable(GPIO_BUTTON);
    
    myQueue = xQueueCreate(3, sizeof(char));
    
    xTaskCreatePinnedToCore(CLICK_Task, "CLICK", 4096, NULL, 10, &Click_handler, 1);
}