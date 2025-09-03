/*
 * Author: Omar Faruk
 *
 * Reference: esp-idf HID device example
 */

#include "hidd_le_prf_int.h"
#include <string.h>
#include "esp_log.h"
#include "ota_manager.h"

/* @file  ota_manager.c
   @brief this code involves ESP32 in BLE peripheral mode and as a GATT server,
          with OTA service for firmware update.On selecting a .bin file by the client,
          the particular file gets written in packets to the OTA partition currently
          available on the device.
   @author Avinashee Tech
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"

#include "nvs_flash.h"
#include "driver/gpio.h"
#include "ota_manager.h"

static const char *TAG = "Bluedroid-OTA";

#define PROFILE_NUM 2      // Number of Application Profiles
#define PROFILE_OTA_ID 0 // OTA Application Profile ID
#define INFO_INST_ID 0     // Device Info service id
#define OTA_INST_ID 1      // OTA service id
#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

/*macros*/
#define GATTS_CHAR_VAL_LEN_MAX 100
#define OTA_VAL_LEN_MAX 256
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
#define REBOOT_DEEP_SLEEP_TIMEOUT 1000
#define REMOVE_BONDED_DEVICES_ON_RESET 1
#define TEST_GPIO 13
#define PROFILE_OTA_ID         1 // OTA Application Profile ID

/*ota variables*/
const esp_partition_t *update_partition;
esp_ota_handle_t update_handle;
TimerHandle_t xTimerOTA;
uint8_t ota_ctrl_val = 0;
uint16_t ota_flag = 0;
bool updating = false;
uint16_t num_pkgs_received = 0;
uint16_t packet_size = 0;

/*
***************************************************************************************************************
GAP Profile
***************************************************************************************************************
*/

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static uint8_t adv_config_done = 0;
uint16_t conn_id = 0;
/*128 bits ota service uuid*/
static uint8_t ota_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xd2, 0xd0, 0x52, 0x4f, 0xa4, 0x74, 0x43, 0xf3, 0x94, 0xb5, 0xb2, 0x97, 0xf3, 0x42, 0x97, 0x6f};

typedef struct
{
    uint8_t company_id[2]; // Espressif Semicoductors ID - 0x02E5
    uint8_t mac_address[6];
} manufacturer_data;

manufacturer_data device_data = {.company_id = {0xE5, 0x02}, .mac_address = {0}};
static uint8_t custom_manufacturer_data[sizeof(device_data)];

/*config primary adv data*/
static esp_ble_adv_data_t esp32_ble_adv_config = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = sizeof(custom_manufacturer_data),
    .p_manufacturer_data = custom_manufacturer_data,
    .service_data_len = 0,
    .p_service_data = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/*config scan response data*/


/*configure esp32 advertising connection parameters*/
static esp_ble_adv_params_t esp32_ble_adv_params = {
    .adv_int_min = 0x0020,
    .adv_int_max = 0x00A0,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/*
 ****************************************************************************************
 GATT Profile
 ****************************************************************************************
 */

/*Attribute table variables*/
uint16_t ble_device_info[IDX_NB_INFO];
uint16_t ble_ota[IDX_NB_OTA];

/*variable holding firmare file data chunks*/
uint8_t ble_ota_data[OTA_VAL_LEN_MAX] = {0};

/*characteristics id's*/
static const uint16_t CTRL_UUID = 0xEE01;
static const uint16_t DATA_UUID = 0xEE02;

static const uint16_t device_info_service_uuid = ESP_GATT_UUID_DEVICE_INFO_SVC;
static const uint16_t manuf_name_character_uuid = ESP_GATT_UUID_MANU_NAME;
static const uint16_t model_num_character_uuid = ESP_GATT_UUID_MODEL_NUMBER_STR;

/*properties of characteristics*/
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;

/*descriptor and readable data of OTA data characteristics*/
const char manuf_name[] = "Espressif";
const char model_num[] = "OTA v1.1";
char ota_ctrl[] = "ota char";
uint8_t ota_cccd[] = {0x00, 0x00};



/// characteristic presentation information
struct prf_char_pres_fmt
{
    /// Unit (The Unit is a UUID)
    uint16_t unit;
    /// Description
    uint16_t description;
    /// Format
    uint8_t format;
    /// Exponent
    uint8_t exponent;
    /// Name space
    uint8_t name_space;
};

// HID report mapping table
static hid_report_map_t hid_rpt_map[HID_NUM_REPORTS];

// HID Report Map characteristic value
// Keyboard report descriptor (using format for Boot interface descriptor)
static const uint8_t hidReportMap[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x02,  // Usage (Mouse)
    0xA1, 0x01,  // Collection (Application)
    0x85, 0x01,  // Report Id (1)
    0x09, 0x01,  //   Usage (Pointer)
    0xA1, 0x00,  //   Collection (Physical)
    0x05, 0x09,  //     Usage Page (Buttons)
    0x19, 0x01,  //     Usage Minimum (01) - Button 1
    0x29, 0x03,  //     Usage Maximum (03) - Button 3
    0x15, 0x00,  //     Logical Minimum (0)
    0x25, 0x01,  //     Logical Maximum (1)
    0x75, 0x01,  //     Report Size (1)
    0x95, 0x03,  //     Report Count (3)
    0x81, 0x02,  //     Input (Data, Variable, Absolute) - Button states
    0x75, 0x05,  //     Report Size (5)
    0x95, 0x01,  //     Report Count (1)
    0x81, 0x01,  //     Input (Constant) - Padding or Reserved bits
    0x05, 0x01,  //     Usage Page (Generic Desktop)
    0x09, 0x30,  //     Usage (X)
    0x09, 0x31,  //     Usage (Y)
    0x09, 0x38,  //     Usage (Wheel)
    0x15, 0x81,  //     Logical Minimum (-127)
    0x25, 0x7F,  //     Logical Maximum (127)
    0x75, 0x08,  //     Report Size (8)
    0x95, 0x03,  //     Report Count (3)
    0x81, 0x06,  //     Input (Data, Variable, Relative) - X & Y coordinate
    0xC0,        //   End Collection
    0xC0,        // End Collection

    0x05, 0x01,  // Usage Pg (Generic Desktop)
    0x09, 0x06,  // Usage (Keyboard)
    0xA1, 0x01,  // Collection: (Application)
    0x85, 0x02,  // Report Id (2)
    //
    0x05, 0x07,  //   Usage Pg (Key Codes)
    0x19, 0xE0,  //   Usage Min (224)
    0x29, 0xE7,  //   Usage Max (231)
    0x15, 0x00,  //   Log Min (0)
    0x25, 0x01,  //   Log Max (1)
    //
    //   Modifier byte
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x08,  //   Report Count (8)
    0x81, 0x02,  //   Input: (Data, Variable, Absolute)
    //
    //   Reserved byte
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x08,  //   Report Size (8)
    0x81, 0x01,  //   Input: (Constant)
    //
    //   LED report
    0x05, 0x08,  //   Usage Pg (LEDs)
    0x19, 0x01,  //   Usage Min (1)
    0x29, 0x05,  //   Usage Max (5)
    0x95, 0x05,  //   Report Count (5)
    0x75, 0x01,  //   Report Size (1)
    0x91, 0x02,  //   Output: (Data, Variable, Absolute)
    //
    //   LED report padding
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x03,  //   Report Size (3)
    0x91, 0x01,  //   Output: (Constant)
    //
    //   Key arrays (6 bytes)
    0x95, 0x06,  //   Report Count (6)
    0x75, 0x08,  //   Report Size (8)
    0x15, 0x00,  //   Log Min (0)
    0x25, 0x65,  //   Log Max (101)
    0x05, 0x07,  //   Usage Pg (Key Codes)
    0x19, 0x00,  //   Usage Min (0)
    0x29, 0x65,  //   Usage Max (101)
    0x81, 0x00,  //   Input: (Data, Array)
    //
    0xC0,        // End Collection
    //
    0x05, 0x0C,   // Usage Pg (Consumer Devices)
    0x09, 0x01,   // Usage (Consumer Control)
    0xA1, 0x01,   // Collection (Application)
    0x85, 0x03,   // Report Id (3)
    0x09, 0x02,   //   Usage (Numeric Key Pad)
    0xA1, 0x02,   //   Collection (Logical)
    0x05, 0x09,   //     Usage Pg (Button)
    0x19, 0x01,   //     Usage Min (Button 1)
    0x29, 0x0A,   //     Usage Max (Button 10)
    0x15, 0x01,   //     Logical Min (1)
    0x25, 0x0A,   //     Logical Max (10)
    0x75, 0x04,   //     Report Size (4)
    0x95, 0x01,   //     Report Count (1)
    0x81, 0x00,   //     Input (Data, Ary, Abs)
    0xC0,         //   End Collection
    0x05, 0x0C,   //   Usage Pg (Consumer Devices)
    0x09, 0x86,   //   Usage (Channel)
    0x15, 0xFF,   //   Logical Min (-1)
    0x25, 0x01,   //   Logical Max (1)
    0x75, 0x02,   //   Report Size (2)
    0x95, 0x01,   //   Report Count (1)
    0x81, 0x46,   //   Input (Data, Var, Rel, Null)
    0x09, 0xE9,   //   Usage (Volume Up)
    0x09, 0xEA,   //   Usage (Volume Down)
    0x15, 0x00,   //   Logical Min (0)
    0x75, 0x01,   //   Report Size (1)
    0x95, 0x02,   //   Report Count (2)
    0x81, 0x02,   //   Input (Data, Var, Abs)
    0x09, 0xE2,   //   Usage (Mute)
    0x09, 0x30,   //   Usage (Power)
    0x09, 0x83,   //   Usage (Recall Last)
    0x09, 0x81,   //   Usage (Assign Selection)
    0x09, 0xB0,   //   Usage (Play)
    0x09, 0xB1,   //   Usage (Pause)
    0x09, 0xB2,   //   Usage (Record)
    0x09, 0xB3,   //   Usage (Fast Forward)
    0x09, 0xB4,   //   Usage (Rewind)
    0x09, 0xB5,   //   Usage (Scan Next)
    0x09, 0xB6,   //   Usage (Scan Prev)
    0x09, 0xB7,   //   Usage (Stop)
    0x15, 0x01,   //   Logical Min (1)
    0x25, 0x0C,   //   Logical Max (12)
    0x75, 0x04,   //   Report Size (4)
    0x95, 0x01,   //   Report Count (1)
    0x81, 0x00,   //   Input (Data, Ary, Abs)
    0x09, 0x80,   //   Usage (Selection)
    0xA1, 0x02,   //   Collection (Logical)
    0x05, 0x09,   //     Usage Pg (Button)
    0x19, 0x01,   //     Usage Min (Button 1)
    0x29, 0x03,   //     Usage Max (Button 3)
    0x15, 0x01,   //     Logical Min (1)
    0x25, 0x03,   //     Logical Max (3)
    0x75, 0x02,   //     Report Size (2)
    0x81, 0x00,   //     Input (Data, Ary, Abs)
    0xC0,           //   End Collection
    0x81, 0x03,   //   Input (Const, Var, Abs)
    0xC0,            // End Collectionq

#if (SUPPORT_REPORT_VENDOR == true)
    0x06, 0xFF, 0xFF, // Usage Page(Vendor defined)
    0x09, 0xA5,       // Usage(Vendor Defined)
    0xA1, 0x01,       // Collection(Application)
    0x85, 0x04,   // Report Id (4)
    0x09, 0xA6,   // Usage(Vendor defined)
    0x09, 0xA9,   // Usage(Vendor defined)
    0x75, 0x08,   // Report Size
    0x95, 0x7F,   // Report Count = 127 Btyes
    0x91, 0x02,   // Output(Data, Variable, Absolute)
    0xC0,         // End Collection
#endif

};

/// Battery Service Attributes Indexes
enum
{
    BAS_IDX_SVC,

    BAS_IDX_BATT_LVL_CHAR,
    BAS_IDX_BATT_LVL_VAL,
    BAS_IDX_BATT_LVL_NTF_CFG,
    BAS_IDX_BATT_LVL_PRES_FMT,

    BAS_IDX_NB,
};

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define PROFILE_NUM            2

#define PROFILE_APP_IDX        0
#define PROFILE_OTA_ID         1 // OTA Application Profile ID

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

hidd_le_env_t hidd_le_env;

// HID report map length
uint8_t hidReportMapLen = sizeof(hidReportMap);
uint8_t hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID report mapping table
//static hidRptMap_t  hidRptMap[HID_NUM_REPORTS];

// HID Information characteristic value
static const uint8_t hidInfo[HID_INFORMATION_LEN] = {
    LO_UINT16(0x0111), HI_UINT16(0x0111),             // bcdHID (USB HID version)
    0x00,                                             // bCountryCode
    HID_KBD_FLAGS                                     // Flags
};


// HID External Report Reference Descriptor
static uint16_t hidExtReportRefDesc = ESP_GATT_UUID_BATTERY_LEVEL;

// HID Report Reference characteristic descriptor, mouse input
static uint8_t hidReportRefMouseIn[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT };


// HID Report Reference characteristic descriptor, key input
static uint8_t hidReportRefKeyIn[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT };

// HID Report Reference characteristic descriptor, LED output
static uint8_t hidReportRefLedOut[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT };

#if (SUPPORT_REPORT_VENDOR  == true)

static uint8_t hidReportRefVendorOut[HID_REPORT_REF_LEN] =
             {HID_RPT_ID_VENDOR_OUT, HID_REPORT_TYPE_OUTPUT};
#endif

// HID Report Reference characteristic descriptor, Feature
static uint8_t hidReportRefFeature[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE };

// HID Report Reference characteristic descriptor, consumer control input
static uint8_t hidReportRefCCIn[HID_REPORT_REF_LEN] =
             { HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT };


/*
 *  Heart Rate PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// hid Service uuid
static uint16_t hid_le_svc = ATT_SVC_HID;
uint16_t            hid_count = 0;
esp_gatts_incl_svc_desc_t incl_svc = {0};

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
///the uuid definition
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t include_service_uuid = ESP_GATT_UUID_INCLUDE_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t hid_info_char_uuid = ESP_GATT_UUID_HID_INFORMATION;
static const uint16_t hid_report_map_uuid    = ESP_GATT_UUID_HID_REPORT_MAP;
static const uint16_t hid_control_point_uuid = ESP_GATT_UUID_HID_CONTROL_POINT;
static const uint16_t hid_report_uuid = ESP_GATT_UUID_HID_REPORT;
static const uint16_t hid_proto_mode_uuid = ESP_GATT_UUID_HID_PROTO_MODE;
static const uint16_t hid_kb_input_uuid = ESP_GATT_UUID_HID_BT_KB_INPUT;
static const uint16_t hid_kb_output_uuid = ESP_GATT_UUID_HID_BT_KB_OUTPUT;
static const uint16_t hid_mouse_input_uuid = ESP_GATT_UUID_HID_BT_MOUSE_INPUT;
static const uint16_t hid_repot_map_ext_desc_uuid = ESP_GATT_UUID_EXT_RPT_REF_DESCR;
static const uint16_t hid_report_ref_descr_uuid = ESP_GATT_UUID_RPT_REF_DESCR;
///the propoty definition
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_write_nr = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

/// battary Service
static const uint16_t battary_svc = ESP_GATT_UUID_BATTERY_SERVICE_SVC;

static const uint16_t bat_lev_uuid = ESP_GATT_UUID_BATTERY_LEVEL;
static const uint8_t   bat_lev_ccc[2] ={ 0x00, 0x00};
static const uint16_t char_format_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;

static uint8_t battary_lev = 50;
/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t bas_att_db[BAS_IDX_NB] =
{
    // Battary Service Declaration
    [BAS_IDX_SVC]               =  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
                                            sizeof(uint16_t), sizeof(battary_svc), (uint8_t *)&battary_svc}},

    // Battary level Characteristic Declaration
    [BAS_IDX_BATT_LVL_CHAR]    = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                                   CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    // Battary level Characteristic Value
    [BAS_IDX_BATT_LVL_VAL]             	= {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&bat_lev_uuid, ESP_GATT_PERM_READ,
                                                                sizeof(uint8_t),sizeof(uint8_t), &battary_lev}},

    // Battary level Characteristic - Client Characteristic Configuration Descriptor
    [BAS_IDX_BATT_LVL_NTF_CFG]     	=  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
                                                          sizeof(uint16_t),sizeof(bat_lev_ccc), (uint8_t *)bat_lev_ccc}},

    // Battary level report Characteristic Declaration
    [BAS_IDX_BATT_LVL_PRES_FMT]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
                                                        sizeof(struct prf_char_pres_fmt), 0, NULL}},
};


/// Full Hid device Database Description - Used to add attributes into the database
static esp_gatts_attr_db_t hidd_le_gatt_db[HIDD_LE_IDX_NB] =
{
            // HID Service Declaration
    [HIDD_LE_IDX_SVC]                       = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                                                             ESP_GATT_PERM_READ_ENCRYPTED, sizeof(uint16_t), sizeof(hid_le_svc),
                                                            (uint8_t *)&hid_le_svc}},

    // HID Service Declaration
    [HIDD_LE_IDX_INCL_SVC]               = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&include_service_uuid,
                                                            ESP_GATT_PERM_READ,
                                                            sizeof(esp_gatts_incl_svc_desc_t), sizeof(esp_gatts_incl_svc_desc_t),
                                                            (uint8_t *)&incl_svc}},

    // HID Information Characteristic Declaration
    [HIDD_LE_IDX_HID_INFO_CHAR]     = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                            ESP_GATT_PERM_READ,
                                                            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                            (uint8_t *)&char_prop_read}},
    // HID Information Characteristic Value
    [HIDD_LE_IDX_HID_INFO_VAL]       = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_info_char_uuid,
                                                            ESP_GATT_PERM_READ,
                                                            sizeof(hids_hid_info_t), sizeof(hidInfo),
                                                            (uint8_t *)&hidInfo}},

    // HID Control Point Characteristic Declaration
    [HIDD_LE_IDX_HID_CTNL_PT_CHAR]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                              ESP_GATT_PERM_READ,
                                                              CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                              (uint8_t *)&char_prop_write_nr}},
    // HID Control Point Characteristic Value
    [HIDD_LE_IDX_HID_CTNL_PT_VAL]    = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_control_point_uuid,
                                                             ESP_GATT_PERM_WRITE,
                                                             sizeof(uint8_t), 0,
                                                             NULL}},

    // Report Map Characteristic Declaration
    [HIDD_LE_IDX_REPORT_MAP_CHAR]   = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                              ESP_GATT_PERM_READ,
                                                              CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                              (uint8_t *)&char_prop_read}},
    // Report Map Characteristic Value
    [HIDD_LE_IDX_REPORT_MAP_VAL]     = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_map_uuid,
                                                              ESP_GATT_PERM_READ,
                                                              HIDD_LE_REPORT_MAP_MAX_LEN, sizeof(hidReportMap),
                                                              (uint8_t *)&hidReportMap}},

    // Report Map Characteristic - External Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_repot_map_ext_desc_uuid,
                                                                        ESP_GATT_PERM_READ,
                                                                        sizeof(uint16_t), sizeof(uint16_t),
                                                                        (uint8_t *)&hidExtReportRefDesc}},

    // Protocol Mode Characteristic Declaration
    [HIDD_LE_IDX_PROTO_MODE_CHAR]            = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                        ESP_GATT_PERM_READ,
                                                                        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                        (uint8_t *)&char_prop_read_write}},
    // Protocol Mode Characteristic Value
    [HIDD_LE_IDX_PROTO_MODE_VAL]               = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_proto_mode_uuid,
                                                                        (ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE),
                                                                        sizeof(uint8_t), sizeof(hidProtocolMode),
                                                                        (uint8_t *)&hidProtocolMode}},

    [HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR]       = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                         ESP_GATT_PERM_READ,
                                                                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                         (uint8_t *)&char_prop_read_notify}},

    [HIDD_LE_IDX_REPORT_MOUSE_IN_VAL]        = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       HIDD_LE_REPORT_MAX_LEN, 0,
                                                                       NULL}},

    [HIDD_LE_IDX_REPORT_MOUSE_IN_CCC]        = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
                                                                      (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
                                                                      sizeof(uint16_t), 0,
                                                                      NULL}},

    [HIDD_LE_IDX_REPORT_MOUSE_REP_REF]       = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       sizeof(hidReportRefMouseIn), sizeof(hidReportRefMouseIn),
                                                                       hidReportRefMouseIn}},
    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_KEY_IN_CHAR]         = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                         ESP_GATT_PERM_READ,
                                                                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                         (uint8_t *)&char_prop_read_notify}},
    // Report Characteristic Value
    [HIDD_LE_IDX_REPORT_KEY_IN_VAL]            = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       HIDD_LE_REPORT_MAX_LEN, 0,
                                                                       NULL}},
    // Report KEY INPUT Characteristic - Client Characteristic Configuration Descriptor
    [HIDD_LE_IDX_REPORT_KEY_IN_CCC]              = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
                                                                      (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
                                                                      sizeof(uint16_t), 0,
                                                                      NULL}},
     // Report Characteristic - Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_KEY_IN_REP_REF]       = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       sizeof(hidReportRefKeyIn), sizeof(hidReportRefKeyIn),
                                                                       hidReportRefKeyIn}},

     // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_LED_OUT_CHAR]         = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                         ESP_GATT_PERM_READ,
                                                                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                         (uint8_t *)&char_prop_read_write_write_nr}},

    [HIDD_LE_IDX_REPORT_LED_OUT_VAL]            = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
                                                                       ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
                                                                       HIDD_LE_REPORT_MAX_LEN, 0,
                                                                       NULL}},
    [HIDD_LE_IDX_REPORT_LED_OUT_REP_REF]      =  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       sizeof(hidReportRefLedOut), sizeof(hidReportRefLedOut),
                                                                       hidReportRefLedOut}},
#if (SUPPORT_REPORT_VENDOR  == true)
    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_VENDOR_OUT_CHAR]        = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                         ESP_GATT_PERM_READ,
                                                                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                         (uint8_t *)&char_prop_read_write_notify}},
    [HIDD_LE_IDX_REPORT_VENDOR_OUT_VAL]         = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
                                                                       ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
                                                                       HIDD_LE_REPORT_MAX_LEN, 0,
                                                                       NULL}},
    [HIDD_LE_IDX_REPORT_VENDOR_OUT_REP_REF]     = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       sizeof(hidReportRefVendorOut), sizeof(hidReportRefVendorOut),
                                                                       hidReportRefVendorOut}},
#endif
    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_CC_IN_CHAR]         = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                         ESP_GATT_PERM_READ,
                                                                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                         (uint8_t *)&char_prop_read_notify}},
    // Report Characteristic Value
    [HIDD_LE_IDX_REPORT_CC_IN_VAL]            = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       HIDD_LE_REPORT_MAX_LEN, 0,
                                                                       NULL}},
    // Report KEY INPUT Characteristic - Client Characteristic Configuration Descriptor
    [HIDD_LE_IDX_REPORT_CC_IN_CCC]              = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
                                                                      (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED),
                                                                      sizeof(uint16_t), 0,
                                                                      NULL}},
     // Report Characteristic - Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_CC_IN_REP_REF]       = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       sizeof(hidReportRefCCIn), sizeof(hidReportRefCCIn),
                                                                       hidReportRefCCIn}},

    // Boot Keyboard Input Report Characteristic Declaration
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                        ESP_GATT_PERM_READ,
                                                                        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                        (uint8_t *)&char_prop_read_notify}},
    // Boot Keyboard Input Report Characteristic Value
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL]   = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_kb_input_uuid,
                                                                        ESP_GATT_PERM_READ,
                                                                        HIDD_LE_BOOT_REPORT_MAX_LEN, 0,
                                                                        NULL}},
    // Boot Keyboard Input Report Characteristic - Client Characteristic Configuration Descriptor
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_NTF_CFG]  = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
                                                                              (ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE),
                                                                              sizeof(uint16_t), 0,
                                                                              NULL}},

    // Boot Keyboard Output Report Characteristic Declaration
    [HIDD_LE_IDX_BOOT_KB_OUT_REPORT_CHAR]    = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                              ESP_GATT_PERM_READ,
                                                                              CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                              (uint8_t *)&char_prop_read_write}},
    // Boot Keyboard Output Report Characteristic Value
    [HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL]      = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_kb_output_uuid,
                                                                              (ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE),
                                                                              HIDD_LE_BOOT_REPORT_MAX_LEN, 0,
                                                                              NULL}},

    // Boot Mouse Input Report Characteristic Declaration
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                              ESP_GATT_PERM_READ,
                                                                              CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                              (uint8_t *)&char_prop_read_notify}},
    // Boot Mouse Input Report Characteristic Value
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL]   = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_mouse_input_uuid,
                                                                              ESP_GATT_PERM_READ,
                                                                              HIDD_LE_BOOT_REPORT_MAX_LEN, 0,
                                                                              NULL}},
    // Boot Mouse Input Report Characteristic - Client Characteristic Configuration Descriptor
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG]    = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
                                                                                      (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
                                                                                      sizeof(uint16_t), 0,
                                                                                      NULL}},

    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_CHAR]                    = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
                                                                         ESP_GATT_PERM_READ,
                                                                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                                         (uint8_t *)&char_prop_read_write}},
    // Report Characteristic Value
    [HIDD_LE_IDX_REPORT_VAL]                      = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       HIDD_LE_REPORT_MAX_LEN, 0,
                                                                       NULL}},
    // Report Characteristic - Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_REP_REF]               = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_ref_descr_uuid,
                                                                       ESP_GATT_PERM_READ,
                                                                       sizeof(hidReportRefFeature), sizeof(hidReportRefFeature),
                                                                       hidReportRefFeature}},
};



/* Full Database Description - Used to add attributes into the database */

// Device Information Service Attribute Table
static esp_gatts_attr_db_t ble_gatt_info_db[IDX_NB_INFO] = {

    // primary service Device Information - IDX_SVC_INFO
    [IDX_SVC_INFO] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&primary_service_uuid, .perm = ESP_GATT_PERM_READ, .max_length = sizeof(device_info_service_uuid), .length = sizeof(device_info_service_uuid), .value = (uint8_t *)&device_info_service_uuid}},

    // manufacturer name characteristics declaration - IDX_CHAR_MANUF_NAME
    [IDX_CHAR_MANUF_NAME] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&character_declaration_uuid, .perm = ESP_GATT_PERM_READ, .max_length = CHAR_DECLARATION_SIZE, .length = CHAR_DECLARATION_SIZE, .value = (uint8_t *)&char_prop_read}},

    // manufacturer name characteristics value - IDX_CHAR_MANUF_NAME_VAL
    [IDX_CHAR_MANUF_NAME_VAL] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&manuf_name_character_uuid, .perm = ESP_GATT_PERM_READ, .max_length = sizeof(manuf_name), .length = sizeof(manuf_name), .value = (uint8_t *)manuf_name}},

    // model number characteristics declaration - IDX_CHAR_MODEL_NUM
    [IDX_CHAR_MODEL_NUM] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&character_declaration_uuid, .perm = ESP_GATT_PERM_READ, .max_length = CHAR_DECLARATION_SIZE, .length = CHAR_DECLARATION_SIZE, .value = (uint8_t *)&char_prop_read}},

    // model number characteristics value - IDX_CHAR_MODEL_NUM_VAL
    [IDX_CHAR_MODEL_NUM_VAL] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&model_num_character_uuid, .perm = ESP_GATT_PERM_READ, .max_length = sizeof(model_num), .length = sizeof(model_num), .value = (uint8_t *)model_num}},
};

// OTA Service Attribute Table
static esp_gatts_attr_db_t ble_gatt_ota_db[IDX_NB_OTA] = {

    // primary service OTA - IDX_SVC_OTA
    [IDX_SVC_OTA] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&primary_service_uuid, .perm = ESP_GATT_PERM_READ, .max_length = sizeof(ota_service_uuid), .length = sizeof(ota_service_uuid), .value = (uint8_t *)ota_service_uuid}},

    // OTA control characterisitics declaration - IDX_CHAR_CTRL
    [IDX_CHAR_CTRL] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&character_declaration_uuid, .perm = ESP_GATT_PERM_READ, .max_length = CHAR_DECLARATION_SIZE, .length = CHAR_DECLARATION_SIZE, .value = (uint8_t *)&char_prop_read_write_notify}},

    // OTA control characteristics value - IDX_CHAR_VAL_CTRL
    [IDX_CHAR_VAL_CTRL] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&CTRL_UUID, .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, .max_length = GATTS_CHAR_VAL_LEN_MAX, .length = sizeof(ota_ctrl), .value = (uint8_t *)&ota_ctrl}},

    // OTA control ccc descriptor - IDX_CHAR_CFG_CTRL
    [IDX_CHAR_CFG_CTRL] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&character_client_config_uuid, .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, .max_length = sizeof(uint16_t), .length = sizeof(ota_cccd), .value = (uint8_t *)&ota_cccd}},

    // OTA data characteristics declaration - IDX_CHAR_DATA
    [IDX_CHAR_DATA] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&character_declaration_uuid, .perm = ESP_GATT_PERM_READ, .max_length = CHAR_DECLARATION_SIZE, .length = CHAR_DECLARATION_SIZE, .value = (uint8_t *)&char_prop_write}},

    // OTA data characteristics value - IDX_CHAR_VAL_DATA
    [IDX_CHAR_VAL_DATA] = {{.auto_rsp = ESP_GATT_AUTO_RSP}, {.uuid_length = ESP_UUID_LEN_16, .uuid_p = (uint8_t *)&DATA_UUID, .perm = ESP_GATT_PERM_WRITE, .max_length = OTA_VAL_LEN_MAX, .length = sizeof(ble_ota_data), .value = (uint8_t *)ble_ota_data}},
};

static void hid_add_id_tbl(void);

void esp_hidd_prf_cb_hdl(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch(event) {
        case ESP_GATTS_REG_EVT: {
            esp_ble_gap_config_local_icon (ESP_BLE_APPEARANCE_GENERIC_HID);
            esp_hidd_cb_param_t hidd_param;
            hidd_param.init_finish.state = param->reg.status;
            if(param->reg.app_id == HIDD_APP_ID) {
                hidd_le_env.gatt_if = gatts_if;
                if(hidd_le_env.hidd_cb != NULL) {
                    (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_REG_FINISH, &hidd_param);
                    hidd_le_create_service(hidd_le_env.gatt_if);
                }
            }
            if(param->reg.app_id == BATTRAY_APP_ID) {
                hidd_param.init_finish.gatts_if = gatts_if;
                 if(hidd_le_env.hidd_cb != NULL) {
                    (hidd_le_env.hidd_cb)(ESP_BAT_EVENT_REG, &hidd_param);
                }

            }

            break;
        }
        case ESP_GATTS_CONF_EVT: {
            break;
        }
        case ESP_GATTS_CREATE_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT: {
            esp_hidd_cb_param_t cb_param = {0};
			ESP_LOGI(HID_LE_PRF_TAG, "HID connection establish, conn_id = %x",param->connect.conn_id);
			memcpy(cb_param.connect.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            cb_param.connect.conn_id = param->connect.conn_id;
            hidd_clcb_alloc(param->connect.conn_id, param->connect.remote_bda);
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
            if(hidd_le_env.hidd_cb != NULL) {
                (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_CONNECT, &cb_param);
            }
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
			 if(hidd_le_env.hidd_cb != NULL) {
                    (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_DISCONNECT, NULL);
             }
            hidd_clcb_dealloc(param->disconnect.conn_id);
            break;
        }
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_WRITE_EVT: {
            esp_hidd_cb_param_t cb_param = {0};
            if (param->write.handle == hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_LED_OUT_VAL]) {
                cb_param.led_write.conn_id = param->write.conn_id;
                cb_param.led_write.report_id = HID_RPT_ID_LED_OUT;
                cb_param.led_write.length = param->write.len;
                cb_param.led_write.data = param->write.value;
                (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT, &cb_param);
            }
#if (SUPPORT_REPORT_VENDOR == true)
            if (param->write.handle == hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VENDOR_OUT_VAL] &&
                hidd_le_env.hidd_cb != NULL) {
                cb_param.vendor_write.conn_id = param->write.conn_id;
                cb_param.vendor_write.report_id = HID_RPT_ID_VENDOR_OUT;
                cb_param.vendor_write.length = param->write.len;
                cb_param.vendor_write.data = param->write.value;
                (hidd_le_env.hidd_cb)(ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT, &cb_param);
            }
#endif
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->add_attr_tab.num_handle == BAS_IDX_NB &&
                param->add_attr_tab.svc_uuid.uuid.uuid16 == ESP_GATT_UUID_BATTERY_SERVICE_SVC &&
                param->add_attr_tab.status == ESP_GATT_OK) {
                incl_svc.start_hdl = param->add_attr_tab.handles[BAS_IDX_SVC];
                incl_svc.end_hdl = incl_svc.start_hdl + BAS_IDX_NB -1;
                ESP_LOGI(HID_LE_PRF_TAG, "%s(), start added the hid service to the stack database. incl_handle = %d",
                           __func__, incl_svc.start_hdl);
                esp_ble_gatts_create_attr_tab(hidd_le_gatt_db, gatts_if, HIDD_LE_IDX_NB, 0);
            }
            if (param->add_attr_tab.num_handle == HIDD_LE_IDX_NB &&
                param->add_attr_tab.status == ESP_GATT_OK) {
                memcpy(hidd_le_env.hidd_inst.att_tbl, param->add_attr_tab.handles,
                            HIDD_LE_IDX_NB*sizeof(uint16_t));
                ESP_LOGI(HID_LE_PRF_TAG, "hid svc handle = %x",hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
                hid_add_id_tbl();
		        esp_ble_gatts_start_service(hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
            } else {
                esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);
            }
            break;
         }

        default:
            break;
    }
}

void hidd_le_create_service(esp_gatt_if_t gatts_if)
{
    /* Here should added the battery service first, because the hid service should include the battery service.
       After finish to added the battery service then can added the hid service. */
    esp_ble_gatts_create_attr_tab(bas_att_db, gatts_if, BAS_IDX_NB, 0);

}

void hidd_le_init(void)
{

    // Reset the hid device target environment
    memset(&hidd_le_env, 0, sizeof(hidd_le_env_t));
}

void hidd_clcb_alloc (uint16_t conn_id, esp_bd_addr_t bda)
{
    uint8_t                   i_clcb = 0;
    hidd_clcb_t      *p_clcb = NULL;

    for (i_clcb = 0, p_clcb= hidd_le_env.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++) {
        if (!p_clcb->in_use) {
            p_clcb->in_use      = true;
            p_clcb->conn_id     = conn_id;
            p_clcb->connected   = true;
            memcpy (p_clcb->remote_bda, bda, ESP_BD_ADDR_LEN);
            break;
        }
    }
    return;
}

bool hidd_clcb_dealloc (uint16_t conn_id)
{
    uint8_t              i_clcb = 0;
    hidd_clcb_t      *p_clcb = NULL;

    for (i_clcb = 0, p_clcb= hidd_le_env.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++) {
            memset(p_clcb, 0, sizeof(hidd_clcb_t));
            return true;
    }

    return false;
}

extern void ota_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = esp_hidd_prf_cb_hdl,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
        [PROFILE_OTA_ID] = {
        // OTA Application Profile
        .gatts_cb = ota_profile_event_handler, // above declared profile handler function
        .gatts_if = ESP_GATT_IF_NONE,            // means that the Application Profile is not linked to any client yet
    },

};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        ESP_LOGI(HID_LE_PRF_TAG, "Reg app, app_id %04x, status %d",param->reg.app_id, param->reg.status);

        if (param->reg.status == ESP_GATT_OK)
        {   
            profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
            profile_tab[PROFILE_OTA_ID].gatts_if = gatts_if;
            
        }
        else
        {
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) 
        {
            if (gatts_if == ESP_GATT_IF_NONE ||gatts_if == profile_tab[idx].gatts_if) 
            {
                /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */

                if (profile_tab[idx].gatts_cb) {
                    profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


esp_err_t hidd_register_cb(void)
{
	esp_err_t status;
	status = esp_ble_gatts_register_callback(gatts_event_handler);
	return status;
}

void hidd_set_attr_value(uint16_t handle, uint16_t val_len, const uint8_t *value)
{
    hidd_inst_t *hidd_inst = &hidd_le_env.hidd_inst;
    if(hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle &&
        hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle) {
        esp_ble_gatts_set_attr_value(handle, val_len, value);
    } else {
        ESP_LOGE(HID_LE_PRF_TAG, "%s error:Invalid handle value.",__func__);
    }
    return;
}

void hidd_get_attr_value(uint16_t handle, uint16_t *length, uint8_t **value)
{
    hidd_inst_t *hidd_inst = &hidd_le_env.hidd_inst;
    if(hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle &&
        hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle){
        esp_ble_gatts_get_attr_value(handle, length, (const uint8_t **)value);
    } else {
        ESP_LOGE(HID_LE_PRF_TAG, "%s error:Invalid handle value.", __func__);
    }

    return;
}

static void hid_add_id_tbl(void)
{
     // Mouse input report
      hid_rpt_map[0].id = hidReportRefMouseIn[0];
      hid_rpt_map[0].type = hidReportRefMouseIn[1];
      hid_rpt_map[0].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL];
      hid_rpt_map[0].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL];
      hid_rpt_map[0].mode = HID_PROTOCOL_MODE_REPORT;

      // Key input report
      hid_rpt_map[1].id = hidReportRefKeyIn[0];
      hid_rpt_map[1].type = hidReportRefKeyIn[1];
      hid_rpt_map[1].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_VAL];
      hid_rpt_map[1].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_CCC];
      hid_rpt_map[1].mode = HID_PROTOCOL_MODE_REPORT;

      // Consumer Control input report
      hid_rpt_map[2].id = hidReportRefCCIn[0];
      hid_rpt_map[2].type = hidReportRefCCIn[1];
      hid_rpt_map[2].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_CC_IN_VAL];
      hid_rpt_map[2].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_CC_IN_CCC];
      hid_rpt_map[2].mode = HID_PROTOCOL_MODE_REPORT;

      // LED output report
      hid_rpt_map[3].id = hidReportRefLedOut[0];
      hid_rpt_map[3].type = hidReportRefLedOut[1];
      hid_rpt_map[3].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_LED_OUT_VAL];
      hid_rpt_map[3].cccdHandle = 0;
      hid_rpt_map[3].mode = HID_PROTOCOL_MODE_REPORT;

      // Boot keyboard input report
      // Use same ID and type as key input report
      hid_rpt_map[4].id = hidReportRefKeyIn[0];
      hid_rpt_map[4].type = hidReportRefKeyIn[1];
      hid_rpt_map[4].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL];
      hid_rpt_map[4].cccdHandle = 0;
      hid_rpt_map[4].mode = HID_PROTOCOL_MODE_BOOT;

      // Boot keyboard output report
      // Use same ID and type as LED output report
      hid_rpt_map[5].id = hidReportRefLedOut[0];
      hid_rpt_map[5].type = hidReportRefLedOut[1];
      hid_rpt_map[5].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL];
      hid_rpt_map[5].cccdHandle = 0;
      hid_rpt_map[5].mode = HID_PROTOCOL_MODE_BOOT;

      // Boot mouse input report
      // Use same ID and type as mouse input report
      hid_rpt_map[6].id = hidReportRefMouseIn[0];
      hid_rpt_map[6].type = hidReportRefMouseIn[1];
      hid_rpt_map[6].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL];
      hid_rpt_map[6].cccdHandle = 0;
      hid_rpt_map[6].mode = HID_PROTOCOL_MODE_BOOT;

      // Feature report
      hid_rpt_map[7].id = hidReportRefFeature[0];
      hid_rpt_map[7].type = hidReportRefFeature[1];
      hid_rpt_map[7].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VAL];
      hid_rpt_map[7].cccdHandle = 0;
      hid_rpt_map[7].mode = HID_PROTOCOL_MODE_REPORT;


  // Setup report ID map
  hid_dev_register_reports(HID_NUM_REPORTS, hid_rpt_map);
}



/*BLE security keys exchanged between devices for pairing/bonding*/
static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
    char *key_str = NULL;
    switch (key_type)
    {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC: // Peer Encryption Key
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID: // Peer Identity Resolving Key
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC: // Local Encryption Key
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID: // Local Identity Resolving Key
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;
    }

    return key_str;
}

/*BLE security authentication mode requested by device for connection with peer*/
static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
    char *auth_str = NULL;
    switch (auth_req)
    {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND: // Secure connections and Bonding support
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND: // Secure connections, Man In The Middle protection, Bonding supported
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
    }

    return auth_str;
}



/**
 * @brief GATT profile event handler(gets called for every individual profiles.....)
 * @param event : Event type
 * @param gatts_if : GATT server access interface, normally
 *                   different gatts_if correspond to different profile
 * @param param : callback parameter, currently is union type
 * @retval None
 */

void ota_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGV(TAG, "OTA event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:                                                                   // triggered on gatt app profile register                                           // set device name to local device
        esp_ble_gap_config_local_privacy(true);                                               // triggers ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT
        esp_ble_gatts_create_attr_tab(ble_gatt_info_db, gatts_if, IDX_NB_INFO, INFO_INST_ID); // create attribute table (Device Info service first)
                                                                                              // triggers ESP_GATTS_CREAT_ATTR_TAB_EVT
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "Read Event");
        break;
    case ESP_GATTS_WRITE_EVT: // triggered when client attempts to write characteristics/descriptor value

        ESP_LOGI(TAG, "attribute handle:%d, len:%d", param->write.handle, param->write.len);

        if (param->write.handle == ble_ota[IDX_CHAR_CFG_CTRL] && param->write.len == 2)
        { // cccd value
            ESP_LOGI(TAG, "OTA CCCD");
            esp_log_buffer_hex(TAG, param->write.value, 2);
            ota_flag = param->write.value[0] | (param->write.value[1] << 8);
        }
        else if (param->write.handle == ble_ota[IDX_CHAR_VAL_CTRL] && param->write.len == 1)
        { // ctrl value, example - OTA_REQUEST to start ota process
            ESP_LOGI(TAG, "OTA CTRL");
            esp_log_buffer_hex(TAG, param->write.value, 1);
            ota_ctrl_val = param->write.value[0];
        }
        else if (param->write.handle == ble_ota[IDX_CHAR_VAL_DATA] && updating == false)
        { // ota data packet size update
            ESP_LOGI(TAG, "OTA PKT SIZE");
            esp_log_buffer_hex(TAG, param->write.value, 2);
            packet_size = param->write.value[0] | (param->write.value[1] << 8);
        }
        else if (param->write.handle == ble_ota[IDX_CHAR_VAL_DATA] && updating == true)
        { // ota data main firmware push (updating flag set in ota_main task)
            memset(ble_ota_data, 0, sizeof(ble_ota_data));
            memcpy(ble_ota_data, param->write.value, packet_size); // copy received data to buffer
            // write received data to partition
            esp_err_t err = esp_ota_write(update_handle, (const void *)ble_ota_data, packet_size);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_ota_write failed (%s)!",
                         esp_err_to_name(err));
            }
            num_pkgs_received++;
            ESP_LOGI(TAG, "Received packet %d", num_pkgs_received);
        }
        else
        {
            // nop
            __asm__ __volatile__("nop");
        }

        break;
    case ESP_GATTS_MTU_EVT: // triggered after client request for updating MTU
        ESP_LOGI(TAG, "for connection id:%u", param->mtu.conn_id);
        ESP_LOGI(TAG, "requested MTU size from client:%u", param->mtu.mtu);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT");
        // esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        /* start advertising again when missing the connect */
        esp_ble_gap_start_advertising(&esp32_ble_adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    { // triggered after create attr command
        ESP_LOGI(TAG, "The number handle = %x", param->add_attr_tab.num_handle);
        if (param->create.status == ESP_GATT_OK)
        {
            if (param->add_attr_tab.num_handle == IDX_NB_INFO)
            { // device information service
                memcpy(ble_device_info, param->add_attr_tab.handles, sizeof(ble_device_info));
                esp_ble_gatts_start_service(ble_device_info[IDX_SVC_INFO]);
                ESP_LOGI(TAG, "ATT Table\n");
                ESP_LOGI(TAG, "%s-%d", "IDX_SVC_INFO", ble_device_info[IDX_SVC_INFO]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_MANUF_NAME", ble_device_info[IDX_CHAR_MANUF_NAME]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_MANUF_NAME_VAL", ble_device_info[IDX_CHAR_MANUF_NAME_VAL]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_MODEL_NUM", ble_device_info[IDX_CHAR_MODEL_NUM]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_MODEL_NUM_VAL", ble_device_info[IDX_CHAR_MODEL_NUM_VAL]);
                esp_ble_gatts_create_attr_tab(ble_gatt_ota_db, gatts_if, IDX_NB_OTA, OTA_INST_ID); // create ota attribute table next

            }
            else if (param->add_attr_tab.num_handle == IDX_NB_OTA)
            { // ota service
                memcpy(ble_ota, param->add_attr_tab.handles, sizeof(ble_ota));
                esp_ble_gatts_start_service(ble_ota[IDX_SVC_OTA]);
                ESP_LOGI(TAG, "%s-%d", "IDX_SVC_OTA", ble_ota[IDX_SVC_OTA]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_CTRL", ble_ota[IDX_CHAR_CTRL]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_VAL_CTRL", ble_ota[IDX_CHAR_VAL_CTRL]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_CFG_CTRL", ble_ota[IDX_CHAR_CFG_CTRL]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_DATA", ble_ota[IDX_CHAR_DATA]);
                ESP_LOGI(TAG, "%s-%d", "IDX_CHAR_VAL_DATA", ble_ota[IDX_CHAR_VAL_DATA]);
            }
            else
            {
                // NOP
                __asm__ __volatile__("nop");
            }
        }
        else
        {
            ESP_LOGE(TAG, " Create attribute table failed, error code = %x", param->create.status);
        }
        break;
    }
    default:
        break;
    }
}



/**
 * @brief  diagnostic function to check new firmware
 * @param  None
 * @retval None
 * @note   basic bluetooth init function is implemented as a diagnostic to check boot
 * time status of new firmware updated via OTA
 */
bool diagnostic(void)
{
    int ret = gpio_get_level(TEST_GPIO);
    if (!ret)
    {
        return false;
    }
    else
    {
        gpio_set_level(TEST_GPIO, 0);
    }

    return true;
}

/**
 * @brief  Timer Callback function
 * @param  None
 * @retval None
 * @note   called after 30 second from boot and checks the ota partition state and
 *         undertakes action accordingly
 */
void vTimerCallback(TimerHandle_t xTimer)
{

    /*get current running partition*/
    const esp_partition_t *app_partition = esp_ota_get_running_partition();
    esp_err_t ret;
    esp_ota_img_states_t ota_app_state;
    ret = esp_ota_get_state_partition(app_partition, &ota_app_state);
    if (ret == ESP_OK)
    {
        if (ota_app_state == ESP_OTA_IMG_PENDING_VERIFY)
        { // CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE option enabled.
            // run diagnostics function
            bool diagnostics_is_ok = diagnostic();
            if (diagnostics_is_ok)
            {
                ESP_LOGI(TAG, "Diagnostic successful!! application is running..");
                esp_ota_mark_app_valid_cancel_rollback();
            }
            else
            {
                ESP_LOGE(TAG, "Diagnostic failed!! rolling back to last app..");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
        else if (ota_app_state == ESP_OTA_IMG_VALID)
        {
            ESP_LOGI(TAG, "running image already verified...........................");
        }
        else
        {
            // nop
            __asm__ __volatile__("nop");
        }
    }

    gpio_set_level(TEST_GPIO, 0); // exit with led status turned low
}





void ota_manager(void *)
{
    
    ESP_LOGI(TAG, "OTA MANAGER STARTED...");


    const esp_partition_t *app_partition = esp_ota_get_running_partition();
    esp_err_t ret;

    switch (app_partition->address)
    { // look into starting address of partition in flash
    case 0x10000:
        ESP_LOGI(TAG, "running factory app..............................................");
        break;
    case 0x110000:
        ESP_LOGI(TAG, "running ota_0 app................................................");
        break;
    case 0x210000:
        ESP_LOGI(TAG, "running ota_1 app................................................");
        break;
    default:
        ESP_LOGI(TAG, "running unknown app..............................................");
        break;
    }

    // create one shot timer of 30 seconds to check firmware stability
    xTimerOTA = xTimerCreate("OTA timer", pdMS_TO_TICKS(30000), pdFALSE, (void *)0, vTimerCallback);
    if (xTimerStart(xTimerOTA, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "timer start failed!!");
    }
    else
    {
        ESP_LOGI(TAG, "timer started sucessfully!!");
    }

    // for diagnostics check
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TEST_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(TEST_GPIO, 1); // set logic high


    /*
    ***********************************************************************************************
    ***************************************Task Creation*******************************************
    ***********************************************************************************************
    */

    /*create task for ota*/
   esp_err_t err, ack;

    while (1)
    {

        /*check ota control characteristics descriptor*/
        if (ota_flag == 0x0001)
        {

            uint8_t ack_val = 0;
            if (ota_ctrl_val == OTA_REQUEST)
            {
                update_partition = esp_ota_get_next_update_partition(NULL);                        // get the next free OTA partition
                err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle); // start the ota update

                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "esp_ota_begin failed (%s)",
                             esp_err_to_name(err));
                    esp_ota_abort(update_handle);
                    ack_val = OTA_REQUEST_NAK;
                }
                else
                {
                    ack_val = OTA_REQUEST_ACK;
                    updating = true;                                  // flag changes to true and upcoming data recognized as firmware chunks
                    ESP_LOGI(TAG, "Packet size is: %d", packet_size); // retrieve the packet size from OTA data
                    num_pkgs_received = 0;
                }

                ack = esp_ble_gatts_send_indicate(profile_tab[PROFILE_OTA_ID].gatts_if, conn_id, ble_ota[IDX_CHAR_VAL_CTRL], sizeof(ack_val), (uint8_t *)&ack_val, false);
                if (ack == 0)
                {
                    ESP_LOGI(TAG, "sent data:%d", ack);
                    ESP_LOGI(TAG, "conn_id:%d", conn_id);
                }
                ESP_LOGI(TAG, "OTA request acknowledgement has been sent.");
                ota_ctrl_val = 0xff;
            }
            else if (ota_ctrl_val == OTA_DONE)
            {
                updating = false;
                err = esp_ota_end(update_handle); // end the OTA and start validation
                if (err != ESP_OK)
                {
                    if (err == ESP_ERR_OTA_VALIDATE_FAILED)
                    {
                        ESP_LOGE(TAG, "Image validation failed, image is corrupted!");
                    }
                    else
                    {
                        ESP_LOGE(TAG, "esp_ota_end failed (%s)!",
                                 esp_err_to_name(err));
                    }
                }
                else
                {
                    err = esp_ota_set_boot_partition(update_partition); // select the new partition for the next boot
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!",
                                 esp_err_to_name(err));
                    }
                }

                // set the control value
                if (err != ESP_OK)
                {
                    ack_val = OTA_DONE_NAK;
                }
                else
                {
                    ack_val = OTA_DONE_ACK;
                }
                ack = esp_ble_gatts_send_indicate(profile_tab[PROFILE_OTA_ID].gatts_if, conn_id, ble_ota[IDX_CHAR_VAL_CTRL], sizeof(ack_val), (uint8_t *)&ack_val, false);
                if (ack == 0)
                {
                    ESP_LOGI(TAG, "sent data:%d", ack);
                    ESP_LOGI(TAG, "conn_id:%d", conn_id);
                }
                ESP_LOGI(TAG, "OTA DONE acknowledgement has been sent.");
                if (err == ESP_OK)
                {
                    ESP_LOGI(TAG, "Preparing to restart!");
                    vTaskDelay(pdMS_TO_TICKS(REBOOT_DEEP_SLEEP_TIMEOUT));
                    esp_restart(); // restart the ESP to finish the OTA
                }
            }
        }
        else
        {
            // nop
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}