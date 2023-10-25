#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "rtc_wdt.h"
#include "freertos/timers.h"

#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include <string.h>

/************************************************
*                   NOTE                        *
*    This lab has not had requirements yet      *
*        So i tested the wifi scanner           *
*           and wifi AP, STA                    *
*                                               * 
*************************************************/
               

/************************************************
*               LAB ID DEFINE                   * 
*************************************************/
#define     LAB6
// #define     LAB5
// #define     LAB4
// #define     LAB3
// #define     LAB2
// #define     LAB1

 

/************************************************
*               DEFINE                          * 
*************************************************/
#define UART_PIN_TX                         (1)
#define UART_PIN_RX                         (3)
#define UART_BAUDRATE                       (115200)
#define UART_BUFFER_SIZE                    (200)


/* Disable watchdog */
#define CONFIG_ESP_TASK_WDT_INIT            0
#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0        (0)
#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1        (0)



#define STA_NAME                           "MANGDAY KTX H1-518 4G"
#define STA_PASS                           "20202024"

#define AP_NAME                             "Test AP ESP32"
#define AP_PASS                             "123456789"

/************************************************
*               VARIABLES                       * 
*************************************************/
uint16_t apNum;
wifi_ap_record_t apRecord[10];

char wifiName[20];
char wifiPass[20];

static QueueHandle_t uartQueue_QueueHandler;


// static char uartRxBuff[200];
static char uartTxBuff[100];

static uint8_t queueTxBuff[100];


/************************************************
*              SUB FUNCTION DEFINE              * 
*************************************************/
void UART0_Init(void);
void UART0_WriteBytes(char* str, uint16_t length);
uint8_t UART0_ReadBytes(char* buff);


/************************************************
*               TASK DEFINE                     * 
*************************************************/


 
/************************************************
*               CALLBACK FUNCTION DEFINE        * 
*************************************************/
void WifiEventHandler(void* arg, esp_event_base_t eventBase, int32_t eventId, void* eventData);



/************************************************
*               MAIN FUNCTION                   * 
*************************************************/
void app_main(void)
{

    uint16_t i;
    uint16_t msgLength;
    char msg[20];

    nvs_flash_init();


    /* Disable watchdog */
    rtc_wdt_protect_off();    // Turns off the automatic wdt service
    rtc_wdt_disable();         // Turn it on manually

    /* Init uart for debugging */
    UART0_Init();    

    /* Event handler */
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, WifiEventHandler, NULL);


    /* Init Wifi */
    wifi_init_config_t  wifiInitConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&wifiInitConfig) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_APSTA) );
    
    wifi_config_t wifiConfigSta = {
        .sta = {
            .ssid = STA_NAME,
            .password = STA_PASS,
            .bssid_set = 0
        },
    };

    wifi_config_t wifiConfigAp = {
        .ap = {
            .ssid = AP_NAME,
            .password = AP_PASS,
            .ssid_len = 0,
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .ssid_hidden = 0,
            .max_connection = 4,
            .beacon_interval = 100
        }
    };

    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifiConfigSta) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &wifiConfigAp) );

    ESP_ERROR_CHECK( esp_wifi_start() );

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_wifi_scan_start(NULL, 1);

    vTaskDelay(10000 / portTICK_PERIOD_MS);
    esp_wifi_connect();


    i = 0;
    while(1){
        
        // if((i < apNum) && (i < 10)){
        //     UART0_WriteBytes( uartTxBuff, sprintf(uartTxBuff, "%d. %d | %d | %s\n", 
        //                                 i, apRecord[i].primary, apRecord[i].rssi, apRecord[i].ssid));
        //     i++;
        // }

        // vTaskDelay(100);
    }

    
    
    
    esp_restart();
}







void WifiEventHandler(void* arg, esp_event_base_t eventBase, int32_t eventId, void* eventData){
    uint16_t i;


    if(eventBase != WIFI_EVENT){
        return;
    }

    
    switch(eventId){
        case WIFI_EVENT_STA_START:
            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "Wifi STA Started!\n"));

        break;
        case WIFI_EVENT_AP_START:
            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "Wifi AP Started!\n"));

        break;
        case WIFI_EVENT_SCAN_DONE:
            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "Wifi Scanner Started!\n"));
           
            esp_wifi_scan_get_ap_num(&apNum);
            esp_wifi_scan_get_ap_records(&apNum, apRecord);
                
            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "Wifi found: %d\n", apNum));

            i = 0;
            while((i < apNum) && (i < 10)){
                UART0_WriteBytes( uartTxBuff, sprintf(uartTxBuff, "%d. %d | %d | %s\n", 
                                            i, apRecord[i].primary, apRecord[i].rssi, apRecord[i].ssid));
                i++;

                vTaskDelay(100);
            }
        break;
        case WIFI_EVENT_STA_CONNECTED:
            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "Wifi connected!\n"));
        break;
        case WIFI_EVENT_STA_DISCONNECTED:
            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "Wifi disconnected!\n"));

        break;
    }
    


}




void UART0_Init(void){
    uart_config_t uartConfig = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB
    };

    /* Config uart param */
    ESP_ERROR_CHECK( uart_param_config(UART_NUM_0, &uartConfig) );

    /* Assign PIN */
    // Set UART pins(TX, RX, RTS, CTS)
    ESP_ERROR_CHECK( uart_set_pin(UART_NUM_0, UART_PIN_TX, UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) ); 

    /* Install uart driver */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10,
                                &uartQueue_QueueHandler, 0) );

}


void UART0_WriteBytes(char* str, uint16_t length){
    uart_tx_chars(UART_NUM_0, (const char*)str, length);
}

uint8_t UART0_ReadBytes(char* buff){
    uint8_t length;

    ESP_ERROR_CHECK( uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length) );

    if(length == 0){
        return 0;
    }

    length = uart_read_bytes(UART_NUM_0, buff, length, 100);
    return length;
}