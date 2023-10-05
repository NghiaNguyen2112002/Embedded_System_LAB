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

/************************************************
*               LAB ID DEFINE                   * 
*************************************************/
// #define     LAB5
// #define     LAB4
// #define     LAB3
// #define     LAB2
#define     LAB1

 

/************************************************
*               DEFINE                          * 
*************************************************/
#define UART_PIN_TX                         (1)
#define UART_PIN_RX                         (3)
#define UART_BAUDRATE                       (115200)
#define UART_BUFFER_SIZE                    (200)


/* Disable watchdog */
#define CONFIG_ESP_TASK_WDT_INIT 0

/************************************************
*               VARIABLES                       * 
*************************************************/


/************************************************
*              SUB FUNCTION DEFINE              * 
*************************************************/


/************************************************
*               TASK DEFINE                     * 
*************************************************/


 
/************************************************
*               CALLBACK FUNCTION DEFINE        * 
*************************************************/



/************************************************
*               MAIN FUNCTION                   * 
*************************************************/


void app_main(void)
{
    TickType_t lastWakeup;

    /* Disable watchdog */
    rtc_wdt_disable();
    rtc_wdt_protect_off();

    lastWakeup = xTaskGetTickCount();
    

    while(1){
        printf("Hello world!\n");
        vTaskDelayUntil(&lastWakeup, 1000 / portTICK_PERIOD_MS);
    }

    
    esp_restart();
}
