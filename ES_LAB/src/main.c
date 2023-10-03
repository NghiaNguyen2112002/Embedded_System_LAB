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


#include <string.h>


/************************************************
*               LAB ID DEFINE                   * 
*************************************************/
#define     LAB5
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
#define CONFIG_ESP_TASK_WDT_INIT 0


/************************************************
*               VARIABLES                       * 
*************************************************/

// static TimerHandle_t onsTimer0_Handler;
static TimerHandle_t aurTimer0_Handler;
static TimerHandle_t aurTimer1_Handler;

static QueueHandle_t uartQueue_QueueHandler;


// static char uartRxBuff[200];
static char uartTxBuff[200];

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
void SoftTimerCallback(TimerHandle_t timerID);


/************************************************
*               MAIN FUNCTION                   * 
*************************************************/
void app_main(void)
{
    uint16_t i;

    /* Disable watchdog */
    rtc_wdt_disable();
    rtc_wdt_protect_off();

    UART0_Init();    

    // onsTimer0_Handler = xTimerCreate("onsTM0", 1000 / portTICK_PERIOD_MS, pdFALSE, (void*)0, SoftTimerCallback);
    aurTimer0_Handler = xTimerCreate("aurTM0", 500 / portTICK_PERIOD_MS, pdTRUE, (void*)0, SoftTimerCallback);
    aurTimer1_Handler = xTimerCreate("aurTM1", 1500 / portTICK_PERIOD_MS, pdTRUE, (void*)0, SoftTimerCallback);

    // xTimerStart(onsTimer0_Handler, (TickType_t)0);
    xTimerStart(aurTimer0_Handler, (TickType_t)0);
    xTimerStart(aurTimer1_Handler, (TickType_t)0);


    while(1){

    }
    
    
    esp_restart();
}




void SoftTimerCallback(TimerHandle_t xTimer){
    uint8_t dmy;


    /* Increase timerID until it reach the number of desired expired times */
    dmy = pvTimerGetTimerID(xTimer);
    vTimerSetTimerID(xTimer, dmy + 1);

    if(xTimer == aurTimer0_Handler){
        if(dmy < 10){
            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "%d.Auto Reload timer 0 triggered!\n", dmy));
        }
        else {
            xTimerStop(xTimer, (TickType_t)0);
        }
    }
    else if(xTimer == aurTimer1_Handler){
        if(dmy < 5){
            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "%d.Auto Reload timer 1 triggered!\n", dmy));
        }
        else {
            xTimerStop(xTimer, (TickType_t)0);
        }
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