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
#define     LAB2
// #define     LAB1


/************************************************
*               DEFINE                          * 
*************************************************/
#define UART_PIN_TX                         (1)
#define UART_PIN_RX                         (3)
#define UART_BAUDRATE                       (115200)
#define UART_BUFFER_SIZE                    (200)

#define PIN_BUTTON                          (18)

/* Disable watchdog */
#define CONFIG_ESP_TASK_WDT_INIT 0


/************************************************
*               VARIABLES                       * 
*************************************************/
TaskHandle_t PrintStuID_Handler;
TaskHandle_t BTPolling_Handler;



/************************************************
*              SUB FUNCTION DEFINE              * 
*************************************************/



/************************************************
*               TASK DEFINE                     * 
*************************************************/
void PrintStuID(void* param);
void BTPolling(void* param);

 
/************************************************
*               CALLBACK FUNCTION DEFINE        * 
*************************************************/



/************************************************
*               MAIN FUNCTION                   * 
*************************************************/
void app_main(void)
{

   /* Disable watchdog */
    rtc_wdt_disable();
    rtc_wdt_protect_off();

    printf("Begin\n");
    
    xTaskCreate(PrintStuID, "PrintStuID", 2048, NULL, 4, &PrintStuID_Handler);
    xTaskCreate(BTPolling, "BTPolling", 2048, NULL, 3, &BTPolling_Handler);


    while(1){

    }

    
    esp_restart();
}



void PrintStuID(void* param){
    uint16_t i = 0;
    TickType_t lastWakeup;


    while(1){
        lastWakeup = xTaskGetTickCount();

        printf("%d.Nguyen Trung Nghia 2013875\n", i++);
        // vTaskDelay(1000);
        vTaskDelayUntil(&lastWakeup,  1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);

}

void BTPolling(void* param){
    uint16_t i = 0;

    gpio_config_t gpioConfig;
    /* Config BUTTON pin */
    gpioConfig.pin_bit_mask = (1 << PIN_BUTTON);
    gpioConfig.mode = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpioConfig);

    while(1){
        while(gpio_get_level(PIN_BUTTON) != 0);
        while(gpio_get_level(PIN_BUTTON) == 0);

        printf("%d.ESP32\n", i++);

    }

    vTaskDelete(NULL);
}