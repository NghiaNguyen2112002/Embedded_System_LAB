#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/************************************************
*               LAB ID DEFINE                   * 
*************************************************/
#define     LAB3
// #define     LAB2
// #define     LAB1

/************************************************
*               VARIABLES DEFINE                * 
*************************************************/
TaskHandle_t PrintStuID_Handler;
TaskHandle_t BTPolling_Handler;


/************************************************
*               FUNCTION DEFINE                 * 
*************************************************/
void PrintStuID(void* param);
void BTPolling(void* param);



/************************************************
*               MAIN FUNCTION                   * 
*************************************************/
void app_main(void)
{

    printf("Begin\n");
    
    xTaskCreatePinnedToCore(PrintStuID, "PrintStuID", 2048, NULL, 4, &PrintStuID_Handler, 0);
    xTaskCreatePinnedToCore(BTPolling, "BTPolling", 2048, NULL, 3, &BTPolling_Handler, 1);


    while(1){

    }

    
    esp_restart();
}



void PrintStuID(void* param){
    uint16_t i = 0;

    while(1){
        printf("%d.Nguyen Trung Nghia 2013875\n", i++);
        // vTaskDelay(1000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);

}

void BTPolling(void* param){
    uint16_t i = 0;

    gpio_config_t gpioConfig;
    /* Config BUTTON pin */
    gpioConfig.pin_bit_mask = (1 << 18);
    gpioConfig.mode = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpioConfig);

    while(1){
        while(gpio_get_level(18) != 0);
        printf("%d.ESP32\n", i++);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    
    vTaskDelete(NULL);
}