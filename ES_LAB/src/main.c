#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define  LAB1


void app_main(void)
{

    

    while(1){
        printf("Hello world!\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    
    esp_restart();
}
