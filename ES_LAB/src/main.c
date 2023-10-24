#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "rtc_wdt.h"
// #include "FreeRTOSConfig.h"

#include <string.h>




/************************************************
*               LAB ID DEFINE                   * 
*************************************************/
#define     LAB3
// #define     LAB2
// #define     LAB1

/************************************************
*               NOTE                            * 
*************************************************/
//    To change the scheduler type, we change the value of 
//     configUSE_PREEMPTION and configUSE_TIME_SLICING 
//     in the FreeRTOSConfig.h
//         1.Prioritized Pre-emptive Scheduling with Time Slicing
//             configUSE_PREEMPTION   1
//             configUSE_TIME_SLICING 1

//         2.Prioritized Pre-emptive Scheduling without Time Slicing
//             configUSE_PREEMPTION   1
//             configUSE_TIME_SLICING 0

//         3.Co-operative Scheduling
//             configUSE_PREEMPTION   0
//             configUSE_TIME_SLICING 0

        



/************************************************
*               DEFINE                          * 
*************************************************/
#define UART_PIN_TX                         (1)
#define UART_PIN_RX                         (3)
#define UART_BAUDRATE                       (115200)
#define UART_BUFFER_SIZE                    (200)


/* Disable watchdog */
#define CONFIG_ESP_TASK_WDT_INIT                        (0)
#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0        (0)
#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1        (0)





/************************************************
*               VARIABLES DEFINE                * 
*************************************************/
TaskHandle_t Task1_Handler;
TaskHandle_t Task2_Handler;
TaskHandle_t Task3_Handler;
TaskHandle_t Task4_Handler;

static QueueHandle_t uartQueue_QueueHandler;


// static char uartRxBuff[200];
// static char uartTxBuff[200];


/************************************************
*              SUB FUNCTION DEFINE              * 
*************************************************/
void UART0_Init(void);
void UART0_WriteBytes(char* str, uint16_t length);
uint8_t UART0_ReadBytes(char* buff);




/************************************************
*               TASK DEFINE                     * 
*************************************************/
void Task1(void* param);
void Task2(void* param);
void Task3(void* param);
void Task4(void* param);


/************************************************
*               MAIN FUNCTION                   * 
*************************************************/
void app_main(void)
{
    
    /* Disable watchdog */
    
    rtc_wdt_disable();
    rtc_wdt_protect_off();

    UART0_Init();    

    // xQueueCreate(5, sizeof(msgQueueTx));

    xTaskCreatePinnedToCore(Task1, "Task1", 2048, NULL, 4, &Task1_Handler, 0);
    xTaskCreatePinnedToCore(Task2, "Task2", 2048, NULL, 4, &Task2_Handler, 0);
    xTaskCreatePinnedToCore(Task3, "Task3", 2048, NULL, 6, &Task3_Handler, 1);
    xTaskCreatePinnedToCore(Task4, "Task4", 2048, NULL, 8, &Task4_Handler, 1);



    while(1){

    }

    
    esp_restart();
}












void Task1(void* param){
    uint16_t i;
    uint32_t timeCount1;
    char str[100];

    timeCount1 = 0;
    i = 0;

    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1){
        while (timeCount1++ < 10000000);
        timeCount1 = 0;

        UART0_WriteBytes(str, sprintf(str, "%d.Task1\n", i++));
    
    }
    

    vTaskDelete(NULL);
}

void Task2(void* param){
    uint16_t i;
    uint32_t timeCount2;
    char str[100];

    timeCount2 = 0;
    i = 0;

    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1){
        while (timeCount2++ < 10000000);
        timeCount2 = 0;

        UART0_WriteBytes(str, sprintf(str, "%d.Task2\n", i++));
    
    }
    

    vTaskDelete(NULL);
}

void Task3(void* param){
    uint16_t i;
    uint32_t timeCount3;
    char str[100];

    timeCount3 = 0;
    i = 0;

    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1){
        while (timeCount3++ < 10000000);
        timeCount3 = 0;

        UART0_WriteBytes(str, sprintf(str, "%d.Task3\n", i++));
    
    }
    

    vTaskDelete(NULL);
}


void Task4(void* param){
    uint16_t i;
    uint32_t timeCount4;
    char str[100];

    timeCount4 = 0;
    i = 0;

    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1){
        while (timeCount4++ < 10000000);
        timeCount4 = 0;

        UART0_WriteBytes(str, sprintf(str, "%d.Task4\n", i++));
    
    }
    

    vTaskDelete(NULL);
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