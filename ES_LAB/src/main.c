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
#define     LAB4
// #define     LAB3
// #define     LAB2
// #define     LAB1

/************************************************
*               DEFINE                          * 
*************************************************/
#define UART_BUFFER_SIZE                    (2048)


/************************************************
*               VARIABLES                       * 
*************************************************/
QueueHandle_t uartQueueHandler;



TaskHandle_t SendMsg2Queue_Handler;
TaskHandle_t RevMsgFromQueue_Handler;

int a, b;
uint8_t uartLength;
char uartRxBuff[200];

/************************************************
*              SUB FUNCTION DEFINE              * 
*************************************************/
void UART0_Init(void);
void UART0_WriteBytes(char* str, uint16_t length);
uint8_t UART0_ReadBytes(char* buff);


/************************************************
*               TASK DEFINE                     * 
*************************************************/
void SendMsg2Queue(void* param);
void RevMsgFromQueue(void* param);





/************************************************
*               MAIN FUNCTION                   * 
*************************************************/
void app_main(void)
{
    UART0_Init();

    // printf("Begin\n");
    
    // xTaskCreatePinnedToCore(PrintStuID, "PrintStuID", 2048, NULL, 4, &PrintStuID_Handler, 0);
    // xTaskCreatePinnedToCore(BTPolling, "BTPolling", 2048, NULL, 3, &BTPolling_Handler, 1);

    while(1){

        UART0_WriteBytes("asdads", 6);

        uartLength = UART0_ReadBytes(&uartRxBuff[0]);
        if(uartLength > 0){
            UART0_WriteBytes(&uartRxBuff[0], uartLength);
        }


        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // scanf("%s", &msg[0]);
        // if(msg )
        // printf("Get:%d, %d", a, b);
        // msg = getchar();
        // printf(msg);
    }
    
    
    esp_restart();
}


void SendMsg2Queue(void* param){


    while(1){


    }


    vTaskDelete(NULL);
}

void RevMsgFromQueue(void* param){



    while(1){

    }

    vTaskDelete(NULL);
}









void UART0_Init(void){
    uart_config_t uartConfig = {
        .baud_rate = 115200,
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
    ESP_ERROR_CHECK( uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) ); 

    /* Install uart driver */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10,
                                &uartQueueHandler, 0) );

}


void UART0_WriteBytes(char* str, uint16_t length){
    uart_tx_chars(UART_NUM_0, (const char*)str, length);
}

uint8_t UART0_ReadBytes(char* buff){
    uint16_t length;

    ESP_ERROR_CHECK( uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length) );

    if(length == 0){
        return 0;
    }

    length = uart_read_bytes(UART_NUM_0, buff, length, 100);


    return length;
}