#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "rtc_wdt.h"

#include <string.h>


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
#define UART_PIN_TX                         (1)
#define UART_PIN_RX                         (3)
#define UART_BAUDRATE                       (115200)
#define UART_BUFFER_SIZE                    (200)

#define NO_RECEIVER                         (3)

/* Disable watchdog */
#define CONFIG_ESP_TASK_WDT_INIT 0

/************************************************
*               VARIABLES                       * 
*************************************************/
static QueueHandle_t uartQueue_QueueHandler;
static QueueHandle_t msgQueue_QueueHandler[NO_RECEIVER];

static TaskHandle_t MsgDistributer_TaskHandler;
static TaskHandle_t MsgReceiver_TaskHandler[NO_RECEIVER];

uint16_t i;

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
void MsgQueueDistribution(void* param);
void MsgQueueReception_0(void* param);
void MsgQueueReception_1(void* param);
void MsgQueueReception_2(void* param);

 


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

    for(i = 0; i < NO_RECEIVER; i++){
        msgQueue_QueueHandler[i] = xQueueCreate(5, sizeof(queueTxBuff) / sizeof(uint8_t));

        
        if(msgQueue_QueueHandler[i] == 0){
            UART0_WriteBytes( uartTxBuff,  sprintf(uartTxBuff, "Failed to create queue at: %d!\n", i) );
        }

    }


    xTaskCreatePinnedToCore(MsgQueueDistribution, "MsgDis", 2048, NULL, 4, &MsgDistributer_TaskHandler, 1);
    xTaskCreatePinnedToCore(MsgQueueReception_0, "MsgRev1", 2048, NULL, 3, &MsgReceiver_TaskHandler[0], 0);
    xTaskCreatePinnedToCore(MsgQueueReception_1, "MsgRev2", 2048, NULL, 3, &MsgReceiver_TaskHandler[1], 0);
    xTaskCreatePinnedToCore(MsgQueueReception_2, "MsgRev3", 2048, NULL, 3, &MsgReceiver_TaskHandler[2], 0);




    while(1){

    }
    
    
    esp_restart();
}


void MsgQueueDistribution(void* param){
    char rxBuff[200];
    char processBuff[200];
    
    uint8_t rxBuffLength;
    uint8_t revId = 0;


    while(1){
        rxBuffLength = UART0_ReadBytes(&rxBuff[0]);
        if(rxBuffLength > 0){
            // UART0_WriteBytes(&rxBuff[0], rxBuffLength);

            /*
             *  Process MSG
             *  Received format: [id]:[msg]
             *  Send format: [msgLength]:[msg]
             */
            
            /* Detect ID */
            for(i = 0; i < rxBuffLength; i++){
                processBuff[i] = rxBuff[i];
                if(rxBuff[i] == ':'){
                    processBuff[i] = '\0';
                    revId = atoi(processBuff);
                    break;
                }
            }

            rxBuff[rxBuffLength] = '\0';
            
            sprintf(processBuff, "%d%s", (rxBuffLength-i-1), rxBuff + i);

            if(i >= rxBuffLength){
                UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff,"ERROR: Wrong format!\n"));
            }
            else if(revId >= NO_RECEIVER){
                UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff,"ERROR: Task ID is invalid!\n"));
            }
            else if(xQueueSend(msgQueue_QueueHandler[revId], processBuff, 0) != pdTRUE){
                UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff,"ERROR: Queue is full!\n"));
            }

           
            
            /* clear buffer */
            // memset(uartRxBuff, 0, 200 * sizeof(char));
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }


    vTaskDelete(NULL);
}


void MsgQueueReception_0(void* param){
    char revBuff[100];
    char processBuff[4];

    uint8_t i;
    uint8_t revBuffLength = 0;

    while(1){

        if(xQueueReceive(msgQueue_QueueHandler[0], &(revBuff), (TickType_t)5)){

            /*
             *  Process MSG
             *  Received format: [msgLength]:[msg]
             *  debug send format: [msg]
             */

            for(i = 0; i < 4; i++){
                processBuff[i] = revBuff[i];

                if(revBuff[i] == ':'){
                    processBuff[i] = '\0';
                    revBuffLength = atoi(processBuff);
                    break;
                }
            }

            revBuff[i + revBuffLength + 1] = '\0';

            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "ID 0 Received: %s\n", revBuff + i + 1));

            /* clear revBuff */
            // memset(revBuff, 0, 100*sizeof(char));

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }


    }

    vTaskDelete(NULL);
}

void MsgQueueReception_1(void* param){
    char revBuff[100];
    char processBuff[4];

    uint8_t i;
    uint8_t revBuffLength = 0;

    while(1){

        if(xQueueReceive(msgQueue_QueueHandler[1], &(revBuff), (TickType_t)5)){

            /*
             *  Process MSG
             *  Received format: [msgLength]:[msg]
             *  debug send format: [msg]
             */

            for(i = 0; i < 4; i++){
                processBuff[i] = revBuff[i];

                if(revBuff[i] == ':'){
                    processBuff[i] = '\0';
                    revBuffLength = atoi(processBuff);
                    break;
                }
            }

            revBuff[i + revBuffLength + 1] = '\0';

            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "ID 1 Received: %s\n", revBuff + i + 1));

            /* clear revBuff */
            // memset(revBuff, 0, 100*sizeof(char));

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }


    }

    vTaskDelete(NULL);
}

void MsgQueueReception_2(void* param){
     char revBuff[100];
    char processBuff[4];

    uint8_t i;
    uint8_t revBuffLength = 0;

    while(1){

        if(xQueueReceive(msgQueue_QueueHandler[2], &(revBuff), (TickType_t)5)){

            /*
             *  Process MSG
             *  Received format: [msgLength]:[msg]
             *  debug send format: [msg]
             */

            for(i = 0; i < 4; i++){
                processBuff[i] = revBuff[i];

                if(revBuff[i] == ':'){
                    processBuff[i] = '\0';
                    revBuffLength = atoi(processBuff);
                    break;
                }
            }

            revBuff[i + revBuffLength + 1] = '\0';

            UART0_WriteBytes(uartTxBuff, sprintf(uartTxBuff, "ID 2 Received: %s\n", revBuff + i + 1));

            /* clear revBuff */
            // memset(revBuff, 0, 100*sizeof(char));

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }


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