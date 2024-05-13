/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "time.h"
#include <string.h>
/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)

static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)




// RECEIVER ESP





static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, 14, 13, ECHO_TEST_RTS, ECHO_TEST_CTS));

    typedef struct Mag_info
    {
        char rfid[50];
        char mag_id[20];
        char sku[10];
        char qty_remain[10];
        char status[20];
        char install_time[15];
        //time_t exit_time;
    } mag_info;

    // Configure a temporary buffer for the incoming data
    char *data = (char *)malloc(BUF_SIZE);

    // void *ptrVoid= (mag_info*)malloc(sizeof(mag_info));

    // mag_info *magptr= (mag_info *) ptrVoid;

    mag_info *mag_var;

    while (1)
    {
        // Read data from the UART

        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, 200, portMAX_DELAY);

      //  ESP_LOGI(TAG, "no. of bytes read from buffer, len= %d", len);

      //  if (len)
      //  {
            // ESP_LOGI(TAG, "Received rfid: %d", mag_var->rfid);
           // ESP_LOGI(TAG, "Received data string: %s", data);          
       // }
        uart_flush(ECHO_UART_PORT_NUM);


      void slice(const char *str, char *result, size_t start, size_t end)
            {
                strncpy(result, str + start, end - start);
            }
            
            char slicedFoo[10] = "";
            mag_info mag_uno;
            data[38]= '\0';
            
            slice(data, slicedFoo, 0, 8);
            strncpy(&mag_uno.rfid, slicedFoo, 50);
           // ESP_LOGI(TAG, "rfid= %s\n", mag_uno.rfid);
            strncpy(slicedFoo, "",10);
            
//ed77102f30105341100installed1715344248.
            slice(data, slicedFoo, 8, 13);
            strncpy(&mag_uno.mag_id, slicedFoo, 20);
           // ESP_LOGI(TAG, "mag_id= %s\n", mag_uno.mag_id);
            strncpy(slicedFoo, "",10);

            
            slice(data, slicedFoo, 13, 16);
            strncpy(&mag_uno.sku, slicedFoo, 10);   
            strncpy(slicedFoo, "",10);

            slice(data, slicedFoo, 16, 19);
            strncpy(&mag_uno.qty_remain, slicedFoo, 10);
            strncpy(slicedFoo, "",10);

            slice(data, slicedFoo, 19, 28);
            strncpy(&mag_uno.status, slicedFoo, 20);
            strncpy(slicedFoo, "",10);

            slice(data, slicedFoo, 28, 39);
            strncpy(&mag_uno.install_time, slicedFoo, 15);
           // strncpy(slicedFoo, "",10);
            
        if (len) {
            ESP_LOGI(TAG, "\n \n MAGAZINE DETECTED. \n");
            ESP_LOGI(TAG, "The RFID= %s", mag_uno.rfid);
            ESP_LOGI(TAG, "The Magazine ID= %s", mag_uno.mag_id);
            ESP_LOGI(TAG, "The SKU= %s", mag_uno.sku);
            ESP_LOGI(TAG, "The quantity remaining= %s", mag_uno.qty_remain);
            ESP_LOGI(TAG, "The status of the magazine is %s", mag_uno.status);
            ESP_LOGI(TAG, "Time: %s", mag_uno.install_time);
            }
    }



}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE*2, NULL, 10, NULL);
}

// Write data back to the UART
//  uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
// if (len) {
//     //data[len] = '\0';
//     data[len]= '\0';
//     ESP_LOGI(TAG, "Recv str: %s", data);
// }

// vTaskDelay(pdMS_TO_TICKS(500));
//