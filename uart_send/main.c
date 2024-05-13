
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "time.h"
#include "string.h"

#include "pn532.h"

#include <sys/time.h>
#include "esp_system.h"
#include "esp_event.h"

#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"

#include <esp_wifi.h>
#include <stdbool.h>
#include "freertos/queue.h"
#include "esp_intr_types.h"

static void obtain_time(void);
void ntp_time();
void init_wifi(void);
time_t now;

static QueueHandle_t gpio_evt_queue = NULL;

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM 2
#define ECHO_UART_BAUD_RATE 115200
#define ECHO_TASK_STACK_SIZE 2048

static const char *TAG = "UART TEST";

char strftime_buf[64];
const int retry_count = 15;
bool button;
bool released;

#define BUF_SIZE (1024)

#define BLINK_GPIO 2

#define PN532_SCK 19
#define PN532_MOSI 23
#define PN532_SS 22
#define PN532_MISO 18

static pn532_t nfc;

// SENDER ESP

typedef struct Mag_info
{
    char rfid[50];
    int mag_id;
    int sku;
    int qty_remain;
    char status[20];
    long long int install_time;
    //  time_t exit_time;
} mag_info;

mag_info mag_uno;
mag_info mag_dos;


void ntp_time()
{

    while (1)
    {

        // time_t now;
        struct tm timeinfo;           // create structure tm to store time details like hours, min...
        time(&now);                   // gives time in seconds since epoch
        localtime_r(&now, &timeinfo); // converts epoch time to local time
        // Is time set? If not, tm_year will be (1970 - 1900).
        if (timeinfo.tm_year < (2016 - 1900))
        {
            ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
            obtain_time(); // if correct time is not set, then get the time from ntp server.
            time(&now);    // update 'now' variable with current time
        }

        // char strftime_buf[64];

        // Set timezone to Indian Standard Time and print local time
        setenv("TZ", "IST-5.5", 1);
        tzset();
        //  localtime_r(&now, &timeinfo);
        // strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        // ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
        now += 19800;
        // ESP_LOGI(TAG, "The current date/time is: %llu", now);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void obtain_time(void) // get the time from ntp server
{
    ESP_ERROR_CHECK(nvs_flash_init());                // default nvs partition initialized
    ESP_ERROR_CHECK(esp_netif_init());                // initialize tcp/ip protocols
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // default event loop created

    //     /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    //      * Read "Establishing Wi-Fi or Ethernet Connection" section in
    //      * examples/protocols/README.md for more information about this function.
    //      */

    ESP_ERROR_CHECK(example_connect()); // function to configure Wifi through menuconfig

    //      This is the basic default config with one server and starting the service
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org"); // configures to one default ntp server

    esp_netif_sntp_init(&config); // initialize sntp

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    time(&now);
    localtime_r(&now, &timeinfo); // converts the epoch time from ntp to local time

    ESP_ERROR_CHECK(example_disconnect()); // disconnects wifi
    esp_netif_sntp_deinit();               // deinitialize netif module
}

char *struct2str(mag_info mg)
{
    /* get lenght of string required to hold struct values */
    size_t len = 0;
    len = snprintf(NULL, len, "%s,%d,%d,%d,%s", mg.rfid, mg.mag_id, mg.sku, mg.qty_remain,
                   mg.status);

    /* allocate/validate string to hold all values (+1 to null-terminate) */
    char *magstr = calloc(1, sizeof *magstr * len + 1);
    if (!magstr)
    {
        fprintf(stderr, "%s() error: virtual memory allocation failed.\n", __func__);
    }

    /* write/validate struct values to magstr */
    if (snprintf(magstr, len + 1, "%s,%d,%d,%d,%s", mg.rfid, mg.mag_id, mg.sku, mg.qty_remain,
                 mg.status) > len + 1)
    {
        fprintf(stderr, "%s() error: snprintf returned truncated result.\n", __func__);
        return NULL;
    }

    return magstr;
}

void gpio_init(void)
{
    //  gpio_set_direction(GPIO_NUM_18, GPIO_MODE_INPUT);
    gpio_config_t mag_detect = {
        .pin_bit_mask = BIT64(GPIO_NUM_18),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&mag_detect);
}

static void IRAM_ATTR gpioHandler(void *arg)
{   
     uint32_t gpio_num = (uint32_t)arg;
     if (gpio_get_level(gpio_num)==1) {
            button = true;
     }

     if (gpio_get_level(gpio_num)==0) {
            released = true;
     }

}





void gpio_detect(void *pv)
{

    while (1)
    {
        if (gpio_get_level(GPIO_NUM_18) == 1)
        {
            ESP_LOGI(TAG, "button pressed");
            button = true;
            ESP_LOGI(TAG, "button state:%d", button);
        }
        vTaskDelay(100);
        button = false;
    }
}

static void echo_task(void *arg)
{

    pn532_spi_init(&nfc, 21, 23, 22, 19); // clock, miso, mosi, ss
    pn532_begin(&nfc);

    uint32_t versiondata = pn532_getFirmwareVersion(&nfc);
    if (!versiondata)
    {
        ESP_LOGI(TAG, "Didn't find PN53x board");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    pn532_SAMConfig(&nfc);

    ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");
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

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, 4, 5, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // mag_uno = {"a3bpopo", 30105, 341, 100, "installed"};
    // mag_uno.rfid="a3bpopo";
    mag_uno.mag_id = 30105;
    mag_uno.sku = 341;
    mag_uno.qty_remain = 100;
    //mag_uno.status = "installed";

    mag_dos.mag_id = 30105;
    mag_dos.sku = 341;
    mag_dos.qty_remain = 100;

    char mag_str[200];
    char mag_str_dos[200];

    // strncpy(mag_str, struct2str(mag_uno),200);

    while (1)
    {
        uint8_t success;
        uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
        uint8_t uidLength;                     // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
        char rfid[50];

        // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
        // 'uid' will be populated with the UID, and uidLength will indicate
        // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
        success = pn532_readPassiveTargetID(&nfc, PN532_MIFARE_ISO14443A, uid, &uidLength, 0);

        if (success)
        {
            // Display some basic information about the card
            ESP_LOGI(TAG, "Found an ISO14443A card");
            ESP_LOGI(TAG, "UID Length: %d bytes", uidLength);
            ESP_LOGI(TAG, "UID Value:%s", uid);
            // esp_log_buffer_hexdump_internal(TAG, uid, uidLength, ESP_LOG_INFO);

            mag_uno.install_time = now;
            mag_dos.install_time = now;

            sprintf(rfid, "%x%x%x%x", uid[0], uid[1], uid[2], uid[3]);
            ESP_LOGI(TAG, "UID Value after conversion to str: %s", rfid);
            memcpy(mag_uno.rfid, rfid, 50);
            memcpy(mag_dos.rfid, rfid, 50);

            char install[20]= "installed" ;
            memcpy(&mag_uno.status, install, 20);

            char release[20]= "released." ;
            memcpy(&mag_dos.status, release, 20);
            
            ESP_LOGI(TAG, "UID Value after conversion from mag_info struct: %s", mag_uno.rfid);

            sprintf(mag_str, "%s%d%d%d%s%llu", mag_uno.rfid, mag_uno.mag_id, mag_uno.sku, mag_uno.qty_remain, mag_uno.status, mag_uno.install_time);

            sprintf(mag_str_dos, "%s%d%d%d%s%llu", mag_dos.rfid, mag_dos.mag_id, mag_dos.sku, mag_dos.qty_remain, mag_dos.status, mag_dos.install_time);
            ESP_LOGI(TAG, "the magazine string is: %s. Size of string: %d", mag_str, sizeof(mag_str));
            // vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            // PN532 probably timed out waiting for a card
            ESP_LOGI(TAG, "Timed out waiting for a card");
        }

        if (button == true)
        {
           
            int ret = uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)mag_str, sizeof(mag_str));
            ESP_LOGI(TAG, "%d bytes written to TX buffer.", ret);
            //uart_flush(ECHO_UART_PORT_NUM);
        }

       /* if (released == true)
        {   

            int ret2 = uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)mag_str_dos, sizeof(mag_str_dos));
            ESP_LOGI(TAG, "%d bytes written to TX buffer.", ret2);
            uart_flush(ECHO_UART_PORT_NUM);
        }
        */ 

        uart_flush(ECHO_UART_PORT_NUM);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}



void app_main(void)
{
    ESP_LOGI(TAG, "here");
      xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE*3, NULL, 10, NULL);
      xTaskCreate(ntp_time, "time", ECHO_TASK_STACK_SIZE*2, NULL, 5, NULL);
    // xTaskCreate(gpio_detect, "detect", 2048, NULL, 5, NULL);

    gpio_config_t mag_detect = {
        .pin_bit_mask = BIT64(GPIO_NUM_15),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&mag_detect);

    gpio_set_intr_type(GPIO_NUM_15, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_NUM_15, gpioHandler, (void *)GPIO_NUM_15);

    ESP_LOGI(TAG, "here after gpio funcs");
    ESP_LOGI(TAG, "button val is %d", button);

    while (1)
    {
        if (button == true)
        {
            ESP_LOGI(TAG, "button pressed");
            button = false;
        }

        if (released == true)
        {
            ESP_LOGI(TAG, "button released");
            released = false;
        }

        vTaskDelay(100);
    }
}
