#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"

/**
 * This is an example which echos any data it receives on UART back to the sender using RS485 interface in half duplex mode.
 */
#define TAG "RS485_ECHO_APP"

// Configure pins according to your requirements
#define ECHO_TEST_TXD           (17)  // Set TX pin to GPIO17
#define ECHO_TEST_RXD           (16)  // Set RX pin to GPIO16
#define ECHO_TEST_RTS           (33)  // Set RTS pin to GPIO33

#define ECHO_TEST_CTS           (UART_PIN_NO_CHANGE)
#define BUF_SIZE                (127)
#define BAUD_RATE               (CONFIG_ECHO_UART_BAUD_RATE)
#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
#define ECHO_TASK_STACK_SIZE    (CONFIG_ECHO_TASK_STACK_SIZE)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART_PORT          (CONFIG_ECHO_UART_PORT_NUM)
#define ECHO_READ_TOUT          (3)

static void echo_send(const int port, const char* str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length) {
        ESP_LOGE(TAG, "Send data critical failure.");
        abort();
    }
}

static void echo_task(void *arg)
{
    const int uart_num = ECHO_UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
    ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));

    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    ESP_LOGI(TAG, "UART start receive loop.\r");
    echo_send(uart_num, "Start RS485 UART test.\r\n", 24);

    while (1) {
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);
        if (len > 0) {
            echo_send(uart_num, "\r\n", 2);
            char prefix[] = "RS485 Received: [";
            echo_send(uart_num, prefix, (sizeof(prefix) - 1));
            ESP_LOGI(TAG, "Received %u bytes:", len);
            printf("[ ");
            for (int i = 0; i < len; i++) {
                printf("0x%.2X ", (uint8_t)data[i]);
                echo_send(uart_num, (const char*)&data[i], 1);
                if (data[i] == '\r') {
                    echo_send(uart_num, "\n", 1);
                }
            }
            printf("] \n");
            echo_send(uart_num, "]\r\n", 3);
        } else {
            // Send the message "kla" when no data is received
            const char* message = "kla";
            echo_send(uart_num, message, strlen(message));  // Send "kla"
            ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10)); // Wait for transmission to finish
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
}
