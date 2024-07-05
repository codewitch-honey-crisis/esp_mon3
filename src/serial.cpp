#include "interface.hpp"
#include <memory.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#define SERIAL_QUEUE_SIZE 64
#define SERIAL_BUF_SIZE (sizeof(read_status_t)*SERIAL_QUEUE_SIZE+SERIAL_QUEUE_SIZE)
static QueueHandle_t serial_queue;
static QueueHandle_t serial_out_queue;
const char* TAG = "Serial";
static void serial_task(void *arg)
{
    read_status_t status;
    uart_event_t event;
    size_t size = 0;
    uint8_t* p;
    int s;
    int pos;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(SERIAL_BUF_SIZE);
    bzero(dtmp, SERIAL_BUF_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(serial_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                p=dtmp+size; 
                s = uart_read_bytes(UART_NUM_0, p, event.size, portMAX_DELAY);
                if(s>0) {
                    size+=s;
                    //ESP_LOGI(TAG, "[SERIAL DATA RECEIVED]");
                    //puts("[Serial data received]");
                    if(size>=sizeof(read_status_t)) {
                        memcpy(&status,p,sizeof(read_status_t));
                        if(size>sizeof(read_status_t)) {
                            memmove(dtmp,dtmp+sizeof(read_status_t),size-sizeof(read_status_t));
                            size-=sizeof(read_status_t);
                            --size;
                        }
                        xQueueSend(serial_out_queue,&status,portMAX_DELAY);
                        //ESP_LOGI(TAG, "[PACKET POSTED]");
                        //puts("[Packet posted]");
                    }
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                //ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(serial_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                //ESP_LOGI(TAG, "ring buffer full");
                puts("[Serial buffer full]");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(serial_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(UART_NUM_0, &buffered_size);
                pos = uart_pattern_pop_pos(UART_NUM_0);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(UART_NUM_0);
                } else {
                    uart_read_bytes(UART_NUM_0, dtmp, pos, 100 / portTICK_PERIOD_MS);
                }
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

bool serial_read_packet(read_status_t* out_status) {
    if(pdTRUE==xQueueReceive(serial_out_queue,out_status,0)) {
        return true;
    }
    return false;
}

bool serial_init() {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    TaskHandle_t task_handle = nullptr;
    serial_out_queue = xQueueCreate(SERIAL_QUEUE_SIZE,sizeof(read_status_t));
    if(serial_out_queue==nullptr) {
        ESP_LOGE(TAG,"Unable to create serial out queue");
        goto error;
    }
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config;
    memset(&uart_config,0,sizeof(uart_config));
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    //Install UART driver, and get the queue.
    if(ESP_OK!=uart_driver_install(UART_NUM_0, SERIAL_BUF_SIZE * 2, SERIAL_BUF_SIZE * 2, 20, &serial_queue, 0)) {
        ESP_LOGE(TAG,"Unable to install uart driver");
        goto error;
    }
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Create a task to handler UART event from ISR
    
    xTaskCreate(serial_task, "serial_task", 2048, NULL, 12, &task_handle);
    if(task_handle==nullptr) {
        ESP_LOGE(TAG,"Unable to create uart event task");
        goto error;
    }
    return true;
error:
    if(serial_out_queue!=nullptr) {
        vQueueDelete(serial_out_queue);
        serial_out_queue = nullptr;
    }
    if(serial_queue!=nullptr) {
        uart_driver_delete(UART_NUM_0);
        serial_queue = nullptr;
    }
    if(task_handle!=nullptr) {
        vTaskDelete(task_handle);
    }
    return false;
}