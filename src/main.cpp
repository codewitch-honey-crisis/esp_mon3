#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <stdio.h>
#include <stdint.h>
#include "esp_lcd_panel_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
extern "C" void app_main();
#endif
#include <uix.hpp>
using namespace gfx;
using namespace uix;
#include <ui.hpp>
#include <interface.hpp>
#ifdef M5STACK_CORE2
#include <m5core2_power.hpp>
#endif
#include "serial.hpp"
// label string data
static char cpu_sz[32];
static char gpu_sz[32];

// so we don't redraw temps
static int old_cpu_temp=-1;
static int old_gpu_temp=-1;

// signal timer for disconnection detection
static uint32_t timeout_ts = 0;

// indicated the connection status
static bool connected = false;

#ifdef M5STACK_CORE2
m5core2_power power;
#endif

static uint32_t get_ms() {
#ifdef ARDUINO
    return millis();
#else
    return ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount()));
#endif
}

void update_all() {
    // timeout for disconnection detection (1 second)
    if(timeout_ts!=0 && get_ms()>timeout_ts+1000) {
        timeout_ts = 0;
        display_screen(disconnected_screen);
        connected = false;
    }
    // listen for incoming serial
    bool done = false;
    uint8_t tmp;
    read_status_t old_data;
    memset(&old_data,0,sizeof(old_data));
    while(!done) {        
        read_status_t data;
        if(serial_read_packet(&data)) { // if data received...
            // reset the disconnect timeout
            timeout_ts = get_ms(); 
            if(!connected) {
                display_screen(main_screen);
                connected = true;
            }
            // update the CPU graph buffer (usage)
            if (cpu_buffers[0].full()) {
                cpu_buffers[0].get(&tmp);
            }
            cpu_buffers[0].put((data.cpu_usage/100.0f)*255);
            // update the bar and label values (usage)
            cpu_values[0]=data.cpu_usage/100.0f;
            // update the CPU graph buffer (temperature)
            if (cpu_buffers[1].full()) {
                cpu_buffers[1].get(&tmp);
            }
            cpu_buffers[1].put((data.cpu_temp/(float)data.cpu_temp_max)*255);
            if(data.cpu_temp>cpu_max_temp) {
                cpu_max_temp = data.cpu_temp;
            }
            // update the bar and label values (temperature)
            cpu_values[1]=data.cpu_temp/(float)data.cpu_temp_max;
            // force a redraw of the CPU bar and graph
            cpu_graph.invalidate();
            if(data.cpu_temp!=old_data.cpu_temp || data.cpu_usage!=old_data.cpu_usage) {
                cpu_bar.invalidate();
            }
            // update CPU the label (temperature)
            if(old_cpu_temp!=data.cpu_temp) {
                old_cpu_temp = data.cpu_temp;
                sprintf(cpu_sz,"%dC",data.cpu_temp);
                cpu_temp_label.text(cpu_sz);
            }
            // update the GPU graph buffer (usage)
            if (gpu_buffers[0].full()) {
                gpu_buffers[0].get(&tmp);
            }
            gpu_buffers[0].put((data.gpu_usage/100.0f)*255);
            // update the bar and label values (usage)
            gpu_values[0] = data.gpu_usage/100.0f;
            // update the GPU graph buffer (temperature)
            if (gpu_buffers[1].full()) {
                gpu_buffers[1].get(&tmp);
            }
            gpu_buffers[1].put((data.gpu_temp/(float)data.gpu_temp_max)*255);
            if(data.gpu_temp>gpu_max_temp) {
                gpu_max_temp = data.gpu_temp;
            }
            // update the bar and label values (temperature)
            gpu_values[1] = data.gpu_temp/(float)data.gpu_temp_max;
            // force a redraw of the GPU bar and graph
            gpu_graph.invalidate();
            if(data.gpu_temp!=old_data.gpu_temp || data.gpu_usage!=old_data.gpu_usage) {
                gpu_bar.invalidate();
            }
            // update GPU the label (temperature)
            if(old_gpu_temp!=data.gpu_temp) {
                old_gpu_temp = data.gpu_temp;
                sprintf(gpu_sz,"%dC",data.gpu_temp);
                gpu_temp_label.text(gpu_sz);   
            }
        } else {
            done = true;
        }
        old_data=data;
    }
    display_update();
}
void initialize_common() {
    display_init();
    serial_init();
    esp_log_level_set("application", ESP_LOG_INFO);
    puts("Booted");
    // initialize the disconnected screen (ui.cpp)
    disconnected_screen_init();
    
    // initialize the main screen (ui.cpp)
    main_screen_init();

    display_screen(disconnected_screen);
}
#ifdef ARDUINO
void setup() {
    // enable the power pins, as necessary
#ifdef T_DISPLAY_S3
    pinMode(15, OUTPUT); 
    digitalWrite(15, HIGH);
#elif defined(S3_T_QT)
    pinMode(4, OUTPUT); 
    digitalWrite(4, HIGH);
#endif
#ifdef M5STACK_CORE2
    power.initialize();
#endif

    initialize_common();
}

void loop() {
    update_all();    
}
#else
void loop_task(void* state) {
    while(true) {
        update_all();
        vTaskDelay(5);
    }
}
void app_main() {
    // open stdin as binary
    freopen(NULL, "rb", stdin);
#ifdef T_DISPLAY_S3
    gpio_set_direction((gpio_num_t)15,GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)15,1);
#elif defined(S3_T_QT)
    gpio_set_direction((gpio_num_t)4,GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)4,1);
#endif
#ifdef M5STACK_CORE2
    power.initialize();
#endif
    initialize_common();
    TaskHandle_t htask = nullptr;
    xTaskCreate(loop_task,"loop_task",5000,nullptr,uxTaskPriorityGet(nullptr),&htask);
    if(htask==nullptr) {
        printf("Unable to create loop task\n");
    }
    vTaskSuspend(nullptr);
}
#endif