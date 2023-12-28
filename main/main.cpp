#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "hal/gpio_hal.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "bluetooth_handler.h"
#include "screen_handler.h"

#include <chrono>
#include <atomic>

#define CLK_PIN 17
#define DT_PIN 15
#define SW_PIN 33
#define BATTERY_LVL_PIN 27

#define ISR_PINS (1 << CLK_PIN | 1 << SW_PIN)


extern "C"{
    void app_main(void);
}


//QueueHandle_t qHandle;
QueueHandle_t qHandle = xQueueCreate( 10, sizeof(bool));

BluetoothHandler* btInstance = BluetoothHandler::getInstance();
ScreenHandler* displayInstance = ScreenHandler::getInstance();

std::atomic<bool> increaseVolume{false};
std::atomic<bool> toggle{false};
SemaphoreHandle_t btnEventSem{};

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    static int clk_prev{1};
    int clk_curr = gpio_get_level(static_cast<gpio_num_t>(CLK_PIN));
    int bt_curr = gpio_get_level(static_cast<gpio_num_t>(DT_PIN));
    int step = 5;
    static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); 

    if(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() >= 5000 && clk_prev != clk_curr){

        if(bt_curr != clk_curr)
            increaseVolume = true;
        else
            increaseVolume = false;
        xQueueSend(qHandle, &increaseVolume, 0);
        clk_prev = clk_curr;
        begin = std::chrono::steady_clock::now();
    }
}

void foo(void* arg){
    bool increaseCurrentVolume{};
    for(;;){
        BaseType_t status = xQueueReceive(qHandle, &increaseCurrentVolume, portMAX_DELAY);
        if(status == pdTRUE)
            btInstance->volume_set_by_local_host(increaseCurrentVolume);
        vTaskDelay(20);
    }
}

static void IRAM_ATTR toggle_interrupt_handler(void *args)
{
    xSemaphoreGive(btnEventSem);
}

void interpret_btn_press(void* arg){
    for(;;){
        xSemaphoreTake(btnEventSem, portMAX_DELAY);
        vTaskDelay(20); //incase bounce?
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        while(!gpio_get_level(static_cast<gpio_num_t>(SW_PIN))){
            vTaskDelay(20);
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() >= 2000){
            printf("PAIRING %" PRId64" ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count());
            btInstance->getInstance()->enter_pairing_mode();
        }
        else{
            printf("togge %" PRId64"ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count());
            btInstance->getInstance()->toggle_play_pause();
        }
    }
}

void read_battery(void* arg){
    float maxVoltage{16.8};
    int maxAdcValue{4095};
    float x;
    const TickType_t xDelay = 10000 / portTICK_PERIOD_MS;
    adc1_config_width(ADC_WIDTH_BIT_12); // Configure ADC to 12-bit mode
    
    for(;;){
        uint32_t voltage =  (float)adc1_get_raw(ADC1_CHANNEL_0) / 4095.0 * 100 /** 16.8  * (22000.0/21428)*/;
        displayInstance->set_battery_level(voltage);
        printf("battery %lu %% raw %d\n", voltage, adc1_get_raw(ADC1_CHANNEL_0) );
        vTaskDelay(xDelay);
    }
}





void app_main(void)
{
    puts("########################################");
    puts("######### application startup  #########");
    puts("########################################");
 
    uint8_t bt_addr[] = {0x48u, 0x01u, 0xc5u, 0x77u, 0xf6u, 0xa4u};
    btnEventSem = xSemaphoreCreateBinary();
    if(btnEventSem == nullptr)
        printf("######### btnEventSem failed #########\n");

    if(!btInstance->init())
        printf("######### init bluetooth failed #########\n");

    if(!displayInstance->init())
        printf("######### init displayInstance failed #########\n");

    printf("######### Bluetooth started #########\n");

    //esp_err_t error = esp_a2d_sink_connect(bt_addr);//[48:01:c5:77:f6:a4]
    //printf("Auto connect to phone %d\n", error);




    gpio_config_t ioConf{};

    ioConf.intr_type = GPIO_INTR_POSEDGE;
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<<CLK_PIN;
    ioConf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&ioConf);

    //ioConf.intr_type = GPIO_INTR_NEGEDGE;
    //ioConf.mode = GPIO_MODE_INPUT;
    //ioConf.pin_bit_mask = 1ULL<< SW_PIN;
    //ioConf.pull_up_en = GPIO_PULLUP_ENABLE;
    //gpio_config(&ioConf);

    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<<BATTERY_LVL_PIN;
    gpio_config(&ioConf);


    TaskHandle_t foo_handle;
    xTaskCreate(foo, "foo", 4048, nullptr, configMAX_PRIORITIES - 2, &foo_handle);
    
    //TaskHandle_t read_battery_handle;
    //xTaskCreate(read_battery, "read_battery", 4048, nullptr, configMAX_PRIORITIES - 10, &read_battery_handle);

    //TaskHandle_t interpret_btn_press_handle;
    //xTaskCreate(interpret_btn_press, "interpret_btn_press", 4048, nullptr, configMAX_PRIORITIES -10, &interpret_btn_press_handle);
    

    if(gpio_install_isr_service(0))
        puts("gpio_install_isr_service failed");
    //if(gpio_isr_handler_add(static_cast<gpio_num_t>(SW_PIN), toggle_interrupt_handler, (void *)SW_PIN))
    //    puts("gpio_isr_handler_add failed");
    if(gpio_isr_handler_add(static_cast<gpio_num_t>(CLK_PIN), gpio_interrupt_handler, (void *)CLK_PIN))
        puts("gpio_isr_handler_add failed");

}


