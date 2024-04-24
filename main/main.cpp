#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <chrono>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"

#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "hal/gpio_hal.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "bluetooth_handler.h"
#include "screen_handler.h"

#define CLK_PIN 17
#define DT_PIN 15
#define SW_PIN 33

#define ISR_PINS (1 << CLK_PIN | 1 << SW_PIN)


extern "C"{
    void app_main(void);
}

struct RotaryValues{
    int clk;
    int dt;
};


//QueueHandle_t qHandle;
QueueHandle_t qHandle = xQueueCreate( 10, sizeof(bool));

BluetoothHandler* btInstance = BluetoothHandler::getInstance();
ScreenHandler* displayInstance = ScreenHandler::getInstance();

//std::atomic<bool> increaseVolume{false};
std::atomic<bool> toggle{false};
SemaphoreHandle_t btnEventSem{};


/*
static void IRAM_ATTR volume_up_gpio_handler(void *args){
    static int clk_prev{1};
    static int bt_prev{1};

    int clk_curr = gpio_get_level(static_cast<gpio_num_t>(CLK_PIN));
    int bt_curr = gpio_get_level(static_cast<gpio_num_t>(DT_PIN));
    int step = 5;
    static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); 

    if(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() >= 10 && clk_prev != clk_curr){

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
    const TickType_t xDelay = 2 / portTICK_PERIOD_MS;
    for(;;){
        BaseType_t status = xQueueReceive(qHandle, &increaseCurrentVolume, portMAX_DELAY);
        if(status == pdTRUE)
            btInstance->volume_set_by_local_host(increaseCurrentVolume);
        vTaskDelay(xDelay);
    }
}
*/


static void IRAM_ATTR volume_up_gpio_handler(void *args){
    static RotaryValues prevValues{1,1}; 
    RotaryValues values{}; 
    bool increaseVolume{};

    values.clk  = gpio_get_level(static_cast<gpio_num_t>(CLK_PIN));
    values.dt = gpio_get_level(static_cast<gpio_num_t>(DT_PIN));

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); 
    static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() >= 500 && values.clk != prevValues.clk){
        if(values.clk != values.dt)
            increaseVolume = true;
        else
            increaseVolume = false;

        xQueueSend(qHandle, &increaseVolume, 0);
        begin = std::chrono::steady_clock::now();
        prevValues.clk = values.clk;
        prevValues.dt = values.dt;
    }
}


void foo(void* arg){
    bool increaseVolume{};
    const TickType_t xDelay = 2 / portTICK_PERIOD_MS;
    for(;;){
        BaseType_t status = xQueueReceive(qHandle, &increaseVolume, portMAX_DELAY);
        if(status == pdTRUE){            
            btInstance->volume_set_by_local_host(increaseVolume);
        }
        vTaskDelay(xDelay);
    }
}




static void IRAM_ATTR toggle_interrupt_handler(void *args)
{
    xSemaphoreGive(btnEventSem);
}

static void interpret_btn_press(void* arg){
    auto lastAtivation = std::chrono::high_resolution_clock::now();
    for(;;){
        xSemaphoreTake(btnEventSem, portMAX_DELAY);
        auto begin = std::chrono::high_resolution_clock::now();

        if(std::chrono::duration_cast<std::chrono::milliseconds>(begin-lastAtivation).count() < 50)
            continue;
        else
            lastAtivation = std::chrono::high_resolution_clock::now();

        while(!gpio_get_level(static_cast<gpio_num_t>(SW_PIN))){
            if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - begin).count() >= 2000)
                break;
        }
        auto end = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() >= 2000){
            printf("PAIRING %" PRId64" ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count());
            btInstance->getInstance()->enter_pairing_mode();
        }
        else{
            printf("core %d toggle %" PRId64"ms\n", xPortGetCoreID(), std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count());
            btInstance->getInstance()->toggle_play_pause();
        }
    }
}


void shutdown_sequence(){
    //display low power

    //disable music

    //toggle gpio to disable 
}

void read_battery(void* arg){
    float maxVoltage{16.8};
    int maxAdcValue{4096};
    int resistorRealValue{22000};
    float resistorIdealValue{21739.13};
    float measureCompensation{1.04}; //to compensate for measured battery voltage vs irl measured

    const TickType_t xDelay = 10000 / portTICK_PERIOD_MS;
    adc1_config_width(ADC_WIDTH_BIT_12); // Configure ADC to 12-bit mode
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    std::vector<float>medianVoltage;
    int batterySamples{256};
    float averageMedianVoltage{};


    for(;;){
            //gather large amount of samples due to volatility
            for(int i=0;i<batterySamples;i++){
                float voltage =  (float)adc1_get_raw(ADC1_CHANNEL_0) / maxAdcValue  * maxVoltage  * (resistorRealValue/resistorIdealValue) * measureCompensation;
                medianVoltage.emplace_back(voltage);
            }
            
            //sort the values, calculate the average value from a couple of median values
            std::sort (medianVoltage.begin(), medianVoltage.begin()+batterySamples); 
            for(int i=((batterySamples/2) - 5);i<((batterySamples/2)+5);i++)
                averageMedianVoltage += medianVoltage.at(i);
            averageMedianVoltage = averageMedianVoltage / 10.0;

            
            int batteryLevel{4};
            bool isTooLow{false};
            if(averageMedianVoltage <= 15.0 && averageMedianVoltage >= 14.0)
                batteryLevel--;
            else if(averageMedianVoltage <= 14.0 && averageMedianVoltage >= 13.0)
                batteryLevel -= 2;
            else if(averageMedianVoltage <= 13.0 && averageMedianVoltage >= 12.5)
                batteryLevel -=3;
            else if(averageMedianVoltage <= 12.5){
                isTooLow = true;
            }

            displayInstance->set_battery_level(batteryLevel, isTooLow);
            printf("battery %.2fv lvl %d raw %d\n", averageMedianVoltage, batteryLevel, adc1_get_raw(ADC1_CHANNEL_0));
            medianVoltage.clear();

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


    gpio_config_t ioConf{};

    ioConf.intr_type = GPIO_INTR_ANYEDGE;
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<<CLK_PIN;
    ioConf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&ioConf);


    ioConf.intr_type = GPIO_INTR_NEGEDGE;
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<< SW_PIN;
    ioConf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&ioConf);

    /*
    ioConf.intr_type = GPIO_INTR_DISABLE; //ioConf.intr_type = GPIO_INTR_NEGEDGE;
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<< DT_PIN;
    ioConf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&ioConf);

    */
 

    ioConf.intr_type = GPIO_INTR_DISABLE; //ioConf.intr_type = GPIO_INTR_NEGEDGE;
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<< DT_PIN;
    ioConf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&ioConf);



    TaskHandle_t foo_handle;
    xTaskCreatePinnedToCore(foo, "foo", 4048, nullptr, configMAX_PRIORITIES - 3, &foo_handle, 1);
    
    TaskHandle_t read_battery_handle;
    xTaskCreatePinnedToCore(read_battery, "read_battery", 4048, nullptr, configMAX_PRIORITIES - 20, &read_battery_handle, 1);

    TaskHandle_t interpret_btn_press_handle;
    xTaskCreatePinnedToCore(interpret_btn_press, "interpret_btn_press", 4048, nullptr, configMAX_PRIORITIES -10, &interpret_btn_press_handle, 1);
    

    if(gpio_install_isr_service(0))
        puts("gpio_install_isr_service failed");
    if(gpio_isr_handler_add(static_cast<gpio_num_t>(SW_PIN), toggle_interrupt_handler, (void *)SW_PIN))
        puts("toggle_interrupt_handler failed");
   if(gpio_isr_handler_add(static_cast<gpio_num_t>(CLK_PIN), volume_up_gpio_handler, (void *)CLK_PIN))
        puts("CLK volume_up_gpio_handler failed");
   // if(gpio_isr_handler_add(static_cast<gpio_num_t>(DT_PIN), volume_up_gpio_handler, (void *)DT_PIN))
   //     puts("DT volume_up_gpio_handler failed");


}


