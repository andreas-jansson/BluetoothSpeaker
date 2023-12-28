#ifndef I2S_HANDLER_H
#define I2S_HANDLER_H

#include <inttypes.h>
#include <iostream>
#include "driver/i2s_std.h"
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>


#include "freertos/ringbuf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"


#define RINGBUF_HIGHEST_WATER_LEVEL    (32 * 1024)
#define RINGBUF_PREFETCH_WATER_LEVEL   (20 * 1024)

class I2sHandler{

    static I2sHandler* instance;

    std::uint32_t m_pin_bck{26};    //bck
    std::uint32_t m_pin_ws{25};     //lck
    std::uint32_t m_pin_data{22};   //din

    I2sHandler();

public:
    enum {
        RINGBUFFER_MODE_PROCESSING,    /* ringbuffer is buffering incoming audio data, I2S is working */
        RINGBUFFER_MODE_PREFETCHING,   /* ringbuffer is buffering incoming audio data, I2S is waiting */
        RINGBUFFER_MODE_DROPPING       /* ringbuffer is not buffering (dropping) incoming audio data, I2S is working */
    };

    static uint16_t ringbuffer_mode;
    static RingbufHandle_t s_ringbuf_i2s;
    static SemaphoreHandle_t s_i2s_write_semaphore;

    size_t write_ringbuf(const uint8_t *data, size_t size);
    static TaskHandle_t s_bt_i2s_task_handle;
    static i2s_chan_handle_t m_tx_chan;

    static I2sHandler* getInstance();
    void i2s_driver_install(void );
    void i2s_driver_uninstall(void);
    void i2s_task_shut_down(void);
    bool i2s_task_start_up(void);

    static void i2s_task_handler(void *arg);


    I2sHandler(const I2sHandler& obj) = delete; 
};



#endif //I2S_HANDLER_H
