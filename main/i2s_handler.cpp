#include "i2s_handler.h"

#include <string.h>

I2sHandler* I2sHandler::instance;
TaskHandle_t I2sHandler::s_bt_i2s_task_handle{nullptr};
RingbufHandle_t I2sHandler::s_ringbuf_i2s{nullptr};
SemaphoreHandle_t I2sHandler::s_i2s_write_semaphore{nullptr};
uint16_t I2sHandler::ringbuffer_mode{RINGBUFFER_MODE_PROCESSING};
i2s_chan_handle_t I2sHandler::m_tx_chan{0};


I2sHandler::I2sHandler(){
    puts("I2sHandler has been created!");
}

I2sHandler* I2sHandler::getInstance()
{

if (instance == nullptr)
    instance = new I2sHandler(); 


return instance;
}

void I2sHandler::i2s_driver_install(void)
{
    printf("%s %d core %d:\n",__func__, __LINE__,xPortGetCoreID());
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = static_cast<gpio_num_t>(m_pin_bck),
            .ws = static_cast<gpio_num_t>(m_pin_ws),
            .dout = static_cast<gpio_num_t>(m_pin_data),
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        
        },
    };
    
    /* enable I2S */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &m_tx_chan, nullptr));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(m_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(m_tx_chan));
}

void I2sHandler::i2s_driver_uninstall(void)
{
    puts(__func__);
    ESP_ERROR_CHECK(i2s_channel_disable(m_tx_chan));
    ESP_ERROR_CHECK(i2s_del_channel(m_tx_chan));
}

void I2sHandler::i2s_task_shut_down(void)
{
    puts(__func__);
    if (s_bt_i2s_task_handle) {
        vTaskDelete(s_bt_i2s_task_handle);
        s_bt_i2s_task_handle = nullptr;
    }
    if (s_ringbuf_i2s) {
        vRingbufferDelete(s_ringbuf_i2s);
        s_ringbuf_i2s = nullptr;
    }
    if (s_i2s_write_semaphore) {
        vSemaphoreDelete(s_i2s_write_semaphore);
        s_i2s_write_semaphore = nullptr;
    }
}

#include <ESP.h>

bool I2sHandler::i2s_task_start_up(void)
{

    printf("ringbuffer data empty! mode changed: RINGBUFFER_MODE_PREFETCHING\n");
    ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    if ((s_i2s_write_semaphore = xSemaphoreCreateBinary()) == nullptr) {
        printf("%s, Semaphore create failed\n", __func__);
        return false;
    }
    size_t freeHeap = ESP.getFreeHeap();
    printf("Free Heap: %u\n", freeHeap);
    size_t maxHeap = ESP.getMaxAllocHeap();
    printf("Max free Heap: %u\n", maxHeap);
    printf("%s created sem %p\n", __func__, s_i2s_write_semaphore);
    if ((s_ringbuf_i2s = xRingbufferCreate(RINGBUF_HIGHEST_WATER_LEVEL, RINGBUF_TYPE_BYTEBUF)) == nullptr) {
        printf("%s, ringbuffer create failed\n", __func__);
        return false;
    }

    xTaskCreate(i2s_task_handler, "BtI2STask", 4048, nullptr, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle);
    return true;
}

void I2sHandler::i2s_task_handler(void *arg)
{
    puts(__func__);
    uint8_t *data = nullptr;
    size_t item_size = 0;
    /**
     * The total length of DMA buffer of I2S is:
     * `dma_frame_num * dma_desc_num * i2s_channel_num * i2s_data_bit_width / 8`.
     * Transmit `dma_frame_num * dma_desc_num` bytes to DMA is trade-off.
     */
    const size_t item_size_upto = 240 * 6;
    size_t bytes_written = 0;

    for (;;) {
        if (pdTRUE == xSemaphoreTake(s_i2s_write_semaphore, portMAX_DELAY)) {
            printf("%s took sem\n",__func__);
            for (;;) {
                item_size = 0;
                /* receive data from ringbuffer and write it to I2S DMA transmit buffer */
                data = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &item_size, (TickType_t)pdMS_TO_TICKS(20), item_size_upto);
                if (item_size == 0) {
                    printf("ringbuffer underflowed! mode changed: RINGBUFFER_MODE_PREFETCHING\n");
                    ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
                    break;
                }
                i2s_channel_write(m_tx_chan, data, item_size, &bytes_written, portMAX_DELAY);
                vRingbufferReturnItem(s_ringbuf_i2s, (void *)data);
            }
        }
    }
}



size_t I2sHandler::write_ringbuf(const uint8_t *data, size_t size)
{
    size_t item_size = 0;
    BaseType_t done = pdFALSE;

    if (ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
        printf("ringbuffer is full, drop this packet!\n");
        vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
        if (item_size <= RINGBUF_PREFETCH_WATER_LEVEL) {
            printf("ringbuffer data decreased! mode changed: RINGBUFFER_MODE_PROCESSING\n");
            ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        }
        return 0;
    }
 
    done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (TickType_t)0);

    if (!done) {
        printf("ringbuffer overflowed, ready to decrease data! mode changed: RINGBUFFER_MODE_DROPPING\n");
        ringbuffer_mode = RINGBUFFER_MODE_DROPPING;
    }

    if (ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
        if (item_size >= RINGBUF_PREFETCH_WATER_LEVEL) {
            printf("ringbuffer data increased! mode changed: RINGBUFFER_MODE_PROCESSING\n");
            ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
            if (pdFALSE == xSemaphoreGive(s_i2s_write_semaphore)) {
                printf("semphore give failed\n");
            }
        }
    }

    return done ? size : 0;
}
