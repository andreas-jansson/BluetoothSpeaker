#ifndef SCREEN_HANDLER_H
#define SCREEN_HANDLER_H

#include <string>
#include <map>

#include "Arduino.h"
#include <TFT_eSPI.h>
#include <SPI.h>

#include "freertos/task.h"
#include "freertos/semphr.h"




enum playStatus
{
    paused,
    stopped,
    playing
};


enum connectedStatusEnum
{
    connected,
    disconnected,
    pairing
};




class ScreenHandler
{
    static ScreenHandler *instance;
    static TaskHandle_t s_display_task_handle;
    static TFT_eSPI m_tft;
    static TFT_eSprite s_backgroundSprite;
    static TFT_eSprite s_playBtnSprite;
    static TFT_eSprite s_pauseBtnSprite;
    static TFT_eSprite s_muteBtnSprite;
    static TFT_eSprite s_volumeTxtSprite;


    std::uint32_t m_volume{30};
    std::string m_artist{"Lets play Music"};
    std::string m_track{""};
    static playStatus m_playStatus;
    std::uint8_t m_maxCharacters{12};
    connectedStatusEnum connectedStatus{disconnected};
    static constexpr char *m_a2d_conn_state_str[4] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};

    
    static SemaphoreHandle_t m_volume_lock;

    std::uint32_t m_batteryLevel{};

    ScreenHandler();
    void buffer_image();
    void draw_battery(TFT_eSprite& a_sprite);
    static void t_display_handler(void *arg);

public:
 

    static ScreenHandler *getInstance();
    bool init();
    void update_volume(uint32_t a_volume);
    void update_artist(std::string a_name);
    void update_track(std::string a_name);
    void update_track_elapsed_time();
    void update_play_status(playStatus isPlaying);
    void set_connected_status(connectedStatusEnum a_status){connectedStatus = a_status;}
    void set_battery_level(std::uint32_t a_level);

    ScreenHandler(const ScreenHandler &obj) = delete;
};

#endif // SCREEN_HANDLER_H
