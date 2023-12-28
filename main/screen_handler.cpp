#include "screen_handler.h"

#include "images/play_btn.h"
#include "images/stop_btn.h"
#include "images/muted_btn.h"
#include "images/pause_btn.h"
#include "images/wallpaper.h"
#include "font_goo.h"
#include "font_hemi.h"

ScreenHandler *ScreenHandler::instance;
TaskHandle_t ScreenHandler::s_display_task_handle{nullptr};
SemaphoreHandle_t ScreenHandler::m_volume_lock{nullptr};
playStatus ScreenHandler::m_playStatus{stopped};
TFT_eSPI ScreenHandler::m_tft{TFT_eSPI()};
TFT_eSprite ScreenHandler::s_backgroundSprite{TFT_eSprite(&m_tft)};
TFT_eSprite ScreenHandler::s_playBtnSprite{TFT_eSprite(&m_tft)};
TFT_eSprite ScreenHandler::s_muteBtnSprite{TFT_eSprite(&m_tft)};
TFT_eSprite ScreenHandler::s_volumeTxtSprite{TFT_eSprite(&m_tft)};
TFT_eSprite ScreenHandler::s_pauseBtnSprite{TFT_eSprite(&m_tft)};


ScreenHandler::ScreenHandler()
{
    puts("ScreenHandler created!");
}

ScreenHandler *ScreenHandler::getInstance()
{

    if (instance == nullptr)
        instance = new ScreenHandler();

    return instance;
}

bool ScreenHandler::init()
{
    m_tft.init();
    m_tft.setRotation(3);

    s_backgroundSprite.createSprite(240, 135);
    s_backgroundSprite.setSwapBytes(true);

    s_backgroundSprite.setColorDepth(8);
    s_backgroundSprite.pushImage(0 ,0 , 240, 135, wallPaper);
    s_backgroundSprite.loadFont(fontHemi);

    s_playBtnSprite.createSprite(48, 48);
    s_playBtnSprite.pushImage(0, 0, 48, 48, playBtn);

    s_pauseBtnSprite.createSprite(48, 48);
    s_pauseBtnSprite.pushImage(0, 0, 48, 48, pauseBtn);

    s_muteBtnSprite.createSprite(48, 48);
    s_muteBtnSprite.pushImage(0, 0, 48, 48, muteBtn);

    s_volumeTxtSprite.createSprite(50, 50);
    s_volumeTxtSprite.setTextColor(TFT_WHITE);
    s_volumeTxtSprite.setColorDepth(8);
    s_volumeTxtSprite.loadFont(fontHemi);

    m_volume_lock = xSemaphoreCreateMutex();
    if (xTaskCreate(t_display_handler, "displayTask", 20480, nullptr, 4, &s_display_task_handle) != pdPASS)
    {
        puts("t_display_handler task failed");
        return false;
    }
    return true;
}


void ScreenHandler::buffer_image()
{
    
    s_backgroundSprite.pushImage(0 ,0 , 240, 135, wallPaper);

    s_volumeTxtSprite.fillSprite(TFT_BLACK);
    s_backgroundSprite.drawString(m_artist.c_str(), 30, 40);
    s_backgroundSprite.drawString(m_track.c_str(), 30, 65);
    s_backgroundSprite.drawString(m_track.c_str(), 100, 65);




    if (m_volume == 0)
    {
        s_volumeTxtSprite.pushToSprite(&s_backgroundSprite, 0, 0, TFT_BLACK);
        s_muteBtnSprite.pushToSprite(&s_backgroundSprite, 2, 2, TFT_BLACK);
    }
    else
    {
        s_backgroundSprite.drawString(String(m_volume), 10, 10);
    }

    //s_backgroundSprite.drawString(String(m_batteryLevel, 2), 90, 10);
    draw_battery(s_backgroundSprite);

      // render media control
    if (m_playStatus == paused || m_playStatus == stopped)
        s_playBtnSprite.pushToSprite(&s_backgroundSprite, 90, 85, TFT_BLACK);
    else
        s_pauseBtnSprite.pushToSprite(&s_backgroundSprite, 90, 85, TFT_BLACK);


    s_backgroundSprite.pushSprite(0, 0);
}

void ScreenHandler::t_display_handler(void *arg)
{

    const TickType_t xDelay = 200 / portTICK_PERIOD_MS;
    for (;;)
    {
        instance->buffer_image();
        vTaskDelay(xDelay);
    }
}

void ScreenHandler::update_volume(uint32_t a_volume)
{
    m_volume = a_volume;
}

void ScreenHandler::update_artist(std::string a_name)
{
    if(a_name.size() >= m_maxCharacters)
        m_artist = a_name.substr(0, m_maxCharacters) + "..";
    else   
        m_artist = a_name;
}

void ScreenHandler::update_track(std::string a_name)
{
    if(a_name.size() >= m_maxCharacters)
        m_track = a_name.substr(0, m_maxCharacters) + "..";
    else    
        m_track = a_name;

}

void ScreenHandler::update_track_elapsed_time()
{
}

void ScreenHandler::update_play_status(playStatus a_playStatus)
{
    m_playStatus = a_playStatus;
    printf("m_playStatus is %d\n", m_playStatus);
}


void ScreenHandler::set_battery_level(std::uint32_t a_level){
    if(a_level > 4 || a_level < 0)
        return;
    
    m_batteryLevel = a_level;
}

void ScreenHandler::draw_battery(TFT_eSprite& a_sprite){

    constexpr int bat_height{18};
    constexpr int bat_width{30};
    constexpr int bat_thickness{3};


    a_sprite.fillRect(200, 10, bat_width, bat_thickness, TFT_WHITE);       // -----
    a_sprite.fillRect(200, 20, bat_width, bat_thickness, TFT_WHITE);       // -----

    a_sprite.fillRect(200, 10, bat_thickness, bat_height, TFT_WHITE);       // |
    a_sprite.fillRect(225, 10, bat_thickness, bat_height, TFT_WHITE);       //      |

    a_sprite.fillRect(225, 16, 6, 5, TFT_WHITE);                            //       *


}

