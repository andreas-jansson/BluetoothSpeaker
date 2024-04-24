#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include <string>
#include <inttypes.h>
#include <atomic>

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "freertos/ringbuf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sys/lock.h"

class BluetoothHandler{
    static BluetoothHandler* instance;


    enum AvcrpEnum{
        APP_RC_CT_TL_GET_CAPS,
        APP_RC_CT_TL_GET_META_DATA,
        APP_RC_CT_TL_RN_TRACK_CHANGE,
        APP_RC_CT_TL_RN_PLAYBACK_CHANGE,
        APP_RC_CT_TL_RN_PLAY_POS_CHANGE
    };
    static constexpr int APP_DELAY_VALUE{50}; //50ms 

    static std::uint32_t m_pkt_cnt;               /* count for audio packet */
    static esp_a2d_audio_state_t m_audio_state;/* audio stream datapath state */
    static constexpr char *m_a2d_conn_state_str[4] = {"Disconnected", "Connecting", "Connected", "Disconnecting"}; /* connection state in string */
    static constexpr char *m_a2d_audio_state_str[3] = {"Suspended", "Stopped", "Started"}; /* audio stream datapath state in string */
    static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;/* AVRC target notification capability bit mask */
    static _lock_t m_volume_lock;
    static uint8_t m_volume;                 /* local volume value */
    static bool m_volume_notify;                 /* notify volume change or not */
    static uint8_t c_volumeMax;                 
    static uint8_t c_volumeStepSize;
    static uint8_t m_connectedMacAddr[6];    
    static bool s_isConnected;       
    static TaskHandle_t s_reconnect_handle;



    BluetoothHandler();
    bool init_bt();
    bool init_blueroid();
    bool init_flash();
    bool init_callbacks();

    static void save_mac_to_nvs(uint8_t* a_addr);
    static void reconnect_last_device(void* arg);
    static void forget_last_device();
    static void bt_av_new_track(void);
    static void bt_av_playback_changed(void);
    static void bt_av_play_pos_changed(void);
    static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    static void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
    static void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);
    static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
    static void a2d_data_cb(const uint8_t *data, uint32_t len);
    static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param);
    static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
    static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param);
    static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
    static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter);
    static void new_track(void);
    static void playback_changed(void);
    static void play_pos_changed(void);



public:
    std::string m_deviceName{"C-model.finalfinal"};
    static std::atomic_uint8_t master_volume;

    static BluetoothHandler* getInstance();
    bool init();
    size_t write_ringbuf(const uint8_t *data, size_t size);
    static void volume_set_by_controller(uint8_t volume);
    static void volume_set_by_local_host(bool increaseVolume); 
    void enter_pairing_mode();
    void toggle_play_pause();


    BluetoothHandler(const BluetoothHandler& obj) = delete; 

};



#endif //BLUETOOTH_HANDLER_H
