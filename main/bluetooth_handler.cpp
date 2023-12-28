#include "bluetooth_handler.h"


#include "i2s_handler.h"
#include "screen_handler.h"

BluetoothHandler* BluetoothHandler::instance = nullptr; 

_lock_t BluetoothHandler::m_volume_lock;
std::uint8_t BluetoothHandler::m_volume{0};
std::uint8_t BluetoothHandler::c_volumeMax{100};
std::uint8_t BluetoothHandler::c_volumeStepSize{5};
std::uint32_t BluetoothHandler::m_pkt_cnt{ESP_A2D_AUDIO_STATE_STOPPED};
esp_a2d_audio_state_t BluetoothHandler::m_audio_state{ESP_A2D_AUDIO_STATE_STOPPED};
bool BluetoothHandler::m_volume_notify{false};
esp_avrc_rn_evt_cap_mask_t BluetoothHandler::s_avrc_peer_rn_cap{};
std::atomic_uint8_t BluetoothHandler::master_volume{30};
std::uint8_t BluetoothHandler::m_connectedMacAddr[6]{};

BluetoothHandler::BluetoothHandler(){
    puts("BluetoothHandler has been created!");
}

BluetoothHandler* BluetoothHandler::getInstance()
{

if (instance == nullptr) 
    instance = new BluetoothHandler(); 

return instance;
}

bool BluetoothHandler::init(){

    if(!init_flash())
        return false;
    if(!init_bt())
        return false;
    if(!init_blueroid())
        return false;
    if(!init_callbacks())
        return false;
    return true;
}

bool BluetoothHandler::init_bt(){
    puts(__func__);

    esp_err_t err{};
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        printf("initialize controller failed: %s\n", esp_err_to_name(err));
        return false;
    }
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        printf("enable controller failed: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;

}

bool BluetoothHandler::init_blueroid(){
    puts(__func__);
    esp_err_t err{};
    if ((err = esp_bluedroid_init()) != ESP_OK) {
        printf("initialize bluedroid failed: %s\n", esp_err_to_name(err));
        return false;
    }
    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        printf("enable bluedroid failed: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;

}

bool BluetoothHandler::init_callbacks(){

    puts(__func__);
    assert(esp_bt_dev_set_device_name(m_deviceName.c_str())== ESP_OK);
    assert(esp_bt_gap_register_callback(bt_app_gap_cb) == ESP_OK);
    assert(esp_avrc_ct_init() ==ESP_OK);
    assert(esp_avrc_ct_register_callback(bt_app_rc_ct_cb) == ESP_OK);
    assert(esp_avrc_tg_init() == ESP_OK);
    assert(esp_avrc_tg_register_callback(bt_app_rc_tg_cb) == ESP_OK);
    esp_avrc_rn_evt_cap_mask_t evt_set = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);
    assert(esp_a2d_sink_init() == ESP_OK);
    assert(esp_a2d_register_callback(&bt_app_a2d_cb) == ESP_OK);
    assert(esp_a2d_sink_register_data_callback(a2d_data_cb) == ESP_OK);
    assert(esp_a2d_sink_get_delay_value() == ESP_OK);
    assert(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE) == ESP_OK);


    return true;
}

bool BluetoothHandler::init_flash(){
    /* initialize NVS — it is used to store PHY calibration data */
        esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    return true;
} 


uint8_t *volume_control_changeVolume(const uint8_t *data, uint8_t *outputData, size_t size, uint8_t volume) {

    const int numBytesShifted = 2;
    int16_t pcmData;
    size_t h = 0;
    double  deviderRight = (((double)volume*0.2)/ 100);
    double  deviderLeft = (((double)volume*0.04)/ 100);
    bool isLeftChannel{true};
    bool isNegative{false};
    
    memcpy(outputData, data, size);
    static std::uint64_t counter{0};
    for (h = 0; h < size; h += numBytesShifted) {
        pcmData = ((uint16_t) data[h + 1] << 8) | data[h];  

        isNegative = pcmData & 0x80;

        if (isNegative) {
            pcmData = (~pcmData) + 0x1;
        }

        if(isLeftChannel)
            pcmData = (pcmData * deviderLeft);
        else
            pcmData = (pcmData * deviderRight);

        if (isNegative) {
            pcmData = (~pcmData) + 0x1;
        }
        outputData[h] = pcmData;
        outputData[h+1] = pcmData >> 8;
        isLeftChannel = !isLeftChannel;

        if(counter % 20000 == 0)
            printf("data: %d %d\n", outputData[h], outputData[h+1]);
        counter++;
    }
    return outputData;
}

void BluetoothHandler::a2d_data_cb(const uint8_t *data, uint32_t len)
{
    uint8_t *volumedData = (uint8_t *)malloc(sizeof(uint8_t)*len);
    I2sHandler::getInstance()->write_ringbuf(volume_control_changeVolume(data, volumedData, len, m_volume), len);
    //I2sHandler::getInstance()->write_ringbuf(data, len);

    free(volumedData);

    if (++m_pkt_cnt % 100 == 0) {
        printf("Audio packet count: %lu\n", m_pkt_cnt);
    }
}

void BluetoothHandler::bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        bt_av_hdl_a2d_evt(event, param);
        break;
    }
    default:
        printf("Invalid A2DP event: %d\n", event);
        break;
    }
}

void BluetoothHandler::bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    puts(__func__);
    uint8_t *btDeviceAddr = nullptr; /*!< remote bluetooth device address */

    switch(event) {
    /* when authentication completed, this event comes */
    case ESP_BT_GAP_AUTH_CMPL_EVT: 
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            printf("authentication success: %s", param->auth_cmpl.device_name);
        else
            printf("authentication failed, status: %d", param->auth_cmpl.stat);
        break;
    /* when GAP mode changed, this event comes */
    case ESP_BT_GAP_MODE_CHG_EVT:
        printf("ESP_BT_GAP_MODE_CHG_EVT mode: %d", param->mode_chg.mode);
        break;
    /* when ACL connection completed, this event comes */
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        btDeviceAddr = (uint8_t *)param->acl_conn_cmpl_stat.bda; 
        m_connectedMacAddr[0] = btDeviceAddr[0];
        m_connectedMacAddr[1] = btDeviceAddr[1];
        m_connectedMacAddr[2] = btDeviceAddr[2];
        m_connectedMacAddr[3] = btDeviceAddr[3];
        m_connectedMacAddr[4] = btDeviceAddr[4];
        m_connectedMacAddr[5] = btDeviceAddr[5];

        ScreenHandler::getInstance()->set_connected_status(connected);

        printf("ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT Connected to [%02x:%02x:%02x:%02x:%02x:%02x], status: 0x%x",
                 btDeviceAddr[0], btDeviceAddr[1], btDeviceAddr[2], btDeviceAddr[3], btDeviceAddr[4], btDeviceAddr[5], param->acl_conn_cmpl_stat.stat);
        break;
    /* when ACL disconnection completed, this event comes */
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        btDeviceAddr = (uint8_t *)param->acl_disconn_cmpl_stat.bda;
        ScreenHandler::getInstance()->set_connected_status(disconnected);
        printf("ESP_BT_GAP_ACL_DISC_CMPL_STAT_EVT Disconnected from [%02x:%02x:%02x:%02x:%02x:%02x], reason: 0x%x",
                 btDeviceAddr[0], btDeviceAddr[1], btDeviceAddr[2], btDeviceAddr[3], btDeviceAddr[4], btDeviceAddr[5], param->acl_disconn_cmpl_stat.reason);
        break;
    /* others */
    default: 
        printf("unhandled event: %d", event);
        break;
    }
}

void BluetoothHandler::volume_set_by_controller(uint8_t volume)
{
    
    printf("Volume is set by remote controller to: %"PRIu32"%%  value: %x\n", (uint32_t)volume * 100 / 0x7f, volume);
    _lock_acquire(&m_volume_lock);
    m_volume = (uint32_t)volume * 100 / 0x7f;
    _lock_release(&m_volume_lock);

    ScreenHandler::getInstance()->update_volume(static_cast<std::uint32_t>(volume * 100 / 0x7f));
}

void BluetoothHandler::volume_set_by_local_host(bool increaseVolume)
{

    _lock_acquire(&m_volume_lock);

    if((m_volume + c_volumeStepSize >= c_volumeMax) && increaseVolume)
        m_volume = c_volumeMax;
    else if((m_volume - c_volumeStepSize <= 0) && !increaseVolume){
        m_volume = 0;
    }
    else{
        m_volume += increaseVolume ? c_volumeStepSize : -c_volumeStepSize;
    }
    printf("Volume %d is %s\n",m_volume, increaseVolume? "increased" : "decreased");


    //m_volume = (uint32_t)m_volume * 100 / 0x7f;

    _lock_release(&m_volume_lock);

    ScreenHandler::getInstance()->update_volume(static_cast<std::uint32_t>(m_volume));



    ///* set the volume in protection of lock */
    //_lock_acquire(&m_volume_lock);
    //m_volume = volume;
    //_lock_release(&m_volume_lock);


    /* send notification response to remote AVRCP controller */
    if (m_volume_notify) {
        esp_avrc_rn_param_t rn_param;
        rn_param.volume = m_volume;
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
        m_volume_notify = false;
    }
}

void BluetoothHandler::bt_av_new_track(void)
{
    /* request metadata */
    uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE |
                        ESP_AVRC_MD_ATTR_ARTIST |
                        ESP_AVRC_MD_ATTR_ALBUM |
                        ESP_AVRC_MD_ATTR_GENRE;
    esp_avrc_ct_send_metadata_cmd(APP_RC_CT_TL_GET_META_DATA, attr_mask);

    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_TRACK_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_TRACK_CHANGE,
                                                   ESP_AVRC_RN_TRACK_CHANGE, 0);
    }
}

void BluetoothHandler::bt_av_playback_changed(void)
{
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAYBACK_CHANGE,
                                                   ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
    }
}

void BluetoothHandler::bt_av_play_pos_changed(void)
{
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_POS_CHANGED)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAY_POS_CHANGE,
                                                   ESP_AVRC_RN_PLAY_POS_CHANGED, 10);
    }
}

void BluetoothHandler::bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param)
{
    printf("%s event: %d\n", __func__, event);

    esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t *)(p_param);

    switch (event) {
    /* when passthrough commanded, this event comes */
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        printf("AVRC passthrough cmd: key_code 0x%x, key_state %d\n", rc->psth_cmd.key_code, rc->psth_cmd.key_state);
        break;
    }
    /* when absolute volume command from remote device set, this event comes */
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        printf("AVRC set absolute volume: %d%%\n", (int)rc->set_abs_vol.volume * 100 / 0x7f);
        volume_set_by_controller(rc->set_abs_vol.volume);
        break;
    }
    /* when notification registered, this event comes */
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
        printf("AVRC register event notification: %d, param: 0x%"PRIx32"\n", rc->reg_ntf.event_id, rc->reg_ntf.event_parameter);
        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            m_volume_notify = true;
            esp_avrc_rn_param_t rn_param;
            rn_param.volume = m_volume;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT: {
        printf("AVRC remote features: %"PRIx32", CT features: %x", rc->rmt_feats.feat_mask, rc->rmt_feats.ct_feat_flag);
        break;
    }
    /* others */
    default:
        printf("%s unhandled event: %d", __func__, event);
        break;
    }
}

void BluetoothHandler::bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param)
{
    printf(" event: %d\n", event);

    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        printf("AVRC conn_state event: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]\n",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected) {
            /* get remote supported event_ids of peer AVRCP Target */
            esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        } else {
            /* clear peer notification capability record */
            s_avrc_peer_rn_cap.bits = 0;
        }
        break;
    }
    /* when passthrough responsed, this event comes */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        printf("AVRC passthrough rsp: key_code 0x%x, key_state %d, rsp_code %d\n", rc->psth_rsp.key_code,
                    rc->psth_rsp.key_state, rc->psth_rsp.rsp_code);
        break;
    }
    /* when metadata responsed, this event comes */
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        printf("AVRC metadata rsp: attribute id 0x%x, %s\n", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);

        switch(rc->meta_rsp.attr_id){
            case 0x1:
            ScreenHandler::getInstance()->update_track(reinterpret_cast<const char*>(rc->meta_rsp.attr_text));
            break;
            case 0x2:
            ScreenHandler::getInstance()->update_artist(reinterpret_cast<const char*>(rc->meta_rsp.attr_text));
            break;
        }

        free(rc->meta_rsp.attr_text);
        break;
    }
    /* when notified, this event comes */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        printf("AVRC event notification: %d\n", rc->change_ntf.event_id);
        bt_av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        printf("AVRC remote features %"PRIx32", TG features %x", rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
        break;
    }
    /* when notification capability of peer device got, this event comes */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        printf("remote rn_cap: count %d, bitmask 0x%x", rc->get_rn_caps_rsp.cap_count,
                 rc->get_rn_caps_rsp.evt_set.bits);
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
        bt_av_new_track();
        bt_av_playback_changed();
        bt_av_play_pos_changed();
        break;
    }
    /* others */
    default:
        printf("%s unhandled event: %d\n", __func__, event);
        break;
    }
}

void BluetoothHandler::bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter)
{
    switch (event_id) {
    /* when new track is loaded, this event comes */
    case ESP_AVRC_RN_TRACK_CHANGE:
        bt_av_new_track();
        break;
    /* when track status changed, this event comes */
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
        printf("Playback status changed: 0x%x\n", event_parameter->playback);
        bt_av_playback_changed();
        break;
    /* when track playing position changed, this event comes */
    case ESP_AVRC_RN_PLAY_POS_CHANGED:
        printf("Play position changed: %"PRIu32"-ms\n", event_parameter->play_pos);
        bt_av_play_pos_changed();
        break;
    /* others */
    default:
        printf("unhandled event: %d\n", event_id);
        break;
    }
}

void BluetoothHandler::new_track(void)
{
    /* request metadata */
    uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE |
                        ESP_AVRC_MD_ATTR_ARTIST |
                        ESP_AVRC_MD_ATTR_ALBUM |
                        ESP_AVRC_MD_ATTR_GENRE;
    esp_avrc_ct_send_metadata_cmd(APP_RC_CT_TL_GET_META_DATA, attr_mask);

    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_TRACK_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_TRACK_CHANGE,
                                                   ESP_AVRC_RN_TRACK_CHANGE, 0);
    }
}

void BluetoothHandler::playback_changed(void)
{
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAYBACK_CHANGE,
                                                   ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
    }
}

void BluetoothHandler::play_pos_changed(void)
{
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_POS_CHANGED)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAY_POS_CHANGE,
                                                   ESP_AVRC_RN_PLAY_POS_CHANGED, 10);
    }
}

void BluetoothHandler::bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        bt_app_alloc_meta_buffer(param);
        /* fall through */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        bt_av_hdl_avrc_ct_evt(event, param);
        break;
    }
    default:
        printf("Invalid AVRC event: %d\n", event);
        break;
    }
}
  
void BluetoothHandler::bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
    case ESP_AVRC_TG_SET_PLAYER_APP_VALUE_EVT:
        bt_av_hdl_avrc_tg_evt(event, param);
        break;
    default:
        printf("Invalid AVRC event: %d\n", event);
        break;
    }
}

void BluetoothHandler::bt_av_hdl_a2d_evt(uint16_t event, void *p_param)
{
    printf("%s event: %d\n", __func__, event);

    esp_a2d_cb_param_t *a2d = nullptr;

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        printf("A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]\n",
            m_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            I2sHandler::getInstance()->i2s_driver_uninstall();
            I2sHandler::getInstance()->i2s_task_shut_down();

            for(auto &it : m_connectedMacAddr)
                it = 0;

        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED){
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            assert(I2sHandler::getInstance()->i2s_task_start_up()==true);
           
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
            I2sHandler::getInstance()->i2s_driver_install();
        }
        break;
    }
    /* when audio stream transmission state changed, this event comes */
    case ESP_A2D_AUDIO_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        printf("A2DP audio state: %s %d\n", m_a2d_audio_state_str[a2d->audio_stat.state], a2d->audio_stat.state);
        m_audio_state = a2d->audio_stat.state;

        if(a2d->audio_stat.state >= 0 ||  a2d->audio_stat.state <= 2)
            ScreenHandler::getInstance()->update_play_status(static_cast<playStatus>(a2d->audio_stat.state));


        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            m_pkt_cnt = 0;
        }
        break;
    }
    /* when audio codec is configured, this event comes */
    case ESP_A2D_AUDIO_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        printf("A2DP audio stream configuration, codec type: %d\n", a2d->audio_cfg.mcc.type);
        /* for now only SBC stream is supported */
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            int sample_rate = 16000;
            int ch_count = 2;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6)) {
                sample_rate = 32000;
            } else if (oct0 & (0x01 << 5)) {
                sample_rate = 44100;
            } else if (oct0 & (0x01 << 4)) {
                sample_rate = 48000;
            }

            if (oct0 & (0x01 << 3)) {
                ch_count = 1;
            }
        
            i2s_channel_disable(I2sHandler::getInstance()->m_tx_chan);
            i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<std::uint32_t>(sample_rate));
            i2s_std_slot_config_t slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, static_cast<i2s_slot_mode_t>(ch_count));
            i2s_channel_reconfig_std_clock(I2sHandler::getInstance()->m_tx_chan, &clk_cfg);
            i2s_channel_reconfig_std_slot(I2sHandler::getInstance()->m_tx_chan, &slot_cfg);
            i2s_channel_enable(I2sHandler::getInstance()->m_tx_chan);

            printf("Configure audio player: %x-%x-%x-%x",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            printf("Audio player configured, sample rate: %d\n", sample_rate);
        }
        break;
    }
    /* when a2dp init or deinit completed, this event comes */
    case ESP_A2D_PROF_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_INIT_SUCCESS == a2d->a2d_prof_stat.init_state) {
            printf("A2DP PROF STATE: Init Complete\n");
        } else {
            printf("A2DP PROF STATE: Deinit Complete\n");
        }
        break;
    }
    /* When protocol service capabilities configured, this event comes */
    case ESP_A2D_SNK_PSC_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        printf("protocol service capabilities configured: 0x%x \n", a2d->a2d_psc_cfg_stat.psc_mask);
        if (a2d->a2d_psc_cfg_stat.psc_mask & ESP_A2D_PSC_DELAY_RPT) {
            printf("Peer device support delay reportin\n");
        } else {
            printf("Peer device unsupport delay reporting\n");
        }
        break;
    }
    /* when set delay value completed, this event comes */
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_SET_INVALID_PARAMS == a2d->a2d_set_delay_value_stat.set_state) {
            printf("Set delay report value: fail\n");
        } else {
            printf("Set delay report value: success, delay_value: %u * 1/10 ms\n", a2d->a2d_set_delay_value_stat.delay_value);
        }
        break;
    }
    /* when get delay value completed, this event comes */
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        printf("Get delay report value: delay_value: %u * 1/10 ms\n", a2d->a2d_get_delay_value_stat.delay_value);
        /* Default delay value plus delay caused by application layer */
        esp_a2d_sink_set_delay_value(a2d->a2d_get_delay_value_stat.delay_value + APP_DELAY_VALUE);
        break;
    }
    /* others */
    default:
        printf("%s unhandled event: %d\n", __func__, event);
        break;
    }
}

void BluetoothHandler::bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{

    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);

    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;
    rc->meta_rsp.attr_text = attr_text;
}

void BluetoothHandler::enter_pairing_mode(){
    bool isValidMac{false};
     for(auto &it : m_connectedMacAddr)
        if(it != 0)
            isValidMac = true;
    if(true)
        return;

    esp_a2d_source_disconnect(m_connectedMacAddr);
}


void BluetoothHandler::toggle_play_pause(){

}

