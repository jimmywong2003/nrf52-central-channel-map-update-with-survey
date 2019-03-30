# BLE_Central_ChannelMapUpdate_Survey

## Description
-----------------------------------------
This example is shown how to use the Quality Survey API (Softdevice S132/S140 v6.x or later) for doing the channel survey and then how to send the channel map update at the central role.

In the release note of the S132/S140 v6.x or later, it has started to support.

* Quality of Service (QoS) Channel survey -- API for channel survey (noise measurement)
* Support for setting channel map for the Observer role
* Channel number for RSSI measurement is now available for connections
* Channel number for RSSI measurement is now available in advertising reports

This feature (QoS) provides measurements of the energy levels on the Bluetooth Low Energy channels to the application. The application can use
this information to determine the noise floor on a per channel basis and set an adapted channel map to avoid busy channels.
When the feature is enabled, BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT events will periodically report the measured energy levels
for each channel. The channel energy is reported in ble_gap_evt_qos_channel_survey_report_t::channel_energy
[BLE_GAP_CHANNEL_COUNT], indexed by the Channel Index. The SoftDevice will attempt to measure energy levels and deliver reports
with the average interval specified in interval_us.
Note: To make the channel survey feature available to the application, ble_gap_cfg_role_count_t::
qos_channel_survey_role_available must be set. This is done using the sd_ble_cfg_set() API.

```c
/* Make Channel Survey feature available to the application */
ble_cfg_t cfg;
cfg.role_count.qos_channel_survey_role_available = 1;
sd_ble_cfg_set(..., &cfg, ...);
```

```c
/* Start receiving channel survey continuously. */
uint32_t err_code = sd_ble_gap_qos_channel_survey_start(BLE_GAP_QOS_CHANNEL_SURVEY_INTERVAL_CONTINUOUS);
```

```c
/* A new measurement is ready. */
case BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT:
{
    for (i = 0; i < BLE_GAP_CHANNEL_COUNT; i++)
    {
        rssi = p_ble_evt->evt.gap_evt.params.qos_channel_survey_report.
        channel_energy[i];
    }
}
```

```c
/* Stop receiving channel survey. */
err_code = sd_ble_gap_qos_channel_survey_stop()
```

# Requirements
------------
- nRF5 SDK version 15.3.0
- Softdevice S140v6.1.0
- nRF52840 DK 
- Segger Embedded Studio IDE (SES) Project

## Instruction

* By pressing the button 1 at the central role to do the QoS (Channel Survey)
* After then pressing the button 2 at the central role to do the channel map update

```c
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {
        case LEDBUTTON_BUTTON_PIN:
                err_code = ble_lbs_led_status_send(&m_ble_lbs_c, button_action);
                if (err_code != NRF_SUCCESS &&
                    err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                    err_code != NRF_ERROR_INVALID_STATE)
                {
                        APP_ERROR_CHECK(err_code);
                }
                if (err_code == NRF_SUCCESS)
                {
                        NRF_LOG_INFO("LBS write LED state %d", button_action);
                }
                break;

        case START_CHANEL_SURVERY_START_PIN:
                if (button_action == APP_BUTTON_PUSH)
                {
                        NRF_LOG_INFO("Press the button to start the channel survey");
                        err_code = connection_channel_survey_start();
                        APP_ERROR_CHECK(err_code);
                }
                break;

        case UPDATE_CHANNEL_MAP_PIN:
                if (button_action == APP_BUTTON_PUSH)
                {
                        if (get_channel_map_status())
                        {
                                channel_map_request_update(m_conn_handle, MINIMUM_CHANNEL_SURVEY_SELECTION);
                        }
                }
                break;
        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}
```

## Note
-----------------------------------------

The project may need modifications to work with later versions or other boards. 

To compile it, clone the repository in the /nRF5_SDK_15.3.0/examples/ directory.

For detail description, you can refer to the link [https://jimmywongbluetooth.wordpress.com/2019/02/23/channel-survey-qos-and-channel-map-update-at-the-ble-central-role/](https://jimmywongbluetooth.wordpress.com/2019/02/23/channel-survey-qos-and-channel-map-update-at-the-ble-central-role/).
