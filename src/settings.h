#ifndef __ESPUINO_SETTINGS_H__
    #define __ESPUINO_SETTINGS_H__
        #include "Arduino.h"
        #include "values.h"
#if __has_include("settings-override.h")
    #include "settings-override.h"
#else
    //######################### INFOS ####################################
    // This is the general configfile for ESPuino-configuration.

    //################## HARDWARE-PLATFORM ###############################
    /* Make sure to also edit the configfile, that is specific for your platform.
    If in doubts (your develboard is not listed) use HAL 1
    1: Wemos Lolin32                        => settings-lolin32.h
    2: ESP32-A1S Audiokit                   => settings-espa1s.h
    3: Wemos Lolin D32                      => settings-lolin_D32.h
    4: Wemos Lolin D32 pro                  => settings-lolin_D32_pro.h
    5: Lilygo T8 (V1.7)                     => settings-ttgo_t8.h
    6: ESPuino complete                     => settings-complete.h
    7: Lolin D32 pro SDMMC Port-Expander    => settings-lolin_d32_pro_sdmmc_pe.h
    8: AZDelivery ESP32 NodeMCU             => settings-azdelivery_sdmmc.h
    99: custom                              => settings-custom.h
    more to come...
    */
    #ifndef HAL             // Will be set by platformio.ini. If using Arduini-IDE you have to set HAL according your needs!
        #define HAL 1       // HAL 1 = LoLin32, 2 = ESP32-A1S-AudioKit, 3 = Lolin D32, 4 = Lolin D32 pro; ... 99 = custom
    #endif


    //########################## MODULES #################################
    //#define PORT_EXPANDER_ENABLE          // When enabled, buttons can be connected via port-expander PCA9555 (https://forum.espuino.de/t/einsatz-des-port-expanders-pca9555/306)
    //#define I2S_COMM_FMT_LSB_ENABLE       // Enables FMT instead of MSB for I2S-communication-format. Used e.g. by PT2811. Don't enable for MAX98357a, AC101 or PCM5102A)
    #ifndef MDNS_ENABLE
        #define MDNS_ENABLE                 // When enabled, you don't have to handle with ESPuino's IP-address. If hostname is set to "ESPuino", you can reach it via ESPuino.local
    #elif MDNS_ENABLE == 0
        #undef MDNS_ENABLE
    #endif
    //#define MQTT_ENABLE                   // Make sure to configure mqtt-server and (optionally) username+pwd
    #ifndef FTP_ENABLE
        #define FTP_ENABLE                  // Enables FTP-server; DON'T FORGET TO ACTIVATE AFTER BOOT BY PRESSING PAUSE + NEXT-BUTTONS (IN PARALLEL)!
    #elif FTP_ENABLE == 0
        #undef FTP_ENABLE
    #endif
    #ifndef NEOPIXEL_ENABLE
        #define NEOPIXEL_ENABLE             // Don't forget configuration of NUM_LEDS if enabled
    #elif NEOPIXEL_ENABLE == 0
        #undef NEOPIXEL_ENABLE
    #endif
    //#define NEOPIXEL_REVERSE_ROTATION     // Some Neopixels are adressed/soldered counter-clockwise. This can be configured here.
    #ifndef LANGUAGE
        #define LANGUAGE DE                 // DE = deutsch; EN = english
    #endif
    //#define STATIC_IP_ENABLE              // Enables static IP-configuration (change static ip-section accordingly)
    #ifndef HEADPHONE_ADJUST_ENABLE
        #define HEADPHONE_ADJUST_ENABLE     // Used to adjust (lower) volume for optional headphone-pcb (refer maxVolumeSpeaker / maxVolumeHeadphone) and to enable stereo (if PLAY_MONO_SPEAKER is set)
    #elif HEADPHONE_ADJUST_ENABLE == 0
        #undef HEADPHONE_ADJUST_ENABLE
    #endif
    #ifndef PLAY_STEREO_SPEAKER
        #define PLAY_MONO_SPEAKER           // If only one speaker is used enabling mono should make sense. Please note: headphones is always stereo (if HEADPHONE_ADJUST_ENABLE is active)
    #endif
    #ifndef SHUTDOWN_IF_SD_BOOT_FAILS
        #define SHUTDOWN_IF_SD_BOOT_FAILS   // Will put ESP to deepsleep if boot fails due to SD. Really recommend this if there's in battery-mode no other way to restart ESP! Interval adjustable via deepsleepTimeAfterBootFails.
    #elif SHUTDOWN_IF_SD_BOOT_FAILS == 0
        #undef SHUTDOWN_IF_SD_BOOT_FAILS
    #endif
    #ifndef MEASURE_BATTERY_VOLTAGE
        #define MEASURE_BATTERY_VOLTAGE     // Enables battery-measurement via GPIO (ADC) and voltage-divider
    #elif MEASURE_BATTERY_VOLTAGE == 0
        #undef MEASURE_BATTERY_VOLTAGE
    #endif
    //#define MEASURE_BATTERY_MAX17055      // Enables battery-measurement via external fuel gauge (MAX17055)
    //#define SHUTDOWN_ON_BAT_CRITICAL      // Whether to turn off on critical battery-level (only used if MEASURE_BATTERY_XXX is active)
    //#define PLAY_LAST_RFID_AFTER_REBOOT   // When restarting ESPuino, the last RFID that was active before, is recalled and played
    //#define USE_LAST_VOLUME_AFTER_REBOOT  // Remembers the volume used at last shutdown after reboot
    #ifndef USEROTARY_ENABLE
        #define USEROTARY_ENABLE            // If rotary-encoder is used (don't forget to review WAKEUP_BUTTON if you disable this feature!)
    #elif USEROTARY_ENABLE == 0
        #undef USEROTARY_ENABLE
    #endif
    //#define BLUETOOTH_ENABLE              // If enabled and bluetooth-mode is active, you can stream to your ESPuino via bluetooth (a2dp-sink).
    //#define IR_CONTROL_ENABLE             // Enables remote control (https://forum.espuino.de/t/neues-feature-fernsteuerung-per-infrarot-fernbedienung/265)
    #ifndef CACHED_PLAYLIST_ENABLE
        #define CACHED_PLAYLIST_ENABLE      // Enables playlist-caching (infos: https://forum.espuino.de/t/neues-feature-cached-playlist/515)
    #elif CACHED_PLAYLIST_ENABLE == 0
        #undef CACHED_PLAYLIST_ENABLE
    #endif
    //#define PAUSE_WHEN_RFID_REMOVED       // Playback starts when card is applied and pauses automatically, when card is removed (https://forum.espuino.de/t/neues-feature-pausieren-wenn-rfid-karte-entfernt-wurde/541)
    //#define SAVE_PLAYPOS_BEFORE_SHUTDOWN  // When playback is active and mode audiobook was selected, last play-position is saved automatically when shutdown is initiated
    //#define SAVE_PLAYPOS_WHEN_RFID_CHANGE // When playback is active and mode audiobook was selected, last play-position is saved automatically for old playlist when new RFID-tag is applied


    //################## select SD card mode #############################
    #ifndef SD_MMC_1BIT_MODE
        #define SD_MMC_1BIT_MODE            // run SD card in SD-MMC 1Bit mode (using GPIOs 15 + 14 + 2 is mandatory!)
    #elif SD_MMC_1BIT_MODE == 0
        #undef SD_MMC_1BIT_MODE
    #endif
    //#define SINGLE_SPI_ENABLE             // If only one SPI-instance should be used instead of two (not yet working!)


    //################## select RFID reader ##############################
    #if !defined(RFID_READER_TYPE_MFRC522_I2C) && ! defined(RFID_READER_TYPE_PN5180)
        #define RFID_READER_TYPE_MFRC522_SPI  // use MFRC522 via SPI
    #endif
    //#define RFID_READER_TYPE_MFRC522_I2C  // use MFRC522 via I2C
    //#define RFID_READER_TYPE_PN5180       // use PN5180 via SPI
    #if (defined(RFID_READER_TYPE_MFRC522_SPI) && (defined(RFID_READER_TYPE_MFRC522_I2C) || defined(RFID_READER_TYPE_PN5180))) || (defined(RFID_READER_TYPE_MFRC522_I2C) && defined(RFID_READER_TYPE_PN5180))
        #error Only one of RFID_READER_TYPE_MFRC522_I2C, RFID_READER_TYPE_MFRC522_I2C and RFID_READER_TYPE_PN5180 must be defined
    #endif

    #ifdef RFID_READER_TYPE_MFRC522_I2C
        #ifndef MFRC522_ADDR
            #define MFRC522_ADDR 0x28       // default I2C-address of MFRC522
        #endif
    #endif

    #ifdef RFID_READER_TYPE_PN5180
        //#define PN5180_ENABLE_LPCD        // Wakes up ESPuino if RFID-tag was applied while deepsleep is active. Only ISO-14443-tags are supported for wakeup!
    #endif

    #if defined(RFID_READER_TYPE_MFRC522_I2C) || defined(RFID_READER_TYPE_MFRC522_SPI)
        #ifndef RFID_GAIN
            #define RFID_GAIN 0x07<<4       // Sensitivity of RC522. For possible values see reference: https://forum.espuino.de/uploads/default/original/1X/9de5f8d35cbc123c1378cad1beceb3f51035cec0.png
        #endif
        constexpr uint8_t rfidGain = RFID_GAIN;
    #endif


    //############# Port-expander-configuration ######################
    #ifdef PORT_EXPANDER_ENABLE
        #ifndef EXPANDER_I2C_ADDRESS
            #define EXPANDER_I2C_ADDRESS 0x20 // I2C-address of PCA9555 (0x20 is true if PCA's pins A0+A1+A2 are pulled to GND)
        #endif
        constexpr uint8_t expanderI2cAddress = EXPANDER_I2C_ADDRESS;
    #endif

    //################## BUTTON-Layout ##################################
    /* German documentation: https://forum.espuino.de/t/das-dynamische-button-layout/224
    Please note the following numbers as you need to know them in order to define actions for buttons.
    Even if you don't use all of them, their numbers won't change
        0: NEXT_BUTTON
        1: PREVIOUS_BUTTON
        2: PAUSEPLAY_BUTTON
        3: ROTARYENCODER_BUTTON
        4: BUTTON_4
        5: BUTTON_5

    Don't forget to enable/configure those buttons you want to use in your develboard-specific config (e.g. settings-custom.h)

    Single-buttons [can be long or short] (examples):
        BUTTON_0_SHORT => Button 0 (NEXT_BUTTON) pressed shortly
        BUTTON_3_SHORT => Button 3 (ROTARYENCODER_BUTTON) pressed shortly
        BUTTON_4_LONG => Button 4 (BUTTON_4) pressed long

    Multi-buttons [short only] (examples):
        BUTTON_MULTI_01 => Buttons 0+1 (NEXT_BUTTON + PREVIOUS_BUTTON) pressed in parallel
        BUTTON_MULTI_12 => Buttons 1+2 (PREV_BUTTON + PAUSEPLAY_BUTTON) pressed in parallel

    Actions:
        To all of those buttons, an action can be assigned freely.
        Please have a look at values.h to look up actions available (>=100 can be used)
        If you don't want to assign an action or you don't use a given button: CMD_NOTHING has to be set
    */
    // *****BUTTON*****        *****ACTION*****
    #ifndef BUTTON_0_SHORT
        #define BUTTON_0_SHORT    CMD_NEXTTRACK
    #endif
    #ifndef BUTTON_1_SHORT
        #define BUTTON_1_SHORT    CMD_PREVTRACK
    #endif
    #ifndef BUTTON_2_SHORT
        #define BUTTON_2_SHORT    CMD_PLAYPAUSE
    #endif
    #ifndef BUTTON_3_SHORT
        #define BUTTON_3_SHORT    CMD_MEASUREBATTERY
    #endif
    #ifndef BUTTON_4_SHORT
        #define BUTTON_4_SHORT    CMD_SEEK_BACKWARDS
    #endif
    #ifndef BUTTON_5_SHORT
        #define BUTTON_5_SHORT    CMD_SEEK_FORWARDS
    #endif

    #ifndef BUTTON_0_LONG
        #define BUTTON_0_LONG     CMD_LASTTRACK
    #endif
    #ifndef BUTTON_1_LONG
        #define BUTTON_1_LONG     CMD_FIRSTTRACK
    #endif
    #ifndef BUTTON_2_LONG
        #define BUTTON_2_LONG     CMD_PLAYPAUSE
    #endif
    #ifndef BUTTON_3_LONG
        #define BUTTON_3_LONG     CMD_SLEEPMODE
    #endif
    #ifndef BUTTON_4_LONG
        #define BUTTON_4_LONG     CMD_NOTHING
    #endif
    #ifndef BUTTON_5_LONG
        #define BUTTON_5_LONG     CMD_NOTHING
    #endif

    #ifndef BUTTON_MULTI_01
        #define BUTTON_MULTI_01   CMD_NOTHING   //CMD_TOGGLE_WIFI_STATUS (disabled now to prevent children from unwanted WiFi-disable)
    #endif
    #ifndef BUTTON_MULTI_02
        #define BUTTON_MULTI_02   CMD_ENABLE_FTP_SERVER
    #endif
    #ifndef BUTTON_MULTI_03
        #define BUTTON_MULTI_03   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_04
        #define BUTTON_MULTI_04   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_05
        #define BUTTON_MULTI_05   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_12
        #define BUTTON_MULTI_12   CMD_TELL_IP_ADDRESS
    #endif
    #ifndef BUTTON_MULTI_13
        #define BUTTON_MULTI_13   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_14
        #define BUTTON_MULTI_14   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_15
        #define BUTTON_MULTI_15   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_23
        #define BUTTON_MULTI_23   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_24
        #define BUTTON_MULTI_24   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_25
        #define BUTTON_MULTI_25   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_34
        #define BUTTON_MULTI_34   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_35
        #define BUTTON_MULTI_35   CMD_NOTHING
    #endif
    #ifndef BUTTON_MULTI_45
        #define BUTTON_MULTI_45   CMD_NOTHING
    #endif

    //#################### Various settings ##############################

    // Serial-logging-configuration
    #ifndef SERIAL_LOGLEVEL
        #define SERIAL_LOGLEVEL LOGLEVEL_DEBUG          // Current loglevel for serial console
    #endif

    // Static ip-configuration
    #ifdef STATIC_IP_ENABLE
        #ifndef LOCAL_IP
            #define LOCAL_IP   192,168,2,100            // ESPuino's IP
        #endif
        #ifndef GATEWAY_IP
            #define GATEWAY_IP 192,168,2,1              // IP of the gateway/router
        #endif
        #ifndef SUBNET_IP
            #define SUBNET_IP  255,255,255,0            // Netmask of your network (/24 => 255.255.255.0)
        #endif
        #ifndef DNS_IP
            #define DNS_IP     192,168,2,1              // DNS-server of your network; in private networks it's usually the gatewy's IP
        #endif
    #endif

    // Buttons (better leave unchanged if in doubts :-))
    #ifndef BUTTON_DEBOUNCE_INTERVAL
        #define BUTTON_DEBOUNCE_INTERVAL 50             // Interval in ms to software-debounce buttons
    #endif
    #ifndef INTERVAL_TO_LONG_PRESS
        #define INTERVAL_TO_LONG_PRESS 700              // Interval in ms to distinguish between short and long press of previous/next-button
    #endif
    constexpr uint8_t buttonDebounceInterval = BUTTON_DEBOUNCE_INTERVAL;
    constexpr uint16_t intervalToLongPress = INTERVAL_TO_LONG_PRESS;

    // RFID-RC522
    #ifndef RFID_SCAN_INTERVAL
        #define RFID_SCAN_INTERVAL 250                  // Interval-time in ms (how often is RFID read?)
    #endif

    // Automatic restart
    #ifdef SHUTDOWN_IF_SD_BOOT_FAILS
        #ifndef DEEPSLEEP_TIME_AFTER_BOOT_FAILS
            #define DEEPSLEEP_TIME_AFTER_BOOT_FAILS 20  // Automatic restart takes place if boot was not successful after this period (in seconds)
        #endif
        constexpr uint32_t deepsleepTimeAfterBootFails = DEEPSLEEP_TIME_AFTER_BOOT_FAILS;
    #endif

    // FTP
    // Nothing to be configured here...
    // Default user/password is esp32/esp32 but can be changed via webgui

    // ESPuino will create a WiFi if joing existing WiFi was not possible. Name can be configured here.
    #ifndef AP_SSID
        #define AP_SSID "ESPuino";                      // Access-point's SSID
    #endif
    #ifndef BT_DEVICE
        #define BT_DEVICE AP_SSID                       // Name of your ESPuino as Bluetooth-device. The default name is the name of the AP SSID
    #endif
    constexpr const char accessPointNetworkSSID[] PROGMEM = AP_SSID;
    constexpr const char nameBluetoothDevice[] PROGMEM = BT_DEVICE;

    // Where to store the backup-file for NVS-records
    #ifndef BACKUP_FILE
        #define BACKUP_FILE "/backup.txt"               // File is written every time a (new) RFID-assignment via GUI is done
    #endif
    #ifndef PLAYLIST_CACHE_FILE
        #define PLAYLIST_CACHE_FILE "playlistcache.csv" // Filename that is used for caching playlists
    #endif
    constexpr const char backupFile[] PROGMEM = BACKUP_FILE;
    constexpr const char playlistCacheFile[] PROGMEM = PLAYLIST_CACHE_FILE;

    //#################### Settings for optional Modules##############################
    // (optinal) Neopixel
    #ifdef NEOPIXEL_ENABLE
        #ifndef NUM_LEDS
            #define NUM_LEDS 24                         // number of LEDs
        #endif
        #ifndef CHIPSET
            #define CHIPSET WS2812B                     // type of Neopixel
        #endif
        #ifndef COLOR_ORDER
            #define COLOR_ORDER GRB
        #endif
        #ifndef PROGRESS_HUE_START
            #define PROGRESS_HUE_START 85               // Start and end hue of mulitple-LED progress indicator. Hue ranges from basically 0 - 255, but you can also set numbers outside this range to get the desired effect (e.g. 85-215 will go from green to purple via blue, 341-215 start and end at exactly the same color but go from green to purple via yellow and red)
        #endif
        #ifndef PROGRESS_HUE_END
            #define PROGRESS_HUE_END -1
        #endif
        #ifndef LED_OFFSET
            #define LED_OFFSET 0                        // shifts the starting LED in the original direction of the neopixel ring
        #elif LED_OFFSET < 0 || LED_OFFSET >= NUM_LEDS
            #error LED_OFFSET must be between 0 and NUM_LEDS-1
        #endif
    #endif

    #if defined(MEASURE_BATTERY_VOLTAGE) || defined(MEASURE_BATTERY_MAX17055)
        #if defined(MEASURE_BATTERY_VOLTAGE) && defined(MEASURE_BATTERY_MAX17055)
            #error Only set one of MEASURE_BATTERY_VOLTAGE and MEASURE_BATTERY_MAX17055
        #endif
        #define BATTERY_MEASURE_ENABLE                  // Don't change. Set automatically if any method of battery monitoring is selected.
        #ifndef BATTERY_CHECK_INTERVAL
            #define BATTERY_CHECK_INTERVAL 10           // How often battery is measured (in minutes) (This is a default value that can be changed via GUI)
        #endif
        constexpr uint8_t s_batteryCheckInterval = BATTERY_CHECK_INTERVAL;
    #endif

    #ifdef MEASURE_BATTERY_VOLTAGE
        // (optional) Default-voltages for battery-monitoring via Neopixel; can be changed later via WebGUI
        #ifndef BATTERY_WARNING_LOW
            #define BATTERY_WARNING_LOW 3.4             // If battery-voltage is <= this value, a cyclic warning will be indicated by Neopixel (can be changed via GUI)
        #endif
        #ifndef BATTERY_WARNING_CRITICAL
            #define BATTERY_WARNING_CRITICAL 3.0        // If battery-voltage is <= this value, assume battery near-empty. Set to 0V to disable.
        #endif
        #ifndef BATTERY_VOLTAGE_INDICATOR_LOW
            #define BATTERY_VOLTAGE_INDICATOR_LOW 3.0   // Lower range for Neopixel-voltage-indication (0 leds) (can be changed via GUI)
        #endif
        #ifndef BATTERY_VOLTAGE_INDICATOR_HIGH
            #define BATTERY_VOLTAGE_INDICATOR_HIGH 4.2  // Upper range for Neopixel-voltage-indication (all leds) (can be changed via GUI)
        #endif
        constexpr float s_warningLowVoltage = BATTERY_WARNING_LOW;
        constexpr float s_warningCriticalVoltage = BATTERY_WARNING_CRITICAL;
        constexpr float s_voltageIndicatorLow = BATTERY_VOLTAGE_INDICATOR_LOW;
        constexpr float s_voltageIndicatorHigh = BATTERY_VOLTAGE_INDICATOR_HIGH;
    #endif

    #ifdef MEASURE_BATTERY_MAX17055
    // TODO
        #ifndef BATTERY_LOW
            #define BATTERY_LOW 15.0                    // low percentage
        #endif
        #ifndef BATTERY_CRITICAL
            #define BATTERY_CRITICAL 5.0                // critical percentage
        #endif
        constexpr float s_batteryLow = BATTERY_LOW;
        constexpr float s_batteryCritical = BATTERY_CRITICAL;

        #ifndef BATTERY_CAPACITY
            #define BATTERY_CAPACITY 6000               // design cap of battery (mAh)
        #endif
        #ifndef BATTERY_EMPTY_VOLTAGE
            #define BATTERY_EMPTY_VOLTAGE 300           // empty voltage in 10mV
        #endif
        #ifndef BATTERY_RECOVERY_VOLTAGE
            #define BATTERY_RECOVERY_VOLTAGE 360        // recovery voltage in 10mV
        #endif
        #ifndef BATTERY_CHEMISTRY
            #define BATTERY_CHEMISTRY 0x60              // 0 = Li-Ion, 0x20 = NCR, 0x60 = LiFePO4
        #endif
        #ifndef BATTERY_RESIST_SENSOR
            #define BATTERY_RESIST_SENSOR 0.01          // current sense resistor, currently non-default values might lead to problems
        #endif
        #ifndef BATTERY_V_CHARGE
            #define BATTERY_V_CHARGE false              // true if charge voltage is greater than 4.275V
        #endif
        constexpr uint16_t s_batteryCapacity = BATTERY_CAPACITY;
        constexpr uint16_t s_emptyVoltage = BATTERY_EMPTY_VOLTAGE;
        constexpr uint16_t s_recoveryVoltage = BATTERY_RECOVERY_VOLTAGE;
        constexpr uint8_t  s_batteryChemistry = BATTERY_CHEMISTRY;
        constexpr float s_resistSensor = BATTERY_RESIST_SENSOR;
        constexpr bool s_vCharge = BATTERY_V_CHARGE;
    #endif

    // enable I2C if necessary
    #if defined(RFID_READER_TYPE_MFRC522_I2C) || defined(PORT_EXPANDER_ENABLE) || defined(MEASURE_BATTERY_MAX17055)
        #define I2C_2_ENABLE
    #endif

    // (optinal) Headphone-detection (leave unchanged if in doubts...)
    #ifdef HEADPHONE_ADJUST_ENABLE
        #ifndef HEADPHONE_LAST_DETECTION_DEBOUNCE
            #define HEADPHONE_LAST_DETECTION_DEBOUNCE 1000        // Debounce-interval in ms when plugging in headphone
        #endif
        constexpr uint16_t headphoneLastDetectionDebounce = HEADPHONE_LAST_DETECTION_DEBOUNCE;
    #endif

    // Seekmode-configuration
    #ifndef JUMP_OFFSET
        #define JUMP_OFFSET 30                                    // Offset in seconds to jump for commands CMD_SEEK_FORWARDS / CMD_SEEK_BACKWARDS
    #endif
    constexpr uint8_t jumpOffset = JUMP_OFFSET;

    // (optional) Topics for MQTT
    #ifdef MQTT_ENABLE
        #ifndef MQTT_RETRY_INTERVAL
            #define MQTT_RETRY_INTERVAL 60                        // Try to reconnect to MQTT-server every (n) seconds if connection is broken
        #endif
        #ifndef MQTT_MAX_RETRIES_PER_INTERVAL
            #define MQTT_MAX_RETRIES_PER_INTERVAL 1               // Number of retries per time-interval (mqttRetryInterval). mqttRetryInterval 60 / mqttMaxRetriesPerInterval 1 => once every 60s
        #endif
        constexpr uint16_t mqttRetryInterval = MQTT_RETRY_INTERVAL;
        constexpr uint8_t mqttMaxRetriesPerInterval = MQTT_MAX_RETRIES_PER_INTERVAL;

        #ifndef DEVICE_HOSTNAME
            #define DEVICE_HOSTNAME "ESP32-ESPuino"         // Name that is used for MQTT
        #endif

        #ifndef MQTT_TOPIC_SLEEP_CMND
            #define MQTT_TOPIC_SLEEP_CMND "Cmnd/ESPuino/Sleep"
        #endif
        #ifndef MQTT_TOPIC_SLEEP_STATE
            #define MQTT_TOPIC_SLEEP_STATE "State/ESPuino/Sleep"
        #endif
        #ifndef MQTT_TOPIC_RFID_CMND
            #define MQTT_TOPIC_RFID_CMND "Cmnd/ESPuino/Rfid"
        #endif
        #ifndef MQTT_TOPIC_RFID_STATE
            #define MQTT_TOPIC_RFID_STATE "State/ESPuino/Rfid"
        #endif
        #ifndef MQTT_TOPIC_TRACK_STATE
            #define MQTT_TOPIC_TRACK_STATE "Cmnd/ESPuino/TrackControl"
        #endif
        #ifndef MQTT_TOPIC_TRACK_CONTROL_CMND
            #define MQTT_TOPIC_TRACK_CONTROL_CMND "Cmnd/ESPuino/TrackControl"
        #endif
        #ifndef MQTT_TOPIC_COVER_CHANGED_STATE
            #define MQTT_TOPIC_COVER_CHANGED_STATE "State/ESPuino/CoverChanged"
        #endif
        #ifndef MQTT_TOPIC_LOUDNESS_CMND
            #define MQTT_TOPIC_LOUDNESS_CMND "Cmnd/ESPuino/Loudness"
        #endif
        #ifndef MQTT_TOPIC_LOUDNESS_STATE
            #define MQTT_TOPIC_LOUDNESS_STATE "State/ESPuino/Loudness"
        #endif
        #ifndef MQTT_TOPIC_SLEEP_TIMER_CMND
            #define MQTT_TOPIC_SLEEP_TIMER_CMND "Cmnd/ESPuino/SleepTimer"
        #endif
        #ifndef MQTT_TOPIC_SLEEP_TIMER_STATE
            #define MQTT_TOPIC_SLEEP_TIMER_STATE "State/ESPuino/SleepTimer"
        #endif
        #ifndef MQTT_TOPIC_STATE
            #define MQTT_TOPIC_STATE "State/ESPuino/State"
        #endif
        #ifndef MQTT_TOPIC_CURRENT_IPV4_IP
            #define MQTT_TOPIC_CURRENT_IPV4_IP "State/ESPuino/IPv4"
        #endif
        #ifndef MQTT_TOPIC_LOCK_CONTROLS_CMND
            #define MQTT_TOPIC_LOCK_CONTROLS_CMND "Cmnd/ESPuino/LockControls"
        #endif
        #ifndef MQTT_TOPIC_LOCK_CONTROL_STATE
            #define MQTT_TOPIC_LOCK_CONTROL_STATE "State/ESPuino/LockControls"
        #endif
        #ifndef MQTT_TOPIC_PLAYMODE_STATE
            #define MQTT_TOPIC_PLAYMODE_STATE "State/ESPuino/Playmode"
        #endif
        #ifndef MQTT_TOPIC_REPEAT_MODE_CMND
            #define MQTT_TOPIC_REPEAT_MODE_CMND "Cmnd/ESPuino/RepeatMode"
        #endif
        #ifndef MQTT_TOPIC_REPEAT_MODE_STATE
            #define MQTT_TOPIC_REPEAT_MODE_STATE "State/ESPuino/RepeatMode"
        #endif
        #ifndef MQTT_TOPIC_LED_BRIGHTNESS_CMND
            #define MQTT_TOPIC_LED_BRIGHTNESS_CMND "Cmnd/ESPuino/LedBrightness"
        #endif
        #ifndef MQTT_TOPIC_LED_BRIGHTNESS_STATE
            #define MQTT_TOPIC_LED_BRIGHTNESS_STATE "State/ESPuino/LedBrightness"
        #endif
        #ifndef MQTT_TOPIC_WIFI_RSSI_STATE
            #define MQTT_TOPIC_WIFI_RSSI_STATE "State/ESPuino/WifiRssi"
        #endif
        #ifndef MQTT_TOPIC_SOFTWARE_REVISION_STATE
            #define MQTT_TOPIC_SOFTWARE_REVISION_STATE "State/ESPuino/SoftwareRevision"
        #endif
        #ifndef MQTT_TOPIC_BATTERY_VOLTAGE
            #define MQTT_TOPIC_BATTERY_VOLTAGE "State/ESPuino/Voltage"
        #endif
        #ifndef MQTT_TOPIC_BATTERY_SOC
            #define MQTT_TOPIC_BATTERY_SOC "State/ESPuino/Battery"
        #endif
        constexpr const char topicSleepCmnd[] PROGMEM = MQTT_TOPIC_SLEEP_CMND;
        constexpr const char topicSleepState[] PROGMEM = MQTT_TOPIC_SLEEP_STATE;
        constexpr const char topicRfidCmnd[] PROGMEM = MQTT_TOPIC_RFID_CMND;
        constexpr const char topicRfidState[] PROGMEM = MQTT_TOPIC_RFID_STATE;
        constexpr const char topicTrackState[] PROGMEM = MQTT_TOPIC_TRACK_STATE;
        constexpr const char topicTrackControlCmnd[] PROGMEM = MQTT_TOPIC_TRACK_CONTROL_CMND;
        constexpr const char topicCoverChangedState[] PROGMEM = MQTT_TOPIC_COVER_CHANGED_STATE;
        constexpr const char topicLoudnessCmnd[] PROGMEM = MQTT_TOPIC_LOUDNESS_CMND;
        constexpr const char topicLoudnessState[] PROGMEM = MQTT_TOPIC_LOUDNESS_STATE;
        constexpr const char topicSleepTimerCmnd[] PROGMEM = MQTT_TOPIC_SLEEP_TIMER_CMND;
        constexpr const char topicSleepTimerState[] PROGMEM = MQTT_TOPIC_SLEEP_TIMER_STATE;
        constexpr const char topicState[] PROGMEM = MQTT_TOPIC_STATE;
        constexpr const char topicCurrentIPv4IP[] PROGMEM = MQTT_TOPIC_CURRENT_IPV4_IP;
        constexpr const char topicLockControlsCmnd[] PROGMEM = MQTT_TOPIC_LOCK_CONTROLS_CMND;
        constexpr const char topicLockControlsState[] PROGMEM = MQTT_TOPIC_LOCK_CONTROL_STATE;
        constexpr const char topicPlaymodeState[] PROGMEM = MQTT_TOPIC_PLAYMODE_STATE;
        constexpr const char topicRepeatModeCmnd[] PROGMEM = MQTT_TOPIC_REPEAT_MODE_CMND;
        constexpr const char topicRepeatModeState[] PROGMEM = MQTT_TOPIC_REPEAT_MODE_STATE;
        constexpr const char topicLedBrightnessCmnd[] PROGMEM = MQTT_TOPIC_LED_BRIGHTNESS_CMND;
        constexpr const char topicLedBrightnessState[] PROGMEM = MQTT_TOPIC_LED_BRIGHTNESS_STATE;
        constexpr const char topicWiFiRssiState[] PROGMEM = MQTT_TOPIC_WIFI_RSSI_STATE;
        constexpr const char topicSRevisionState[] PROGMEM = MQTT_TOPIC_SOFTWARE_REVISION_STATE;
        #ifdef BATTERY_MEASURE_ENABLE
            constexpr const char topicBatteryVoltage[] PROGMEM = MQTT_TOPIC_BATTERY_VOLTAGE;
            constexpr const char topicBatterySOC[] PROGMEM     = MQTT_TOPIC_BATTERY_SOC;
        #endif
    #endif

    // !!! MAKE SURE TO EDIT PLATFORM SPECIFIC settings-****.h !!!
    #if (HAL == 1)
        #include "settings-lolin32.h"                       // Contains all user-relevant settings for Wemos Lolin32
    #elif (HAL == 2)
        #include "settings-espa1s.h"                        // Contains all user-relevant settings for ESP32-A1S Audiokit
    #elif (HAL == 3)
        #include "settings-lolin_d32.h"                     // Contains all user-relevant settings for Wemos Lolin D32
    #elif (HAL == 4)
        #include "settings-lolin_d32_pro.h"                 // Contains all user-relevant settings for Wemos Lolin D32 pro
    #elif (HAL == 5)
        #include "settings-ttgo_t8.h"                       // Contains all user-relevant settings for Lilygo TTGO T8 1.7
    #elif (HAL == 6)
        #include "settings-complete.h"                      // Contains all user-relevant settings for ESPuino complete
    #elif (HAL == 7)
        #include "settings-lolin_d32_pro_sdmmc_pe.h"        // Pre-configured settings for ESPuino Lolin D32 pro with SDMMC + port-expander (https://forum.espuino.de/t/espuino-minid32pro-lolin-d32-pro-mit-sd-mmc-und-port-expander-smd/866)
    #elif (HAL == 8)
        #include "settings-azdelivery_sdmmc.h"              // Pre-configured settings for AZ Delivery ESP32 NodeMCU / Devkit C (https://forum.espuino.de/t/az-delivery-esp32-nodemcu-devkit-c-mit-sd-mmc-und-pn5180-als-rfid-leser/634)
    #elif (HAL == 9)
        #include "settings-lolin_d32_sdmmc_pe.h"            // Pre-configured settings for ESPuino Lolin D32 pro with SDMMC + port-expander (https://forum.espuino.de/t/espuino-minid32-pro-lolin-d32-pro-mit-sd-mmc-und-port-expander-smd/866)
    #elif (HAL == 99)
        #include "settings-custom.h"                        // Contains all user-relevant settings custom-board
    #elif (HAL == 100)
        #if defined(HAL_SETTINGS_FILE) && __has_include(HAL_SETTINGS_FILE)
            #include HAL_SETTINGS_FILE
        #else
            #error define HAL_SETTINGS_FILE if you want to use HAL=100
        #endif
    #endif

    //#define ENABLE_ESPUINO_DEBUG                            // Needs modification of platformio.ini (https://forum.espuino.de/t/rfid-mit-oder-ohne-task/353/21); better don't enable unless you know what you're doing :-)
#endif //settings_override
#endif
