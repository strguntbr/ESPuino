#ifndef __ESPUINO_SETTINGS_LOLIN_D32_PRO_H__
#define __ESPUINO_SETTINGS_LOLIN_D32_PRO_H__
    #include "Arduino.h"

    //######################### INFOS ####################################
    /* This is a PCB-specific config-file for *Wemos Lolin32 D32 pro with port-expander PCA9555PW and SD_MMC*.
    PCB: t.b.a.
    Forum: https://forum.espuino.de/t/espuino-minid32-pro-lolin-d32-pro-mit-sd-mmc-und-port-expander-smd/866
    Infos: https://www.wemos.cc/en/latest/d32/d32_pro.html
    Schematics Lolin D32 pro: https://www.wemos.cc/en/latest/_static/files/sch_d32_pro_v2.0.0.pdf
    Schematics PCB: t.b.a.
    Caveats: GPIO35 (battery monitoring) + don't use internal SD-slot as it's to capable of SD_MMC because of its pin-layout!
    Status:
        tested with PN5180 + SD_MMC (by biologist79)
    */

    //################## GPIO-configuration ##############################
    // Please note: GPIOs 34, 35, 36, 39 are input-only and don't have internal pullup-resistors.
    // So if connecting a button to these, make sure to add a 10k-pullup-resistor for each button.
    // Further infos: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
    // GPIOs 16+17 are not available for D32 pro as they're used to internal purposes (PSRAM).
    // All GPIOs >=100 and <= 115 are connected to a port-expander
    #ifdef SD_MMC_1BIT_MODE
        //  (MOSI)    15  CMD
        //  (SCK)     14  SCK
        //  (MISO)     2  D0
    #else
        #error SPI-SD IS NOT SUPPORTED BY THIS PCB - DO NOT USE INTERNAL SD-READER!
    #endif

    // RFID (via SPI)
    #define RST_PIN                         99          // Used as dummy for RC522
    #ifndef RFID_CS
        #define RFID_CS                     21          // GPIO for chip select (RFID)
    #endif
    #ifndef RFID_MOSI
        #define RFID_MOSI                   23          // GPIO for master out slave in (RFID)
    #endif
    #ifndef RFID_MISO
        #define RFID_MISO                   19          // GPIO for master in slave out (RFID)
    #endif
    #ifndef RFID_SCK
        #define RFID_SCK                    18          // GPIO for clock-signal (RFID)
    #endif

    #ifdef RFID_READER_TYPE_PN5180
        #ifndef RFID_BUSY
            #define RFID_BUSY               33          // PN5180 BUSY PIN
        #endif
        #ifndef RFID_RST
            #define RFID_RST                22          // PN5180 RESET PIN
        #endif
        #ifndef RFID_IRQ
            #define RFID_IRQ                99          // Needs to be adjusted to 106 if LPCD-mode is desired!
        #endif
    #endif

    // I2S (DAC)
    #ifndef I2S_DOUT
        #define I2S_DOUT                    25          // Digital out (I2S)
    #endif
    #ifndef I2S_BCLK
        #define I2S_BCLK                    27          // BCLK (I2S)
    #endif
    #ifndef I2S_LRC
        #define I2S_LRC                     26          // LRC (I2S)
    #endif

    // Rotary encoder
    #ifdef USEROTARY_ENABLE
        //#define REVERSE_ROTARY                        // To reverse encoder's direction; switching CLK / DT in hardware does the same
        #ifndef ROTARYENCODER_CLK
            #define ROTARYENCODER_CLK       34          // rotary encoder's CLK
        #endif
        #ifndef ROTARYENCODER_DT
            #define ROTARYENCODER_DT        39          // 39 = 'VN'; rotary encoder's DT
        #endif
    #endif

    // Amp enable (optional)
    #ifndef GPIO_PA_EN
        #define GPIO_PA_EN                  108         // To enable/disable amp for loudspeaker; connected to port-expander
    #endif

    // Control-buttons
    #ifndef NEXT_BUTTON
        #define NEXT_BUTTON                 102         // Next-Button: connected to port-expander
    #endif
    #ifndef PREVIOUS_BUTTON
        #define PREVIOUS_BUTTON             100         // Prev-Button: connected to port-expander
    #endif
    #ifndef PAUSEPLAY_BUTTON
        #define PAUSEPLAY_BUTTON            101         // Play-Button: connected to port-expander
    #endif
    #ifndef ROTARYENCODER_BUTTON
        #define ROTARYENCODER_BUTTON        103         // Set to 99 to disable the button; connected to port-expander
    #endif
    #ifndef BUTTON_4
        #define BUTTON_4                    104         // Button 4: connected to port-expander
    #endif
    #ifndef BUTTON_5
        #define BUTTON_5                    105         // Button 5: connected to port-expander
    #endif

    //#define BUTTONS_LED                   114         // Powers the LEDs of the buttons

    // Channels of port-expander can be read cyclic or interrupt-driven. It's strongly recommended to use the interrupt-way!
    // Infos: https://forum.espuino.de/t/einsatz-des-port-expanders-pca9555/306
    #ifdef PORT_EXPANDER_ENABLE
        #ifndef PE_INTERRUPT_PIN
            #define PE_INTERRUPT_PIN        36          // GPIO that is used to receive interrupts from port-expander + to wake up ESP32
        #endif
    #endif

    // I2C-configuration (necessary for PCA9555)
    #ifdef I2C_2_ENABLE
        #ifndef ext_IIC_CLK
            #define ext_IIC_CLK             4           // i2c-SCL (clock)
        #endif
        #ifndef ext_IIC_DATA
            #define ext_IIC_DATA            13          // i2c-SDA (data)
        #endif
    #endif

    // Wake-up button => this also is the interrupt-pin if port-expander is enabled!
    // Please note: only RTC-GPIOs (0, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, 39, 99) can be used! Set to 99 to DISABLE.
    // Please note #2: this button can be used as interrupt-pin for port-expander. If so, all pins connected to port-expander can wake up ESPuino.
    #ifndef WAKEUP_BUTTON
        #define WAKEUP_BUTTON               36          // Defines the button that is used to wake up ESPuino from deepsleep; set to 99 to disable
    #endif

    // Power-control
    #ifndef POWER
        #define POWER                       32          // GPIO used to drive transistor-circuit, that switches off peripheral devices while ESP32-deepsleep
    #elif POWER == 0
        #undef POWER
    #endif
    #ifdef POWER
        //#define INVERT_POWER                          // If enabled, use inverted logic for POWER circuit, that means peripherals are turned off by writing HIGH
    #endif

    // (optional) Neopixel
    #ifndef LED_PIN
        #define LED_PIN                     12          // GPIO for Neopixel-signaling
    #endif

    // (optinal) Headphone-detection
    #ifdef HEADPHONE_ADJUST_ENABLE
        //#define DETECT_HP_ON_HIGH                      // Per default headphones are supposed to be connected if HT_DETECT is LOW. DETECT_HP_ON_HIGH will change this behaviour to HIGH.
        #ifndef HP_DETECT
            #define HP_DETECT               107          // GPIO that detects, if there's a plug in the headphone jack or not; connected to port-expander
        #endif
    #endif

    // (optional) Monitoring of battery-voltage via ADC
    #ifdef MEASURE_BATTERY_VOLTAGE
        #ifndef VOLTAGE_READ_PIN
            #define VOLTAGE_READ_PIN        35          // GPIO used to monitor battery-voltage. Don't change, it's built in
        #endif
        #ifndef REFERENCE_VOLTAGE
            #define REFERENCE_VOLTAGE       3.3         // Voltage between 3.3V and GND-pin at the develboard in battery-mode (disconnect USB!)
        #endif
        #ifndef OFFSET_VOLTAGE
            #define OFFSET_VOLTAGE          0.1         // If voltage measured by ESP isn't 100% accurate, you can add an correction-value here
        #endif
        constexpr float referenceVoltage = REFERENCE_VOLTAGE;
        constexpr float offsetVoltage = OFFSET_VOLTAGE;
    #endif

    // (optional) For measuring battery-voltage a voltage-divider is already onboard. Connect a LiPo and use it!
    #ifdef MEASURE_BATTERY_VOLTAGE
        constexpr uint16_t rdiv1 = 100;                 // Don't change, it's built in
        constexpr uint16_t rdiv2 = 100;                 // Don't change, it's built in
    #endif

    #ifdef IR_CONTROL_ENABLE
        #ifndef IRLED_PIN
            #define IRLED_PIN               5               // GPIO where IR-receiver is connected (only tested with VS1838B)
        #endif
        #ifndef IR_DEBOUNCE
            #define IR_DEBOUNCE             200             // Interval in ms to wait at least for next signal (not used for actions volume up/down)
        #endif

        // Actions available. Use your own remote control and have a look at the console for "Command=0x??". E.g. "Protocol=NEC Address=0x17F Command=0x68 Repeat gap=39750us"
        // Make sure to define a hex-code not more than once as this will lead to a compile-error
        // https://forum.espuino.de/t/neues-feature-fernsteuerung-per-infrarot-fernbedienung/265
        #ifndef RC_PLAY
            #define RC_PLAY                 0x68            // command for play
        #endif
        #ifndef RC_PAUSE
            #define RC_PAUSE                0x67            // command for pause
        #endif
        #ifndef RC_NEXT
            #define RC_NEXT                 0x6b            // command for next track of playlist
        #endif
        #ifndef RC_PREVIOUS
            #define RC_PREVIOUS             0x6a            // command for previous track of playlist
        #endif
        #ifndef RC_FIRST
            #define RC_FIRST                0x6c            // command for first track of playlist
        #endif
        #ifndef RC_LAST
            #define RC_LAST                 0x6d            // command for last track of playlist
        #endif
        #ifndef RC_VOL_UP
            #define RC_VOL_UP               0x1a            // Command for volume up (one step)
        #endif
        #ifndef RC_VOL_DOWN
            #define RC_VOL_DOWN             0x1b            // Command for volume down (one step)
        #endif
        #ifndef RC_MUTE
            #define RC_MUTE                 0x1c            // Command to mute ESPuino
        #endif
        #ifndef RC_SHUTDOWN
            #define RC_SHUTDOWN             0x2a            // Command for deepsleep
        #endif
        #ifndef RC_BLUETOOTH
            #define RC_BLUETOOTH            0x72            // Command to enable/disable bluetooth
        #endif
        #ifndef RC_FTP
            #define RC_FTP                  0x65            // Command to enable FTP-server
        #endif
    #endif
#endif