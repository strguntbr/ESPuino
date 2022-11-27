#include <Arduino.h>
#include "settings.h"
#include "Rfid.h"
#include "Log.h"
#include "MemX.h"
#include "Queues.h"
#include "System.h"
#include <esp_task_wdt.h>
#include "AudioPlayer.h"

#if defined RFID_READER_TYPE_MFRC522_SPI || defined RFID_READER_TYPE_MFRC522_I2C
    #ifdef RFID_READER_TYPE_MFRC522_SPI
        #include <MFRC522.h>
    #endif
    #if defined(RFID_READER_TYPE_MFRC522_I2C) || defined(PORT_EXPANDER_ENABLE)
        #include "Wire.h"
    #endif
    #ifdef RFID_READER_TYPE_MFRC522_I2C
        #include <MFRC522_I2C.h>
    #endif

    extern unsigned long Rfid_LastRfidCheckTimestamp;
    static void Rfid_Task(void *parameter);

    #ifdef RFID_READER_TYPE_MFRC522_I2C
        extern TwoWire i2cBusTwo;
        static MFRC522_I2C mfrc522(MFRC522_ADDR, MFRC522_RST_PIN, &i2cBusTwo);
    #endif
    #ifdef RFID_READER_TYPE_MFRC522_SPI
        static MFRC522 mfrc522(RFID_CS, RST_PIN);
    #endif

    static bool Mfrc522_RequestA(void) {
        uint8_t bufferATQA[2];
        uint8_t bufferSize = sizeof(bufferATQA);
        MFRC522::StatusCode result = mfrc522.PICC_RequestA(bufferATQA, &bufferSize);
        return result == MFRC522::STATUS_OK;    
    }

    static bool Mfrc522_WakeupA(void) {
        uint8_t bufferATQA[2];
        uint8_t bufferSize = sizeof(bufferATQA);
        MFRC522::StatusCode result = mfrc522.PICC_WakeupA(bufferATQA, &bufferSize);
        return result == MFRC522::STATUS_OK;
    }

    static void Mfrc522_HaltA(void) {
        MFRC522::StatusCode result = mfrc522.PICC_HaltA();
        if (result != MFRC522::STATUS_OK) {
            Log_Println((char *) FPSTR(mfrc522HaltFailed), LOGLEVEL_NOTICE);
        }
    }

    void Rfid_Init(void) {
        #ifdef RFID_READER_TYPE_MFRC522_SPI
            SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS);
            SPI.setFrequency(1000000);
        #endif

        // Init RC522 Card-Reader
        #if defined(RFID_READER_TYPE_MFRC522_I2C) || defined(RFID_READER_TYPE_MFRC522_SPI)
            mfrc522.PCD_Init();
            mfrc522.PCD_SetAntennaGain(rfidGain);
            delay(50);
            Log_Println((char *) FPSTR(rfidScannerReady), LOGLEVEL_DEBUG);

            xTaskCreatePinnedToCore(
                Rfid_Task,              /* Function to implement the task */
                "rfid",                 /* Name of the task */
                1536,                   /* Stack size in words */
                NULL,                   /* Task input parameter */
                2 | portPRIVILEGE_BIT,  /* Priority of the task */
                NULL,                   /* Task handle. */
                1                       /* Core where the task should run */
            );
        #endif
    }

    void Rfid_Task(void *parameter) {
        const TickType_t xFrequency = RFID_SCAN_INTERVAL / portTICK_PERIOD_MS;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        bool tagCurrentlyActive = false;
        byte cardId[cardIdSize];
        #ifdef PAUSE_WHEN_RFID_REMOVED
            const bool pauseWhenRfidRemoved = true;
        #else
            const bool pauseWhenRfidRemoved = false;
        #endif

        for (;;) {
            if (!tagCurrentlyActive) {
                if (Mfrc522_RequestA()) {
                    if (mfrc522.PICC_ReadCardSerial()) {
                        tagCurrentlyActive = true;
                        if (pauseWhenRfidRemoved && memcmp(cardId, mfrc522.uid.uidByte, cardIdSize) == 0) {
                            // same tag reapplied
                            if (gPlayProperties.pausePlay && System_GetOperationMode() != OPMODE_BLUETOOTH_SINK) {
                                AudioPlayer_TrackControlToQueueSender(PAUSEPLAY);       // ... play/pause instead (but not for BT)
                            }
                            Log_Println((char *) FPSTR(rfidTagReapplied), LOGLEVEL_NOTICE);
                        } else {
                            memcpy(cardId, mfrc522.uid.uidByte, cardIdSize);
                            String cardIdString;
                            for (uint8_t i=0u; i < cardIdSize; i++) {
                                char num[4];
                                snprintf(num, sizeof(num), "%03d", cardId[i]);
                                cardIdString += num;
                            }
                            xQueueSend(gRfidCardQueue, cardIdString.c_str(), 0);
                            Log_Print((char *) FPSTR(rfidTagDetected), LOGLEVEL_NOTICE);
                            for (uint8_t i=0u; i < cardIdSize; i++) {
                                snprintf(Log_Buffer, Log_BufferLength, "%02x%s", cardId[i], (i < cardIdSize - 1u) ? "-" : "\n");
                                Log_Print(Log_Buffer, LOGLEVEL_NOTICE);
                            }
                        }
                    }
                    Mfrc522_HaltA();
                }
            } else {
                if (Mfrc522_WakeupA()) {
                    if (mfrc522.PICC_ReadCardSerial() && memcmp(cardId, mfrc522.uid.uidByte, cardIdSize) == 0) {
                        // still found same tag
                        Mfrc522_HaltA();
                        continue;
                    }
                    Mfrc522_HaltA();
                }
                tagCurrentlyActive = false;
                #ifdef PAUSE_WHEN_RFID_REMOVED
                    Log_Println((char *) FPSTR(rfidTagRemoved), LOGLEVEL_NOTICE);
                    if (!gPlayProperties.pausePlay && System_GetOperationMode() != OPMODE_BLUETOOTH_SINK) {
                        AudioPlayer_TrackControlToQueueSender(PAUSEPLAY);
                    }
                #endif
            }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void Rfid_Cyclic(void) {
        // Not necessary as cyclic stuff performed by task Rfid_Task()
    }

    void Rfid_Exit(void) {
        #ifndef RFID_READER_TYPE_MFRC522_I2C
            mfrc522.PCD_SoftPowerDown();
        #endif
    }

    void Rfid_WakeupCheck(void) {
    }

#endif
