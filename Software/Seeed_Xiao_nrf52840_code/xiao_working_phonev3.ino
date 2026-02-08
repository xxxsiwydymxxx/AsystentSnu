#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <bluefruit.h>

#define LEDR 11
#define LEDG 12
#define LEDB 13

PulseOximeter pox;
uint32_t tsLastReport = 0;
bool sensorWorking = false;

const uint16_t CUSTOM_SERVICE_UUID = 0xFF00;
const uint16_t CUSTOM_CHAR_UUID    = 0xFF01;

BLEService        customService = BLEService(CUSTOM_SERVICE_UUID);
BLECharacteristic customChar    = BLECharacteristic(CUSTOM_CHAR_UUID);

void onBeatDetected() {
    // Flash Green
    digitalWrite(LEDG, LOW); delay(20); digitalWrite(LEDG, HIGH);
}

void setup() {
    pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);

    Serial.begin(115200);
    
    // Sensor Init
    if (!pox.begin()) {
        sensorWorking = false;
        digitalWrite(LEDR, LOW); 
    } else {
        sensorWorking = true;
        pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
        pox.setOnBeatDetectedCallback(onBeatDetected);
        digitalWrite(LEDG, LOW); delay(200); digitalWrite(LEDG, HIGH);
    }

    Bluefruit.begin();
    Bluefruit.setName("XIAO_Health_Native");

    customService.begin();

    // --- FIX IS HERE ---
    customChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    // Allow Open Write access so the Pico can Subscribe
    customChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    // -------------------
    
    customChar.setFixedLen(2); 
    customChar.begin();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(customService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.start(0); 
}

void loop() {
    if (sensorWorking) pox.update();

    if (millis() - tsLastReport > 1000) {
        uint8_t bpm = 0;
        uint8_t spo2 = 0;

        if (sensorWorking) {
            bpm = (uint8_t)pox.getHeartRate();
            spo2 = pox.getSpO2();
        } else {
            bpm = 65; spo2 = 98; // Dummy data
        }

        if (Bluefruit.connected()) {
            if (customChar.notifyEnabled()) {
                uint8_t packet[2] = {bpm, spo2};
                customChar.notify(packet, 2);
                Serial.print("Sent -> HR:"); Serial.print(bpm);
                Serial.print(" SpO2:"); Serial.println(spo2);
            } else {
                Serial.println("Connected - Waiting for Pico to Subscribe...");
            }
        }
        tsLastReport = millis();
    }
}