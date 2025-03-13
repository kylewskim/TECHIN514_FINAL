#include <Arduino.h>
#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

// BLE 서비스 및 특성 설정
BLEService myService("40808a19-ee3d-49df-bfbb-e76f5bcbafde");  
BLEIntCharacteristic angleCharacteristic("53d800b6-f31c-4650-8304-64964e55012a", BLERead | BLENotify);

LSM6DS3 myIMU(I2C_MODE, 0x6A);
float angle;
const int N = 5;  
int angleBuffer[N] = {0};
int bufferIndex = 0;
int filtered_angle = 0;

unsigned long previousMillis = 0;
const long interval = 1000;

int movingAverageFilter(int newValue) {
    static int sum = 0;
    sum -= angleBuffer[bufferIndex];  
    angleBuffer[bufferIndex] = newValue;  
    sum += newValue;
    bufferIndex = (bufferIndex + 1) % N;
    return sum / N;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    if (myIMU.begin() != 0) {
        Serial.println("❌ Sensor Error!");
        while (1);
    } else {
        Serial.println("✅ Sensor OK!");
    }

    Serial.println("🚀 Starting BLE...");

    // BLE 초기화
    if (!BLE.begin()) {
        Serial.println("❌ BLE initialization failed!");
        while (1);
    }

    BLE.setLocalName("KYLE_SD");
    BLE.setAdvertisedService(myService);

    // BLE 서비스 및 특성 추가
    myService.addCharacteristic(angleCharacteristic);
    BLE.addService(myService);

    // BLE 광고 시작
    BLE.advertise();
    Serial.println("✅ BLE Advertising started! You can connect to it.");
}

void loop() {
    BLEDevice central = BLE.central();  // BLE 중앙 기기 탐색

    if (central) {
        Serial.print("✅ Connected to central: ");
        Serial.println(central.address());

        while (central.connected()) {
            float accelX = myIMU.readFloatAccelX();
            float accelY = myIMU.readFloatAccelY();
            float accelZ = myIMU.readFloatAccelZ();

            angle = atan2(accelX, accelZ) * 180.0 / M_PI;
            filtered_angle = (int)movingAverageFilter(angle);

            Serial.print("Raw: ");
            Serial.print(angle);
            Serial.print(" | Filtered: ");
            Serial.println(filtered_angle);

            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
                angleCharacteristic.writeValue(filtered_angle);
                Serial.print("📡 Notify sent: ");
                Serial.println(filtered_angle);
                previousMillis = currentMillis;
            }
            delay(1000);
        }

        Serial.print("❌ Disconnected from central: ");
        Serial.println(central.address());
    }
}