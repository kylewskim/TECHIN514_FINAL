#include <Arduino.h>
#include <SwitecX25.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>  
#include <RotaryEncoder.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define SCREEN_WIDTH 128  
#define SCREEN_HEIGHT 128 
#define OLED_RESET -1     
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET, 1000000, 100000);

// #define A_PLUS 18
// #define A_MINUS 17
#define A_PLUS A0
#define A_MINUS A1
#define B_PLUS 8
#define B_MINUS 9
#define STEPS 945

#define ROTARY_B 5 // BUTTON
#define ROTARY_A1 6 // CLK
#define ROTARY_A2 7 // DT

#define LED_R 35
#define LED_G 37
#define LED_B 36

static BLEUUID serviceUUID("40808a19-ee3d-49df-bfbb-e76f5bcbafde");
static BLEUUID charUUID("53d800b6-f31c-4650-8304-64964e55012a");

static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

int status = 1;  // 1: Goal 설정 -> 2: OKIE 화면 -> 3: Ready 화면 (BLE 데이터 수신)
int goal = 1;
int receivedAngle = 0;
bool lowAngleDetected = false; // ✅ 초록 → 빨간 사이클 감지용

bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const int debounceDelay = 50; // 버튼 디바운싱 딜레이
String BLEstatus = "";

SwitecX25 motor1(STEPS, A_PLUS, A_MINUS, B_PLUS, B_MINUS);
RotaryEncoder *encoder = nullptr;

// ✅ 인터럽트 기반 로터리 엔코더 감지 함수
void checkPosition() {
    encoder->tick();
}

// ✅ BLE Notify 수신 콜백 (데이터 수신 시 status = 3)
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

    Serial.println("🔔 BLE Notify Callback 실행됨!");
    
    if (length == 4) {
        receivedAngle = (pData[0]) | (pData[1] << 8) | (pData[2] << 16) | (pData[3] << 24);
        Serial.print("📡 Received Angle: ");
        Serial.println(receivedAngle);

        // ✅ LED 상태 업데이트 & 목표 개수 감소
        updateLEDandGoal();
        
        int motorTargetPos = receivedAngle * 3.25 + 270;
        Serial.print("🌀 Motor Target Position: ");
        Serial.println(motorTargetPos);
        motor1.setPosition(motorTargetPos); // 모터 이동 명령
    }

    // 최초 데이터 수신 시 status 변경
    if (status == 2) {
        status = 3;
        Serial.println("✅ Status 변경: 3 (Ready!)");
    }
}

// BLE Client Callback
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("❌ Disconnected from BLE server.");
    BLEDevice::getScan()->start(10, false);
  }
};

// ✅ BLE 기기 스캔 결과 처리
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.println("🔍 Found BLE Device!");
    BLEstatus = "Searching device";
    if (advertisedDevice.isAdvertisingService(serviceUUID)) {
        Serial.println("✅ Target BLE Device Found! Connecting...");
        BLEstatus = "Target device found";
        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
    }
  }
};

void setup() {
    Serial.begin(115200);

    pinMode(A_PLUS, OUTPUT);  // 디지털 입력으로 설정
    pinMode(A_MINUS, OUTPUT);

    encoder = new RotaryEncoder(ROTARY_A1, ROTARY_A2, RotaryEncoder::LatchMode::TWO03);
    pinMode(ROTARY_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ROTARY_A1), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_A2), checkPosition, CHANGE);

    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);

  motor1.zero();
  Serial.println("ZERO1");
  delay(1000);
  motor1.update();

  // // **🔹 전원 켜질 때 항상 한 방향으로 이동하여 위치 보정**
  // Serial.println("Resetting motor position...");
  // motor1.setPosition(-STEPS);  // 충분히 반시계 방향으로 이동
  // while (motor1.currentStep != motor1.targetStep) {
  //     motor1.update();
  //     delay(5);
  // }

  // // **🔹 현재 위치를 0으로 설정**
  // motor1.zero();
  // Serial.println("Motor position reset complete!");
    // 벽이 있는 방향(최소 위치)으로 충분히 이동

    display.begin(0x3D, true);
    display.display();
    display.clearDisplay();
    
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(36, 64);
    display.println(F("SITUP"));
    display.display();
      motor1.setPosition(-STEPS);  
  Serial.println("-STEPS");
  Serial.println(motor1.currentStep);
  while (motor1.currentStep != motor1.targetStep) {
      motor1.update();
      delay(5);
  }
  Serial.println("DONE");

  // 이 위치를 기준점(0)으로 설정
  motor1.zero();
  Serial.println("Calibration complete! Motor is now at position 0.");

  // 이제 gauge 사용 범위 (180도)만큼 사용할 수 있음
  motor1.setPosition(STEPS);  // 사용 범위의 중앙으로 이동
  while (motor1.currentStep != motor1.targetStep) {
      motor1.update();
      delay(5);
  }
  motor1.setPosition(STEPS/2);  // 사용 범위의 중앙으로 이동
  while (motor1.currentStep != motor1.targetStep) {
      motor1.update();
      delay(5);
  }

    display.clearDisplay();

    // ✅ BLE 초기화
    Serial.println("🚀 Starting BLE Client...");
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(499);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(10, false);
}

void loop() {
    if (status == 1) {
      handleRotaryEncoder();
      handleButtonPress();
      updateGoalDisplay();
    } else if (status == 2) {
      updateOKIE();
    } else if (status == 3) {
      updateReady();
    } else if (status == 4) {
      updateComplete();
    }

    if (doConnect) {
        connectToBLEServer();
    }

    if (motor1.currentStep != motor1.targetStep && status == 3) {
        motor1.update();
    }
    delay(1);
}

// 🔄 **로터리 인코더 값 감지 + 음수 방지**
void handleRotaryEncoder() {
    static int lastPos = 0;
    encoder->tick();

    int newPos = encoder->getPosition();
    if (newPos != lastPos) {  
        if (newPos > lastPos) {
            goal++;
        } else {
            goal--;
            if (goal < 1) goal = 1;
        }
        lastPos = newPos;
    }
}

// 🔘 **버튼 입력 감지 (status 변경)**
void handleButtonPress() {
    if (digitalRead(ROTARY_B) == LOW && !buttonPressed) {
        delay(debounceDelay);
        if (digitalRead(ROTARY_B) == LOW) {
            status = 2;
            buttonPressed = true;
        }
    }
    if (digitalRead(ROTARY_B) == HIGH) {
        buttonPressed = false;
    }
}

// 🖥 **목표 설정 화면 업데이트**
void updateGoalDisplay() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(54, 64);
    display.println(goal);
    display.setTextSize(1);
    display.setCursor(8, 120);
    display.println(BLEstatus);
    display.display();
}

// 🖥 **OKIE 화면 업데이트**
void updateOKIE() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.clearDisplay();
    display.setCursor(54, 64);
    display.println("3");
    display.display();
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    delay(1000);
    display.clearDisplay();
    display.setCursor(54, 64);
    display.println("2");
    display.display();
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
    delay(1000);
    display.clearDisplay();
    display.setCursor(54, 64);
    display.println("1");
    display.display();
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
    delay(1000);
    status = 3;
}

// 🖥 **Ready 화면 업데이트**
void updateReady() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(54, 64);
    display.println(goal);
    display.setTextSize(1);
    display.setCursor(48, 100);
    display.println(receivedAngle);
    display.display();
}

void updateComplete() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(32, 64);
  display.print("DONE");
  display.display();
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  delay(200);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  delay(200);
    digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  delay(200);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  delay(200);
    digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  delay(200);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  delay(200);
    digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  delay(200);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  delay(200);
    digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  delay(200);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  delay(2000);
  status = 1;
}

// ✅ **LED 상태 업데이트 및 goal 감소 처리**
void updateLEDandGoal() {
    if (receivedAngle <= 20 && status == 3) {
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_R, LOW);
        lowAngleDetected = true;
    } else if (receivedAngle >= 70 && status == 3) {
        digitalWrite(LED_G, LOW);
        digitalWrite(LED_R, HIGH);
        if (lowAngleDetected) {
            goal--;
            lowAngleDetected = false;
            digitalWrite(LED_R, LOW);
            digitalWrite(LED_G, LOW);
            digitalWrite(LED_B, LOW);
            if (goal == 0) status = 4;
        }
    }
}

void connectToBLEServer() {
    Serial.println("🔗 BLE 서버 연결 시도...");
    BLEClient* pClient = BLEDevice::createClient();
    Serial.println(" - ✅ Created client");
    BLEstatus = "Created client";
    updateGoalDisplay();

    pClient->setClientCallbacks(new MyClientCallback());
    if (!pClient->connect(myDevice)) {
        Serial.println("❌ Failed to connect.");
        BLEDevice::getScan()->start(10, false);
        return;
    }

    Serial.println(" - ✅ Connected to server");
    BLEstatus = "Connected to s
    
    erver";
    updateGoalDisplay();

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (!pRemoteService) {
        Serial.println("❌ Failed to find service UUID.");
        pClient->disconnect();
        BLEDevice::getScan()->start(10, false);
        return;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (!pRemoteCharacteristic) {
        Serial.println("❌ Failed to find characteristic UUID.");
        pClient->disconnect();
        BLEDevice::getScan()->start(10, false);
        return;
    }

    if (pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
        Serial.println("✅ Notify 등록 완료! BLE 데이터 수신 대기 중...");
        BLEstatus = "Notify Ready";
        delay(1000);
    }

    connected = true;
    doConnect = false;
}