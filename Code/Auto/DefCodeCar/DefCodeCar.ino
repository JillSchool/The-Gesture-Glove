// Includes //
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <stdint.h>
#include <stdbool.h>

// BLE Data //
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_SERVER_MAC      "08:3A:F2:59:B8:D2"

// Motor Pins //
const int output1 = 25;    
const int output2 = 26;    
const int output3 = 14;    
const int output4 = 13;    

// BLE Data //
BLEClient* pClient;
BLERemoteService* pRemoteService;
BLERemoteCharacteristic* pRemoteCharacteristic;

bool connected = false;
unsigned long lastReceivedTime = 0;
const unsigned long timeout = 5000;  // in ms

// Motor control functions //
void forward() {
  digitalWrite(output1, HIGH);      
  digitalWrite(output2, LOW);       
  digitalWrite(output3, HIGH);      
  digitalWrite(output4, LOW);       
}

void backward() {
  digitalWrite(output1, LOW);
  digitalWrite(output2, HIGH);
  digitalWrite(output3, LOW);
  digitalWrite(output4, HIGH);
}

void backwardLeft() {
  digitalWrite(output1, LOW);
  digitalWrite(output2, HIGH);
  digitalWrite(output3, LOW);
  digitalWrite(output4, LOW);
}

void backwardRight() {
  digitalWrite(output1, LOW);
  digitalWrite(output2, LOW);
  digitalWrite(output3, LOW);
  digitalWrite(output4, HIGH);
}

void left() {
  digitalWrite(output1, LOW);
  digitalWrite(output2, LOW);
  digitalWrite(output3, HIGH);
  digitalWrite(output4, LOW);
}

void leftSharp() {
  digitalWrite(output1, LOW);
  digitalWrite(output2, HIGH);
  digitalWrite(output3, HIGH);
  digitalWrite(output4, LOW);
}

void right() {
  digitalWrite(output1, HIGH);
  digitalWrite(output2, LOW);
  digitalWrite(output3, LOW);
  digitalWrite(output4, LOW);
}

void rightSharp() {
  digitalWrite(output1, HIGH);
  digitalWrite(output2, LOW);
  digitalWrite(output3, LOW);
  digitalWrite(output4, HIGH);
}

void stopMotors() {
  digitalWrite(output1, LOW);
  digitalWrite(output2, LOW);
  digitalWrite(output3, LOW);
  digitalWrite(output4, LOW);
}

// Masks //
#define FLAG_MASK          0b1100000000
#define DIRECTION_MASK     0b0000001111

// Special //
#define HARDWARE_INTERRUPT 0b1000000000
#define REM                0b0100000000

// Direction //
#define STATE_STIL         0b0000000000
#define RECHTS             0b0000001000
#define RECHTS_VOOR        0b0000001001
#define RECHTS_ACHTER      0b0000001010
#define LINKS              0b0000000100
#define LINKS_VOOR         0b0000000101
#define LINKS_ACHTER       0b0000000110
#define ONDER              0b0000000010
#define BOVEN              0b0000000001

// Received data //
void handleReceivedData(uint16_t data) {
  uint8_t flagBits = (data & FLAG_MASK) >> 8;
  uint8_t directionBits = data & DIRECTION_MASK;

  switch (flagBits) {
    case 0b10:
      Serial.println("Hardware interrupt");
      stopMotors();
      return;
    case 0b01:
      Serial.println("Brake");
      stopMotors();
      return;
    case 0b00:
      break;
    default:
      Serial.println("Unknown flag");
      stopMotors();
      return;
  }

  switch (directionBits) {
    case STATE_STIL:
      Serial.println("IDLE");
      stopMotors();
      break;
    case RECHTS:
      Serial.println("Right");
      rightSharp();
      break;
    case RECHTS_VOOR:
      Serial.println("Right + Forward");
      right();
      break;
    case RECHTS_ACHTER:
      Serial.println("Right + Reverse");
      backwardRight();
      break;
    case LINKS:
      Serial.println("Left");
      leftSharp();
      break;
    case LINKS_VOOR:
      Serial.println("Left + Forward");
      left();
      break;
    case LINKS_ACHTER:
      Serial.println("Left + Reverse");
      backwardLeft();
      break;
    case ONDER:
      Serial.println("Reverse");
      backward();
      break;
    case BOVEN:
      Serial.println("Forward");
      forward();
      break;
    default:
      Serial.println("Unknown direction");
      stopMotors();
      break;
  }
}

// Setup //
void setup() {
  Serial.begin(115200);

  pinMode(output1, OUTPUT);
  pinMode(output2, OUTPUT);
  pinMode(output3, OUTPUT);
  pinMode(output4, OUTPUT);

  BLEDevice::init("");
  pClient = BLEDevice::createClient();
  Serial.println("BLE Client Initialized.");

  BLEAddress bleAddress(BLE_SERVER_MAC);
  if (pClient->connect(bleAddress)) {
    Serial.println("Successfully connected to the BLE server!");

    pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
    if (pRemoteService) {
      pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
      if (pRemoteCharacteristic) {
        Serial.println("Found characteristic!");
        connected = true;
      } else {
        Serial.println("Characteristic not found.");
      }
    } else {
      Serial.println("Service not found.");
    }
  } else {
    Serial.println("Failed to connect to the BLE server.");
  }

  stopMotors();
}

// Main loop //
void loop() {
  if (connected) {
    if (!pClient->isConnected()) {
      Serial.println("Disconnected from BLE server.");
      connected = false;
    } else {
      if (pRemoteCharacteristic) {
        String value = pRemoteCharacteristic->readValue();
        if (value.length() == 10) {
          Serial.print("Received data: ");
          Serial.println(value);

          lastReceivedTime = millis();

          // Zet 10-bit binaire string om naar integer
          uint16_t data = 0;
          for (int i = 0; i < 10; i++) {
            data |= (value.charAt(i) == '1' ? 1 : 0) << (9 - i);
          }

          handleReceivedData(data);
        }
      }

      if (millis() - lastReceivedTime > timeout) {
        Serial.println("Disconnected due to inactivity.");
        connected = false;
      }
    }
  } else {
    Serial.println("Waiting for connection...");
    BLEAddress bleAddress(BLE_SERVER_MAC);
    if (pClient->connect(bleAddress)) {
      Serial.println("Reconnected to BLE server!");
      connected = true;
    }
    stopMotors();
  }

  delay(100);
}
