#include <Adafruit_GFX.h>//////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1331.h>//////////////////////////////////////////////////////////////////////////////////
#include <SPI.h>/////////////////////////////////FINAL/////////////////////////////////////////////////////////
#include <MPU6500_WE.h>////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>//////////////////////////////////////////////////////////////////////////////////////////////
#include "esp_task_wdt.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Ticker.h>

///// Pin and Device Configuration /////
// Display pins
const uint8_t sclk = 14;   // Serial clock pin
const uint8_t mosi = 13;   // Master Out Slave In pin
const uint8_t cs = 15;     // Chip Select pin
const uint8_t dc = 16;     // Data/Command pin
const uint8_t rst = 4;     // Reset pin

// IMU pins
const uint8_t scl = 22;    // I2C clock pin for IMU
const uint8_t sda = 21;    // I2C data pin for IMU
const uint8_t addr = 0x68; // I2C address for MPU6500

// Input pins
const uint8_t button = 36; // Onboard button pin
const uint8_t flex = 2;    // Analog pin for flex sensor
const uint8_t battery = 35;// Analog pin for battery voltage

// Watchdog timeout setting
const int wdtTimeoutMs = 100;  // Reset if no response in 5 seconds

///// Threshold Configuration /////
// IMU tresholds
const int imuActive = 20;    // Minimum value to detect movement
const int imuDeadZone = 15;  // Value below which movement is ignored
const int rawPrintThreshold = 10; // Minimum change before printing raw values

///// PID Controller Configuration /////
float kp = 0.6;       // Proportional gain
float ki = 0.1;       // Integral gain
float kd = 0.005;     // Derivative gain
float lastErrorX = 0; // Previous X error for derivative term
float lastErrorY = 0; // Previous Y error for derivative term
float integralX = 0;  // Accumulated X error for integral term
float integralY = 0;  // Accumulated Y error for integral term
float integralLimit = 100.0; // Limit to prevent integral windup

///// Angle Tracking /////
// IMU angle variables
float rawX = 0.0;     // Raw X angle from integration
float rawY = 0.0;     // Raw Y angle from integration
float pidX = 0.0;// PID-filtered X angle
float pidY = 0.0;// PID-filtered Y angle
int lastPrintX = 0;   // Last printed X value
int lastPrintY = 0;   // Last printed Y value

///// Timing Configuration /////
// Ticker object for slow data updates
Ticker slowDataTicker;

// Flag to indicate when a slow data update should occur
volatile bool slowDataFlag = false;

unsigned long lastUpdate = 0; // Last sensor update time

// State tracking
bool hwStopClicked = false;  // Tracks if hardware stop button was pressed

///// Connection /////
#define DEVICE_NAME "GloveESP32"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

esp_bd_addr_t clientAddress;
int rssiValue = 0;

///// Data structures /////
// Structure for real-time sensor readings
struct SensorData {
  float gyroX, gyroY, gyroZ;  // IMU gyroscope readings
  uint8_t flexValue;          // Flex sensor reading (0-100%)
};

// Structure for slow-updating status data
struct SlowData {
  uint8_t gloveBat;     // Glove battery level (0-100%)
  uint8_t endBat;       // End device battery level (0-100%)
  uint8_t bleStrength;  // Bluetooth connection strength (0-100%)
  uint8_t endSpeed;     // End device speed (0-100%)
};

// Device orientation enum with bit flags for multi-direction support
enum Orient {
  CENTER   = 0,         // No movement detected
  FORWARD  = 1 << 0,    // Forward movement
  BACKWARD = 1 << 1,    // Backward movement
  LEFT     = 1 << 2,    // Left movement
  RIGHT    = 1 << 3     // Right movement
};

// Current state of the ESP controller
struct EspState {
  bool braking;         // True when braking is active
  bool stopped;         // True when device is stopped
  uint8_t orientation;  // Current orientation (can combine multiple directions)
};  

// Initialize current state
EspState curState = {false, false, CENTER}; // Can use combined values like (FORWARD | LEFT)

///// Task and queue handles /////
TaskHandle_t SensorTask;    // Handle for sensor reading task
TaskHandle_t DataTask;      // Handle for data processing task
QueueHandle_t sensorQueue;  // Queue for passing sensor data between tasks

///// Object initialization /////
// Initialize display
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);
// Initialize IMU
MPU6500_WE myMPU6500 = MPU6500_WE(addr);

///// UI Icons - 8x10 bitmap arrays /////
// Battery indicator icon
const uint8_t batteryIcon[] = {
  0b00111000,
  0b01111100,  
  0b10000010,  
  0b10111010,  
  0b10111010,
  0b10111010,
  0b10111010,
  0b10111010, 
  0b10000010,  
  0b01111100
};

// Glove indicator icon
const uint8_t gloveIcon[] = {
  0b00001000,
  0b00101010,
  0b00101010,
  0b10101010,
  0b10111101,
  0b10111101,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100
};

// Wheel indicator icon
const uint8_t wheelIcon[] = {
  0b00111100,
  0b01100110,
  0b11000011,
  0b11100111,
  0b10111101,
  0b10011001,
  0b10011001,
  0b11011011,
  0b01111110,
  0b00111100
};

// Stop indicator icon
const uint8_t stoppedIcon[] = {
  0b11000011,
  0b01100110,
  0b01111110,
  0b00111100,
  0b00011100,
  0b00011100,
  0b00111100,
  0b01111110,
  0b11100111,
  0b11000011
};

// Connection indicator icon
const uint8_t connectionIcon[] = {
  0b00000011,
  0b00000011,  
  0b00000011,  
  0b00011011,  
  0b00011011,
  0b00011011,
  0b11011011,
  0b11011011, 
  0b11011011,  
  0b11011011
};

// Speed indicator icon
const uint8_t speedIcon[] = {
  0b00000000,
  0b00000000,  
  0b00011000,  
  0b00100100,  
  0b01000110,
  0b01001110,
  0b01011010,
  0b1001001, 
  0b10000001,
  0b10000001
};

///// Function declarations /////
void sensebuttonPressed();
void handlePossibleSleep();
void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void sendBleData(BLECharacteristic* ptr_characteristic, const char* message);
SlowData readSlowData();
float pidController(float target, float current, float &lastError, float &integral);
SensorData readSensors();
void sensorTaskLoop(void * pvParameters);
void displayBLEConnect();
void displayLayout();
void displayFastData(SensorData data);
void displaySlowData(SlowData sData);
void sendData(SensorData data);
void dataTaskLoop(void * pvParameters);

///// Power Management Functions and Interrupts /////

/**
 * Interrupt service routine for onboard button press
 * Toggles the stopped state and sets the hwStopClicked flag
 */
void sensebuttonPressed() {
  hwStopClicked = true;
  curState.stopped = !curState.stopped;
}

/**
 * Handles sleep logic after device has been stopped
 * Waits for 5 seconds before going to sleep if no other input is received
 */
void handlePossibleSleep() {
  sendBleData();
  const int waitMs = 5000;    // Wait 5 seconds before sleep
  const int interval = 100;   // Check every 100ms
  int waited = 0;

  // Update UI to show sleep pending
  display.fillRect(66, 3, 11, 7, 0x0000);
  display.setCursor(66, 3);
  display.print("yes");
  display.fillCircle(73, 42, 19, 0x0000);
  display.fillCircle(73, 42, 10, 0xF800);
  curState.braking = true;
  hwStopClicked = false;

  // Wait loop with early exit if button pressed again
  while (waited < waitMs) {
    if (hwStopClicked == true) {
      hwStopClicked = false;  // reset flag
      display.fillRect(66, 3, 17, 8, 0x0000);
      display.setCursor(66, 3);
      display.print("no");
      return;  // button pressed again - don't sleep
    }
    esp_task_wdt_reset();  // Reset watchdog timer
    vTaskDelay(interval / portTICK_PERIOD_MS);
    waited += interval;
  }

  // If still stopped after timeout, go to deep sleep
  if (curState.stopped == true) {
    display.fillScreen(0x0000); // Clear before sleep
    display.setCursor(40, 30);
    display.print("Going to sleep...");
    delay(500);
    esp_task_wdt_deinit();  // Disable watchdog before sleep
    // Configure wakeup on button press
    esp_sleep_enable_ext0_wakeup((gpio_num_t)button, 0);
    esp_deep_sleep_start();  // Enter deep sleep mode
  }
  esp_task_wdt_reset();
}

/**
 * Software interupt handler
 */
void triggerSlowDataUpdate() {
  slowDataFlag = true;
}

// BLE Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
    deviceConnected = true;
    memcpy(clientAddress, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

/**
 * Measures BLE strength
 */
void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT) {
    rssiValue = param->read_rssi_cmpl.rssi;
    int strength = map(rssiValue, -100, -40, 0, 100);
    strength = constrain(strength, 0, 100);
    Serial.printf("RSSI: %d dBm (%d%%)\n", rssiValue, strength);
    Serial.print("Signal: [");
    for (int i = 0; i < 10; i++) {
      Serial.print(i < strength / 10 ? "=" : " ");
    }
    Serial.println("]");
  }
}

///// Bluetooth Functions /////

/**
 * Sends the sensor data over BLE to connected devices
 */
void sendBleData() {
  if (deviceConnected) {
    static char buffer[11]; // 8 chars for orientation + 2 chars for states + null
    
    // First two chars for braking and stopped
    buffer[0] = curState.stopped ? '1' : '0';
    buffer[1] = curState.braking ? '1' : '0';
    Serial.println(curState.stopped);
    Serial.println(buffer[0]);
    
    // Next 8 chars for orientation as a binary representation
    for (int i = 0; i < 8; i++) {
      buffer[2+i] = (curState.orientation & (1 << (7-i))) ? '1' : '0';
    }
    buffer[10] = '\0'; // Null terminator 

    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();
  }
}

///// Sensor Functions /////

/**
 * Measures all less critical data like battery and connections status
 * Currently only the BLE strength status works
 */
SlowData readSlowData() {
  SlowData sData;
  
  // Calculate battery percentage from voltage reading
  float minVolt = 3.2;  // Minimum battery voltage
  float maxVolt = 4.2;  // Maximum battery voltage
  float voltage = analogRead(battery);
  int percent = ((voltage - minVolt) / (maxVolt - minVolt)) * 100;
  sData.gloveBat = constrain(percent, 0, 100); // Ensure value is between 0-100%
  
  // Update BLE strength using RSSI
  esp_ble_gap_read_rssi(clientAddress); // Request RSSI
  int strength = map(rssiValue, -100, -40, 0, 100); // Clearer indicator for most people
  sData.bleStrength = constrain(strength, 0, 100); // BLE signal strength
  
  // Placeholder values
  sData.endBat = 98;    // End device battery // Not integrated yet
  sData.endSpeed = 1;    // End device speed   // Not integrated yet
  
  return sData;
}

/**
 * PID controller function for smoothing sensor data
 * 
 * @param target The target value (usually 0 for IMU drift compensation)
 * @param current The current sensor reading
 * @param lastError Reference to store last error value
 * @param integral Reference to store integral term
 * @return The PID-filtered output value
 */
float pidController(float target, float current, float &lastError, float &integral) {
  float error = target - current;  // Calculate error

  // Reset integral if error is small to prevent oscillation
  if (fabs(error) < 10) {
    integral = 0;
  } else {
    integral += error;
  }

  // Constrain the integral to avoid wind-up
  integral = constrain(integral, -integralLimit, integralLimit);

  float derivative = error - lastError;  // Calculate derivative
  lastError = error;  // Update last error

  // Calculate the PID output and invert the sign
  float pidOutput = kp * error + ki * integral + kd * derivative;

  // Flip the sign of the PID output
  return -pidOutput;
}

/**
 * Reads all sensor data and applies filtering
 * @return SensorData structure with processed readings
 */
SensorData readSensors() {
  SensorData data;
  
  // Read flex sensor and convert to percentage
  data.flexValue = (analogRead(flex) / 3200.0) * 100;
  Serial.print("flex reading: ");
  Serial.println(data.flexValue);
  
  // Read raw gyroscope values
  xyzFloat gyrValue = myMPU6500.getGyrValues();

  // Apply PID filtering to the gyroscope values
  float pidGyrX = pidController(0.0, gyrValue.x, lastErrorX, integralX);
  float pidGyrY = pidController(0.0, gyrValue.y, lastErrorY, integralY);

  // Calculate time delta for angle integration
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdate) / 1000.0;  // Time difference in seconds
  lastUpdate = currentTime;  // Update last update time

  // Integrate raw angles using gyroscope values
  rawX += gyrValue.x * dt;
  rawY += gyrValue.y * dt;

  // Integrate PID-filtered angles
  pidX += pidGyrX * dt;
  pidY += pidGyrY * dt;

  // Constrain angles to ±90 degrees to prevent unrealistic values
  rawX = constrain(rawX, -90.0, 90.0);
  rawY = constrain(rawY, -90.0, 90.0);
  pidX = constrain(pidX, -90.0, 90.0);
  pidY = constrain(pidY, -90.0, 90.0);

  // Smoothly reduce PID values when raw angles approach zero
  float smoothingFactor = 0.1;  // How quickly PID values decay to zero

  if (fabs(rawX) < 1.0) {  // If X is close to zero
    pidX *= (1 - smoothingFactor);  // Gradually decay PID X value
  }

  if (fabs(rawY) < 1.0) {  // If Y is close to zero
    pidY *= (1 - smoothingFactor);  // Gradually decay PID Y value
  }

  // Round values to integers for display and debugging
  int rawIntX = round(rawX);
  int rawIntY = round(rawY);
  int pidIntX = round(pidX);
  int pidIntY = round(pidY);

  // Check if the raw data has changed enough to print (reduces console spam)
  bool rawChanged = abs(rawIntX - lastPrintX) >= rawPrintThreshold ||
                    abs(rawIntY - lastPrintY) >= rawPrintThreshold;

  if (rawChanged) {
    // Snap to nearest multiple of 10 for raw data
    rawIntX = (rawIntX / 10) * 10;
    rawIntY = (rawIntY / 10) * 10;

    // Update last printed values
    lastPrintX = rawIntX;
    lastPrintY = rawIntY;
  }

  // Always print PID data for debugging
  Serial.print("PIDX: "); 
  Serial.print(pidIntX); 
  Serial.print("\t");
  Serial.print("PIDY: "); 
  Serial.println(pidIntY);
  
  // Store filtered values in the result structure
  data.gyroX = pidIntX;
  data.gyroY = pidIntY;
  
  return data;
}

/**
 * FreeRTOS task for continuous sensor polling
 * Runs on core 0 and sends data to the queue for processing
 */
void sensorTaskLoop(void * pvParameters) {
  esp_task_wdt_add(NULL);  // Register task to watchdog
  
  while(true) {
    SensorData data = readSensors();  // Read sensor data
    
    // Send data to queue with 50ms timeout
    BaseType_t status = xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(50));
    
    if (status != pdPASS) {
      // Display there is a delay or fault between the sensors and display
      Serial.println("Data was measured but not sent to the display");
    }
    
    esp_task_wdt_reset();  // Reset watchdog timer
  }
}

///// Display and UI Functions /////

/**
 * Draws the initial display layout with all UI elements
 * Should be called everytime before starting to display sensor data
 */
void displayLayout() {
  display.fillScreen(0x0000); // Black background
  
  // Layout divider lines
  display.drawFastVLine(50, 0, 64, 0xFFFF);  // Vertical divider
  display.drawFastHLine(0, 32, 50, 0xFFFF);  // Left section horizontal divider
  display.drawFastHLine(50, 20, 46, 0xFFFF); // Right section top divider
  
  // Gyroscope visualization elements
  display.drawCircle(73, 42, 20, 0xFFFF);   // Outer circle
  display.fillCircle(73, 42, 2, 0x07E0);    // Inner circle (green indicator)
  
  // Status icons and labels for left panel
  display.drawBitmap(1, 3, batteryIcon, 8, 10, 0xFFFF);    // Glove battery icon
  display.drawBitmap(10, 3, gloveIcon, 8, 10, 0xFFFF);     // Glove icon
  display.setCursor(19, 5);
  display.print(":");                                      // Separator
  
  display.drawBitmap(1, 35, batteryIcon, 8, 10, 0xFFFF);   // End device battery icon
  display.drawBitmap(10, 35, wheelIcon, 8, 10, 0xFFFF);    // Wheel icon
  display.setCursor(19, 37);
  display.print(":");                                      // Separator
  
  display.drawBitmap(1, 16, connectionIcon, 8, 10, 0xFFFF); // Connection icon
  display.setCursor(10, 18);
  display.print(":");                                       // Separator
  
  display.drawBitmap(1, 48, speedIcon, 8, 10, 0xFFFF);      // Speed icon
  display.setCursor(10, 50);
  display.print(":");                                       // Separator
  
  // Status icons and labels for right panel
  display.drawBitmap(52, 3, stoppedIcon, 8, 10, 0xFFFF);    // Stop icon
  display.setCursor(61, 5);
  display.print(":");                                       // Separator
}

/**
 * Shows when the esp is advertising its bluetooth
 */
void displayBLEConnect(){
  display.fillScreen(0x0000); // Clear before sleep
  display.setCursor(1, 25);
  display.print("Searching for  bluetooth...");
}

/**
 * Updates the display with rapidly changing sensor data
 * Shows brake status and direction indicators
 * 
 * @param data Current sensor readings to display
 */
void displayFastData(SensorData data) {
  // Check for coming out of stop state
  if (hwStopClicked == true) {
    display.fillRect(66, 3, 17, 8, 0x0000);  // Clear previous text
    display.setCursor(66, 3);
    display.print("no");                     // Update stop indicator
    hwStopClicked = false;
  }
  
  // Display flex sensor activation (brake control)
  if (data.flexValue <= 30 && curState.braking == true) {
    // Release brake
    display.fillCircle(73, 42, 10, 0x0000);        // Clear brake indicator
    display.fillCircle(73, 42, 5, 0x07E0);         // Show green "go" indicator
    curState.braking = false;
  } else if (data.flexValue > 30 && curState.braking == false) {
    // Apply brake
    display.fillCircle(73, 42, 10, 0xF800);        // Show red "brake" indicator
    curState.braking = true;
  }
  
  // Handle forward/backward direction indicators
  if (data.gyroY < -imuActive && !(curState.orientation & FORWARD)) {
    // Draw forward and delete backward indicators
    display.fillCircle(73, 57, 4, 0x0000);        // Clear bottom indicator
    display.fillCircle(73, 27, 4, 0xFFFF);        // Show top indicator
    curState.orientation |= FORWARD;              // Add FORWARD flag
    curState.orientation &= ~BACKWARD;            // Remove BACKWARD flag if active
    curState.orientation &= ~CENTER;              // Remove CENTER flag
  }
  else if (data.gyroY > imuActive && !(curState.orientation & BACKWARD)) {
    // Draw backward and delete forward indicators
    display.fillCircle(73, 27, 4, 0x0000);        // Clear top indicator
    display.fillCircle(73, 57, 4, 0xFFFF);        // Show bottom indicator
    curState.orientation |= BACKWARD;             // Add BACKWARD flag
    curState.orientation &= ~FORWARD;             // Remove FORWARD flag if active
    curState.orientation &= ~CENTER;              // Remove CENTER flag
  }
  else if (abs(data.gyroY) < imuDeadZone) {
    // Clear both forward/backward indicators when in deadzone
    display.fillCircle(73, 27, 4, 0x0000);        // Clear top indicator
    display.fillCircle(73, 57, 4, 0x0000);        // Clear bottom indicator
    curState.orientation &= ~FORWARD;             // Remove FORWARD flag
    curState.orientation &= ~BACKWARD;            // Remove BACKWARD flag
  }
  
  // Handle left/right direction indicators
  if (data.gyroX > imuActive && !(curState.orientation & LEFT)) {
    // Draw left and delete right indicators
    display.fillCircle(88, 42, 4, 0x0000);        // Clear right indicator
    display.fillCircle(58, 42, 4, 0xFFFF);        // Show left indicator
    curState.orientation |= LEFT;                 // Add LEFT flag
    curState.orientation &= ~RIGHT;               // Remove RIGHT flag if active
    curState.orientation &= ~CENTER;              // Remove CENTER flag
  }
  else if (data.gyroX < -imuActive && !(curState.orientation & RIGHT)) {
    // Draw right and delete left indicators
    display.fillCircle(58, 42, 4, 0x0000);        // Clear left indicator
    display.fillCircle(88, 42, 4, 0xFFFF);        // Show right indicator
    curState.orientation |= RIGHT;                // Add RIGHT flag
    curState.orientation &= ~LEFT;                // Remove LEFT flag if active
    curState.orientation &= ~CENTER;              // Remove CENTER flag
  }
  else if (abs(data.gyroX) < imuDeadZone) {
    // Clear both left/right indicators when in deadzone
    display.fillCircle(58, 42, 4, 0x0000);        // Clear left indicator
    display.fillCircle(88, 42, 4, 0x0000);        // Clear right indicator
    curState.orientation &= ~LEFT;                // Remove LEFT flag
    curState.orientation &= ~RIGHT;               // Remove RIGHT flag
  }
  
  // Handle CENTER case (no active directions)
  if (curState.orientation == 0) {
    // All directions are already cleared in the deadzone checks above
    curState.orientation = CENTER;                // Set CENTER flag
  }
  else if (curState.orientation != CENTER) {
    // Make sure CENTER is not mixed with active directions
    curState.orientation &= ~CENTER;              // Remove CENTER flag
  }
}

/**
 * Updates the display with slowly changing system status data
 * Shows battery levels, connection status, and speed
 * 
 * @param sData Current systems statusses data to display
 */
void displaySlowData(SlowData sData) {
  // Clear previous status values (each text field is 5x7 pixels + 1px spacing)
  display.fillRect(24, 5, 17, 7, 0x0000);    // Glove battery field
  display.fillRect(24, 37, 26, 7, 0x0000);   // End device battery field
  display.fillRect(15, 18, 17, 7, 0x0000);   // Connection strength field
  display.fillRect(15, 50, 11, 7, 0x0000);   // End device speed field
  
  // Update status values
  display.setCursor(24, 5);
  display.print(sData.gloveBat);             // Show glove battery percentage
  
  display.setCursor(24, 37);
  display.print(sData.endBat);               // Show end device battery percentage
  
  display.setCursor(15, 18);
  display.print(sData.bleStrength);          // Show Bluetooth connection strength
  
  display.setCursor(15, 50);
  display.print(sData.endSpeed);             // Show end device speed value
}

/**
 * FreeRTOS task for data processing and display
 * Runs on core 1 and handles UI updates and data transmission
 */
void dataTaskLoop(void * pvParameters) {
  displayLayout();  // Initialize the UI
  esp_task_wdt_add(NULL);  // Register task to watchdog
  
  while (true) {
    SensorData data;
    // Wait for new sensor data from the queue
    if (xQueueReceive(sensorQueue, &data, portMAX_DELAY)) {
      if (curState.stopped == false) {
        // Normal operation mode
        displayFastData(data);  // Update display
        if (!deviceConnected && oldDeviceConnected) {
          curState.stopped = true;
          Serial.println("Client disconnected, restarting advertising...");
          pServer->startAdvertising();
          Serial.println("Waiting for a client to connect...");
          displayBLEConnect();
          while (!deviceConnected) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
          }
          curState.stopped = false;
          oldDeviceConnected = false;
          Serial.println("Client connected!");
          displayLayout();
        } else if (deviceConnected && !oldDeviceConnected) {
          oldDeviceConnected = true;
        }
      } else {
        // Stopped mode - check for sleep condition
        if (hwStopClicked == true && curState.stopped == true) {
          handlePossibleSleep();
        }
      }
      if (deviceConnected) {
        sendBleData();
      }
    }
    // Check if the slow data update flag has been set by the timer interrupt
    if (slowDataFlag) {
      slowDataFlag = false;  // Reset the flag
      
      // Process slow data update
      SlowData sData = readSlowData();
      displaySlowData(sData);
    }
    esp_task_wdt_reset();  // Reset watchdog timer
  }
}

void setup() {
  Serial.begin(115200);  // Initialize serial communication

  // Configure input pins
  pinMode(flex, INPUT);
  pinMode(battery, INPUT);

  // Configure button with pullup (button press pulls pin LOW)
  pinMode(button, INPUT_PULLUP);
  // Attach interrupt for button press detection
  attachInterrupt(digitalPinToInterrupt(button), sensebuttonPressed, FALLING);

  // Configure software interrupt
  // Setup ticker to trigger slow data updates every 2000ms
  slowDataTicker.attach_ms(2000, triggerSlowDataUpdate);

  // Initialize display
  display.begin();
  display.setTextColor(0xFFFF);  // White text
  display.setTextSize(1);        // Normal text size
  Serial.println("Display Initialized!");

  // Initialize IMU
  Wire.begin();
  if (!myMPU6500.init()) {
    Serial.println("MPU6500 does not respond");
  } else {
    Serial.println("MPU6500 is connected");
  }
  
  // Calibrate IMU sensor
  Serial.println("Calibrating... Keep MPU6500 still");
  delay(1000);
  myMPU6500.autoOffsets();  // Automatically calibrate sensor
  delay(200);               // Wait for calibration to complete
  Serial.println("Calibration complete.");

  // Initialize time tracking for sensor integration
  lastUpdate = millis();

  // Configure watchdog timer
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = wdtTimeoutMs,              // Convert seconds to milliseconds
    .idle_core_mask = (1 << 0) | (1 << 1),         // Apply watchdog to both cores
    .trigger_panic = true,                         // Reset ESP if watchdog triggers
  };

  // Initialize watchdog
  esp_task_wdt_init(&wdt_config);

  // Create queue with space for 2 SensorData structs
  sensorQueue = xQueueCreate(2, sizeof(SensorData));

  if (sensorQueue == NULL) {
    Serial.println("Queue creation failed!");
    while (1);  // Halt if queue creation fails
  }

  Serial.println("Starting BLE work!");

  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Hello World from GloveESP32!");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  esp_ble_gap_register_callback(gapCallback);

  displayBLEConnect();
  while (!deviceConnected) { delay(500); }
  Serial.println("Client connected!");

  // Create FreeRTOS tasks on separate cores
  xTaskCreatePinnedToCore(
    sensorTaskLoop,  // Task function
    "Sensor task",   // Task name
    10000,           // Stack size (bytes)
    NULL,            // Parameters
    1,               // Priority
    &SensorTask,     // Task handle
    0                // Core (0)
  );
  delay(500);  // Allow task to start
  
  xTaskCreatePinnedToCore(
    dataTaskLoop,    // Task function
    "Data task",     // Task name
    10000,           // Stack size (bytes)
    NULL,            // Parameters
    1,               // Priority
    &DataTask,       // Task handle
    1                // Core (1)
  );
  delay(500);  // Allow task to start
}

void loop() {
  // Nothing to do here - all functionality is in FreeRTOS tasks
}