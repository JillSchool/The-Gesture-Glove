#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer*         ble_server;
BLEService*        ble_service;
BLECharacteristic* ble_characteristic;
BLEAdvertising*    ble_advertising;

/**
 * @brief 
 * Sensor IMU: (Left/Right & Front/Rear) rng: -90, +90
 * Sensor Flex: (ADC pin 12bit) rng: 0, 4095
 * 
 */

constexpr const char* const WEBOTS_COMMANDS[] = {
    "0000000000",  // Initial State
    "0100000000",  // Stop / Park

    "0000001000",  // Right turn
    "0000000100",  // Left turn

    "0000000010",  // Backward movement
    "0000000001",  // Forward movement

    "0000000101",  // Left turn + Forward movement
    "0000001001",  // Right turn + Forward movement

    "1000000000",  // HW Interrupt: Stop / Park

    "0000000110",  // Left turn + Backward movement
    "0000001010"   // Right turn + Backward movement
};

void init_ble()
{
    Serial.println("Starting BLE initialization!");

    BLEDevice::init("GloveESP32");

    ble_server         = BLEDevice::createServer();
    ble_service        = ble_server->createService(SERVICE_UUID);
    ble_characteristic = ble_service->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    ble_characteristic->setValue("Hello World from GloveESP32!");
    ble_service->start();

    ble_advertising = BLEDevice::getAdvertising();

    ble_advertising->addServiceUUID(SERVICE_UUID);
    ble_advertising->setScanResponse(true);
}

void start_ble_server()
{
    ble_server->startAdvertising();

    Serial.println("Waiting for a client to connect...");
    while (ble_server->getConnectedCount() == 0) { delay(1000); }

    Serial.println("Client connected!");
}

void send_ble_data(const char* message)
{
    ble_characteristic->setValue(message);
    ble_characteristic->notify();
}

void setup()
{
    Serial.begin(115200);

    init_ble();
    start_ble_server();
}

void loop()
{
    for (const char* cmd : WEBOTS_COMMANDS)
    {
        if (ble_server->getConnectedCount() == 0)
        {
            Serial.println("Client disconnected, restarting BLE server...");
            ble_server->disconnect(ble_server->getConnId());
            // delay(2000);

            start_ble_server();
            break;
        }
        else
        {
            send_ble_data(cmd);
            Serial.println(cmd);
            delay(3000);
        }
    }

    delay(0);
}