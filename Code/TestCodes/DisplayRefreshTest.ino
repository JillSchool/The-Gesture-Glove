#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>

// Define pins for onboard display ssd1331
#define SCLK_PIN  14
#define MOSI_PIN  13
#define CS_PIN    15
#define DC_PIN    16
#define RST_PIN   4

unsigned long startTime;
unsigned long endTime;
unsigned long duration;

// Initialize display
Adafruit_SSD1331 display = Adafruit_SSD1331(CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);

void setup() {
    Serial.begin(115200);
    display.begin();
    display.setTextColor(0xFFFF);
    display.setTextSize(1);
    Serial.println("Display Initialized!");
}

void drawCornersBenchmark() {
    int size = 10;
    int color = 0xFFFF; // White
    int positions[4][2] = {{0, 0}, {display.width() - size, 0}, {0, display.height() - size}, {display.width() - size, display.height() - size}};
    
    for (int i = 0; i < 4; i++) {
        display.fillScreen(0x0000);
        startTime = millis();
        display.fillRect(positions[i][0], positions[i][1], size, size, color);
        endTime = millis();
        duration = endTime - startTime;
        Serial.print("FillRect "); Serial.print(i + 1); Serial.print(" took: "); Serial.print(duration); Serial.println(" ms");
    }
}

void drawExpandingSquaresBenchmark() {
    int maxSize = min(display.width(), display.height()) / 2;
    int step = 5;
    int color = 0xF800; // Red
    
    for (int size = step; size <= maxSize; size += step) {
        display.fillScreen(0x0000);
        startTime = millis();
        display.drawRect((display.width() - size) / 2, (display.height() - size) / 2, size, size, color);
        endTime = millis();
        duration = endTime - startTime;
        Serial.print("DrawRect size "); Serial.print(size); Serial.print(" took: "); Serial.print(duration); Serial.println(" ms");
    }
}

void drawCenterPixelComparison() {
    int x = display.width() / 2;
    int y = display.height() / 2;
    int size = 10;
    int color = 0x07E0; // Green
    
    struct { void (*func)(int, int, int, int, uint16_t); const char* name; } tests[] = {
        {[](int x, int y, int w, int h, uint16_t c) { display.fillRect(x, y, w, h, c); }, "FillRect"},
        {[](int x, int y, int w, int h, uint16_t c) { display.drawRect(x, y, w, h, c); }, "DrawRect"},
        {[](int x, int y, int w, int h, uint16_t c) { display.drawCircle(x, y, w, c); }, "DrawCircle"},
        {[](int x, int y, int w, int h, uint16_t c) { display.fillCircle(x, y, w, c); }, "FillCircle"}
    };

    for (auto& test : tests) {
        display.fillScreen(0x0000);
        startTime = millis();
        test.func(x - size / 2, y - size / 2, size, size, color);
        endTime = millis();
        duration = endTime - startTime;
        Serial.print(test.name); Serial.print(" took: "); Serial.print(duration); Serial.println(" ms");
    }
}

void loop() {
    drawCornersBenchmark();
    delay(2000);
    display.fillScreen(0x0000);
    drawExpandingSquaresBenchmark();
    delay(2000);
    display.fillScreen(0x0000);
    drawCenterPixelComparison();
    delay(5000);
    display.fillScreen(0x0000);
}
