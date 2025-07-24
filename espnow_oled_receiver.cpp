#include <esp_now.h>  
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display settings (adjust if needed)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C // Most SSD1306 OLEDs use 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Helper: Draw word-wrapped text (no mid-word breaks)
void drawWordWrappedText(int x, int y, const char* text, int maxWidth) {
  display.setCursor(x, y);
  int16_t x1, y1;
  uint16_t w, h;
  char buf[256];
  strncpy(buf, text, sizeof(buf)-1);
  buf[sizeof(buf)-1] = 0;
  char* word = strtok(buf, " \n");
  int cursorX = x, cursorY = y;
  while (word) {
    String wordStr(word);
    display.getTextBounds(wordStr, 0, 0, &x1, &y1, &w, &h);
    if (cursorX + w > maxWidth) {
      cursorX = x;
      cursorY += h + 2;
      display.setCursor(cursorX, cursorY);
    }
    display.print(wordStr);
    cursorX += w + display.getCursorX() - cursorX; // update cursorX
    display.print(' ');
    cursorX += display.getCursorX() - cursorX;
    word = strtok(NULL, " \n");
  }
}

void showOledMessage(const char* msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawWordWrappedText(0, 0, msg, SCREEN_WIDTH);
  display.display();
}

char lastMsg[128] = {0};

void onDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
  int copyLen = min(len, (int)sizeof(lastMsg) - 1);
  memcpy(lastMsg, incomingData, copyLen);
  lastMsg[copyLen] = 0; // Null-terminate
  showOledMessage(lastMsg);
}

#define BUTTON_PIN 1

void drawCrosshair() {
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2;
  display.drawLine(centerX, 0, centerX, SCREEN_HEIGHT - 1, SSD1306_WHITE);
  display.drawLine(0, centerY, SCREEN_WIDTH - 1, centerY, SSD1306_WHITE);
}

void showDisarmed() {
  drawCrosshair();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor((SCREEN_WIDTH-8*7)/2, (SCREEN_HEIGHT/2)-8); // Centered
  display.print("DISARMED");
  display.display();
}

void setup() {
  Serial.begin(115200);

  // Set I2C pins for ESP32-C3 (adjust if needed)
  Wire.begin(8, 9); // SDA=8, SCL=9 (change if your board uses different pins)

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    showOledMessage("SSD1306 allocation failed");
    while(1);
  }
  showOledMessage("Waiting for ESP-NOW...");

  // WiFi/ESP-NOW init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    showOledMessage("ESP-NOW init failed!");
    while (1);
  }
  esp_now_register_recv_cb(onDataRecv);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  display.clearDisplay();
  drawCrosshair();
  display.display();
}

void loop() {
  static bool lastButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      showDisarmed();
    }
  }
  lastButtonState = reading;
} 