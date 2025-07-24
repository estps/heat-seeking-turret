#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <WiFi.h>

// MLX90640 thermal camera
Adafruit_MLX90640 mlx = Adafruit_MLX90640();

// Servo objects
Servo panServo;
Servo tiltServo;
Servo triggerServo;

// Pin definitions
#define PAN_SERVO_PIN 18
#define TILT_SERVO_PIN 19
#define I2C_SDA 21
#define I2C_SCL 22
#define TRIGGER_SERVO_PIN 23
#define LED_PIN 2
#define POWER_PIN 1

// Servo limits
#define PAN_MIN -90
#define PAN_MAX 90
#define TILT_MIN 45
#define TILT_MAX 135

// Default position when no target is found
#define DEFAULT_PAN 0
#define DEFAULT_TILT 110

// Current servo positions
int currentPan = DEFAULT_PAN;
int currentTilt = DEFAULT_TILT;

// Tracking parameters
#define TEMP_THRESHOLD 30.0  // Temperature threshold for human detection (°C)
#define MIN_HUMAN_SIZE 4      // Minimum cluster size to consider as human
#define TRACKING_SPEED 2      // Servo movement speed

// Thermal frame buffer
float frame[32*24]; // MLX90640 is 32x24 pixels

// Human tracking variables
int targetX = -1;
int targetY = -1;
bool humanDetected = false;

unsigned long disarmUntil = 0;
bool disarmed = false;

bool hasMLX = false;

typedef struct {
  char msg[8];
} message_t;

// ESP-NOW callback function
void onDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
  message_t msg;
  memcpy(&msg, incomingData, sizeof(msg));
  if (strcmp(msg.msg, "DISARM") == 0) {
    disarmUntil = millis() + 60000;
    disarmed = true;
    Serial.println("DISARM command received via ESP-NOW!");
  }
}

// ESPNOW peer MAC address (replace with your receiver ESP32-C3 MAC)
uint8_t receiverAddress[] = {0x98, 0x3D, 0xAE, 0xAA, 0x8D, 0xAC}; // TODO: Replace with actual MAC

// ESPNOW peer setup
void setupESPNOWPeer() {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for serial monitor to connect
  Serial.println("MLX90640 Human Tracker");
  pinMode(LED_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  
  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }
  esp_now_register_recv_cb(onDataRecv);
  setupESPNOWPeer(); // Add peer for sending
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  triggerServo.setPeriodHertz(50);
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  triggerServo.attach(TRIGGER_SERVO_PIN);
  triggerServo.write(0); // Initial position (safe)
  
  // Center servos to default position
  panServo.write(currentPan + 90); // Map pan range to servo range
  tiltServo.write(currentTilt);
  delay(1000);
  
  // Initialize MLX90640
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found! Running in demo mode.");
    hasMLX = false;
  } else {
    hasMLX = true;
    Serial.println("Found MLX90640");
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setMode(MLX90640_CHESS);
    mlx.setRefreshRate(MLX90640_8_HZ);
  }
}

void loop() {
  // Check if disarmed period has expired
  if (disarmed && millis() > disarmUntil) {
    disarmed = false;
  }

  // If disarmed, immediately set pan to 90 and tilt to 45 (fastest, no scan)
  if (disarmed) {
    currentPan = 90;
    currentTilt = 45;
    panServo.write(currentPan + 90); // Map -90~90 to 0~180
    tiltServo.write(currentTilt);
    triggerServo.write(0); // Always safe when disarmed
    // No delay for fastest response
  } else if (!hasMLX) {
    // If MLX sensor is not available, go to default position
    goToDefaultPosition();
    delay(50);
  } else if (mlx.getFrame(frame) != 0) {
    // Read thermal frame
    Serial.println("Failed to read frame");
  } else {
    // Detect humans in the frame
    detectHumans();
    // Track detected humans or go to default position
    if (humanDetected) {
      trackHuman();
      activateTrigger();
    } else {
      resetTrigger();
      goToDefaultPosition();
    }
    delay(100); // ~10 FPS, adjust based on refresh rate
  }
  printStatus();
}

void detectHumans() {
  humanDetected = false;
  targetX = -1;
  targetY = -1;
  
  // Find the hottest point (likely human)
  float maxTemp = 0;
  int maxX = -1, maxY = -1;
  
  for (int y = 0; y < 24; y++) {
    for (int x = 0; x < 32; x++) {
      float temp = frame[y * 32 + x];
      
      if (temp > maxTemp && temp > TEMP_THRESHOLD) {
        maxTemp = temp;
        maxX = x;
        maxY = y;
      }
    }
  }
  
  // Check if we found a human-sized heat source
  if (maxX != -1 && maxY != -1) {
    // Check surrounding area for human-sized cluster
    int clusterSize = countClusterSize(maxX, maxY);
    
    if (clusterSize >= MIN_HUMAN_SIZE) {
      humanDetected = true;
      targetX = maxX;
      targetY = maxY;
    }
  }
}

int countClusterSize(int centerX, int centerY) {
  int size = 0;
  int radius = 3; // Check 3x3 area around center
  
  for (int y = max(0, centerY - radius); y <= min(23, centerY + radius); y++) {
    for (int x = max(0, centerX - radius); x <= min(31, centerX + radius); x++) {
      if (frame[y * 32 + x] > TEMP_THRESHOLD) {
        size++;
      }
    }
  }
  
  return size;
}

void trackHuman() {
  if (targetX == -1 || targetY == -1) return;
  
  // Calculate center of frame
  int frameCenterX = 16;
  int frameCenterY = 12;
  
  // Calculate error (how far target is from center)
  int errorX = targetX - frameCenterX;
  int errorY = targetY - frameCenterY;
  
  // Move servos to center the target
  if (abs(errorX) > 2) { // Dead zone of 2 pixels
    if (errorX > 0) {
      currentPan = min(PAN_MAX, currentPan + TRACKING_SPEED);
    } else {
      currentPan = max(PAN_MIN, currentPan - TRACKING_SPEED);
    }
    panServo.write(currentPan + 90); // Map -90~90 to 0~180 for servo
  }
  
  if (abs(errorY) > 2) { // Dead zone of 2 pixels
    if (errorY > 0) {
      currentTilt = max(TILT_MIN, currentTilt - TRACKING_SPEED); // Reverse direction for tilt
    } else {
      currentTilt = min(TILT_MAX, currentTilt + TRACKING_SPEED); // Reverse direction for tilt
    }
    tiltServo.write(currentTilt);
  }
}

// NEW Function: Scans tilt up and down while pan stays at default
void scanTiltWhenDisarmed() {
    static int tiltDir = 1; // 1 for up, -1 for down

    // Keep pan servo at the default position
    currentPan = DEFAULT_PAN;
    panServo.write(currentPan + 90);

    // Update tilt position for scanning motion
    currentTilt += tiltDir;

    // Reverse direction when tilt limits are reached
    if (currentTilt >= TILT_MAX) {
        currentTilt = TILT_MAX;
        tiltDir = -1; // Go down
    } else if (currentTilt <= TILT_MIN) {
        currentTilt = TILT_MIN;
        tiltDir = 1; // Go up
    }

    tiltServo.write(currentTilt);
}

// Moves servos to the defined default position
void goToDefaultPosition() {
    currentPan = DEFAULT_PAN;
    currentTilt = DEFAULT_TILT;
    panServo.write(currentPan + 90); // Map 0 to 90 for servo
    tiltServo.write(currentTilt);
}

// Helper to send status string via ESP-NOW
void sendStatusESPNow(const String& status) {
  // Limit message size for ESP-NOW (max 250 bytes, but keep it short)
  char msg[128];
  status.substring(0, sizeof(msg) - 1).toCharArray(msg, sizeof(msg));
  esp_now_send(receiverAddress, (uint8_t*)msg, strlen(msg) + 1);
}

void printStatus() {
  String statusStr = "Status: ";
  if (disarmed) {
    statusStr += "DISARMED (Scanning)";
    unsigned long msLeft = (disarmUntil > millis()) ? (disarmUntil - millis()) : 0;
    statusStr += " | Disarm left: ";
    statusStr += String(msLeft / 1000);
    statusStr += "s";
  } else if (humanDetected) {
    statusStr += "HUMAN DETECTED at (";
    statusStr += String(targetX);
    statusStr += ", ";
    statusStr += String(targetY);
    statusStr += ") - Temp: ";
    statusStr += String(frame[targetY * 32 + targetX]);
    statusStr += " C";
  } else {
    statusStr += "No human detected";
  }
  statusStr += " | Pan: ";
  statusStr += String(currentPan);
  statusStr += " | Tilt: ";
  statusStr += String(currentTilt);

  Serial.println(statusStr);
  // Disarm logic is handled elsewhere, so just send after serial
  sendStatusESPNow(statusStr);
}

void activateTrigger() {
  triggerServo.write(90); // Fire position (adjust as needed)
}

void resetTrigger() {
  triggerServo.write(-90); // Safe position
}
