# ESP32-C3 Human Tracker & Disarm System (Work in Progress)

> **⚠️ Disclaimer:**
> This project is a work in progress. The code is experimental and has not been fully tested. Functionality is not guaranteed, and hardware or software issues may occur. Use at your own risk!

---

## Quick Start: What You Need To Do

1. **Gather Hardware:**
   - 2x ESP32-C3 boards
   - 1x SSD1306 OLED display (I2C, 128x64, address 0x3C)
   - 1x Button (for receiver, connect to GPIO 1)
   - 1x Reed sensor (magnetic switch, for sender, connect to RESET pin)
   - Jumper wires, breadboard, USB cables

2. **Install Software:**
   - [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support
   - Required libraries: `esp_now`, `WiFi`, `Wire`, `Adafruit_GFX`, `Adafruit_SSD1306`, `Arduino`

3. **Wire Components:**
   - **Receiver (espnow_oled_receiver.cpp):**
     - OLED SDA → GPIO 8
     - OLED SCL → GPIO 9
     - Button → GPIO 1 to GND
   - **Sender (nfc_disarm_sender.cpp):**
     - Reed sensor: one side to RESET, other to GND

4. **Configure MAC Address:**
   - In `nfc_disarm_sender.cpp`, set `turretAddress` to the MAC address of your receiver ESP32-C3 (see Serial output of receiver for its MAC).

5. **Upload Code:**
   - Flash `espnow_oled_receiver.cpp` to the receiver ESP32-C3
   - Flash `nfc_disarm_sender.cpp` to the sender ESP32-C3
   - (Optional) Flash `main.cpp` to any ESP32-C3 for LED blink test

6. **Power Up:**
   - Connect both ESP32-C3 boards to USB power
   - Open Serial Monitor at 115200 baud for debug info

7. **Test:**
   - On power-up, sender will transmit a DISARM message
   - Receiver should display the message on the OLED
   - Press the button on the receiver to show "DISARMED"
   - Trigger the reed sensor (magnet) to reset the sender and send DISARM again

---

## About This Project

This repository contains three example scripts for ESP32-C3 microcontrollers, designed to demonstrate a basic wireless human tracking and disarm system using ESP-NOW and an OLED display. The system is not yet fully validated and is intended for experimentation and learning.

### 1. `espnow_oled_receiver.cpp`

**Purpose:**
- Listens for ESP-NOW messages and displays them on an SSD1306 OLED.
- Shows a crosshair by default. When the button is pressed, displays "DISARMED" in large text.

**How it works:**
- Initializes I2C for the OLED (SDA=8, SCL=9).
- Sets up ESP-NOW in WiFi station mode.
- Registers a callback to handle incoming messages, which are displayed on the OLED with word wrapping.
- Button on GPIO 1 (active LOW) triggers the "DISARMED" display.
- Default display is a crosshair overlay.

**Hardware:**
- ESP32-C3
- SSD1306 OLED (I2C, 128x64, address 0x3C)
- Button (GPIO 1 to GND)

**Libraries:**
- `esp_now`, `WiFi`, `Wire`, `Adafruit_GFX`, `Adafruit_SSD1306`

### 2. `nfc_disarm_sender.cpp`

**Purpose:**
- Sends a "DISARM" message via ESP-NOW to a specified peer (the receiver) immediately on startup.

**How it works:**
- On boot, initializes WiFi and ESP-NOW.
- Adds the receiver as a peer using its MAC address.
- Immediately sends a struct containing the string "DISARM".
- **Instead of using NFC, a reed sensor is connected to the RESET pin.** When the reed sensor is triggered (magnet present), the ESP32-C3 resets and sends the DISARM message again.

**Hardware:**
- ESP32-C3
- Reed sensor (magnetic switch)

**Wiring:**
- One side of the reed sensor to RESET pin
- Other side to GND

**Libraries:**
- `esp_now`, `WiFi`, `Arduino`

### 3. `main.cpp`

**Purpose:**
- Simple test script to blink the onboard LED (GPIO 2) every 500ms.

**How it works:**
- Sets GPIO 2 as output in `setup()`.
- In `loop()`, toggles the LED on and off with a 500ms delay.

**Hardware:**
- ESP32-C3

---

## Additional Notes

- **Work in Progress:**
  - The code is not guaranteed to work in all environments or with all ESP32-C3 boards.
  - Pin numbers, I2C addresses, and MAC addresses may need adjustment for your hardware.
  - Error handling is minimal; you may need to debug hardware and software issues.
- **ESP-NOW:**
  - Both sender and receiver must be on the same WiFi channel.
  - MAC addresses must be set correctly for communication.
- **OLED Display:**
  - If the display does not work, check wiring and I2C address (default is 0x3C).
- **Reed Sensor:**
  - Used as a simple hardware trigger for the sender by resetting the ESP32-C3.
- **Testing:**
  - Use Serial Monitor for debugging and to find the receiver's MAC address.

---

## Troubleshooting

- **No message on OLED:**
  - Check ESP-NOW peer MAC addresses
  - Verify both boards are powered and running correct scripts
  - Check OLED wiring and address
- **Button not working:**
  - Ensure button is wired to GPIO 1 and GND
  - Use internal pull-up (already set in code)
- **Reed sensor not triggering:**
  - Confirm wiring to RESET and GND
  - Use a strong magnet
- **General:**
  - Try basic sketches (like `main.cpp`) to verify board operation
  - Check Serial Monitor for error messages

---

## Contributing

Pull requests and suggestions are welcome! This project is experimental and any improvements or bug reports are appreciated. 
