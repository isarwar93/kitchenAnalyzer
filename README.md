# KitchenAnalyzer: Ultra-Low-Power Kitchen Monitoring Device

## Overview
KitchenAnalyzer is a firmware project for an ESP32 LOLIN32-based kitchen monitoring device. It features gas detection, temperature/humidity sensing, motion-based activation with LCD display, and is optimized for battery operation with ultra-low power consumption.

## Features
- MQ-2 gas sensor monitoring (detects LPG, smoke, etc.)
- DHT22 temperature and humidity sensing
- PIR motion sensor for wake-up
- 16x2 I2C LCD display showing real-time sensor data and motion status
- Countdown timer showing remaining seconds until sleep
- Motion-based activation (device wakes only when motion is detected)
- Deep sleep for ultra-low-power operation
- Battery operated with long battery life

## Hardware Requirements
- ESP32 LOLIN32 board
- MQ-2 gas sensor module
- DHT22 temperature/humidity sensor
- PIR motion sensor
- 16x2 I2C LCD display (e.g., with PCF8574 backpack)
- Battery pack (e.g., 3.7V LiPo battery with charging circuit)

## Wiring Connections

### ESP32 LOLIN32 Pinout
- **3.3V**: Power for sensors and LCD
- **GND**: Ground for all components
- **GPIO 32**: MQ-2 analog output (A0)
- **GPIO 13**: DHT22 data pin
- **GPIO 34**: PIR motion sensor output
- **GPIO 17**: I2C SDA for LCD
- **GPIO 19**: I2C SCL for LCD

### Detailed Connections
1. **MQ-2 Gas Sensor**:
   - VCC → ESP32 3.3V
   - GND → ESP32 GND
   - A0 → ESP32 GPIO 32

2. **DHT22 Sensor**:
   - VCC → ESP32 3.3V
   - GND → ESP32 GND
   - DATA → ESP32 GPIO 13

3. **PIR Motion Sensor**:
   - VCC → ESP32 5V (Direct on the board soldered)
   - GND → ESP32 GND
   - OUT → ESP32 GPIO 34

4. **16x2 I2C LCD**:
   - VCC → ESP32 3.3V
   - GND → ESP32 GND
   - SDA → ESP32 GPIO 17
   - SCL → ESP32 GPIO 19

5. **Battery**:
   - Connect battery positive to ESP32 VIN (or through a regulator if needed)
   - Battery negative to ESP32 GND

## Pin Configuration
Pin macros are defined in `include/config.h`:
- `PIN_GAS_SENSOR`: GPIO32 (MQ-2 A0)
- `PIN_TEMP_HUM_SENSOR`: GPIO13 (DHT22 DATA)
- `PIN_MOTION_SENSOR`: GPIO34 (PIR OUT)
- LCD I2C: SDA GPIO17, SCL GPIO19

## Power Optimization
- Device enters deep sleep after 20 seconds of no motion
- Wakes up on motion detection
- LCD backlight turns off during sleep
- Optimized for battery life (estimated months on a single charge)

## LCD Display Format
- Line 1: Gas:XXXX T:XX.X (Gas, Temperature)
- Line 2: H:XX.X M:X XX (Humidity, Motion status 0/1, Remaining seconds 00-20)

## Project Structure
```
kitchenAnalyzer/
├── include/config.h      # Pin macros and configuration
├── src/main.cpp         # Main firmware logic
├── platformio.ini       # PlatformIO project config
├── lib/                 # Custom libraries (if needed)
```

## Build & Flash

### Environment Setup (Linux)

1. Install Python 3 if not already installed:
   ```bash
   sudo apt update
   sudo apt install python3 python3-venv python3-pip
   ```

2. Create and activate a Python virtual environment:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

3. Install PlatformIO in the virtual environment:
   ```bash
   pip install platformio
   ```

4. Initialize the PlatformIO project (if not already done):
   ```bash
   platformio init --board lolin32 --project-dir kitchenAnalyzer
   ```

5. Build and upload firmware to ESP32:
   ```bash
   cd kitchenAnalyzer
   platformio run --target upload
   ```

## Usage
1. Assemble the hardware as per wiring instructions
2. Flash the firmware using PlatformIO
3. Power on with battery
4. Device will sleep until motion is detected
5. When motion detected, LCD turns on and shows sensor data
6. Updates every second, shows countdown timer
7. Sleeps after 20 seconds of no motion

## Customization
- Adjust timeout values in `config.h`
- Calibrate MQ-2 sensor R0 value for accurate readings
- Modify LCD layout in `main.cpp` if needed

## Future Enhancements
The KitchenAnalyzer is designed with IoT capabilities in mind. Planned future enhancements include:
- **WiFi Connectivity**: Integrate ESP32 WiFi for internet access
- **Cloud Database Storage**: Store historical sensor data in a cloud database (e.g., Firebase Realtime Database, AWS IoT Core, or Google Cloud Firestore)
- **Data Logging**: Continuous logging of gas levels, temperature, humidity, and motion events
- **Remote Monitoring**: Web dashboard or mobile app for real-time and historical data visualization
- **Alerts and Notifications**: Email/SMS alerts for gas detection thresholds or abnormal readings
- **Data Analysis**: Trend analysis and predictive maintenance insights
- **OTA Updates**: Over-the-air firmware updates for easy maintenance

These enhancements will transform the device from a local monitor to a full IoT solution while maintaining its low-power, battery-operated design.

## License
See LICENSE file for details.
