#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <MQUnifiedsensor.h>
#include "config.h"

LiquidCrystal_I2C lcd((uint8_t)LCD_I2C_ADDR, (uint8_t)LCD_COLS, (uint8_t)LCD_ROWS);

DHT dht(PIN_TEMP_HUM_SENSOR, DHT22);

MQUnifiedsensor MQ2("ESP-32", 3.3, 12, PIN_GAS_SENSOR, "MQ-2");

unsigned long lastMotionTime = 0;
bool motionDetected = false;

void wokeUpWork(bool setupFunction) {
        bool currentMotion = (analogRead(PIN_MOTION_SENSOR) > 1000); // Adjust threshold as needed
    Serial.print("Motion sensor: ");
    Serial.println(currentMotion ? "HIGH" : "LOW");
    Serial.println(analogRead(PIN_MOTION_SENSOR));
    if (setupFunction) {
        currentMotion = true; // Force motion detection on setup to initialize LCD and read sensors
    }

    float gasValue, temperature, humidity;

    if (currentMotion) {
        if (!motionDetected) {
            // Motion just detected
            motionDetected = true; 
            lcd.backlight(); // Turn on LCD backlight
            Serial.println("Motion detected - LED on, LCD backlight on");
        }
        lastMotionTime = millis();

        // Read and display sensor values
        // int gasValue = analogRead(PIN_GAS_SENSOR);
        MQ2.update();
        gasValue = MQ2.readSensor();
        temperature = dht.readTemperature();
        humidity = dht.readHumidity();

        Serial.print("Gas: ");
        Serial.print(gasValue);
        Serial.print(", Temp: ");
        Serial.print(temperature);
        Serial.print(", Hum: ");
        Serial.println(humidity);

        if (isnan(temperature) || isnan(humidity)) {
            Serial.println("DHT sensor read failed");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Sensor Error");
        } else {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Gas:");
            lcd.print(gasValue, 0);
            lcd.print(" ");
            lcd.setCursor(9, 0);
            lcd.print("T:");
            lcd.print(temperature, 1);
            lcd.setCursor(0, 1);
            lcd.print("H:");
            lcd.print(humidity, 1);
            lcd.print("   M:");
            lcd.print(currentMotion ? 1 : 0);            long remaining = NO_MOTION_TIMEOUT_S;
            lcd.setCursor(14, 1);
            if (remaining < 10) lcd.print("0");
            lcd.print(remaining);            Serial.println("LCD updated with sensor values");
        }

        delay(1000); // Update display every second
    } else {
        if (motionDetected) {
            unsigned long elapsed = millis() - lastMotionTime;
            if (elapsed >= NO_MOTION_TIMEOUT_S * 1000UL) {
                // Turn off LCD and enter deep sleep
                lcd.noBacklight();
                Serial.println("No motion for 20s - LCD off, LED off, entering deep sleep");
                Serial.println("Will wake up on motion");
                esp_deep_sleep_start();
            } else {
                // Read and display sensor values during timeout period
            MQ2.update();
            gasValue = MQ2.readSensor();
            temperature = dht.readTemperature();
            humidity = dht.readHumidity();

            Serial.print("Gas: ");
            Serial.print(gasValue);
            Serial.print(", Temp: ");
            Serial.print(temperature);
            Serial.print(", Hum: ");
            Serial.println(humidity);

            if (!isnan(temperature) && !isnan(humidity)) {
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Gas:");
                lcd.print(gasValue, 0);
                lcd.print(" ");
                lcd.setCursor(9, 0);
                lcd.print("T:");
                lcd.print(temperature, 1);
                lcd.setCursor(0, 1);
                lcd.print("H:");
                lcd.print(humidity, 1);
                lcd.print("   M:");
                lcd.print(currentMotion ? 1 : 0);                
                long remaining = (NO_MOTION_TIMEOUT_S * 1000UL - (millis() - lastMotionTime)) / 1000;
                if (remaining < 0) remaining = 0;
                lcd.setCursor(14, 1);
                if (remaining < 10) lcd.print("0");
                lcd.print(remaining);                
                Serial.println("LCD updated with sensor values");
            }
            delay(1000);
            }
        } else {
            delay(100);
        }
    }
}

void setup() {
    // Serial.begin(115200);
    Serial.println("Kitchen Analyzer starting...");

    pinMode(PIN_GAS_SENSOR, INPUT);
    pinMode(PIN_MOTION_SENSOR, INPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    dht.begin();
    Serial.println("DHT sensor initialized");

    MQ2.init();
    MQ2.setVCC(3.3);
    MQ2.setRegressionMethod(1);
    MQ2.setA(574.25); MQ2.setB(-2.222);
    MQ2.setR0(10.0);
    Serial.println("MQ2 sensor initialized");

    // Initialize I2C
    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
    Serial.println("I2C initialized with custom pins");

    // Configure wake up on motion
    esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN, 1); // Wake up on HIGH
    // esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME_S * 1000000ULL); // Backup timer wake up
    Serial.println("Wake up configured on motion sensor pin and timer");

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    // lcd.print("Kitchen Analyzer");
    Serial.println("LCD initialized");
    // delay(2000);
    lcd.clear();
    Serial.println("Setup complete");
    motionDetected = false; // Reset motion detected flag after setup
    wokeUpWork(true);
}



void loop() {
    wokeUpWork(false);
}
