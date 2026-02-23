# Sentry Data Scout v0.3 🎯

**[PL]** Sentry Data Scout to zaawansowany kontroler markera Spyder MR3 oparty na układzie ESP32. Projekt wykorzystuje asynchroniczną maszynę stanów (RTOS) do jednoczesnego sterowania serwomechanizmami dużej mocy, zbierania danych z sensorów (odrzut, dźwięk, temperatura) oraz obsługi serwera WWW do zdalnego sterowania z telefonu.

**[EN]** Sentry Data Scout is an advanced ESP32-based controller for the Spyder MR3 marker. The project uses an asynchronous state machine (RTOS) to simultaneously control high-power servos, collect sensor data (recoil, audio, temperature), and serve a Web UI for remote smartphone control.

---

## 🚀 Features / Główne funkcje
* **Dual Core RTOS:** Core 0 handles the Web Server/WiFi, while Core 1 executes time-critical hardware tasks without blocking.
* **Web Interface:** Built-in HTML/JS interface for remote configuration (fire rate, power, ammo weight) and triggering.
* **Telemetry Data:**
  * **Recoil (IMU):** BNO055 via I2C for tracking G-force.
  * **Acoustics:** Analog MAX4466 mic for peak & average dB calculation.
  * **Barrel Temp:** DS18B20 1-Wire sensor.
* **Data Logging:** Exports a clean `logs.csv` directly from LittleFS memory.
* **Weather Sync:** Pulls live environmental data (Temp, Hum, Press) via OpenWeather API + GPS.

---

## ⚠️ CRITICAL HARDWARE WARNINGS / OSTRZEŻENIA SPRZĘTOWE

1. **SERVO POWER (DS3218MG & MG945):** * **[PL]** Serwa pobierają 2-3A pod obciążeniem. NIE ZASILAJ ICH Z PINU 5V NA ESP32, bo spalisz układ! Użyj zewnętrznej przetwornicy DC-DC (min. 3A-5A). Pamiętaj o połączeniu masy (GND) przetwornicy, serw i ESP32.
   * **[EN]** Servos draw 2-3A under load. DO NOT POWER THEM FROM THE 5V PIN ON THE ESP32 to prevent brownouts/burning the board. Use an external DC-DC converter (min. 3A-5A). Ensure common ground (GND) across the converter, servos, and ESP32.
2. **TEMP SENSOR (DS18B20):** * Requires a 4.7k Ohm pull-up resistor on the DATA line.
3. **I2C CONNECTIONS (BNO055):** * Must be hard-soldered! Regular dupont wires will disconnect due to marker recoil.

---

## 🛠️ Requirements & Installation

Before uploading, ensure you have the following libraries installed in your Arduino IDE:
* `WiFi.h`, `WebServer.h`, `LittleFS.h`, `HTTPClient.h` (Built-in for ESP32)
* `ArduinoJson`
* `ESP32Servo`
* `Adafruit_BNO055` & `Adafruit_Sensor`
* `DallasTemperature` & `OneWire`

**Setup Steps:**
1. Clone the repository.
2. Enter your Wi-Fi credentials in `ssid` and `password`.
3. Provide your personal `OPENWEATHER_API_KEY`.
4. Flash the code to your ESP32.

---

## 📝 License / Licencja
*(Apache 2.0)*

**Author:** Maciej Kasperek (vonKrappitz)
