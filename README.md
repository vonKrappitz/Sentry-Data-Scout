# Sentry Data Scout v0.4.4 FL 🎯

**[PL]** Sentry Data Scout to zaawansowany kontroler markera Spyder MR3 oparty na układzie ESP32. Projekt wykorzystuje asynchroniczną maszynę stanów (RTOS) do jednoczesnego sterowania serwomechanizmami dużej mocy, zbierania danych z sensorów (odrzut, dźwięk, temperatura) oraz obsługi serwera WWW do zdalnego sterowania z telefonu. Pełna kalibracja serw, diagnostyka sprzętowa, tryb AP offline i zapis danych do CSV — wszystko z poziomu przeglądarki.

**[EN]** Sentry Data Scout is an advanced ESP32-based controller for the Spyder MR3 marker. The project uses an asynchronous state machine (RTOS) to simultaneously control high-power servos, collect sensor data (recoil, audio, temperature), and serve a Web UI for remote smartphone control. Full servo calibration, hardware diagnostics, offline AP mode and CSV data logging — all from a browser.

**Author:** Maciej Kasperek (vonKrappitz)

---

## 🚀 Features / Główne funkcje

* **Dual-Core RTOS:** Core 0 handles Web Server/WiFi, Core 1 executes time-critical hardware tasks without blocking.
* **Web Interface:** Built-in HTML/JS/CSS single-page app with dark/light mode toggle 🌙/🌞
* **Step-by-Step Calibration Wizard:** Guided trigger and N2 screw servo calibration, persisted to flash.
* **Telemetry Data:**
    * **Recoil (IMU):** BNO055 via I2C — peak G-force per shot.
    * **Acoustics:** Analog MAX4466 mic — peak & average dB calculation (~10-40kHz sampling).
    * **Barrel Temp:** DS18B20 1-Wire — async read within 500ms measurement window.
* **Data Logging:** Exports clean `logs.csv` from LittleFS with per-shot telemetry.
* **Weather Sync:** Live environmental data (Temp, Humidity, Pressure) via OpenWeather API + browser GPS.
* **Diagnostics Panel:** Traffic-light health indicators (🟢🟡🔴) for all sensors, WiFi RSSI, heap, voltage.
* **WiFi AP Fallback:** No network in 15s? ESP32 auto-starts its own hotspot (`SentryAP`).
* **Last-Shot Preview:** Barrel temp, G-force and dB shown on status card after each shot.
* **Safety:** Min cadence 500ms enforced, double-start protection (409 Conflict), constrain on all servo inputs.
* **Power Persistence:** Servo power level (0-100%) saved to flash — survives reboot.
* **38 Unit Tests:** Full state machine coverage compiled natively with g++.

---

## ⚠️ CRITICAL HARDWARE WARNINGS / OSTRZEŻENIA SPRZĘTOWE

### 1. 🔌 Servo Power / Zasilanie serw (DS3218MG & MG945)

* **[PL]** Serwa pobierają 2-3A pod obciążeniem. **NIE ZASILAJ ICH Z PINU 5V NA ESP32**, bo spalisz układ! Użyj zewnętrznej przetwornicy DC-DC (min. 3A, zalecane 5A). Pamiętaj o połączeniu masy (GND) przetwornicy, serw i ESP32.
* **[EN]** Servos draw 2-3A under load. **DO NOT POWER THEM FROM THE 5V PIN ON THE ESP32** to prevent brownouts/burning the board. Use an external DC-DC converter (min. 3A, recommended 5A). Ensure common ground (GND) across the converter, servos, and ESP32.

```
    ┌─────────────┐
    │ Battery 7.4V│
    └──────┬──────┘
           │
    ┌──────▼──────┐
    │ UBEC/SBEC   │──── 5V ──→ Servo red wires (power)
    │ DC-DC 5A    │
    └──────┬──────┘
           │
           ├── GND ──→ Servo black wires
           └── GND ──→ ESP32 GND pin  ◄── COMMON GROUND / WSPÓLNA MASA!
```

### 2. 🌡️ Temp Sensor / Czujnik temperatury (DS18B20)
* Requires a **4.7kΩ pull-up resistor** on the DATA line to 3.3V.

### 3. 📐 I2C Connections / Połączenia I2C (BNO055)
* **Must be hard-soldered with silicone insulation!** Regular dupont wires **WILL** disconnect due to marker recoil at the first shot.

### 4. 🎤 Microphone / Mikrofon (MAX4466)
* Power from clean 3.3V. Adjust the gain potentiometer to center position before first calibration.

### 5. 🔋 Battery Voltage (optional / opcjonalne)
* Requires voltage divider on pin 35 (e.g. 100k/100k for up to ~6.6V). Set `VBAT_ENABLED = true` in code after connecting.

---

## 📌 Pin Map / Mapa pinów

| Component / Komponent | Model | ESP32 Pin | Notes / Uwagi |
|---|---|---|---|
| Trigger servo / Serwo spustu | DS3218MG (20kg) | GPIO 18 | 2-3A draw! External PSU! |
| N2 screw servo / Serwo śruby | MG945 (12kg) | GPIO 19 | 1-1.5A additional |
| Barrel temp / Temp lufy | DS18B20 | GPIO 4 | 4.7kΩ pull-up to 3.3V |
| IMU / Accelerometer | BNO055 | I2C SDA:21 SCL:22 | Soldered only! |
| Microphone / Mikrofon | MAX4466 | GPIO 34 (ADC1) | Clean 3.3V, gain pot center |
| Battery voltage (optional) | Voltage divider | GPIO 35 | Set VBAT_ENABLED=true |

---

## 🏗️ Architecture / Architektura

```
    ESP32 Dual-Core
    ┌────────────────────────────────────────────────┐
    │                                                │
    │  CORE 0                    CORE 1              │
    │  ┌──────────────┐          ┌────────────────┐  │
    │  │ WiFi +       │          │ loop()         │  │
    │  │ WebServer    │          │ State Machine  │  │
    │  │ mDNS         │          │ (servo timing) │  │
    │  │ 13 endpoints │          └───────┬────────┘  │
    │  └──────────────┘                  │           │
    │                             ┌──────▼────────┐  │
    │                             │ High-Speed    │  │
    │                             │ Measurements  │  │
    │                             │ IMU + Mic     │  │
    │                             │ ~10-40kHz ADC │  │
    │                             └───────────────┘  │
    │                                                │
    │  Priority 1 on both cores — yield() prevents   │
    │  task starvation and enables fair scheduling.   │
    └────────────────────────────────────────────────┘
```

### 🔄 State Machine / Maszyna stanów

```
                     cmd=start
     ┌──────────┐ ─────────────→ ┌────────────────┐
     │ SYS_IDLE │                │ SHOOTING_PRESS │
     │          │ ←── (last) ─── │ (servo → max)  │
     └────▲─────┘                └───────┬────────┘
          │                              │ 60ms
     cmd=stop                            ▼
     (any state)                 ┌────────────────┐
          │                      │ SHOOTING_      │
          │                      │ RELEASE        │
          │                      └───────┬────────┘
          │                              │
     ┌────┴─────┐                        ▼
     │ PAUSED   │ ←── pause ─── ┌────────────────┐
     │          │               │ SHOOTING_      │
     └────┬─────┘               │ WAIT_500       │
          │ resume              │ (500ms window) │
          ▼                     └───────┬────────┘
     ┌──────────┐                       │
     │ WAITING_ │ ←── cadence ──────────┤ remaining > 0
     │ CADENCE  │                       │
     └──────────┘                       │ requireV?
                                        ▼
                                ┌────────────────┐
                                │ WAITING_FOR_V  │
                                │ (chronograph)  │
                                └────────────────┘
```

---

## 🛠️ Installation / Instalacja

### Required Libraries / Wymagane biblioteki

Built-in (ESP32 Arduino):
* `WiFi.h`, `WebServer.h`, `LittleFS.h`, `HTTPClient.h`, `ESPmDNS.h`

Install via Library Manager:
* `ArduinoJson` (Benoit Blanchon, v6.x)
* `ESP32Servo` (Kevin Harrington)
* `Adafruit BNO055` + `Adafruit Unified Sensor`
* `DallasTemperature` + `OneWire`

### Setup Steps / Kroki instalacji

1. Clone the repository.
2. Open `Sentry_data_scout_0_4_4_FL.ino`
3. Enter your credentials:
   ```cpp
   const char* ssid     = "YOUR_WIFI";
   const char* password = "YOUR_PASSWORD";
   const String OPENWEATHER_API_KEY = "YOUR_KEY";  // free at openweathermap.org
   ```
4. Arduino IDE Board settings:
    * Board: **ESP32 Dev Module**
    * Upload Speed: **921600**
    * Partition Scheme: **Default 4MB with spiffs**
5. Flash the code to your ESP32.
6. Open `http://sentry.local` (or check Serial Monitor for IP).
7. **Run the Calibration Wizard** — it guides you step by step.
8. Fire! 🔥

### WiFi Modes / Tryby WiFi

| Mode | When / Kiedy | Address / Adres | Weather / Pogoda |
|---|---|---|---|
| 📶 STA (client) | Connected to WiFi within 15s | `http://sentry.local` | ✅ Works |
| 📡 AP (hotspot) | No WiFi after 15s timeout | `http://192.168.4.1` | ❌ No internet |

AP network: `SentryAP`, password: `sentry1234` (change in code: `ap_ssid`, `ap_password`).

---

## 📊 CSV Data Format / Format danych CSV

```csv
ID,Timestamp,Typ_Kulki,Waga_Kulki,Odleglosc,Temp_Otoczenia,Wilgotnosc,Cisnienie,Temp_Lufy_500ms,Max_Odrzut_G,Peak_dB,Sredni_dB,V_ms
1,12345,szorstka,3.20,5.00,22.50,45.00,1013.00,28.30,8.50,95.20,88.10,0.0
UPDATE_V,1,85.30
```

* `V_ms = 0.0` → velocity not yet measured / prędkość jeszcze nie zmierzona
* `UPDATE_V,ID,value` → velocity appended later from chronograph / dopisanie prędkości z chronografu

---

## 📁 Files on ESP32 / Pliki na ESP32

```
LittleFS:
  /logs.csv            — Shot log / Log strzałów
  /calibration.json    — Servo calibration + power level / Kalibracja + moc
```

---

## 🧪 Unit Tests / Testy jednostkowe

File: `test_sentry_0_4_4.cpp` — **38 tests, 10 groups, 100% PASS ✅**

```bash
g++ -std=c++17 -o test_sentry test_sentry_0_4_4.cpp && ./test_sentry
```

| Group / Grupa | Tests | Scope / Zakres |
|---|---|---|
| State Machine Flow | 5 | Full shot cycle, burst of 3, 60ms/500ms timing |
| Pause / Resume | 3 | Cadence pause, timer reset, remaining preserved |
| V Input / Chronograph | 3 | WAITING_FOR_V → CADENCE or IDLE |
| Safety Guards | 6 | Double-start 409, cadence floor 500ms, constrain |
| Stop Command | 3 | Stop from every possible state |
| Power Mapping | 5 | 0/50/100%, asymmetric calibrated ranges |
| Voltage Calculation | 4 | ADC 0/2048/4095, different divider ratios |
| Last Shot Data | 3 | Init, save, update on each shot |
| Reset Logs | 1 | Counter and flag zeroed |
| Edge Cases | 5 | millis() overflow, resume from non-PAUSED, count=0 |

---

## 📜 Changelog

### v0.3 → v0.4
* ✅ Servo calibration system with LittleFS persistence / System kalibracji z zapisem
* ✅ mDNS — `http://sentry.local`
* ✅ Power slider 0-100% mapped to calibrated range / Suwak mocy na skalibrowany zakres
* ✅ Live servo preview during calibration / Podgląd serwa na żywo

### v0.4 → v0.4.1
* ✅ Step-by-step Calibration Wizard / Krokowy wizard kalibracyjny
* ✅ Weight preset selector + custom input / Select wagi z presetami + "Inna"
* ✅ Input validation (constrain) on calibration / Walidacja na endpoincie kalibracji

### v0.4.1 → v0.4.2
* ✅ HTTP POST for all state-changing endpoints / POST dla operacji zmieniających stan
* ✅ Auto-hide wizard if calibration exists / Auto-ukrywanie wizarda
* ✅ Toast notification system (green/red) / System toastów
* ✅ Checkmark ✓ feedback on wizard steps / Ptaszki na krokach wizarda
* ✅ Bilingual comments (EN/PL) restored / Przywrócone komentarze EN/PL

### v0.4.2 → v0.4.3
* ✅ Diagnostics panel with traffic-light indicators / Panel diagnostyczny 🟢🟡🔴
* ✅ BNO055, DS18B20, Mic, RSSI, Heap, Uptime monitoring
* ✅ Battery voltage monitoring infrastructure / Infrastruktura monitoringu napięcia
* ✅ Power level persistence across reboots / Trwały zapis mocy
* ✅ Double-start protection (409 Conflict) / Ochrona przed podwójnym startem

### v0.4.3 → v0.4.4 FL 🏁
* ✅ WiFi AP fallback (15s timeout → SentryAP hotspot)
* ✅ Light/Dark mode toggle 🌙🌞 / Tryb jasny/ciemny
* ✅ Last-shot preview on status card / Podgląd ostatniego strzału
* ✅ Reset logs with confirm() dialog / Reset logów z potwierdzeniem
* ✅ Minimum cadence 500ms enforced (backend + UI) / Min kadencja 500ms
* ✅ WiFi mode badge (STA/AP) / Wskaźnik trybu WiFi
* ✅ 38 unit tests — all green / 38 testów — wszystkie zielone ✅

---

## 📝 License / Licencja

*(Apache 2.0)*

**Author:** Maciej Kasperek (vonKrappitz)
