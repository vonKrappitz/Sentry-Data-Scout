/**
 * =======================================================================================
 * Project: Sentry Data Scout
 * Author: Maciej Kasperek (vonKrappitz)
 * Version: 0.4.3
 * [EN] Description: Spyder MR3 marker controller — Web Server + LittleFS + State Machine
 *      Calibration Wizard, mDNS, dual-core RTOS, Diagnostics panel, Power persistence.
 * [PL] Opis: Kontroler markera Spyder MR3 — Web Serwer + LittleFS + Maszyna Stanów
 *      Wizard kalibracyjny, mDNS, RTOS dwurdzeniowy, Panel diagnostyczny, Zapis mocy.
 * ---------------------------------------------------------------------------------------
 * [EN]
 * CRITICAL HARDWARE NOTES — READ BEFORE CONNECTING!
 *
 * 1. SERVO POWER SUPPLY (DS3218MG and MG945):
 *    - DS3218MG is a powerful servo (20kg) that can draw 2-3A when stalled.
 *      MG945 adds 1-1.5A.
 *    - ABSOLUTELY DO NOT power them from the 5V (VIN) pin on ESP32!
 *      A voltage drop will reset the chip (Brownout) or burn the regulator.
 *    - SOLUTION: Use an external DC-DC Step-Down converter (UBEC/SBEC)
 *      with minimum 3A (5A recommended) output at 5V or 6V.
 *    - CONNECTION: Red wires from servos → positive (+) of the converter.
 *    - CRITICAL (COMMON GROUND): Black/Brown wires from servos, negative (-)
 *      of the converter, AND GND pin on ESP32 MUST be connected together.
 *      Without common ground, the PWM signal will not be interpreted correctly.
 *
 * 2. TEMPERATURE SENSOR (DS18B20 on pin 4):
 *    - 1-Wire module. Requires soldering a 4.7k Ohm pull-up resistor
 *      between the DATA pin and 3.3V.
 *
 * 3. VIBRATION SENSOR (BNO055 on I2C):
 *    - SDA (usually 21) and SCL (usually 22) pins.
 *    - Requires absolutely rigid, SOLDERED connections with silicone insulation.
 *      Standard jumper wires ("duponts") WILL disconnect at the first shot.
 *
 * 4. MICROPHONE (MAX4466 on pin 34):
 *    - Power with clean 3.3V from ESP32. The built-in potentiometer on the back
 *      of the module allows Gain adjustment. Set to middle before first calibration.
 *
 * 5. BATTERY VOLTAGE MONITORING (optional, pin 35):
 *    - Requires external voltage divider (e.g. 100k/100k for up to ~6.6V).
 *    - Connect battery+ through divider to pin 35. Without divider reads 0V.
 *    - Adjust VBAT_DIVIDER_RATIO to match your resistor values.
 *    - If not connected, diagnostics will show "N/A" for voltage.
 *
 * Required libraries: WiFi, WebServer, LittleFS, HTTPClient, ArduinoJson,
 *   ESP32Servo, Adafruit_BNO055, DallasTemperature, ESPmDNS
 * ---------------------------------------------------------------------------------------
 * [PL]
 * KRYTYCZNE UWAGI SPRZĘTOWE — PRZECZYTAJ ZANIM PODŁĄCZYSZ!
 *
 * 1. ZASILANIE SERWOMECHANIZMÓW (DS3218MG i MG945):
 *    - DS3218MG to potężne serwo (20kg), przy zablokowaniu pociąga 2-3A.
 *      MG945 dorzuca swoje 1-1.5A.
 *    - ABSOLUTNIE NIE zasilaj ich z pinu 5V (VIN) na ESP32!
 *      Spadek napięcia zresetuje układ (Brownout) lub spali stabilizator.
 *    - ROZWIĄZANIE: Zewnętrzna przetwornica DC-DC Step-Down (UBEC/SBEC)
 *      o wydajności min 3A (zalecane 5A), 5V lub 6V na wyjściu.
 *    - PODŁĄCZENIE: Czerwone kable od serw → plus (+) wyjścia przetwornicy.
 *    - KRYTYCZNE (WSPÓLNA MASA): Czarne/Brązowe kable od serw, minus (-)
 *      przetwornicy ORAZ pin GND na ESP32 MUSZĄ być ze sobą połączone.
 *      Bez wspólnej masy sygnał PWM nie zostanie poprawnie odczytany.
 *
 * 2. CZUJNIK TEMPERATURY (DS18B20 na pinu 4):
 *    - Moduł 1-Wire. Wymaga wlutowania rezystora podciągającego (pull-up)
 *      4.7k Ohm pomiędzy pinem DATA a zasilaniem 3.3V.
 *
 * 3. CZUJNIK DRGAŃ (BNO055 na I2C):
 *    - Piny SDA (21) i SCL (22).
 *    - Wymagane LUTOWANE, sztywne połączenia z izolacją silikonową.
 *      Zwykłe kable zworkowe ("duponty") rozłączą się przy pierwszym strzale.
 *
 * 4. MIKROFON (MAX4466 na pinie 34):
 *    - Zasil z czystego 3.3V od ESP32. Potencjometr z tyłu modułu reguluje
 *      wzmocnienie (Gain). Ustaw na środek przed pierwszą kalibracją.
 *
 * 5. MONITORING NAPIĘCIA BATERII (opcjonalny, pin 35):
 *    - Wymaga zewnętrznego dzielnika napięcia (np. 100k/100k dla ~6.6V max).
 *    - Podłącz baterię+ przez dzielnik do pinu 35. Bez dzielnika czyta 0V.
 *    - Dostosuj VBAT_DIVIDER_RATIO do swoich rezystorów.
 *    - Jeśli niepodłączony, diagnostyka pokaże "N/A".
 *
 * Wymagane biblioteki: WiFi, WebServer, LittleFS, HTTPClient, ArduinoJson,
 *   ESP32Servo, Adafruit_BNO055, DallasTemperature, ESPmDNS
 * =======================================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESPmDNS.h>

/*
 * =======================================================================================
 * [EN] NETWORK CONFIGURATION / [PL] KONFIGURACJA SIECI
 * =======================================================================================
 */
const char* ssid     = "SSID_here";
const char* password = "PASSWORD_here";
const String OPENWEATHER_API_KEY = "API_here";

/*
 * =======================================================================================
 * [EN] PIN DEFINITIONS (adapted for ESP32)
 * [PL] DEFINICJE PINÓW (dostosowane do ESP32)
 * =======================================================================================
 */
#define SERVO_SPUST_PIN 18  // [EN] Trigger: DS3218MG signal wire / [PL] Spust: kabel sygnałowy DS3218MG
#define SERVO_SRUBA_PIN 19  // [EN] N2 Screw: MG945 signal wire / [PL] Śruba N2: kabel sygnałowy MG945
#define DS18B20_PIN      4  // [EN] Reminder: 4.7k pull-up needed! / [PL] Przypomnienie: rezystor 4.7k!
#define MIC_PIN         34  // [EN] Analog mic (ADC1 only works with WiFi!) / [PL] Mikrofon analogowy (tylko ADC1 działa z WiFi!)

/*
 * [EN] OPTIONAL: Battery voltage monitoring via voltage divider on ADC pin.
 *   With 100k/100k divider: VBAT_DIVIDER_RATIO = 2.0 (reads up to ~6.6V).
 *   With 100k/47k divider: VBAT_DIVIDER_RATIO ≈ 3.13 (reads up to ~10.3V).
 *   Set VBAT_ENABLED to false if no divider is connected.
 * [PL] OPCJONALNIE: Monitoring napięcia baterii przez dzielnik napięcia na ADC.
 *   Przy dzielniku 100k/100k: VBAT_DIVIDER_RATIO = 2.0 (odczyt do ~6.6V).
 *   Przy dzielniku 100k/47k: VBAT_DIVIDER_RATIO ≈ 3.13 (odczyt do ~10.3V).
 *   Ustaw VBAT_ENABLED na false jeśli dzielnik nie jest podłączony.
 */
#define VBAT_PIN        35
#define VBAT_ENABLED    false    // [EN] Set true when voltage divider is connected / [PL] Ustaw true gdy dzielnik jest podłączony
#define VBAT_DIVIDER_RATIO 2.0  // [EN] Adjust to your resistor values / [PL] Dostosuj do swoich rezystorów

/*
 * =======================================================================================
 * [EN] SERVO CONFIGURATION
 *   Default Servo library values (1000-2000us) give only ~90° on DS3218MG.
 *   Extended range 500-2500us provides full 180° and torque spectrum.
 * [PL] KONFIGURACJA SERWOMECHANIZMÓW
 *   Domyślne wartości (1000-2000us) dają na DS3218MG tylko ~90°.
 *   Rozszerzony zakres 500-2500us daje pełne 180° i spektrum momentu.
 * =======================================================================================
 */
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

/*
 * =======================================================================================
 * [EN] GLOBAL OBJECTS / [PL] OBIEKTY GLOBALNE
 * =======================================================================================
 */
WebServer server(80);
Servo serwoSpust;
Servo serwoSruba;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensorsBarrel(&oneWire);

/*
 * =======================================================================================
 * [EN] CALIBRATION DATA (persisted to LittleFS as /calibration.json)
 *   spust_min = rest position (trigger released), spust_max = fire position
 *   sruba_min = N2 screw fully closed, sruba_max = N2 screw max open
 *   power_pct = last set power level (0-100%), restored on boot
 * [PL] DANE KALIBRACYJNE (zapisywane do LittleFS jako /calibration.json)
 *   spust_min = pozycja spoczynkowa, spust_max = pozycja strzału
 *   sruba_min = śruba N2 zamknięta, sruba_max = śruba N2 max otwarta
 *   power_pct = ostatnio ustawiony poziom mocy (0-100%), przywracany po restarcie
 * =======================================================================================
 */
int cal_spust_min = 0, cal_spust_max = 180;
int cal_sruba_min = 0, cal_sruba_max = 180;
int cal_power_pct = 50; // [EN] Persisted power level / [PL] Zapisywany poziom mocy

/*
 * =======================================================================================
 * [EN] GLOBAL VARIABLES & SESSION DATA / [PL] ZMIENNE GLOBALNE I DANE SESJI
 * =======================================================================================
 */
volatile bool isShooting = false;
volatile float sharedMaxG  = 0.0;
volatile float sharedPeakDB = 0.0;
volatile float sharedAvgDB  = 0.0;

/*
 * [EN] States for the shooting state machine (see v0.4.2 comments for details)
 * [PL] Stany maszyny strzelającej (szczegóły w komentarzach v0.4.2)
 */
enum SystemState {
  SYS_IDLE,
  SYS_SHOOTING_PRESS,
  SYS_SHOOTING_RELEASE,
  SYS_SHOOTING_WAIT_500,
  SYS_WAITING_CADENCE,
  SYS_PAUSED,
  SYS_WAITING_FOR_V
};
volatile SystemState sysState = SYS_IDLE;
volatile int  shotsRemaining  = 0;
volatile bool pauseRequested  = false;
volatile bool requireVInput   = false;
unsigned long stateStartTime  = 0;

// [EN] Weather data from OpenWeatherMap / [PL] Dane pogodowe z OpenWeatherMap
float currentEnvTemp  = 0.0;
float currentEnvHum   = 0.0;
float currentEnvPress = 0.0;

/*
 * [EN] Session Configuration (default GUI values)
 * [PL] Konfiguracja Sesji (domyślne wartości GUI)
 */
String currentAmmoType    = "szorstka";
float  currentAmmoWeight  = 3.2;
float  currentDistance     = 5.0;   // [m]
int    currentCadenceMs   = 1000;   // [ms]
int    totalShotsFired    = 0;

/*
 * =======================================================================================
 * [EN] DIAGNOSTICS STATE — updated periodically for health monitoring
 *   bnoOK: true if BNO055 initialized successfully in setup()
 *   lastBarrelTemp: cached from last DS18B20 read (-127 = disconnected, 85 = reset/error)
 * [PL] STAN DIAGNOSTYCZNY — aktualizowany okresowo do monitorowania zdrowia
 *   bnoOK: true jeśli BNO055 zainicjowany poprawnie w setup()
 *   lastBarrelTemp: cache z ostatniego odczytu DS18B20 (-127 = rozłączony, 85 = reset/błąd)
 * =======================================================================================
 */
bool   bnoOK = false;
float  lastBarrelTemp = -127.0;

/*
 * =======================================================================================
 * [EN] FILE SYSTEM — CSV LOGGING (LittleFS)
 * [PL] SYSTEM PLIKÓW — LOGOWANIE CSV (LittleFS)
 * =======================================================================================
 */
void appendLogToCSV(int id, String typ, float waga, float dystans,
                    float tempLufy500, float maxRecoil, float peakDB, float avgDB) {
  File file = LittleFS.open("/logs.csv", FILE_APPEND);
  if (!file) return;
  file.printf("%d,%lu,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0.0\n",
              id, millis(), typ.c_str(), waga, dystans,
              currentEnvTemp, currentEnvHum, currentEnvPress,
              tempLufy500, maxRecoil, peakDB, avgDB);
  file.close();
}

void updateVelocityInLog(int id, float v) {
  File file = LittleFS.open("/logs.csv", FILE_APPEND);
  if (file) {
    file.printf("UPDATE_V,%d,%.2f\n", id, v);
    file.close();
  }
}

/*
 * =======================================================================================
 * [EN] CALIBRATION PERSISTENCE (LittleFS → /calibration.json)
 *   Now includes power_pct so the power slider position survives reboot.
 * [PL] TRWAŁOŚĆ KALIBRACJI (LittleFS → /calibration.json)
 *   Teraz zawiera power_pct, więc pozycja suwaka mocy przeżywa restart.
 * =======================================================================================
 */
void saveCalibration() {
  File f = LittleFS.open("/calibration.json", FILE_WRITE);
  if (!f) return;
  StaticJsonDocument<256> doc;
  doc["spust_min"] = cal_spust_min;
  doc["spust_max"] = cal_spust_max;
  doc["sruba_min"] = cal_sruba_min;
  doc["sruba_max"] = cal_sruba_max;
  doc["power_pct"] = cal_power_pct;
  serializeJson(doc, f);
  f.close();
}

void loadCalibration() {
  File f = LittleFS.open("/calibration.json", FILE_READ);
  if (!f) return;
  StaticJsonDocument<256> doc;
  deserializeJson(doc, f);
  cal_spust_min = doc["spust_min"] | 0;
  cal_spust_max = doc["spust_max"] | 180;
  cal_sruba_min = doc["sruba_min"] | 0;
  cal_sruba_max = doc["sruba_max"] | 180;
  cal_power_pct = doc["power_pct"] | 50; // [EN] Default 50% if not saved / [PL] Domyślnie 50% jeśli nie zapisano
  f.close();
}

bool isCalibrated() {
  return LittleFS.exists("/calibration.json");
}

/*
 * =======================================================================================
 * [EN] VOLTAGE READING (optional hardware)
 *   Reads battery voltage through voltage divider on VBAT_PIN.
 *   Returns 0.0 if VBAT_ENABLED is false.
 *   ESP32 ADC: 12-bit (0-4095), reference 3.3V. Multiply by divider ratio.
 * [PL] ODCZYT NAPIĘCIA (opcjonalny sprzęt)
 *   Czyta napięcie baterii przez dzielnik na VBAT_PIN.
 *   Zwraca 0.0 jeśli VBAT_ENABLED jest false.
 *   ESP32 ADC: 12-bit (0-4095), referencja 3.3V. Pomnóż przez ratio dzielnika.
 * =======================================================================================
 */
float readBatteryVoltage() {
  if (!VBAT_ENABLED) return 0.0;
  int raw = analogRead(VBAT_PIN);
  float volts = (raw / 4095.0) * 3.3 * VBAT_DIVIDER_RATIO;
  return volts;
}

/*
 * =======================================================================================
 * [EN] HIGH-SPEED MEASUREMENT TASK (CORE 1)
 *   Runs on Core 1 alongside loop(). Samples IMU (BNO055) and microphone (MAX4466)
 *   at maximum ADC speed (~10-40 kHz) during the 500ms measurement window.
 *
 *   CRITICAL FOR RTOS:
 *   yield() releases CPU time to other tasks with the same priority (1).
 *   This allows loop() on the same core to count down the 500ms elapsed time
 *   and clear the isShooting flag. Without yield(), the system would deadlock.
 *   Priority is set to 1 (same as loop()) to prevent Task Starvation.
 *
 * [PL] ZADANIE SZYBKICH POMIARÓW (CORE 1)
 *   Działa na Core 1 obok loop(). Próbkuje IMU (BNO055) i mikrofon (MAX4466)
 *   z max prędkością ADC (~10-40 kHz) w oknie pomiarowym 500ms.
 *
 *   KRYTYCZNE DLA RTOS:
 *   yield() oddaje czas procesora innym taskom o priorytecie 1.
 *   Bez yield() — deadlock. Priorytet 1 zapobiega Task Starvation.
 * =======================================================================================
 */
void taskHighSpeedMeasurements(void * pvParameters) {
  for (;;) {
    if (isShooting) {
      sharedMaxG = 0.0;
      unsigned long sampleCount = 0;
      double sumSquares = 0;
      int signalMax = 0, signalMin = 4095;

      while (isShooting) {
        // [EN] 1. IMU — linear acceleration (recoil) / [PL] 1. IMU — przyspieszenie liniowe (odrzut)
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        float g = sqrt(accel.x()*accel.x() + accel.y()*accel.y() + accel.z()*accel.z());
        if (g > sharedMaxG) sharedMaxG = g;

        // [EN] 2. Microphone — peak-to-peak and RMS / [PL] 2. Mikrofon — peak-to-peak i RMS
        int sample = analogRead(MIC_PIN);
        if (sample > signalMax) signalMax = sample;
        if (sample < signalMin) signalMin = sample;

        double volts = ((double)sample - 2048.0) / 2048.0;
        sumSquares += (volts * volts);
        sampleCount++;

        yield(); // [EN] CRITICAL — prevents deadlock / [PL] KRYTYCZNE — zapobiega deadlockowi
      }

      // [EN] Microphone calibration (110 dB baseline — adjust with physical dB meter)
      // [PL] Kalibracja mikrofonu (110 dB bazowe — dostosuj z fizycznym decybelomierzem)
      float peakV = (signalMax - signalMin) / 4095.0f;
      sharedPeakDB = 20.0f * log10(peakV > 0.001f ? peakV : 0.001f) + 110.0f;

      if (sampleCount > 0) {
        double rms = sqrt(sumSquares / sampleCount);
        sharedAvgDB = 20.0f * log10(rms > 0.001f ? rms : 0.001f) + 110.0f;
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

/*
 * =======================================================================================
 * [EN] WEB SERVER — INTERFACE (HTML/CSS/JS)
 *   Single-page app with: Calibration Wizard, Fire Control, Velocity Input,
 *   Diagnostics panel with traffic-light health indicators, Toast notifications.
 * [PL] WEB SERVER — INTERFEJS (HTML/CSS/JS)
 *   SPA z: Wizard kalibracyjny, Kontrola ognia, Wpisywanie prędkości,
 *   Panel diagnostyczny z sygnalizacją świetlną zdrowia, Powiadomienia toast.
 * =======================================================================================
 */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Sentry Data Scout v0.4.3</title>
  <style>
    body {font-family:sans-serif;background:#121212;color:#e0e0e0;text-align:center;padding:10px;margin:0}
    .card {background:#1e1e1e;padding:15px;border-radius:8px;margin-bottom:15px;box-shadow:0 4px 8px rgba(0,0,0,0.5)}
    h2 {color:#00ff88;margin-bottom:2px}
    .author {font-size:11px;color:#666;margin-top:2px;margin-bottom:12px}
    h3 {color:#00d4ff;border-bottom:1px solid #444;padding-bottom:5px;margin-top:0}
    button {border:none;color:white;border-radius:5px;cursor:pointer;margin:5px}
    .btn-fire {font-size:22px;font-weight:bold;width:100%;padding:20px;border-radius:8px;margin-top:10px;transition:background 0.3s}
    .btn-green {background:#28a745} .btn-green:active{background:#1e7e34}
    .btn-red {background:#dc3545} .btn-red:active{background:#a71d2a}
    .btn-orange {background:#ff9800;color:#111} .btn-orange:active{background:#e65100}
    .btn-small {background:#0088ff;padding:10px 15px;font-size:14px}
    .btn-cancel {background:#555;font-size:14px;padding:12px;margin-top:5px;width:100%}
    select,input {padding:8px;font-size:14px;border-radius:4px;border:1px solid #555;background:#2c2c2c;color:#fff;margin:3px}
    .input-group {display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;text-align:left}
    .input-group label {flex:1;font-size:14px;color:#bbb}
    input[type=range] {width:100%}

    /* Toast */
    .toast {
      position:fixed;top:20px;left:50%;transform:translateX(-50%);
      background:#28a745;color:white;padding:12px 24px;border-radius:8px;
      font-size:16px;font-weight:bold;z-index:9999;
      opacity:0;transition:opacity 0.3s;pointer-events:none;
      box-shadow:0 4px 12px rgba(0,0,0,0.5);
    }
    .toast.show {opacity:1}
    .toast.error {background:#dc3545}
    .step-done {color:#28a745;font-weight:bold;margin-left:8px}

    /* Diagnostics traffic-light indicators */
    .diag-row {display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #2a2a2a}
    .diag-row:last-child {border-bottom:none}
    .diag-label {font-size:13px;color:#bbb;text-align:left;flex:1}
    .diag-value {font-size:13px;color:#fff;text-align:right;margin-right:8px;min-width:80px}
    .diag-dot {
      width:14px;height:14px;border-radius:50%;flex-shrink:0;
      box-shadow:0 0 6px rgba(0,0,0,0.5);
    }
    .dot-green  {background:#28a745;box-shadow:0 0 8px #28a74580}
    .dot-yellow {background:#ffc107;box-shadow:0 0 8px #ffc10780}
    .dot-red    {background:#dc3545;box-shadow:0 0 8px #dc354580}
    .dot-gray   {background:#666;box-shadow:none}

    /* Collapsible diagnostics */
    .diag-toggle {
      background:none;border:none;color:#00d4ff;font-size:14px;cursor:pointer;
      padding:0;margin:0;text-decoration:underline;
    }
  </style>
</head>
<body>

  <h2>Sentry Data Scout</h2>
  <div class="author">Maciej Kasperek (von Krappitz) | MR3 Platform | v0.4.3</div>

  <div id="toast" class="toast"></div>

  <!-- ===== STATUS CARD ===== -->
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <div style="text-align:left">
        <p style="margin:2px">Seria: <b id="shots">0</b> strzałów</p>
        <p style="margin:2px;font-size:12px;color:#888" id="env">Pogoda: --</p>
      </div>
      <button class="btn-small" onclick="getWeather()">📍 Pogoda</button>
    </div>
  </div>

  <!-- ===== DIAGNOSTICS CARD ===== -->
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">
      <h3 style="margin:0;border:none;padding:0">🔧 Diagnostyka / Diagnostics</h3>
      <button class="diag-toggle" id="diag_toggle" onclick="toggleDiag()">▼ pokaż</button>
    </div>
    <div id="diag_panel" style="display:none">

      <div class="diag-row">
        <span class="diag-label">IMU (BNO055)</span>
        <span class="diag-value" id="d_bno">--</span>
        <span class="diag-dot dot-gray" id="dot_bno"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Temp lufy / Barrel (DS18B20)</span>
        <span class="diag-value" id="d_temp">--</span>
        <span class="diag-dot dot-gray" id="dot_temp"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Mikrofon / Microphone</span>
        <span class="diag-value" id="d_mic">--</span>
        <span class="diag-dot dot-gray" id="dot_mic"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Napięcie / Voltage</span>
        <span class="diag-value" id="d_vbat">--</span>
        <span class="diag-dot dot-gray" id="dot_vbat"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">WiFi RSSI</span>
        <span class="diag-value" id="d_rssi">--</span>
        <span class="diag-dot dot-gray" id="dot_rssi"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Free Heap</span>
        <span class="diag-value" id="d_heap">--</span>
        <span class="diag-dot dot-gray" id="dot_heap"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Uptime</span>
        <span class="diag-value" id="d_uptime">--</span>
        <span class="diag-dot dot-green" id="dot_uptime"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Strzały w sesji / Session shots</span>
        <span class="diag-value" id="d_shots">0</span>
        <span class="diag-dot dot-green"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Serwo Spust / Trigger</span>
        <span class="diag-value" id="d_servo_s">--</span>
        <span class="diag-dot dot-green"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Serwo Śruba / Screw</span>
        <span class="diag-value" id="d_servo_p">--</span>
        <span class="diag-dot dot-green"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Moc / Power</span>
        <span class="diag-value" id="d_power">--</span>
        <span class="diag-dot dot-green"></span>
      </div>

      <div class="diag-row">
        <span class="diag-label">Stan systemu / System state</span>
        <span class="diag-value" id="d_state">--</span>
        <span class="diag-dot dot-green" id="dot_state"></span>
      </div>
    </div>
  </div>

  <!-- ===== CALIBRATION WIZARD CARD ===== -->
  <div class="card">
    <h3>⚙️ Kalibracja Sprzętowa</h3>

    <div style="text-align:left;border-bottom:1px solid #333;padding-bottom:10px;margin-bottom:10px">
      <div class="input-group">
        <span style="color:#00ff88;font-weight:bold">Spust (Trigger):</span>
        <b id="s_val">0°</b>
      </div>
      <div id="wizard_spust">
        <input type="range" id="s_live" min="0" max="180" value="0" oninput="moveLive('spust',this.value)">
        <p style="font-size:12px;color:#aaa;margin:5px 0">
          [EN] 1. Slide until the marker fires → SAVE FIRE. 2. Slide back to slack → SAVE REST.<br>
          [PL] 1. Przesuwaj aż nastąpi strzał → ZAPISZ STRZAŁ. 2. Cofaj do luzu → ZAPISZ LUZ.
        </p>
        <div style="display:flex;gap:5px">
          <button class="btn-small btn-red" style="flex:1" onclick="zapiszKrok('spust','max')">
            🎯 STRZAŁ / FIRE <span id="check_spust_max"></span>
          </button>
          <button class="btn-small btn-green" style="flex:1" onclick="zapiszKrok('spust','min')">
            😌 LUZ / REST <span id="check_spust_min"></span>
          </button>
        </div>
      </div>
      <button id="btn_rekal_spust" class="btn-small" style="display:none;width:100%;background:#555;margin-top:5px" onclick="pokazWizard('spust')">
        🔄 Rekalibruj Spust / Recalibrate Trigger
      </button>
    </div>

    <div style="text-align:left">
      <div class="input-group">
        <span style="color:#00ff88;font-weight:bold">Śruba N2 (Power Screw):</span>
        <b id="p_val">90°</b>
      </div>
      <div id="wizard_sruba">
        <input type="range" id="p_live" min="0" max="180" value="90" oninput="moveLive('sruba',this.value)">
        <p style="font-size:12px;color:#aaa;margin:5px 0">
          [EN] 1. Mount servo at closed position → SAVE ZERO. 2. Slide to max power → SAVE MAX.<br>
          [PL] 1. Załóż serwo przy zamkniętej → ZAPISZ ZERO. 2. Przesuń na max → ZAPISZ MAX.
        </p>
        <div style="display:flex;gap:5px">
          <button class="btn-small" style="flex:1;background:#888" onclick="zapiszKrok('sruba','min')">
            0️⃣ ZERO <span id="check_sruba_min"></span>
          </button>
          <button class="btn-small btn-orange" style="flex:1" onclick="zapiszKrok('sruba','max')">
            🔥 MAX <span id="check_sruba_max"></span>
          </button>
        </div>
      </div>
      <button id="btn_rekal_sruba" class="btn-small" style="display:none;width:100%;background:#555;margin-top:5px" onclick="pokazWizard('sruba')">
        🔄 Rekalibruj Śrubę / Recalibrate Screw
      </button>
    </div>
  </div>

  <!-- ===== FIRE CONTROL CARD ===== -->
  <div class="card">
    <h3>🎯 Kontrola Ognia / Fire Control</h3>

    <div class="input-group">
      <label>Moc / Power (0-100%):</label>
      <div style="display:flex;align-items:center">
        <input type="range" id="p_power" min="0" max="100" value="50" style="width:100px" onchange="setPower(this.value)">
        <span id="power_pct" style="margin-left:5px;width:35px;text-align:right">50%</span>
      </div>
    </div>

    <div class="input-group">
      <label>Tryb / Mode:</label>
      <select id="shot_mode">
        <option value="1">Pojedynczy / Single (1)</option>
        <option value="3">Seria / Burst (3)</option>
        <option value="5">Seria / Burst (5)</option>
      </select>
    </div>

    <div class="input-group">
      <label>Rodzaj kulki / Ammo type:</label>
      <select id="p_type">
        <option value="szorstka">Szorstka / Rough</option>
        <option value="gładka">Gładka / Smooth</option>
      </select>
    </div>

    <div class="input-group">
      <label>Waga / Weight [g]:</label>
      <div style="display:flex;gap:5px">
        <select id="p_weight_sel" onchange="checkCustomWeight()" style="width:70px">
          <option value="3.0">3.0g</option>
          <option value="3.2" selected>3.2g</option>
          <option value="5.0">5.0g</option>
          <option value="9.0">9.0g</option>
          <option value="inna">Inna / Other</option>
        </select>
        <input type="number" id="p_weight_custom" style="display:none;width:60px" placeholder="np 4.5" step="0.1">
      </div>
    </div>

    <div class="input-group">
      <label>Dystans / Distance [m]:</label>
      <select id="p_dist">
        <option value="0">0m</option>
        <option value="5" selected>5m</option>
        <option value="10">10m</option>
        <option value="15">15m</option>
        <option value="20">20m</option>
        <option value="25">25m</option>
        <option value="50">50m</option>
      </select>
    </div>

    <div class="input-group">
      <label>Odstęp / Cadence [ms]:</label>
      <input type="number" id="p_cadence" value="1000" step="100" style="width:80px">
    </div>

    <div class="input-group" style="margin-top:10px;background:#333;padding:10px;border-radius:5px">
      <label style="color:#00ff88;font-weight:bold">Wymagaj V / Require V:</label>
      <input type="checkbox" id="p_req_v" style="width:20px;height:20px">
    </div>

    <button id="btn_action" class="btn-fire btn-green" onclick="toggleAction()">▶ START</button>
    <button class="btn-cancel" onclick="stopAction()">⏹ STOP / CANCEL</button>
  </div>

  <!-- ===== VELOCITY INPUT CARD ===== -->
  <div class="card">
    <h3>Dopisywanie Prędkości / Velocity Input (V)</h3>
    <div class="input-group" style="justify-content:center;gap:10px">
      <input type="number" id="v_id" placeholder="ID" style="width:30%">
      <input type="number" id="v_val" placeholder="V [m/s]" step="0.1" style="width:35%">
      <button class="btn-small" onclick="saveV()">✍️ Zapisz</button>
    </div>
    <br>
    <button class="btn-fire btn-green" style="font-size:16px;padding:15px" onclick="location.href='/logs.csv'">
      📥 Pobierz Logi CSV / Download Logs
    </button>
  </div>

  <span id="sys_state" style="display:none">IDLE</span>

  <script>
    /* ============== TOAST ============== */
    function showToast(msg, isError) {
      var t = document.getElementById('toast');
      t.innerText = msg;
      t.className = 'toast show' + (isError ? ' error' : '');
      setTimeout(function(){ t.className = 'toast'; }, 2000);
    }

    /* ============== DIAGNOSTICS TOGGLE ============== */
    var diagOpen = false;
    function toggleDiag() {
      diagOpen = !diagOpen;
      document.getElementById('diag_panel').style.display = diagOpen ? 'block' : 'none';
      document.getElementById('diag_toggle').innerText = diagOpen ? '▲ ukryj' : '▼ pokaż';
      if (diagOpen) fetchDiag();
    }

    /*
     * [EN] Fetch diagnostics from ESP32 and update traffic-light indicators.
     *   Color logic:
     *     BNO055: green=calibrated, yellow=low calibration, red=not detected
     *     DS18B20: green=valid temp, yellow=85°C (reset), red=-127 (disconnected)
     *     Mic: green=ADC reading in range, yellow=clipping, red=stuck at 0
     *     Voltage: green=>4.5V, yellow=4.0-4.5V, red=<4.0V, gray=disabled
     *     RSSI: green>-60, yellow>-80, red<-80
     *     Heap: green>50kB, yellow>20kB, red<20kB
     * [PL] Pobiera diagnostykę z ESP32 i aktualizuje sygnalizację świetlną.
     */
    function fetchDiag() {
      fetch('/api/diagnostics').then(function(r){ return r.json(); }).then(function(d) {
        // BNO055
        var bnoEl = document.getElementById('d_bno');
        var bnoD = document.getElementById('dot_bno');
        if (!d.bno_ok) {
          bnoEl.innerText = 'BRAK / NOT FOUND'; bnoD.className = 'diag-dot dot-red';
        } else if (d.bno_sys_cal < 2) {
          bnoEl.innerText = 'Kal: ' + d.bno_sys_cal + '/3'; bnoD.className = 'diag-dot dot-yellow';
        } else {
          bnoEl.innerText = 'OK (Kal: ' + d.bno_sys_cal + '/3)'; bnoD.className = 'diag-dot dot-green';
        }

        // DS18B20
        var tmpEl = document.getElementById('d_temp');
        var tmpD = document.getElementById('dot_temp');
        if (d.barrel_temp <= -126) {
          tmpEl.innerText = 'ROZŁĄCZONY / DISCONN'; tmpD.className = 'diag-dot dot-red';
        } else if (d.barrel_temp >= 84.5 && d.barrel_temp <= 85.5) {
          tmpEl.innerText = d.barrel_temp.toFixed(1) + '°C (reset?)'; tmpD.className = 'diag-dot dot-yellow';
        } else {
          tmpEl.innerText = d.barrel_temp.toFixed(1) + '°C'; tmpD.className = 'diag-dot dot-green';
        }

        // Microphone
        var micEl = document.getElementById('d_mic');
        var micD = document.getElementById('dot_mic');
        if (d.mic_raw <= 5) {
          micEl.innerText = 'BRAK SYGNAŁU / NO SIGNAL'; micD.className = 'diag-dot dot-red';
        } else if (d.mic_raw >= 4090) {
          micEl.innerText = 'CLIPPING (' + d.mic_raw + ')'; micD.className = 'diag-dot dot-yellow';
        } else {
          micEl.innerText = 'OK (' + d.mic_raw + ')'; micD.className = 'diag-dot dot-green';
        }

        // Voltage
        var vEl = document.getElementById('d_vbat');
        var vD = document.getElementById('dot_vbat');
        if (!d.vbat_enabled) {
          vEl.innerText = 'N/A'; vD.className = 'diag-dot dot-gray';
        } else if (d.vbat < 4.0) {
          vEl.innerText = d.vbat.toFixed(2) + 'V ⚠️'; vD.className = 'diag-dot dot-red';
        } else if (d.vbat < 4.5) {
          vEl.innerText = d.vbat.toFixed(2) + 'V'; vD.className = 'diag-dot dot-yellow';
        } else {
          vEl.innerText = d.vbat.toFixed(2) + 'V'; vD.className = 'diag-dot dot-green';
        }

        // WiFi RSSI
        var rssiEl = document.getElementById('d_rssi');
        var rssiD = document.getElementById('dot_rssi');
        rssiEl.innerText = d.rssi + ' dBm';
        if (d.rssi > -60) rssiD.className = 'diag-dot dot-green';
        else if (d.rssi > -80) rssiD.className = 'diag-dot dot-yellow';
        else rssiD.className = 'diag-dot dot-red';

        // Free Heap
        var heapEl = document.getElementById('d_heap');
        var heapD = document.getElementById('dot_heap');
        var heapKB = (d.free_heap / 1024).toFixed(1);
        heapEl.innerText = heapKB + ' kB';
        if (d.free_heap > 50000) heapD.className = 'diag-dot dot-green';
        else if (d.free_heap > 20000) heapD.className = 'diag-dot dot-yellow';
        else heapD.className = 'diag-dot dot-red';

        // Uptime
        var sec = Math.floor(d.uptime_ms / 1000);
        var h = Math.floor(sec / 3600); var m = Math.floor((sec % 3600) / 60); var s = sec % 60;
        document.getElementById('d_uptime').innerText = h + 'h ' + m + 'm ' + s + 's';

        // Session shots, servos, power, state
        document.getElementById('d_shots').innerText = d.total_shots;
        document.getElementById('d_servo_s').innerText = 'min:' + d.cal_spust_min + '° max:' + d.cal_spust_max + '°';
        document.getElementById('d_servo_p').innerText = 'min:' + d.cal_sruba_min + '° max:' + d.cal_sruba_max + '°';
        document.getElementById('d_power').innerText = d.power_pct + '%';
        document.getElementById('d_state').innerText = d.state;

        var stD = document.getElementById('dot_state');
        if (d.state === 'IDLE') stD.className = 'diag-dot dot-green';
        else if (d.state === 'PAUSED' || d.state === 'WAITING_V') stD.className = 'diag-dot dot-yellow';
        else stD.className = 'diag-dot dot-red';
      }).catch(function(){ });
    }

    /* [EN] Auto-refresh diagnostics every 3s while panel is open / [PL] Auto-odświeżanie co 3s gdy panel otwarty */
    setInterval(function(){ if (diagOpen) fetchDiag(); }, 3000);

    /* ============== PAGE LOAD — restore calibration + power ============== */
    window.addEventListener('load', function() {
      fetch('/api/get_cal').then(function(r){ return r.json(); }).then(function(cal) {
        if (cal.calibrated) {
          document.getElementById('wizard_spust').style.display = 'none';
          document.getElementById('btn_rekal_spust').style.display = 'block';
          document.getElementById('wizard_sruba').style.display = 'none';
          document.getElementById('btn_rekal_sruba').style.display = 'block';
          showToast('Kalibracja wczytana z pamięci ✓', false);
        }
        // [EN] Restore power slider to saved value / [PL] Przywróć suwak mocy do zapisanej wartości
        if (typeof cal.power_pct !== 'undefined') {
          document.getElementById('p_power').value = cal.power_pct;
          document.getElementById('power_pct').innerText = cal.power_pct + '%';
        }
      }).catch(function(){});
    });

    setInterval(updateStatus, 1000);

    var wizState = { spust: {min:false, max:false}, sruba: {min:false, max:false} };

    function checkCustomWeight() {
      var sel = document.getElementById('p_weight_sel').value;
      document.getElementById('p_weight_custom').style.display = (sel === 'inna') ? 'block' : 'none';
    }

    function moveLive(target, val) {
      document.getElementById(target === 'spust' ? 's_val' : 'p_val').innerText = val + '°';
      fetch('/api/live', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'target='+target+'&val='+val});
    }

    function zapiszKrok(target, mode) {
      var slider = document.getElementById(target === 'spust' ? 's_live' : 'p_live');
      var val = slider.value;
      fetch('/api/set_cal', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'target='+target+'&mode='+mode+'&val='+val
      }).then(function(r) {
        if (r.ok) {
          document.getElementById('check_'+target+'_'+mode).innerHTML = '<span class="step-done">✓</span>';
          wizState[target][mode] = true;
          var labelMap = {spust_min:'LUZ/REST',spust_max:'STRZAŁ/FIRE',sruba_min:'ZERO',sruba_max:'MAX'};
          showToast(labelMap[target+'_'+mode]+' = '+val+'° ✓', false);
          if (wizState[target].min && wizState[target].max) {
            setTimeout(function(){
              document.getElementById('wizard_'+target).style.display = 'none';
              document.getElementById('btn_rekal_'+target).style.display = 'block';
              showToast('Kalibracja '+target+' zakończona! ✓', false);
            }, 800);
          }
        } else { showToast('Błąd zapisu! / Save error!', true); }
      });
    }

    function pokazWizard(target) {
      document.getElementById('wizard_'+target).style.display = 'block';
      document.getElementById('btn_rekal_'+target).style.display = 'none';
      wizState[target] = { min: false, max: false };
      document.getElementById('check_'+target+'_min').innerHTML = '';
      document.getElementById('check_'+target+'_max').innerHTML = '';
    }

    /*
     * [EN] Power slider — saves to flash so it persists across reboots
     * [PL] Suwak mocy — zapisuje do flasha, przeżywa restart
     */
    function setPower(val) {
      document.getElementById('power_pct').innerText = val + '%';
      fetch('/api/power', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'val='+val});
    }

    function getWeather() {
      document.getElementById('env').innerText = 'Szukam GPS / Searching GPS...';
      if (!navigator.geolocation) { showToast('Brak GPS / No GPS', true); return; }
      navigator.geolocation.getCurrentPosition(function(pos) {
        document.getElementById('env').innerText = 'Łączenie z API...';
        fetch('/api/weather', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
          body:'lat='+pos.coords.latitude+'&lon='+pos.coords.longitude
        }).then(function(r){ return r.text(); }).then(function(t){
          showToast(t, false); updateStatus();
        });
      }, function(err){ showToast('GPS Error: '+err.message, true); });
    }

    function updateStatus() {
      fetch('/api/status').then(function(r){ return r.json(); }).then(function(d) {
        document.getElementById('shots').innerText = d.totalShots;
        if (d.envPress > 0) {
          document.getElementById('env').innerText = d.envTemp.toFixed(1)+'°C, '+d.envPress.toFixed(0)+'hPa';
        }
        var state = d.state;
        document.getElementById('sys_state').innerText = state;
        var btn = document.getElementById('btn_action');
        if (state === 'IDLE') {
          btn.className = 'btn-fire btn-green'; btn.innerText = '▶ START';
        } else if (state === 'PAUSED') {
          btn.className = 'btn-fire btn-orange'; btn.innerText = '▶ WZNÓW / RESUME';
        } else if (state === 'WAITING_V') {
          btn.className = 'btn-fire'; btn.style.backgroundColor = '#007bff';
          btn.innerText = '⏳ WPISZ V / ENTER V';
          document.getElementById('v_id').value = d.totalShots;
        } else {
          btn.className = 'btn-fire btn-red'; btn.style.backgroundColor = '';
          btn.innerText = '⏸ PAUZA / PAUSE';
        }
      });
    }

    function toggleAction() {
      var state = document.getElementById('sys_state').innerText;
      if (state === 'IDLE') {
        var selW = document.getElementById('p_weight_sel').value;
        var w = (selW === 'inna') ? document.getElementById('p_weight_custom').value : selW;
        if (!w) w = '3.2';
        var params = 'type='+encodeURIComponent(document.getElementById('p_type').value)
          +'&weight='+w+'&dist='+document.getElementById('p_dist').value
          +'&cadence='+document.getElementById('p_cadence').value
          +'&reqV='+(document.getElementById('p_req_v').checked?'1':'0');
        fetch('/api/params', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:params})
          .then(function(){ return fetch('/api/control', {method:'POST',
            headers:{'Content-Type':'application/x-www-form-urlencoded'},
            body:'cmd=start&count='+document.getElementById('shot_mode').value}); });
      } else if (state === 'PAUSED') {
        fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'cmd=resume'});
      } else if (state === 'WAITING_V') {
        saveV();
      } else {
        fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'cmd=pause'});
      }
      setTimeout(updateStatus, 200);
    }

    function stopAction() {
      fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'cmd=stop'});
      setTimeout(updateStatus, 200);
    }

    function saveV() {
      var id = document.getElementById('v_id').value;
      var val = document.getElementById('v_val').value;
      if (!id || !val) { showToast('Wypełnij ID i V! / Fill ID and V!', true); return; }
      fetch('/api/save_v', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'id='+id+'&v='+val
      }).then(function(r) {
        if (r.ok) {
          showToast('V zapisane! / V saved! (ID:'+id+')', false);
          document.getElementById('v_id').value = '';
          document.getElementById('v_val').value = '';
        }
      });
    }
  </script>
</body>
</html>
)rawliteral";

/*
 * =======================================================================================
 * [EN] WEB SERVER — ENDPOINTS
 *   POST for state-changing ops, GET for read-only.
 * [PL] WEB SERVER — ENDPOINTY
 *   POST dla operacji zmieniających stan, GET dla tylko-odczyt.
 * =======================================================================================
 */
void setupWebEndpoints() {

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

  /* [EN] GET /api/status — system state for GUI polling / [PL] Stan systemu dla pollingu */
  server.on("/api/status", HTTP_GET, []() {
    StaticJsonDocument<256> doc;
    doc["envTemp"]    = currentEnvTemp;
    doc["envHum"]     = currentEnvHum;
    doc["envPress"]   = currentEnvPress;
    doc["totalShots"] = totalShotsFired;
    if      (sysState == SYS_IDLE)          doc["state"] = "IDLE";
    else if (sysState == SYS_PAUSED)        doc["state"] = "PAUSED";
    else if (sysState == SYS_WAITING_FOR_V) doc["state"] = "WAITING_V";
    else                                    doc["state"] = "RUNNING";
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  /*
   * [EN] GET /api/get_cal — calibration data + power level for page load restore.
   * [PL] GET /api/get_cal — dane kalibracji + poziom mocy do przywrócenia przy ładowaniu.
   */
  server.on("/api/get_cal", HTTP_GET, []() {
    StaticJsonDocument<256> doc;
    doc["calibrated"] = isCalibrated();
    doc["spust_min"]  = cal_spust_min;
    doc["spust_max"]  = cal_spust_max;
    doc["sruba_min"]  = cal_sruba_min;
    doc["sruba_max"]  = cal_sruba_max;
    doc["power_pct"]  = cal_power_pct;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  /*
   * ====================================================================================
   * [EN] GET /api/diagnostics — Full hardware health report for the Diagnostics panel.
   *   Returns sensor status, ESP32 internals, calibration values, and session data.
   *   This is a read-only snapshot — no state is modified.
   * [PL] GET /api/diagnostics — Pełny raport zdrowia sprzętu dla panelu diagnostycznego.
   *   Zwraca status czujników, stan ESP32, wartości kalibracji i dane sesji.
   *   Tylko-odczyt — żaden stan nie jest modyfikowany.
   * ====================================================================================
   */
  server.on("/api/diagnostics", HTTP_GET, []() {
    StaticJsonDocument<512> doc;

    // [EN] BNO055 status / [PL] Status BNO055
    doc["bno_ok"] = bnoOK;
    if (bnoOK) {
      uint8_t sysCal, gyroCal, accelCal, magCal;
      bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
      doc["bno_sys_cal"]   = sysCal;   // 0-3, 3=fully calibrated
      doc["bno_gyro_cal"]  = gyroCal;
      doc["bno_accel_cal"] = accelCal;
      doc["bno_mag_cal"]   = magCal;
    } else {
      doc["bno_sys_cal"] = 0;
    }

    // [EN] DS18B20 barrel temperature / [PL] Temperatura lufy DS18B20
    doc["barrel_temp"] = lastBarrelTemp;

    // [EN] Microphone — single ADC read for health check / [PL] Mikrofon — odczyt ADC do health checku
    doc["mic_raw"] = analogRead(MIC_PIN);

    // [EN] Battery voltage / [PL] Napięcie baterii
    doc["vbat_enabled"] = VBAT_ENABLED;
    doc["vbat"] = readBatteryVoltage();

    // [EN] ESP32 internals / [PL] Stan wewnętrzny ESP32
    doc["rssi"]      = WiFi.RSSI();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["uptime_ms"] = millis();

    // [EN] Session and calibration data / [PL] Dane sesji i kalibracji
    doc["total_shots"]   = totalShotsFired;
    doc["cal_spust_min"] = cal_spust_min;
    doc["cal_spust_max"] = cal_spust_max;
    doc["cal_sruba_min"] = cal_sruba_min;
    doc["cal_sruba_max"] = cal_sruba_max;
    doc["power_pct"]     = cal_power_pct;

    // [EN] System state string / [PL] Stan systemu jako string
    if      (sysState == SYS_IDLE)          doc["state"] = "IDLE";
    else if (sysState == SYS_PAUSED)        doc["state"] = "PAUSED";
    else if (sysState == SYS_WAITING_FOR_V) doc["state"] = "WAITING_V";
    else                                    doc["state"] = "RUNNING";

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  /* [EN] POST /api/live — real-time servo move during calibration / [PL] Ruch serwa na żywo */
  server.on("/api/live", HTTP_POST, []() {
    String t = server.arg("target");
    int v = constrain(server.arg("val").toInt(), 0, 180);
    if (t == "spust") serwoSpust.write(v);
    else              serwoSruba.write(v);
    server.send(200, "text/plain", "OK");
  });

  /* [EN] POST /api/set_cal — save calibration step / [PL] Zapis kroku kalibracji */
  server.on("/api/set_cal", HTTP_POST, []() {
    String t = server.arg("target");
    String m = server.arg("mode");
    int v = constrain(server.arg("val").toInt(), 0, 180);
    if (t == "spust") {
      if (m == "min") cal_spust_min = v; else cal_spust_max = v;
    } else {
      if (m == "min") cal_sruba_min = v; else cal_sruba_max = v;
    }
    saveCalibration();
    server.send(200, "text/plain", "OK");
  });

  /*
   * [EN] POST /api/power — Set N2 screw power 0-100%, mapped to calibrated range.
   *   Now also persists power_pct to flash so it survives reboot.
   * [PL] POST /api/power — Ustaw moc 0-100%, zmapowane na skalibrowany zakres.
   *   Teraz również zapisuje power_pct do flasha — przeżywa restart.
   */
  server.on("/api/power", HTTP_POST, []() {
    int p = constrain(server.arg("val").toInt(), 0, 100);
    cal_power_pct = p;
    serwoSruba.write(map(p, 0, 100, cal_sruba_min, cal_sruba_max));
    saveCalibration(); // [EN] Persist power level / [PL] Zapisz poziom mocy
    server.send(200, "text/plain", "OK");
  });

  /* [EN] POST /api/params — session parameters / [PL] Parametry sesji */
  server.on("/api/params", HTTP_POST, []() {
    if (server.hasArg("type"))    currentAmmoType   = server.arg("type");
    if (server.hasArg("weight"))  currentAmmoWeight = server.arg("weight").toFloat();
    if (server.hasArg("dist"))    currentDistance    = server.arg("dist").toFloat();
    if (server.hasArg("cadence")) currentCadenceMs   = server.arg("cadence").toInt();
    if (server.hasArg("reqV"))    requireVInput      = (server.arg("reqV") == "1");
    server.send(200, "text/plain", "OK");
  });

  /*
   * [EN] POST /api/control — start, pause, resume, stop.
   *   Added guard: cmd=start ignored if not in IDLE (prevents double-start).
   * [PL] POST /api/control — start, pause, resume, stop.
   *   Dodano guard: cmd=start ignorowany jeśli nie w IDLE (zapobiega podwójnemu startowi).
   */
  server.on("/api/control", HTTP_POST, []() {
    if (!server.hasArg("cmd")) {
      server.send(400, "text/plain", "Missing cmd / Brak komendy");
      return;
    }
    String cmd = server.arg("cmd");
    if (cmd == "start") {
      if (sysState != SYS_IDLE) {
        server.send(409, "text/plain", "Already running / Już działa"); // [EN] 409 Conflict / [PL] 409 Konflikt
        return;
      }
      shotsRemaining = server.hasArg("count") ? server.arg("count").toInt() : 1;
      pauseRequested = false;
      sysState = SYS_SHOOTING_PRESS;
    } else if (cmd == "pause") {
      pauseRequested = true;
    } else if (cmd == "resume") {
      pauseRequested = false;
      if (sysState == SYS_PAUSED) {
        stateStartTime = millis();
        sysState = SYS_WAITING_CADENCE;
      }
    } else if (cmd == "stop") {
      shotsRemaining = 0;
      pauseRequested = false;
      sysState = SYS_IDLE;
      isShooting = false;
    }
    server.send(200, "text/plain", "OK");
  });

  /* [EN] POST /api/save_v — velocity from chronograph / [PL] Prędkość z chronografu */
  server.on("/api/save_v", HTTP_POST, []() {
    if (!server.hasArg("id") || !server.hasArg("v")) {
      server.send(400, "text/plain", "Missing ID or V / Brak ID lub V");
      return;
    }
    updateVelocityInLog(server.arg("id").toInt(), server.arg("v").toFloat());
    if (sysState == SYS_WAITING_FOR_V) {
      if (shotsRemaining > 0) { stateStartTime = millis(); sysState = SYS_WAITING_CADENCE; }
      else sysState = SYS_IDLE;
    }
    server.send(200, "text/plain", "OK");
  });

  /* [EN] POST /api/weather — fetch from OpenWeatherMap / [PL] Pobierz pogodę */
  server.on("/api/weather", HTTP_POST, []() {
    if (!server.hasArg("lat") || !server.hasArg("lon")) {
      server.send(400, "text/plain", "Missing GPS / Brak GPS");
      return;
    }
    HTTPClient http;
    String url = "https://api.openweathermap.org/data/2.5/weather?lat=" + server.arg("lat")
               + "&lon=" + server.arg("lon") + "&units=metric&appid=" + OPENWEATHER_API_KEY;
    http.begin(url);
    if (http.GET() == HTTP_CODE_OK) {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, http.getString());
      currentEnvTemp  = doc["main"]["temp"];
      currentEnvHum   = doc["main"]["humidity"];
      currentEnvPress = doc["main"]["pressure"];
      server.send(200, "text/plain", "Pogoda OK! / Weather updated!");
    } else {
      server.send(500, "text/plain", "API Error / Błąd API");
    }
    http.end();
  });

  /* [EN] GET /logs.csv — download CSV / [PL] Pobierz CSV */
  server.on("/logs.csv", HTTP_GET, []() {
    File f = LittleFS.open("/logs.csv", "r");
    if (!f) { server.send(404, "text/plain", "Log file empty / Plik logów pusty"); return; }
    server.streamFile(f, "text/csv");
    f.close();
  });
}

/*
 * =======================================================================================
 * [EN] NETWORK TASK (CORE 0) — WiFi + WebServer isolated from hardware timing.
 * [PL] ZADANIE SIECIOWE (CORE 0) — WiFi + WebServer izolowane od timerów sprzętowych.
 * =======================================================================================
 */
void taskNetworkCore0(void * pvParameters) {
  setupWebEndpoints();
  server.begin();
  for (;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

/*
 * =======================================================================================
 * [EN] SETUP / [PL] INICJALIZACJA
 * =======================================================================================
 */
void setup() {
  Serial.begin(115200);

  if (!LittleFS.begin(true)) {
    Serial.println("[ERROR] LittleFS mount failed!");
    return;
  }

  if (!LittleFS.exists("/logs.csv")) {
    File f = LittleFS.open("/logs.csv", FILE_WRITE);
    f.println("ID,Timestamp,Typ_Kulki,Waga_Kulki,Odleglosc,Temp_Otoczenia,Wilgotnosc,Cisnienie,Temp_Lufy_500ms,Max_Odrzut_G,Peak_dB,Sredni_dB,V_ms");
    f.close();
  }

  // [EN] Load calibration + saved power level / [PL] Wczytaj kalibrację + zapisany poziom mocy
  loadCalibration();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  serwoSpust.setPeriodHertz(50);
  serwoSruba.setPeriodHertz(50);
  serwoSpust.attach(SERVO_SPUST_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  serwoSruba.attach(SERVO_SRUBA_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  serwoSpust.write(cal_spust_min);
  // [EN] Restore power to saved level (not hardcoded 50%) / [PL] Przywróć moc do zapisanego poziomu (nie hardkodowane 50%)
  serwoSruba.write(map(cal_power_pct, 0, 100, cal_sruba_min, cal_sruba_max));

  sensorsBarrel.begin();
  sensorsBarrel.setResolution(9);
  sensorsBarrel.setWaitForConversion(false);

  // [EN] BNO055 init with health check / [PL] Inicjalizacja BNO055 ze sprawdzeniem zdrowia
  bnoOK = bno.begin();
  if (!bnoOK) {
    Serial.println("[WARN] BNO055 not detected! Check wiring / I2C address.");
  }

  pinMode(MIC_PIN, INPUT);
  if (VBAT_ENABLED) pinMode(VBAT_PIN, INPUT);

  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());

  MDNS.begin("sentry");

  /*
   * [EN] Dual-core task distribution:
   *   Core 0: WiFi + WebServer — isolated from hardware timing
   *   Core 1: loop() state machine + taskHighSpeedMeasurements (priority 1 both)
   * [PL] Rozkład na rdzenie:
   *   Core 0: WiFi + WebServer — izolowane od timerów
   *   Core 1: loop() maszyna stanów + taskHighSpeedMeasurements (priorytet 1 oba)
   */
  xTaskCreatePinnedToCore(taskNetworkCore0, "TaskNet0", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskHighSpeedMeasurements, "TaskMeas1", 4096, NULL, 1, NULL, 1);

  Serial.println("[OK] Sentry Data Scout v0.4.3 ready → http://sentry.local");
  Serial.println("[DIAG] BNO055: " + String(bnoOK ? "OK" : "FAIL") +
                 " | Power: " + String(cal_power_pct) + "%" +
                 " | VBAT: " + String(VBAT_ENABLED ? "ON" : "OFF"));
}

/*
 * =======================================================================================
 * [EN] MAIN LOOP (CORE 1) — Non-blocking state machine
 * [PL] PĘTLA GŁÓWNA (CORE 1) — Nieblokująca maszyna stanów
 * =======================================================================================
 */
void loop() {
  unsigned long now = millis();

  /*
   * [EN] Background temperature polling + diagnostics cache update.
   *   Every 2s when idle/paused: request temp and cache it for diagnostics.
   * [PL] Odczyt temperatury w tle + aktualizacja cache diagnostycznego.
   *   Co 2s w IDLE/PAUSED: zlecenie temp i cache dla diagnostyki.
   */
  static unsigned long lastSensorTime = 0;
  if (now - lastSensorTime > 2000 && (sysState == SYS_IDLE || sysState == SYS_PAUSED)) {
    sensorsBarrel.requestTemperatures();
    lastSensorTime = now;
  }

  /*
   * [EN] Update barrel temp cache for diagnostics (non-blocking read of last conversion).
   *   getTempCByIndex returns the result of the last requestTemperatures() call.
   *   -127.0 means no device on bus, 85.0 means conversion not ready or power-on reset.
   * [PL] Aktualizacja cache temp lufy dla diagnostyki (nieblokujący odczyt ostatniej konwersji).
   *   -127.0 = brak czujnika, 85.0 = konwersja niegotowa lub reset zasilania.
   */
  static unsigned long lastTempRead = 0;
  if (now - lastTempRead > 2500) {
    lastBarrelTemp = sensorsBarrel.getTempCByIndex(0);
    lastTempRead = now;
  }

  switch (sysState) {
    case SYS_IDLE:
      break;

    case SYS_SHOOTING_PRESS:
      isShooting = true;
      serwoSpust.write(cal_spust_max);
      sensorsBarrel.requestTemperatures();
      stateStartTime = now;
      sysState = SYS_SHOOTING_RELEASE;
      break;

    case SYS_SHOOTING_RELEASE:
      /*
       * [EN] 60ms trigger hold — increased from 50ms (v0.3) after field tests.
       * [PL] 60ms trzymanie spustu — zwiększone z 50ms (v0.3) po testach terenowych.
       */
      if (now - stateStartTime >= 60) {
        serwoSpust.write(cal_spust_min);
        sysState = SYS_SHOOTING_WAIT_500;
      }
      break;

    case SYS_SHOOTING_WAIT_500:
      if (now - stateStartTime >= 500) {
        isShooting = false;
        totalShotsFired++;
        float tempL = sensorsBarrel.getTempCByIndex(0);
        lastBarrelTemp = tempL; // [EN] Update diag cache / [PL] Aktualizuj cache diagnostyczny
        appendLogToCSV(totalShotsFired, currentAmmoType, currentAmmoWeight,
                       currentDistance, tempL, sharedMaxG, sharedPeakDB, sharedAvgDB);
        shotsRemaining--;

        if (requireVInput) {
          sysState = SYS_WAITING_FOR_V;
        } else if (shotsRemaining > 0) {
          stateStartTime = millis();
          sysState = pauseRequested ? SYS_PAUSED : SYS_WAITING_CADENCE;
        } else {
          sysState = SYS_IDLE;
          pauseRequested = false;
        }
      }
      break;

    case SYS_WAITING_CADENCE:
      if (pauseRequested) {
        sysState = SYS_PAUSED;
      } else if (now - stateStartTime >= (unsigned long)currentCadenceMs) {
        sysState = SYS_SHOOTING_PRESS;
      }
      break;

    case SYS_PAUSED:
    case SYS_WAITING_FOR_V:
      break;
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);
}
