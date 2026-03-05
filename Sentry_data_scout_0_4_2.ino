/**
 * =======================================================================================
 * Project: Sentry Data Scout
 * Author: Maciej Kasperek (vonKrappitz)
 * Version: 0.4.2
 * [EN] Description: Spyder MR3 marker controller — Web Server + LittleFS + State Machine
 *      Calibration Wizard with persistent state, mDNS, dual-core RTOS architecture.
 * [PL] Opis: Kontroler markera Spyder MR3 — Web Serwer + LittleFS + Maszyna Stanów
 *      Wizard kalibracyjny z trwałym zapisem, mDNS, architektura dwurdzeniowa RTOS.
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
 *   These define the physical servo range for this specific marker installation.
 *   spust_min = rest position (trigger released), spust_max = fire position (trigger pulled)
 *   sruba_min = N2 screw fully closed, sruba_max = N2 screw maximum open
 * [PL] DANE KALIBRACYJNE (zapisywane do LittleFS jako /calibration.json)
 *   Definiują fizyczny zakres serw dla tej konkretnej instalacji markera.
 *   spust_min = pozycja spoczynkowa (spust zwolniony), spust_max = pozycja strzału
 *   sruba_min = śruba N2 zamknięta, sruba_max = śruba N2 max otwarta
 * =======================================================================================
 */
int cal_spust_min = 0, cal_spust_max = 180;
int cal_sruba_min = 0, cal_sruba_max = 180;

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
 * [EN] States for the shooting state machine
 *   Ensures timing precision and non-blocking asynchronous execution.
 *   SYS_IDLE           — waiting for command
 *   SYS_SHOOTING_PRESS — trigger servo engaged, measurement window opens
 *   SYS_SHOOTING_RELEASE — waiting for trigger release (60ms hold time)
 *   SYS_SHOOTING_WAIT_500 — 500ms measurement window for sensors
 *   SYS_WAITING_CADENCE — delay between shots in a burst
 *   SYS_PAUSED         — user-requested pause mid-burst
 *   SYS_WAITING_FOR_V  — waiting for manual velocity input from chronograph
 * [PL] Stany maszyny strzelającej
 *   Zapewnia precyzję czasową i nieblokującą asynchroniczność.
 * =======================================================================================
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
 *   Saved as JSON so values survive reboot. loadCalibration() called in setup().
 * [PL] TRWAŁOŚĆ KALIBRACJI (LittleFS → /calibration.json)
 *   Zapis w JSON, wartości przeżywają restart. loadCalibration() wywoływana w setup().
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
  f.close();
}

/*
 * [EN] Check if calibration file exists and has been modified from defaults.
 *   Used by /api/get_cal so the GUI can skip the wizard on reload.
 * [PL] Sprawdza czy plik kalibracji istnieje i ma zmienione wartości.
 *   Używane przez /api/get_cal żeby GUI mogło pominąć wizarda po restarcie.
 */
bool isCalibrated() {
  return LittleFS.exists("/calibration.json");
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
 *
 *   Priority is set to 1 (same as loop()) to prevent Task Starvation.
 *   Priority 2 would starve loop() and block the 500ms countdown.
 *
 * [PL] ZADANIE SZYBKICH POMIARÓW (CORE 1)
 *   Działa na Core 1 obok loop(). Próbkuje IMU (BNO055) i mikrofon (MAX4466)
 *   z maksymalną prędkością ADC (~10-40 kHz) w oknie pomiarowym 500ms.
 *
 *   KRYTYCZNE DLA RTOS:
 *   yield() oddaje czas procesora innym taskom o priorytecie 1.
 *   Dzięki temu loop() na tym samym rdzeniu może odliczyć 500ms
 *   i zmienić flagę isShooting. Bez yield() — deadlock.
 *
 *   Priorytet 1 (taki sam jak loop()) zapobiega zagłodzeniu tasku (Task Starvation).
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

        yield(); // [EN] CRITICAL — see comment above / [PL] KRYTYCZNE — patrz komentarz wyżej
      }

      /*
       * [EN] Microphone calibration constant (110 dB baseline).
       *   Adjust after testing with a physical decibel meter.
       * [PL] Stała kalibracji mikrofonu (110 dB bazowe).
       *   Dostosuj po teście z fizycznym decybelomierzem.
       */
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
 *   Served from PROGMEM to save heap. Single-page app with:
 *   - Step-by-step calibration wizard (persists across page reloads)
 *   - Fire control panel with power slider mapped to calibrated range
 *   - Velocity input section for chronograph data
 *   - Toast notifications for user feedback
 * [PL] WEB SERVER — INTERFEJS (HTML/CSS/JS)
 *   Serwowany z PROGMEM żeby oszczędzać RAM. SPA z:
 *   - Krokowy wizard kalibracyjny (przeżywa przeładowanie strony)
 *   - Panel kontroli ognia z suwakiem mocy na skalibrowany zakres
 *   - Sekcja wpisywania prędkości z chronografu
 *   - Powiadomienia toast dla feedbacku użytkownika
 * =======================================================================================
 */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Sentry Data Scout v0.4.2</title>
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

    /* Toast notification */
    .toast {
      position:fixed;top:20px;left:50%;transform:translateX(-50%);
      background:#28a745;color:white;padding:12px 24px;border-radius:8px;
      font-size:16px;font-weight:bold;z-index:9999;
      opacity:0;transition:opacity 0.3s;pointer-events:none;
      box-shadow:0 4px 12px rgba(0,0,0,0.5);
    }
    .toast.show {opacity:1}
    .toast.error {background:#dc3545}

    /* Wizard step checkmark */
    .step-done {color:#28a745;font-weight:bold;margin-left:8px}
  </style>
</head>
<body>

  <h2>Sentry Data Scout</h2>
  <div class="author">Maciej Kasperek (von Krappitz) | MR3 Platform | v0.4.2</div>

  <!-- Toast element -->
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

  <!-- ===== CALIBRATION WIZARD CARD ===== -->
  <div class="card">
    <h3>⚙️ Kalibracja Sprzętowa</h3>

    <!-- TRIGGER (Spust) -->
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

    <!-- N2 SCREW (Śruba) -->
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
    /*
     * [EN] Toast notification system — shows green confirmation or red error
     * [PL] System powiadomień toast — zielone potwierdzenie lub czerwony błąd
     */
    function showToast(msg, isError) {
      var t = document.getElementById('toast');
      t.innerText = msg;
      t.className = 'toast show' + (isError ? ' error' : '');
      setTimeout(function(){ t.className = 'toast'; }, 2000);
    }

    /*
     * [EN] On page load: fetch calibration from ESP32 and auto-hide wizard if already calibrated
     * [PL] Przy załadowaniu strony: pobierz kalibrację z ESP32 i auto-ukryj wizarda jeśli skalibrowano
     */
    window.addEventListener('load', function() {
      fetch('/api/get_cal').then(function(r){ return r.json(); }).then(function(cal) {
        if (cal.calibrated) {
          // [EN] Calibration exists — hide wizards, show recalibrate buttons
          // [PL] Kalibracja istnieje — ukryj wizardy, pokaż przyciski rekalibracji
          document.getElementById('wizard_spust').style.display = 'none';
          document.getElementById('btn_rekal_spust').style.display = 'block';
          document.getElementById('wizard_sruba').style.display = 'none';
          document.getElementById('btn_rekal_sruba').style.display = 'block';
          showToast('Kalibracja wczytana z pamięci ✓', false);
        }
      }).catch(function(){});
    });

    setInterval(updateStatus, 1000);

    /* [EN] Calibration wizard state / [PL] Stan wizarda kalibracji */
    var wizState = {
      spust: { min: false, max: false },
      sruba: { min: false, max: false }
    };

    function checkCustomWeight() {
      var sel = document.getElementById('p_weight_sel').value;
      document.getElementById('p_weight_custom').style.display = (sel === 'inna') ? 'block' : 'none';
    }

    /*
     * [EN] Live servo movement for calibration — uses POST
     * [PL] Ruch serwa na żywo do kalibracji — używa POST
     */
    function moveLive(target, val) {
      document.getElementById(target === 'spust' ? 's_val' : 'p_val').innerText = val + '°';
      fetch('/api/live', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'target='+target+'&val='+val});
    }

    /*
     * [EN] Save calibration step with visual feedback (checkmark + toast)
     * [PL] Zapis kroku kalibracji z feedbackiem wizualnym (ptaszek + toast)
     */
    function zapiszKrok(target, mode) {
      var slider = document.getElementById(target === 'spust' ? 's_live' : 'p_live');
      var val = slider.value;
      fetch('/api/set_cal', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'target='+target+'&mode='+mode+'&val='+val
      }).then(function(r) {
        if (r.ok) {
          // [EN] Show checkmark on the button / [PL] Pokaż ptaszka na przycisku
          document.getElementById('check_'+target+'_'+mode).innerHTML = '<span class="step-done">✓</span>';
          wizState[target][mode] = true;

          var labelMap = {spust_min:'LUZ/REST',spust_max:'STRZAŁ/FIRE',sruba_min:'ZERO',sruba_max:'MAX'};
          showToast(labelMap[target+'_'+mode]+' = '+val+'° ✓', false);

          // [EN] If both steps done — hide wizard / [PL] Oba kroki gotowe — ukryj wizarda
          if (wizState[target].min && wizState[target].max) {
            setTimeout(function(){
              document.getElementById('wizard_'+target).style.display = 'none';
              document.getElementById('btn_rekal_'+target).style.display = 'block';
              showToast('Kalibracja '+target+' zakończona! ✓', false);
            }, 800);
          }
        } else {
          showToast('Błąd zapisu! / Save error!', true);
        }
      });
    }

    function pokazWizard(target) {
      document.getElementById('wizard_'+target).style.display = 'block';
      document.getElementById('btn_rekal_'+target).style.display = 'none';
      wizState[target] = { min: false, max: false };
      document.getElementById('check_'+target+'_min').innerHTML = '';
      document.getElementById('check_'+target+'_max').innerHTML = '';
    }

    function setPower(val) {
      document.getElementById('power_pct').innerText = val + '%';
      fetch('/api/power', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'val='+val});
    }

    function getWeather() {
      document.getElementById('env').innerText = 'Szukam GPS / Searching GPS...';
      if (!navigator.geolocation) { showToast('Brak GPS w przeglądarce / No GPS', true); return; }
      navigator.geolocation.getCurrentPosition(function(pos) {
        document.getElementById('env').innerText = 'Łączenie z API... / Connecting...';
        fetch('/api/weather', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
          body:'lat='+pos.coords.latitude+'&lon='+pos.coords.longitude
        }).then(function(r){ return r.text(); }).then(function(t){
          showToast(t, false);
          updateStatus();
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
          +'&weight='+w
          +'&dist='+document.getElementById('p_dist').value
          +'&cadence='+document.getElementById('p_cadence').value
          +'&reqV='+(document.getElementById('p_req_v').checked ? '1' : '0');

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
 *   State-changing operations use HTTP_POST (security: prevents prefetch, keeps params
 *   out of proxy logs and browser history).
 *   Read-only operations use HTTP_GET.
 * [PL] WEB SERVER — ENDPOINTY
 *   Operacje zmieniające stan używają HTTP_POST (bezpieczeństwo: zapobiega prefetchowi,
 *   trzyma parametry z dala od logów proxy i historii przeglądarki).
 *   Operacje tylko-odczyt używają HTTP_GET.
 * =======================================================================================
 */
void setupWebEndpoints() {
  // [EN] Serve main page / [PL] Serwuj stronę główną
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

  /*
   * [EN] GET /api/status — Read-only system state for GUI polling
   * [PL] GET /api/status — Stan systemu tylko-odczyt dla pollingu GUI
   */
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
   * [EN] GET /api/get_cal — Returns current calibration + whether file exists.
   *   Used on page load to decide if wizard should be shown or hidden.
   * [PL] GET /api/get_cal — Zwraca aktualną kalibrację + czy plik istnieje.
   *   Używane przy ładowaniu strony do decyzji: wizard czy przycisk rekalibracji.
   */
  server.on("/api/get_cal", HTTP_GET, []() {
    StaticJsonDocument<256> doc;
    doc["calibrated"]  = isCalibrated();
    doc["spust_min"]   = cal_spust_min;
    doc["spust_max"]   = cal_spust_max;
    doc["sruba_min"]   = cal_sruba_min;
    doc["sruba_max"]   = cal_sruba_max;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  /*
   * [EN] POST /api/live — Move servo in real-time during calibration
   *   Params: target (spust|sruba), val (0-180)
   * [PL] POST /api/live — Ruch serwa w czasie rzeczywistym przy kalibracji
   *   Parametry: target (spust|sruba), val (0-180)
   */
  server.on("/api/live", HTTP_POST, []() {
    String t = server.arg("target");
    int v = constrain(server.arg("val").toInt(), 0, 180);
    if (t == "spust") serwoSpust.write(v);
    else              serwoSruba.write(v);
    server.send(200, "text/plain", "OK");
  });

  /*
   * [EN] POST /api/set_cal — Save calibration endpoint for a specific servo and mode.
   *   Params: target (spust|sruba), mode (min|max), val (0-180)
   *   Persists to /calibration.json via saveCalibration().
   * [PL] POST /api/set_cal — Zapis kalibracji dla konkretnego serwa i trybu.
   *   Parametry: target (spust|sruba), mode (min|max), val (0-180)
   *   Zapisuje do /calibration.json przez saveCalibration().
   */
  server.on("/api/set_cal", HTTP_POST, []() {
    String t = server.arg("target");
    String m = server.arg("mode");
    int v = constrain(server.arg("val").toInt(), 0, 180); // [EN] Guard against out-of-range / [PL] Ochrona przed wartościami spoza zakresu

    if (t == "spust") {
      if (m == "min") cal_spust_min = v; else cal_spust_max = v;
    } else {
      if (m == "min") cal_sruba_min = v; else cal_sruba_max = v;
    }
    saveCalibration();
    server.send(200, "text/plain", "OK");
  });

  /*
   * [EN] POST /api/power — Set N2 screw power as 0-100% mapped to calibrated range.
   *   This is the main "power dial" for the operator.
   * [PL] POST /api/power — Ustaw moc śruby N2 jako 0-100% zmapowane na skalibrowany zakres.
   *   To jest główny "regulator mocy" dla operatora.
   */
  server.on("/api/power", HTTP_POST, []() {
    int p = constrain(server.arg("val").toInt(), 0, 100);
    serwoSruba.write(map(p, 0, 100, cal_sruba_min, cal_sruba_max));
    server.send(200, "text/plain", "OK");
  });

  /*
   * [EN] POST /api/params — Set session parameters before starting a burst.
   *   All params optional — only provided ones are updated.
   * [PL] POST /api/params — Ustaw parametry sesji przed rozpoczęciem serii.
   *   Wszystkie parametry opcjonalne — aktualizowane są tylko podane.
   */
  server.on("/api/params", HTTP_POST, []() {
    if (server.hasArg("type"))    currentAmmoType   = server.arg("type");
    if (server.hasArg("weight"))  currentAmmoWeight = server.arg("weight").toFloat();
    if (server.hasArg("dist"))    currentDistance    = server.arg("dist").toFloat();
    if (server.hasArg("cadence")) currentCadenceMs   = server.arg("cadence").toInt();
    if (server.hasArg("reqV"))    requireVInput      = (server.arg("reqV") == "1");
    server.send(200, "text/plain", "OK");
  });

  /*
   * [EN] POST /api/control — Main command endpoint: start, pause, resume, stop.
   *   cmd=start requires count param (number of shots in burst).
   * [PL] POST /api/control — Główny endpoint komend: start, pause, resume, stop.
   *   cmd=start wymaga parametru count (liczba strzałów w serii).
   */
  server.on("/api/control", HTTP_POST, []() {
    if (!server.hasArg("cmd")) {
      server.send(400, "text/plain", "Missing cmd / Brak komendy");
      return;
    }
    String cmd = server.arg("cmd");

    if (cmd == "start") {
      shotsRemaining = server.hasArg("count") ? server.arg("count").toInt() : 1;
      pauseRequested = false;
      sysState = SYS_SHOOTING_PRESS;
    } else if (cmd == "pause") {
      pauseRequested = true;
    } else if (cmd == "resume") {
      pauseRequested = false;
      if (sysState == SYS_PAUSED) {
        stateStartTime = millis(); // [EN] Reset cadence timer on resume / [PL] Reset timera kadencji przy wznowieniu
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

  /*
   * [EN] POST /api/save_v — Save manually measured velocity from chronograph.
   *   If system was waiting for V input, resumes the burst or goes idle.
   * [PL] POST /api/save_v — Zapisz ręcznie zmierzoną prędkość z chronografu.
   *   Jeśli system czekał na V, wznawia serię lub przechodzi w IDLE.
   */
  server.on("/api/save_v", HTTP_POST, []() {
    if (!server.hasArg("id") || !server.hasArg("v")) {
      server.send(400, "text/plain", "Missing ID or V / Brak ID lub V");
      return;
    }
    updateVelocityInLog(server.arg("id").toInt(), server.arg("v").toFloat());

    if (sysState == SYS_WAITING_FOR_V) {
      if (shotsRemaining > 0) {
        stateStartTime = millis();
        sysState = SYS_WAITING_CADENCE;
      } else {
        sysState = SYS_IDLE;
      }
    }
    server.send(200, "text/plain", "OK");
  });

  /*
   * [EN] POST /api/weather — Fetch weather from OpenWeatherMap using GPS coords from browser.
   * [PL] POST /api/weather — Pobierz pogodę z OpenWeatherMap używając GPS z przeglądarki.
   */
  server.on("/api/weather", HTTP_POST, []() {
    if (!server.hasArg("lat") || !server.hasArg("lon")) {
      server.send(400, "text/plain", "Missing GPS / Brak GPS");
      return;
    }
    float lat = server.arg("lat").toFloat();
    float lon = server.arg("lon").toFloat();
    HTTPClient http;
    String url = "https://api.openweathermap.org/data/2.5/weather?lat=" + String(lat, 4)
               + "&lon=" + String(lon, 4) + "&units=metric&appid=" + OPENWEATHER_API_KEY;
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

  /*
   * [EN] GET /logs.csv — Download shot logs as CSV file.
   * [PL] GET /logs.csv — Pobierz logi strzałów jako plik CSV.
   */
  server.on("/logs.csv", HTTP_GET, []() {
    File f = LittleFS.open("/logs.csv", "r");
    if (!f) {
      server.send(404, "text/plain", "Log file empty / Plik logów pusty");
      return;
    }
    server.streamFile(f, "text/csv");
    f.close();
  });
}

/*
 * =======================================================================================
 * [EN] NETWORK TASK (CORE 0)
 *   Runs the web server on Core 0, completely separated from hardware control on Core 1.
 *   This ensures that WiFi/HTTP latency never blocks servo timing or sensor reads.
 * [PL] ZADANIE SIECIOWE (CORE 0)
 *   Serwer WWW działa na Core 0, całkowicie oddzielony od sprzętu na Core 1.
 *   Dzięki temu opóźnienia WiFi/HTTP nigdy nie blokują timerów serw ani odczytów.
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
 * [EN] SETUP
 * [PL] INICJALIZACJA
 * =======================================================================================
 */
void setup() {
  Serial.begin(115200);

  // [EN] Initialize filesystem / [PL] Inicjalizacja systemu plików
  if (!LittleFS.begin(true)) {
    Serial.println("[ERROR] LittleFS mount failed!");
    return;
  }

  // [EN] Create CSV header if first boot / [PL] Stwórz nagłówek CSV przy pierwszym uruchomieniu
  if (!LittleFS.exists("/logs.csv")) {
    File f = LittleFS.open("/logs.csv", FILE_WRITE);
    f.println("ID,Timestamp,Typ_Kulki,Waga_Kulki,Odleglosc,Temp_Otoczenia,Wilgotnosc,Cisnienie,Temp_Lufy_500ms,Max_Odrzut_G,Peak_dB,Sredni_dB,V_ms");
    f.close();
  }

  // [EN] Load calibration from flash (survives reboot) / [PL] Wczytaj kalibrację z flasha (przeżywa restart)
  loadCalibration();

  /*
   * [EN] PWM timer allocation for ESP32Servo library.
   *   Two timers for two independent servo channels.
   * [PL] Alokacja timerów PWM dla biblioteki ESP32Servo.
   *   Dwa timery dla dwóch niezależnych kanałów serw.
   */
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  serwoSpust.setPeriodHertz(50); // [EN] Standard 50Hz for hobby servos / [PL] Standardowe 50Hz
  serwoSruba.setPeriodHertz(50);
  serwoSpust.attach(SERVO_SPUST_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  serwoSruba.attach(SERVO_SRUBA_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  // [EN] Set servos to calibrated rest positions / [PL] Ustaw serwa na skalibrowane pozycje spoczynkowe
  serwoSpust.write(cal_spust_min);
  serwoSruba.write(map(50, 0, 100, cal_sruba_min, cal_sruba_max)); // [EN] 50% power on boot / [PL] 50% mocy na starcie

  // [EN] Temperature sensor — async mode to avoid blocking / [PL] Czujnik temp — tryb async żeby nie blokować
  sensorsBarrel.begin();
  sensorsBarrel.setResolution(9);
  sensorsBarrel.setWaitForConversion(false); // [EN] CRITICAL: Prevents 94ms loop blocking! / [PL] KRYTYCZNE: Zapobiega blokowaniu pętli o 94ms!

  // [EN] IMU and microphone / [PL] IMU i mikrofon
  bno.begin();
  pinMode(MIC_PIN, INPUT);

  // [EN] WiFi connection / [PL] Łączenie z WiFi
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());

  // [EN] mDNS — accessible as http://sentry.local / [PL] mDNS — dostępny jako http://sentry.local
  MDNS.begin("sentry");

  /*
   * [EN] Dual-core task distribution:
   *   Core 0: WiFi + WebServer (network task) — isolated from hardware timing
   *   Core 1: loop() state machine + taskHighSpeedMeasurements
   *   Both measurement tasks on Core 1 share priority 1 to enable fair yield() scheduling.
   * [PL] Rozkład zadań na dwa rdzenie:
   *   Core 0: WiFi + WebServer (zadanie sieciowe) — izolowane od timerów sprzętowych
   *   Core 1: loop() maszyna stanów + taskHighSpeedMeasurements
   *   Oba taski pomiarowe na Core 1 mają priorytet 1 dla sprawiedliwego yield().
   */
  xTaskCreatePinnedToCore(taskNetworkCore0, "TaskNet0", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskHighSpeedMeasurements, "TaskMeas1", 4096, NULL, 1, NULL, 1);

  Serial.println("[OK] Sentry Data Scout v0.4.2 ready → http://sentry.local");
}

/*
 * =======================================================================================
 * [EN] MAIN LOOP (CORE 1) — Non-blocking state machine
 *   Controls the shot sequence timing. All waits are handled by comparing millis()
 *   against stateStartTime — no delay() calls, no blocking.
 * [PL] PĘTLA GŁÓWNA (CORE 1) — Nieblokująca maszyna stanów
 *   Steruje sekwencją czasową strzałów. Wszystkie odczekania przez porównanie millis()
 *   ze stateStartTime — zero delay(), zero blokowania.
 * =======================================================================================
 */
void loop() {
  unsigned long now = millis();

  /*
   * [EN] Background temperature polling — only when idle or paused,
   *   every 2 seconds, to keep barrel temp data fresh between bursts.
   * [PL] Odczyt temperatury w tle — tylko w IDLE lub PAUSED,
   *   co 2 sekundy, żeby dane temp lufy były świeże między seriami.
   */
  static unsigned long lastSensorTime = 0;
  if (now - lastSensorTime > 2000 && (sysState == SYS_IDLE || sysState == SYS_PAUSED)) {
    sensorsBarrel.requestTemperatures();
    lastSensorTime = now;
  }

  // [EN] State machine / [PL] Maszyna stanów
  switch (sysState) {
    case SYS_IDLE:
      break;

    case SYS_SHOOTING_PRESS:
      isShooting = true;
      serwoSpust.write(cal_spust_max); // [EN] Pull trigger to calibrated fire position / [PL] Pociągnij spust na skalibrowaną pozycję strzału
      sensorsBarrel.requestTemperatures(); // [EN] Start async temp read / [PL] Zlecenie async odczytu temp
      stateStartTime = now;
      sysState = SYS_SHOOTING_RELEASE;
      break;

    case SYS_SHOOTING_RELEASE:
      /*
       * [EN] 60ms trigger hold time — enough for the sear to release and ball to fire.
       *   Increased from 50ms (v0.3) after field testing showed occasional misfires
       *   with heavy trigger springs. Adjust if needed for your marker.
       * [PL] 60ms czas trzymania spustu — wystarczający na zwolnienie zaczepu i strzał.
       *   Zwiększone z 50ms (v0.3) po testach terenowych z ciężkimi sprężynami spustu.
       *   Dostosuj jeśli potrzeba dla Twojego markera.
       */
      if (now - stateStartTime >= 60) {
        serwoSpust.write(cal_spust_min); // [EN] Release trigger / [PL] Zwolnij spust
        sysState = SYS_SHOOTING_WAIT_500;
      }
      break;

    case SYS_SHOOTING_WAIT_500:
      /*
       * [EN] 500ms measurement window — during this time taskHighSpeedMeasurements
       *   collects IMU and microphone data at max ADC speed.
       *   After 500ms: read barrel temperature (result of async request from PRESS state),
       *   log everything to CSV, decrement burst counter.
       * [PL] Okno pomiarowe 500ms — w tym czasie taskHighSpeedMeasurements
       *   zbiera dane IMU i mikrofonu z max prędkością ADC.
       *   Po 500ms: odczytaj temp lufy (wynik async z PRESS), zapisz do CSV, zmniejsz licznik.
       */
      if (now - stateStartTime >= 500) {
        isShooting = false;
        totalShotsFired++;
        float tempL = sensorsBarrel.getTempCByIndex(0);
        appendLogToCSV(totalShotsFired, currentAmmoType, currentAmmoWeight,
                       currentDistance, tempL, sharedMaxG, sharedPeakDB, sharedAvgDB);

        shotsRemaining--;

        // [EN] Decide next state / [PL] Decyzja o kolejnym stanie
        if (requireVInput) {
          sysState = SYS_WAITING_FOR_V; // [EN] Wait for chronograph input / [PL] Czekaj na dane z chronografu
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
