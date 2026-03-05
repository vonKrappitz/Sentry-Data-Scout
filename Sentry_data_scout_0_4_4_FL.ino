/**
 * =======================================================================================
 * Project: Sentry Data Scout
 * Author: Maciej Kasperek (vonKrappitz)
 * Version: 0.4.4 FL
 * [EN] Description: Spyder MR3 marker controller — Full-featured version with AP fallback,
 *      Diagnostics, Calibration Wizard, Light/Dark mode, Last-shot preview.
 * [PL] Opis: Kontroler markera Spyder MR3 — Pełna wersja z AP fallbackiem,
 *      Diagnostyką, Wizardem kalibracji, trybem Jasny/Ciemny, Podglądem strzału.
 * ---------------------------------------------------------------------------------------
 *
 *  ╔═══════════════════════════════════════════════════════════════════════════════╗
 *  ║  CHANGELOG — NEW FEATURES / NOWE FUNKCJE (tylko pozytywne zmiany)          ║
 *  ╠═══════════════════════════════════════════════════════════════════════════════╣
 *  ║                                                                             ║
 *  ║  v0.3 → v0.4                                                                ║
 *  ║  [EN] + Servo calibration system with LittleFS persistence                  ║
 *  ║       + mDNS — accessible as http://sentry.local                            ║
 *  ║       + Power slider mapped to calibrated range (0-100%)                    ║
 *  ║       + Live servo preview during calibration                               ║
 *  ║  [PL] + System kalibracji serw z zapisem do LittleFS                        ║
 *  ║       + mDNS — dostępny jako http://sentry.local                            ║
 *  ║       + Suwak mocy zmapowany na skalibrowany zakres (0-100%)               ║
 *  ║       + Podgląd serwa na żywo przy kalibracji                               ║
 *  ║                                                                             ║
 *  ║  v0.4 → v0.4.1                                                              ║
 *  ║  [EN] + Step-by-step Calibration Wizard (guided UI)                         ║
 *  ║       + Weight preset selector with custom input option                     ║
 *  ║       + Input validation (constrain) on calibration endpoint                ║
 *  ║  [PL] + Krokowy Wizard kalibracyjny (prowadzony UI)                         ║
 *  ║       + Select wagi z presetami i opcją "Inna"                              ║
 *  ║       + Walidacja wejścia (constrain) na endpoincie kalibracji              ║
 *  ║                                                                             ║
 *  ║  v0.4.1 → v0.4.2                                                            ║
 *  ║  [EN] + HTTP POST for all state-changing endpoints (security)               ║
 *  ║       + /api/get_cal — wizard auto-hides if calibration exists              ║
 *  ║       + Toast notification system (green OK / red error)                    ║
 *  ║       + Checkmark feedback on wizard steps                                  ║
 *  ║       + Bilingual comments (EN/PL) restored throughout code                 ║
 *  ║       + Named network task function (better RTOS stack trace)               ║
 *  ║  [PL] + HTTP POST dla wszystkich endpointów zmieniających stan              ║
 *  ║       + /api/get_cal — wizard auto-ukrywa się jeśli kalibracja istnieje     ║
 *  ║       + System powiadomień toast (zielony OK / czerwony błąd)               ║
 *  ║       + Ptaszki ✓ na krokach wizarda                                        ║
 *  ║       + Przywrócone dwujęzyczne komentarze (EN/PL) w całym kodzie           ║
 *  ║       + Nazwana funkcja network task (lepszy RTOS stack trace)              ║
 *  ║                                                                             ║
 *  ║  v0.4.2 → v0.4.3                                                            ║
 *  ║  [EN] + Diagnostics panel with traffic-light health indicators              ║
 *  ║       + BNO055, DS18B20, Mic, RSSI, Heap, Uptime monitoring                ║
 *  ║       + Battery voltage monitoring infrastructure (VBAT_PIN)                ║
 *  ║       + Power level persistence across reboots                              ║
 *  ║       + Double-start protection (409 Conflict)                              ║
 *  ║       + Serial boot diagnostics summary                                     ║
 *  ║  [PL] + Panel diagnostyczny z sygnalizacją świetlną zdrowia                 ║
 *  ║       + Monitoring BNO055, DS18B20, Mic, RSSI, Heap, Uptime                ║
 *  ║       + Infrastruktura monitoringu napięcia baterii (VBAT_PIN)              ║
 *  ║       + Trwały zapis poziomu mocy (przeżywa restart)                        ║
 *  ║       + Ochrona przed podwójnym startem (409 Conflict)                      ║
 *  ║       + Podsumowanie diagnostyczne na Serial przy starcie                   ║
 *  ║                                                                             ║
 *  ║  v0.4.3 → v0.4.4 FL                                                         ║
 *  ║  [EN] + WiFi AP fallback — auto Access Point if no network in 15s           ║
 *  ║       + Light/Dark mode toggle (persists in browser localStorage)           ║
 *  ║       + Last-shot preview on status card (barrel temp, G, dB)               ║
 *  ║       + Reset logs button with confirm() dialog                             ║
 *  ║       + Minimum cadence safety (≥500ms, enforced in backend + UI)           ║
 *  ║       + WiFi mode indicator in status card (STA vs AP)                      ║
 *  ║  [PL] + WiFi AP fallback — auto Access Point jeśli brak sieci w 15s         ║
 *  ║       + Tryb Jasny/Ciemny (zapamiętywany w przeglądarce)                   ║
 *  ║       + Podgląd ostatniego strzału na karcie statusu (temp, G, dB)          ║
 *  ║       + Przycisk resetu logów z oknem potwierdzenia confirm()               ║
 *  ║       + Bezpieczna minimalna kadencja (≥500ms, wymuszana w backend + UI)    ║
 *  ║       + Wskaźnik trybu WiFi na karcie statusu (STA vs AP)                   ║
 *  ║                                                                             ║
 *  ╚═══════════════════════════════════════════════════════════════════════════════╝
 *
 * ---------------------------------------------------------------------------------------
 * [EN] CRITICAL HARDWARE NOTES — READ BEFORE CONNECTING!
 *
 * 1. SERVO POWER SUPPLY (DS3218MG + MG945):
 *    - NEVER power from ESP32 5V pin! Use external 5A UBEC/SBEC.
 *    - CRITICAL: Common ground between servos, converter, and ESP32 GND.
 *
 * 2. DS18B20 (pin 4): Needs 4.7k pull-up resistor to 3.3V.
 *
 * 3. BNO055 (I2C SDA:21 SCL:22): SOLDERED connections only — no duponts!
 *
 * 4. MAX4466 Mic (pin 34): Power from clean 3.3V. Adjust gain pot to center.
 *
 * 5. Battery voltage (pin 35, optional): Needs voltage divider. Set VBAT_ENABLED.
 *
 * Libraries: WiFi, WebServer, LittleFS, HTTPClient, ArduinoJson, ESP32Servo,
 *   Adafruit_BNO055, DallasTemperature, ESPmDNS
 * ---------------------------------------------------------------------------------------
 * [PL] UWAGI SPRZĘTOWE — skrócona wersja (pełna w v0.4.2)
 *
 * 1. SERWA: NIE zasilaj z ESP32! Zewnętrzna przetwornica 5A. Wspólna masa!
 * 2. DS18B20 (pin 4): Rezystor 4.7k pull-up do 3.3V.
 * 3. BNO055 (I2C): LUTOWANE połączenia! Duponty = rozłączenie przy strzale.
 * 4. MAX4466 (pin 34): Czyste 3.3V. Potencjometr na środek.
 * 5. Bateria (pin 35, opcja): Dzielnik napięcia. Ustaw VBAT_ENABLED.
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

/* ==================== KONFIGURACJA / CONFIG ==================== */
const char* ssid     = "SSID_here";
const char* password = "PASSWORD_here";
const String OPENWEATHER_API_KEY = "API_here";

/*
 * [EN] AP FALLBACK CONFIGURATION
 *   If WiFi doesn't connect within WIFI_TIMEOUT_MS, ESP32 starts its own Access Point.
 *   Connect to "SentryAP" network, open http://192.168.4.1 in browser.
 *   Password must be ≥8 chars for WPA2.
 * [PL] KONFIGURACJA AP FALLBACK
 *   Jeśli WiFi nie połączy się w WIFI_TIMEOUT_MS, ESP32 startuje własny Access Point.
 *   Połącz się z siecią "SentryAP", otwórz http://192.168.4.1 w przeglądarce.
 *   Hasło musi mieć ≥8 znaków dla WPA2.
 */
#define WIFI_TIMEOUT_MS   15000
const char* ap_ssid     = "SentryAP";
const char* ap_password = "sentry1234";
bool wifiIsAP = false; // [EN] True if running in AP mode / [PL] True jeśli działa w trybie AP

/*
 * [EN] MINIMUM CADENCE SAFETY
 *   The 500ms measurement window must complete before the next shot.
 *   Setting cadence below this would overlap measurements and corrupt data.
 * [PL] BEZPIECZNA MINIMALNA KADENCJA
 *   Okno pomiarowe 500ms musi się zakończyć przed kolejnym strzałem.
 *   Kadencja poniżej tej wartości nałożyłaby pomiary i zepsuła dane.
 */
#define MIN_CADENCE_MS 500

/* ==================== PINY / PINS ==================== */
#define SERVO_SPUST_PIN 18
#define SERVO_SRUBA_PIN 19
#define DS18B20_PIN      4
#define MIC_PIN         34
#define VBAT_PIN        35
#define VBAT_ENABLED    false
#define VBAT_DIVIDER_RATIO 2.0
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

/* ==================== OBIEKTY / OBJECTS ==================== */
WebServer server(80);
Servo serwoSpust;
Servo serwoSruba;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensorsBarrel(&oneWire);

/* ==================== KALIBRACJA / CALIBRATION ==================== */
int cal_spust_min = 0, cal_spust_max = 180;
int cal_sruba_min = 0, cal_sruba_max = 180;
int cal_power_pct = 50;

/* ==================== ZMIENNE GLOBALNE / GLOBALS ==================== */
volatile bool  isShooting    = false;
volatile float sharedMaxG    = 0.0;
volatile float sharedPeakDB  = 0.0;
volatile float sharedAvgDB   = 0.0;

enum SystemState { SYS_IDLE, SYS_SHOOTING_PRESS, SYS_SHOOTING_RELEASE,
                   SYS_SHOOTING_WAIT_500, SYS_WAITING_CADENCE, SYS_PAUSED, SYS_WAITING_FOR_V };
volatile SystemState sysState = SYS_IDLE;
volatile int  shotsRemaining = 0;
volatile bool pauseRequested = false;
volatile bool requireVInput  = false;
unsigned long stateStartTime = 0;

float currentEnvTemp = 0.0, currentEnvHum = 0.0, currentEnvPress = 0.0;
String currentAmmoType   = "szorstka";
float  currentAmmoWeight = 3.2;
float  currentDistance    = 5.0;
int    currentCadenceMs  = 1000;
int    totalShotsFired   = 0;

/* [EN] Diagnostics / [PL] Diagnostyka */
bool  bnoOK = false;
float lastBarrelTemp = -127.0;

/*
 * [EN] LAST SHOT DATA — updated after each shot for the status card preview.
 * [PL] DANE OSTATNIEGO STRZAŁU — aktualizowane po każdym strzale dla podglądu.
 */
float lastShotTemp = 0.0;
float lastShotG    = 0.0;
float lastShotPeakDB = 0.0;
float lastShotAvgDB  = 0.0;
bool  hasLastShot  = false; // [EN] False until first shot fired / [PL] False do pierwszego strzału

/* ==================== LOGOWANIE / LOGGING ==================== */
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
  if (file) { file.printf("UPDATE_V,%d,%.2f\n", id, v); file.close(); }
}

/* ==================== KALIBRACJA TRWAŁA / CALIBRATION PERSISTENCE ==================== */
void saveCalibration() {
  File f = LittleFS.open("/calibration.json", FILE_WRITE);
  if (!f) return;
  StaticJsonDocument<256> doc;
  doc["spust_min"] = cal_spust_min; doc["spust_max"] = cal_spust_max;
  doc["sruba_min"] = cal_sruba_min; doc["sruba_max"] = cal_sruba_max;
  doc["power_pct"] = cal_power_pct;
  serializeJson(doc, f);
  f.close();
}

void loadCalibration() {
  File f = LittleFS.open("/calibration.json", FILE_READ);
  if (!f) return;
  StaticJsonDocument<256> doc;
  deserializeJson(doc, f);
  cal_spust_min = doc["spust_min"] | 0;   cal_spust_max = doc["spust_max"] | 180;
  cal_sruba_min = doc["sruba_min"] | 0;   cal_sruba_max = doc["sruba_max"] | 180;
  cal_power_pct = doc["power_pct"] | 50;
  f.close();
}

bool isCalibrated() { return LittleFS.exists("/calibration.json"); }

float readBatteryVoltage() {
  if (!VBAT_ENABLED) return 0.0;
  return (analogRead(VBAT_PIN) / 4095.0) * 3.3 * VBAT_DIVIDER_RATIO;
}

/* ==================== TASK SZYBKICH POMIARÓW / HIGH-SPEED TASK (Core 1) ==================== */
/*
 * [EN] Samples IMU + mic at max ADC speed during 500ms window. yield() prevents deadlock.
 * [PL] Próbkuje IMU + mic z max prędkością ADC w oknie 500ms. yield() zapobiega deadlockowi.
 */
void taskHighSpeedMeasurements(void * pvParameters) {
  for (;;) {
    if (isShooting) {
      sharedMaxG = 0.0;
      unsigned long sampleCount = 0;
      double sumSquares = 0;
      int signalMax = 0, signalMin = 4095;
      while (isShooting) {
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        float g = sqrt(accel.x()*accel.x() + accel.y()*accel.y() + accel.z()*accel.z());
        if (g > sharedMaxG) sharedMaxG = g;
        int sample = analogRead(MIC_PIN);
        if (sample > signalMax) signalMax = sample;
        if (sample < signalMin) signalMin = sample;
        double volts = ((double)sample - 2048.0) / 2048.0;
        sumSquares += (volts * volts);
        sampleCount++;
        yield();
      }
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

/* ==================== HTML / CSS / JS ==================== */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Sentry Data Scout v0.4.4</title>
  <style>
    :root {
      --bg:#121212; --card:#1e1e1e; --text:#e0e0e0; --text2:#bbb; --text3:#888;
      --border:#444; --input-bg:#2c2c2c; --input-border:#555; --accent:#00ff88; --accent2:#00d4ff;
      --highlight:#333;
    }
    .light {
      --bg:#f0f0f0; --card:#ffffff; --text:#1a1a1a; --text2:#555; --text3:#777;
      --border:#ccc; --input-bg:#fff; --input-border:#aaa; --accent:#00aa55; --accent2:#0077cc;
      --highlight:#e8e8e8;
    }
    body {font-family:sans-serif;background:var(--bg);color:var(--text);text-align:center;padding:10px;margin:0;transition:background 0.3s,color 0.3s}
    .card {background:var(--card);padding:15px;border-radius:8px;margin-bottom:15px;box-shadow:0 4px 8px rgba(0,0,0,0.3);transition:background 0.3s}
    h2 {color:var(--accent);margin-bottom:2px}
    .author {font-size:11px;color:var(--text3);margin-top:2px;margin-bottom:12px}
    h3 {color:var(--accent2);border-bottom:1px solid var(--border);padding-bottom:5px;margin-top:0}
    button {border:none;color:white;border-radius:5px;cursor:pointer;margin:5px}
    .btn-fire {font-size:22px;font-weight:bold;width:100%;padding:20px;border-radius:8px;margin-top:10px;transition:background 0.3s}
    .btn-green {background:#28a745} .btn-red {background:#dc3545} .btn-orange {background:#ff9800;color:#111}
    .btn-small {background:#0088ff;padding:10px 15px;font-size:14px}
    .btn-cancel {background:#555;font-size:14px;padding:12px;margin-top:5px;width:100%}
    .btn-danger {background:#dc3545;padding:10px 15px;font-size:13px}
    select,input {padding:8px;font-size:14px;border-radius:4px;border:1px solid var(--input-border);background:var(--input-bg);color:var(--text);margin:3px}
    .input-group {display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;text-align:left}
    .input-group label {flex:1;font-size:14px;color:var(--text2)}
    input[type=range] {width:100%}
    .toast {position:fixed;top:20px;left:50%;transform:translateX(-50%);background:#28a745;color:white;padding:12px 24px;border-radius:8px;font-size:16px;font-weight:bold;z-index:9999;opacity:0;transition:opacity 0.3s;pointer-events:none;box-shadow:0 4px 12px rgba(0,0,0,0.5)}
    .toast.show {opacity:1} .toast.error {background:#dc3545}
    .step-done {color:#28a745;font-weight:bold;margin-left:8px}
    .diag-row {display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid rgba(128,128,128,0.2)}
    .diag-row:last-child {border-bottom:none}
    .diag-label {font-size:13px;color:var(--text2);text-align:left;flex:1}
    .diag-value {font-size:13px;color:var(--text);text-align:right;margin-right:8px;min-width:80px}
    .diag-dot {width:14px;height:14px;border-radius:50%;flex-shrink:0}
    .dot-green {background:#28a745;box-shadow:0 0 8px #28a74580}
    .dot-yellow {background:#ffc107;box-shadow:0 0 8px #ffc10780}
    .dot-red {background:#dc3545;box-shadow:0 0 8px #dc354580}
    .dot-gray {background:#666}
    .diag-toggle {background:none;border:none;color:var(--accent2);font-size:14px;cursor:pointer;padding:0;margin:0;text-decoration:underline}
    .theme-btn {position:fixed;top:8px;right:8px;z-index:100;background:var(--card);border:1px solid var(--border);color:var(--text);border-radius:50%;width:36px;height:36px;font-size:18px;cursor:pointer;box-shadow:0 2px 6px rgba(0,0,0,0.3);display:flex;align-items:center;justify-content:center}
    .last-shot {font-size:12px;color:var(--accent);margin-top:4px;text-align:left}
    .wifi-badge {display:inline-block;padding:2px 8px;border-radius:10px;font-size:11px;font-weight:bold;margin-left:6px}
    .wifi-sta {background:#28a745;color:white} .wifi-ap {background:#ff9800;color:#111}
  </style>
</head>
<body>

  <!-- Theme toggle -->
  <button class="theme-btn" id="theme_btn" onclick="toggleTheme()">🌙</button>

  <h2>Sentry Data Scout</h2>
  <div class="author">Maciej Kasperek (von Krappitz) | MR3 Platform | v0.4.4 FL</div>

  <div id="toast" class="toast"></div>

  <!-- ===== STATUS CARD ===== -->
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <div style="text-align:left">
        <p style="margin:2px">Seria: <b id="shots">0</b> strzałów <span id="wifi_badge" class="wifi-badge wifi-sta">STA</span></p>
        <p style="margin:2px;font-size:12px;color:var(--text3)" id="env">Pogoda: --</p>
        <div id="last_shot_box" class="last-shot" style="display:none">
          Ostatni strzał / Last shot: 🌡️ <span id="ls_temp">--</span>°C | 💥 <span id="ls_g">--</span>G | 🔊 <span id="ls_peak">--</span>dB (avg <span id="ls_avg">--</span>)
        </div>
      </div>
      <button class="btn-small" onclick="getWeather()">📍</button>
    </div>
  </div>

  <!-- ===== DIAGNOSTICS ===== -->
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">
      <h3 style="margin:0;border:none;padding:0">🔧 Diagnostyka</h3>
      <button class="diag-toggle" id="diag_toggle" onclick="toggleDiag()">▼</button>
    </div>
    <div id="diag_panel" style="display:none">
      <div class="diag-row"><span class="diag-label">IMU (BNO055)</span><span class="diag-value" id="d_bno">--</span><span class="diag-dot dot-gray" id="dot_bno"></span></div>
      <div class="diag-row"><span class="diag-label">Temp lufy (DS18B20)</span><span class="diag-value" id="d_temp">--</span><span class="diag-dot dot-gray" id="dot_temp"></span></div>
      <div class="diag-row"><span class="diag-label">Mikrofon</span><span class="diag-value" id="d_mic">--</span><span class="diag-dot dot-gray" id="dot_mic"></span></div>
      <div class="diag-row"><span class="diag-label">Napięcie / Voltage</span><span class="diag-value" id="d_vbat">--</span><span class="diag-dot dot-gray" id="dot_vbat"></span></div>
      <div class="diag-row"><span class="diag-label">WiFi RSSI</span><span class="diag-value" id="d_rssi">--</span><span class="diag-dot dot-gray" id="dot_rssi"></span></div>
      <div class="diag-row"><span class="diag-label">Free Heap</span><span class="diag-value" id="d_heap">--</span><span class="diag-dot dot-gray" id="dot_heap"></span></div>
      <div class="diag-row"><span class="diag-label">Uptime</span><span class="diag-value" id="d_uptime">--</span><span class="diag-dot dot-green"></span></div>
      <div class="diag-row"><span class="diag-label">Sesja / Session</span><span class="diag-value" id="d_shots">0</span><span class="diag-dot dot-green"></span></div>
      <div class="diag-row"><span class="diag-label">Spust cal</span><span class="diag-value" id="d_servo_s">--</span><span class="diag-dot dot-green"></span></div>
      <div class="diag-row"><span class="diag-label">Śruba cal</span><span class="diag-value" id="d_servo_p">--</span><span class="diag-dot dot-green"></span></div>
      <div class="diag-row"><span class="diag-label">Moc / Power</span><span class="diag-value" id="d_power">--</span><span class="diag-dot dot-green"></span></div>
      <div class="diag-row"><span class="diag-label">Stan / State</span><span class="diag-value" id="d_state">--</span><span class="diag-dot dot-green" id="dot_state"></span></div>
    </div>
  </div>

  <!-- ===== CALIBRATION ===== -->
  <div class="card">
    <h3>⚙️ Kalibracja</h3>
    <div style="text-align:left;border-bottom:1px solid var(--border);padding-bottom:10px;margin-bottom:10px">
      <div class="input-group"><span style="color:var(--accent);font-weight:bold">Spust:</span><b id="s_val">0°</b></div>
      <div id="wizard_spust">
        <input type="range" id="s_live" min="0" max="180" value="0" oninput="moveLive('spust',this.value)">
        <p style="font-size:12px;color:var(--text3);margin:5px 0">1. Przesuń na strzał → ZAPISZ STRZAŁ. 2. Cofnij → ZAPISZ LUZ.<br>1. Slide to fire → SAVE FIRE. 2. Slide back → SAVE REST.</p>
        <div style="display:flex;gap:5px">
          <button class="btn-small btn-red" style="flex:1" onclick="zapiszKrok('spust','max')">🎯 STRZAŁ <span id="check_spust_max"></span></button>
          <button class="btn-small btn-green" style="flex:1" onclick="zapiszKrok('spust','min')">😌 LUZ <span id="check_spust_min"></span></button>
        </div>
      </div>
      <button id="btn_rekal_spust" class="btn-small" style="display:none;width:100%;background:#555;margin-top:5px" onclick="pokazWizard('spust')">🔄 Rekalibruj Spust</button>
    </div>
    <div style="text-align:left">
      <div class="input-group"><span style="color:var(--accent);font-weight:bold">Śruba N2:</span><b id="p_val">90°</b></div>
      <div id="wizard_sruba">
        <input type="range" id="p_live" min="0" max="180" value="90" oninput="moveLive('sruba',this.value)">
        <p style="font-size:12px;color:var(--text3);margin:5px 0">1. Załóż serwo → ZAPISZ ZERO. 2. Przesuń na max → ZAPISZ MAX.<br>1. Mount servo → SAVE ZERO. 2. Slide to max → SAVE MAX.</p>
        <div style="display:flex;gap:5px">
          <button class="btn-small" style="flex:1;background:#888" onclick="zapiszKrok('sruba','min')">0️⃣ ZERO <span id="check_sruba_min"></span></button>
          <button class="btn-small btn-orange" style="flex:1" onclick="zapiszKrok('sruba','max')">🔥 MAX <span id="check_sruba_max"></span></button>
        </div>
      </div>
      <button id="btn_rekal_sruba" class="btn-small" style="display:none;width:100%;background:#555;margin-top:5px" onclick="pokazWizard('sruba')">🔄 Rekalibruj Śrubę</button>
    </div>
  </div>

  <!-- ===== FIRE CONTROL ===== -->
  <div class="card">
    <h3>🎯 Kontrola Ognia</h3>
    <div class="input-group"><label>Moc (0-100%):</label><div style="display:flex;align-items:center"><input type="range" id="p_power" min="0" max="100" value="50" style="width:100px" onchange="setPower(this.value)"><span id="power_pct" style="margin-left:5px;width:35px;text-align:right">50%</span></div></div>
    <div class="input-group"><label>Tryb / Mode:</label><select id="shot_mode"><option value="1">1</option><option value="3">3</option><option value="5">5</option></select></div>
    <div class="input-group"><label>Kulka / Ammo:</label><select id="p_type"><option value="szorstka">Szorstka</option><option value="gładka">Gładka</option></select></div>
    <div class="input-group"><label>Waga / Weight [g]:</label><div style="display:flex;gap:5px"><select id="p_weight_sel" onchange="checkCW()" style="width:70px"><option value="3.0">3.0g</option><option value="3.2" selected>3.2g</option><option value="5.0">5.0g</option><option value="9.0">9.0g</option><option value="inna">Inna</option></select><input type="number" id="p_weight_custom" style="display:none;width:60px" placeholder="4.5" step="0.1"></div></div>
    <div class="input-group"><label>Dystans [m]:</label><select id="p_dist"><option value="0">0m</option><option value="5" selected>5m</option><option value="10">10m</option><option value="15">15m</option><option value="20">20m</option><option value="25">25m</option><option value="50">50m</option></select></div>
    <div class="input-group"><label>Kadencja [ms] (min 500):</label><input type="number" id="p_cadence" value="1000" min="500" step="100" style="width:80px"></div>
    <div class="input-group" style="margin-top:10px;background:var(--highlight);padding:10px;border-radius:5px"><label style="color:var(--accent);font-weight:bold">Wymagaj V:</label><input type="checkbox" id="p_req_v" style="width:20px;height:20px"></div>
    <button id="btn_action" class="btn-fire btn-green" onclick="toggleAction()">▶ START</button>
    <button class="btn-cancel" onclick="stopAction()">⏹ STOP</button>
  </div>

  <!-- ===== VELOCITY + LOGS ===== -->
  <div class="card">
    <h3>Prędkość V / Velocity</h3>
    <div class="input-group" style="justify-content:center;gap:10px">
      <input type="number" id="v_id" placeholder="ID" style="width:25%">
      <input type="number" id="v_val" placeholder="V [m/s]" step="0.1" style="width:30%">
      <button class="btn-small" onclick="saveV()">✍️</button>
    </div>
    <br>
    <div style="display:flex;gap:8px;margin-top:5px">
      <button class="btn-fire btn-green" style="font-size:14px;padding:12px;flex:1" onclick="location.href='/logs.csv'">📥 CSV</button>
      <button class="btn-danger" style="flex:0.6" onclick="resetLogs()">🗑️ Reset logów</button>
    </div>
  </div>

  <span id="sys_state" style="display:none">IDLE</span>

  <script>
    function showToast(m,e){var t=document.getElementById('toast');t.innerText=m;t.className='toast show'+(e?' error':'');setTimeout(function(){t.className='toast'},2000)}

    /* ===== THEME ===== */
    function toggleTheme(){
      var b=document.body,isLight=b.classList.toggle('light');
      document.getElementById('theme_btn').innerText=isLight?'🌞':'🌙';
      try{localStorage.setItem('sentry_theme',isLight?'light':'dark')}catch(e){}
    }
    (function(){try{if(localStorage.getItem('sentry_theme')==='light'){document.body.classList.add('light');document.getElementById('theme_btn').innerText='🌞'}}catch(e){}})();

    /* ===== DIAGNOSTICS ===== */
    var diagOpen=false;
    function toggleDiag(){diagOpen=!diagOpen;document.getElementById('diag_panel').style.display=diagOpen?'block':'none';document.getElementById('diag_toggle').innerText=diagOpen?'▲':'▼';if(diagOpen)fetchDiag()}
    function fetchDiag(){
      fetch('/api/diagnostics').then(function(r){return r.json()}).then(function(d){
        var e,dot;
        e=document.getElementById('d_bno');dot=document.getElementById('dot_bno');
        if(!d.bno_ok){e.innerText='BRAK';dot.className='diag-dot dot-red'}
        else if(d.bno_sys_cal<2){e.innerText='Kal:'+d.bno_sys_cal+'/3';dot.className='diag-dot dot-yellow'}
        else{e.innerText='OK ('+d.bno_sys_cal+'/3)';dot.className='diag-dot dot-green'}
        e=document.getElementById('d_temp');dot=document.getElementById('dot_temp');
        if(d.barrel_temp<=-126){e.innerText='BRAK';dot.className='diag-dot dot-red'}
        else if(d.barrel_temp>=84.5&&d.barrel_temp<=85.5){e.innerText=d.barrel_temp.toFixed(1)+'°C ?';dot.className='diag-dot dot-yellow'}
        else{e.innerText=d.barrel_temp.toFixed(1)+'°C';dot.className='diag-dot dot-green'}
        e=document.getElementById('d_mic');dot=document.getElementById('dot_mic');
        if(d.mic_raw<=5){e.innerText='BRAK';dot.className='diag-dot dot-red'}
        else if(d.mic_raw>=4090){e.innerText='CLIP('+d.mic_raw+')';dot.className='diag-dot dot-yellow'}
        else{e.innerText='OK('+d.mic_raw+')';dot.className='diag-dot dot-green'}
        e=document.getElementById('d_vbat');dot=document.getElementById('dot_vbat');
        if(!d.vbat_enabled){e.innerText='N/A';dot.className='diag-dot dot-gray'}
        else if(d.vbat<4.0){e.innerText=d.vbat.toFixed(2)+'V';dot.className='diag-dot dot-red'}
        else if(d.vbat<4.5){e.innerText=d.vbat.toFixed(2)+'V';dot.className='diag-dot dot-yellow'}
        else{e.innerText=d.vbat.toFixed(2)+'V';dot.className='diag-dot dot-green'}
        e=document.getElementById('d_rssi');dot=document.getElementById('dot_rssi');
        e.innerText=d.rssi+'dBm';dot.className='diag-dot dot-'+(d.rssi>-60?'green':d.rssi>-80?'yellow':'red');
        e=document.getElementById('d_heap');dot=document.getElementById('dot_heap');
        e.innerText=(d.free_heap/1024).toFixed(1)+'kB';dot.className='diag-dot dot-'+(d.free_heap>50000?'green':d.free_heap>20000?'yellow':'red');
        var s=Math.floor(d.uptime_ms/1000),h=Math.floor(s/3600),m=Math.floor(s%3600/60);
        document.getElementById('d_uptime').innerText=h+'h '+m+'m '+(s%60)+'s';
        document.getElementById('d_shots').innerText=d.total_shots;
        document.getElementById('d_servo_s').innerText=d.cal_spust_min+'°/'+d.cal_spust_max+'°';
        document.getElementById('d_servo_p').innerText=d.cal_sruba_min+'°/'+d.cal_sruba_max+'°';
        document.getElementById('d_power').innerText=d.power_pct+'%';
        document.getElementById('d_state').innerText=d.state;
        var sd=document.getElementById('dot_state');
        sd.className='diag-dot dot-'+(d.state==='IDLE'?'green':(d.state==='PAUSED'||d.state==='WAITING_V')?'yellow':'red');
      }).catch(function(){})
    }
    setInterval(function(){if(diagOpen)fetchDiag()},3000);

    /* ===== PAGE LOAD ===== */
    window.addEventListener('load',function(){
      fetch('/api/get_cal').then(function(r){return r.json()}).then(function(c){
        if(c.calibrated){
          document.getElementById('wizard_spust').style.display='none';document.getElementById('btn_rekal_spust').style.display='block';
          document.getElementById('wizard_sruba').style.display='none';document.getElementById('btn_rekal_sruba').style.display='block';
          showToast('Kalibracja wczytana ✓',false)
        }
        if(typeof c.power_pct!=='undefined'){document.getElementById('p_power').value=c.power_pct;document.getElementById('power_pct').innerText=c.power_pct+'%'}
        if(c.wifi_ap){document.getElementById('wifi_badge').innerText='AP';document.getElementById('wifi_badge').className='wifi-badge wifi-ap'}
      }).catch(function(){})
    });

    setInterval(updateStatus,1000);
    var wizState={spust:{min:false,max:false},sruba:{min:false,max:false}};
    function checkCW(){document.getElementById('p_weight_custom').style.display=(document.getElementById('p_weight_sel').value==='inna')?'block':'none'}
    function moveLive(t,v){document.getElementById(t==='spust'?'s_val':'p_val').innerText=v+'°';fetch('/api/live',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'target='+t+'&val='+v})}

    function zapiszKrok(t,m){
      var v=document.getElementById(t==='spust'?'s_live':'p_live').value;
      fetch('/api/set_cal',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'target='+t+'&mode='+m+'&val='+v}).then(function(r){
        if(r.ok){
          document.getElementById('check_'+t+'_'+m).innerHTML='<span class="step-done">✓</span>';
          wizState[t][m]=true;
          var L={spust_min:'LUZ',spust_max:'STRZAŁ',sruba_min:'ZERO',sruba_max:'MAX'};
          showToast(L[t+'_'+m]+'='+v+'° ✓',false);
          if(wizState[t].min&&wizState[t].max){setTimeout(function(){document.getElementById('wizard_'+t).style.display='none';document.getElementById('btn_rekal_'+t).style.display='block';showToast('Kalibracja '+t+' OK ✓',false)},800)}
        }else showToast('Błąd!',true)
      })
    }
    function pokazWizard(t){document.getElementById('wizard_'+t).style.display='block';document.getElementById('btn_rekal_'+t).style.display='none';wizState[t]={min:false,max:false};document.getElementById('check_'+t+'_min').innerHTML='';document.getElementById('check_'+t+'_max').innerHTML=''}
    function setPower(v){document.getElementById('power_pct').innerText=v+'%';fetch('/api/power',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'val='+v})}

    function getWeather(){
      document.getElementById('env').innerText='GPS...';
      if(!navigator.geolocation){showToast('Brak GPS',true);return}
      navigator.geolocation.getCurrentPosition(function(p){document.getElementById('env').innerText='API...';
        fetch('/api/weather',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'lat='+p.coords.latitude+'&lon='+p.coords.longitude}).then(function(r){return r.text()}).then(function(t){showToast(t,false);updateStatus()})
      },function(e){showToast('GPS: '+e.message,true)})
    }

    function updateStatus(){
      fetch('/api/status').then(function(r){return r.json()}).then(function(d){
        document.getElementById('shots').innerText=d.totalShots;
        if(d.envPress>0)document.getElementById('env').innerText=d.envTemp.toFixed(1)+'°C, '+d.envPress.toFixed(0)+'hPa';
        var st=d.state;document.getElementById('sys_state').innerText=st;
        var b=document.getElementById('btn_action');
        if(st==='IDLE'){b.className='btn-fire btn-green';b.innerText='▶ START';b.style.backgroundColor=''}
        else if(st==='PAUSED'){b.className='btn-fire btn-orange';b.innerText='▶ WZNÓW';b.style.backgroundColor=''}
        else if(st==='WAITING_V'){b.className='btn-fire';b.style.backgroundColor='#007bff';b.innerText='⏳ WPISZ V';document.getElementById('v_id').value=d.totalShots}
        else{b.className='btn-fire btn-red';b.style.backgroundColor='';b.innerText='⏸ PAUZA'}
        /* Last shot preview */
        if(d.hasLastShot){
          document.getElementById('last_shot_box').style.display='block';
          document.getElementById('ls_temp').innerText=d.lsTemp.toFixed(1);
          document.getElementById('ls_g').innerText=d.lsG.toFixed(2);
          document.getElementById('ls_peak').innerText=d.lsPeak.toFixed(1);
          document.getElementById('ls_avg').innerText=d.lsAvg.toFixed(1);
        }
      })
    }

    function toggleAction(){
      var st=document.getElementById('sys_state').innerText;
      if(st==='IDLE'){
        var selW=document.getElementById('p_weight_sel').value;
        var w=(selW==='inna')?document.getElementById('p_weight_custom').value:selW;
        if(!w)w='3.2';
        /* [EN] Enforce min cadence in UI / [PL] Wymuszenie min kadencji w UI */
        var cad=parseInt(document.getElementById('p_cadence').value)||1000;
        if(cad<500){cad=500;document.getElementById('p_cadence').value='500';showToast('Min kadencja: 500ms!',true)}
        var p='type='+encodeURIComponent(document.getElementById('p_type').value)+'&weight='+w+'&dist='+document.getElementById('p_dist').value+'&cadence='+cad+'&reqV='+(document.getElementById('p_req_v').checked?'1':'0');
        fetch('/api/params',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p}).then(function(){return fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'cmd=start&count='+document.getElementById('shot_mode').value})})
      }else if(st==='PAUSED')fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'cmd=resume'});
      else if(st==='WAITING_V')saveV();
      else fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'cmd=pause'});
      setTimeout(updateStatus,200)
    }
    function stopAction(){fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'cmd=stop'});setTimeout(updateStatus,200)}

    function saveV(){
      var id=document.getElementById('v_id').value,val=document.getElementById('v_val').value;
      if(!id||!val){showToast('Wypełnij ID i V!',true);return}
      fetch('/api/save_v',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'id='+id+'&v='+val}).then(function(r){if(r.ok){showToast('V OK (ID:'+id+')',false);document.getElementById('v_id').value='';document.getElementById('v_val').value=''}})
    }

    /* [EN] Reset logs with confirmation / [PL] Reset logów z potwierdzeniem */
    function resetLogs(){
      if(!confirm('Na pewno usunąć WSZYSTKIE logi?\nDelete ALL logs for real?'))return;
      fetch('/api/reset_logs',{method:'POST'}).then(function(r){return r.text()}).then(function(t){showToast(t,false);updateStatus()})
    }
  </script>
</body>
</html>
)rawliteral";

/* ==================== ENDPOINTY / ENDPOINTS ==================== */
void setupWebEndpoints() {
  server.on("/", HTTP_GET, [](){ server.send(200, "text/html", index_html); });

  /* GET /api/status — includes last-shot data + wifi mode */
  server.on("/api/status", HTTP_GET, []() {
    StaticJsonDocument<384> doc;
    doc["envTemp"] = currentEnvTemp; doc["envHum"] = currentEnvHum; doc["envPress"] = currentEnvPress;
    doc["totalShots"] = totalShotsFired;
    if      (sysState == SYS_IDLE)          doc["state"] = "IDLE";
    else if (sysState == SYS_PAUSED)        doc["state"] = "PAUSED";
    else if (sysState == SYS_WAITING_FOR_V) doc["state"] = "WAITING_V";
    else                                    doc["state"] = "RUNNING";
    doc["wifiAP"] = wifiIsAP;
    // [EN] Last shot preview data / [PL] Dane podglądu ostatniego strzału
    doc["hasLastShot"] = hasLastShot;
    if (hasLastShot) {
      doc["lsTemp"] = lastShotTemp; doc["lsG"] = lastShotG;
      doc["lsPeak"] = lastShotPeakDB; doc["lsAvg"] = lastShotAvgDB;
    }
    String r; serializeJson(doc, r); server.send(200, "application/json", r);
  });

  /* GET /api/get_cal — calibration + power + wifi mode for page restore */
  server.on("/api/get_cal", HTTP_GET, []() {
    StaticJsonDocument<256> doc;
    doc["calibrated"] = isCalibrated();
    doc["spust_min"] = cal_spust_min; doc["spust_max"] = cal_spust_max;
    doc["sruba_min"] = cal_sruba_min; doc["sruba_max"] = cal_sruba_max;
    doc["power_pct"] = cal_power_pct; doc["wifi_ap"] = wifiIsAP;
    String r; serializeJson(doc, r); server.send(200, "application/json", r);
  });

  /* GET /api/diagnostics — full health report */
  server.on("/api/diagnostics", HTTP_GET, []() {
    StaticJsonDocument<512> doc;
    doc["bno_ok"] = bnoOK;
    if (bnoOK) {
      uint8_t sc, gc, ac, mc;
      bno.getCalibration(&sc, &gc, &ac, &mc);
      doc["bno_sys_cal"] = sc;
    } else doc["bno_sys_cal"] = 0;
    doc["barrel_temp"] = lastBarrelTemp;
    doc["mic_raw"] = analogRead(MIC_PIN);
    doc["vbat_enabled"] = VBAT_ENABLED; doc["vbat"] = readBatteryVoltage();
    doc["rssi"] = WiFi.RSSI(); doc["free_heap"] = ESP.getFreeHeap(); doc["uptime_ms"] = millis();
    doc["total_shots"] = totalShotsFired;
    doc["cal_spust_min"] = cal_spust_min; doc["cal_spust_max"] = cal_spust_max;
    doc["cal_sruba_min"] = cal_sruba_min; doc["cal_sruba_max"] = cal_sruba_max;
    doc["power_pct"] = cal_power_pct;
    if      (sysState == SYS_IDLE) doc["state"] = "IDLE";
    else if (sysState == SYS_PAUSED) doc["state"] = "PAUSED";
    else if (sysState == SYS_WAITING_FOR_V) doc["state"] = "WAITING_V";
    else doc["state"] = "RUNNING";
    String r; serializeJson(doc, r); server.send(200, "application/json", r);
  });

  server.on("/api/live", HTTP_POST, []() {
    String t = server.arg("target"); int v = constrain(server.arg("val").toInt(), 0, 180);
    if (t == "spust") serwoSpust.write(v); else serwoSruba.write(v);
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/set_cal", HTTP_POST, []() {
    String t = server.arg("target"), m = server.arg("mode");
    int v = constrain(server.arg("val").toInt(), 0, 180);
    if (t == "spust") { if (m == "min") cal_spust_min = v; else cal_spust_max = v; }
    else { if (m == "min") cal_sruba_min = v; else cal_sruba_max = v; }
    saveCalibration(); server.send(200, "text/plain", "OK");
  });

  /* POST /api/power — persists power level to flash */
  server.on("/api/power", HTTP_POST, []() {
    int p = constrain(server.arg("val").toInt(), 0, 100);
    cal_power_pct = p;
    serwoSruba.write(map(p, 0, 100, cal_sruba_min, cal_sruba_max));
    saveCalibration();
    server.send(200, "text/plain", "OK");
  });

  /*
   * [EN] POST /api/params — enforces MIN_CADENCE_MS safety floor.
   * [PL] POST /api/params — wymusza bezpieczne minimum kadencji MIN_CADENCE_MS.
   */
  server.on("/api/params", HTTP_POST, []() {
    if (server.hasArg("type"))    currentAmmoType   = server.arg("type");
    if (server.hasArg("weight"))  currentAmmoWeight = server.arg("weight").toFloat();
    if (server.hasArg("dist"))    currentDistance    = server.arg("dist").toFloat();
    if (server.hasArg("cadence")) {
      int c = server.arg("cadence").toInt();
      currentCadenceMs = max(c, (int)MIN_CADENCE_MS); // [EN] Safety floor / [PL] Bezpieczne minimum
    }
    if (server.hasArg("reqV")) requireVInput = (server.arg("reqV") == "1");
    server.send(200, "text/plain", "OK");
  });

  /* POST /api/control — with double-start guard */
  server.on("/api/control", HTTP_POST, []() {
    if (!server.hasArg("cmd")) { server.send(400, "text/plain", "No cmd"); return; }
    String cmd = server.arg("cmd");
    if (cmd == "start") {
      if (sysState != SYS_IDLE) { server.send(409, "text/plain", "Already running"); return; }
      shotsRemaining = server.hasArg("count") ? server.arg("count").toInt() : 1;
      pauseRequested = false; sysState = SYS_SHOOTING_PRESS;
    } else if (cmd == "pause") pauseRequested = true;
    else if (cmd == "resume") { pauseRequested = false; if (sysState == SYS_PAUSED) { stateStartTime = millis(); sysState = SYS_WAITING_CADENCE; } }
    else if (cmd == "stop") { shotsRemaining = 0; pauseRequested = false; sysState = SYS_IDLE; isShooting = false; }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/save_v", HTTP_POST, []() {
    if (!server.hasArg("id") || !server.hasArg("v")) { server.send(400, "text/plain", "Missing data"); return; }
    updateVelocityInLog(server.arg("id").toInt(), server.arg("v").toFloat());
    if (sysState == SYS_WAITING_FOR_V) {
      if (shotsRemaining > 0) { stateStartTime = millis(); sysState = SYS_WAITING_CADENCE; }
      else sysState = SYS_IDLE;
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/weather", HTTP_POST, []() {
    if (!server.hasArg("lat") || !server.hasArg("lon")) { server.send(400, "text/plain", "No GPS"); return; }
    HTTPClient http;
    String url = "https://api.openweathermap.org/data/2.5/weather?lat=" + server.arg("lat") + "&lon=" + server.arg("lon") + "&units=metric&appid=" + OPENWEATHER_API_KEY;
    http.begin(url);
    if (http.GET() == HTTP_CODE_OK) {
      DynamicJsonDocument doc(1024); deserializeJson(doc, http.getString());
      currentEnvTemp = doc["main"]["temp"]; currentEnvHum = doc["main"]["humidity"]; currentEnvPress = doc["main"]["pressure"];
      server.send(200, "text/plain", "Pogoda OK!");
    } else server.send(500, "text/plain", "API Error");
    http.end();
  });

  /*
   * [EN] POST /api/reset_logs — Deletes CSV and recreates with header. Resets shot counter.
   * [PL] POST /api/reset_logs — Usuwa CSV i tworzy nowy z nagłówkiem. Resetuje licznik strzałów.
   */
  server.on("/api/reset_logs", HTTP_POST, []() {
    LittleFS.remove("/logs.csv");
    File f = LittleFS.open("/logs.csv", FILE_WRITE);
    if (f) {
      f.println("ID,Timestamp,Typ_Kulki,Waga_Kulki,Odleglosc,Temp_Otoczenia,Wilgotnosc,Cisnienie,Temp_Lufy_500ms,Max_Odrzut_G,Peak_dB,Sredni_dB,V_ms");
      f.close();
    }
    totalShotsFired = 0;
    hasLastShot = false;
    server.send(200, "text/plain", "Logi wyczyszczone / Logs cleared ✓");
  });

  server.on("/logs.csv", HTTP_GET, []() {
    File f = LittleFS.open("/logs.csv", "r");
    if (!f) { server.send(404, "text/plain", "Empty"); return; }
    server.streamFile(f, "text/csv"); f.close();
  });
}

/* ==================== NETWORK TASK (Core 0) ==================== */
void taskNetworkCore0(void * pvParameters) {
  setupWebEndpoints();
  server.begin();
  for (;;) { server.handleClient(); vTaskDelay(10 / portTICK_PERIOD_MS); }
}

/* ==================== SETUP ==================== */
void setup() {
  Serial.begin(115200);
  if (!LittleFS.begin(true)) { Serial.println("[ERROR] LittleFS!"); return; }

  if (!LittleFS.exists("/logs.csv")) {
    File f = LittleFS.open("/logs.csv", FILE_WRITE);
    f.println("ID,Timestamp,Typ_Kulki,Waga_Kulki,Odleglosc,Temp_Otoczenia,Wilgotnosc,Cisnienie,Temp_Lufy_500ms,Max_Odrzut_G,Peak_dB,Sredni_dB,V_ms");
    f.close();
  }

  loadCalibration();

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  serwoSpust.setPeriodHertz(50); serwoSruba.setPeriodHertz(50);
  serwoSpust.attach(SERVO_SPUST_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  serwoSruba.attach(SERVO_SRUBA_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  serwoSpust.write(cal_spust_min);
  serwoSruba.write(map(cal_power_pct, 0, 100, cal_sruba_min, cal_sruba_max));

  sensorsBarrel.begin(); sensorsBarrel.setResolution(9); sensorsBarrel.setWaitForConversion(false);
  bnoOK = bno.begin();
  if (!bnoOK) Serial.println("[WARN] BNO055 not found!");
  pinMode(MIC_PIN, INPUT);
  if (VBAT_ENABLED) pinMode(VBAT_PIN, INPUT);

  /*
   * [EN] WiFi connection with AP fallback.
   *   Tries STA mode for WIFI_TIMEOUT_MS. If fails, starts AP mode.
   *   In AP mode: no internet (weather won't work), but all local features work.
   * [PL] WiFi z fallbackiem AP.
   *   Próbuje STA przez WIFI_TIMEOUT_MS. Jeśli nie — startuje AP.
   *   W trybie AP: brak internetu (pogoda nie zadziała), ale lokalne funkcje OK.
   */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Connecting (STA)");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiIsAP = false;
    Serial.println("\n[WiFi] STA Connected: " + WiFi.localIP().toString());
  } else {
    // [EN] Fallback to AP mode / [PL] Fallback do trybu AP
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_password);
    wifiIsAP = true;
    Serial.println("\n[WiFi] AP Mode: " + String(ap_ssid) + " → http://192.168.4.1");
    Serial.println("[WiFi] Password: " + String(ap_password));
  }

  MDNS.begin("sentry");

  xTaskCreatePinnedToCore(taskNetworkCore0, "TaskNet0", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskHighSpeedMeasurements, "TaskMeas1", 4096, NULL, 1, NULL, 1);

  Serial.println("[OK] Sentry Data Scout v0.4.4 FL → http://sentry.local");
  Serial.println("[DIAG] BNO:" + String(bnoOK ? "OK" : "FAIL") +
                 " | Pwr:" + String(cal_power_pct) + "%" +
                 " | WiFi:" + String(wifiIsAP ? "AP" : "STA") +
                 " | VBAT:" + String(VBAT_ENABLED ? "ON" : "OFF"));
}

/* ==================== LOOP (Core 1) ==================== */
void loop() {
  unsigned long now = millis();

  static unsigned long lastSensorTime = 0;
  if (now - lastSensorTime > 2000 && (sysState == SYS_IDLE || sysState == SYS_PAUSED)) {
    sensorsBarrel.requestTemperatures(); lastSensorTime = now;
  }
  static unsigned long lastTempRead = 0;
  if (now - lastTempRead > 2500) { lastBarrelTemp = sensorsBarrel.getTempCByIndex(0); lastTempRead = now; }

  switch (sysState) {
    case SYS_IDLE: break;

    case SYS_SHOOTING_PRESS:
      isShooting = true;
      serwoSpust.write(cal_spust_max);
      sensorsBarrel.requestTemperatures();
      stateStartTime = now;
      sysState = SYS_SHOOTING_RELEASE;
      break;

    case SYS_SHOOTING_RELEASE:
      if (now - stateStartTime >= 60) { serwoSpust.write(cal_spust_min); sysState = SYS_SHOOTING_WAIT_500; }
      break;

    case SYS_SHOOTING_WAIT_500:
      if (now - stateStartTime >= 500) {
        isShooting = false;
        totalShotsFired++;
        float tempL = sensorsBarrel.getTempCByIndex(0);
        lastBarrelTemp = tempL;
        appendLogToCSV(totalShotsFired, currentAmmoType, currentAmmoWeight, currentDistance, tempL, sharedMaxG, sharedPeakDB, sharedAvgDB);

        // [EN] Update last-shot preview / [PL] Aktualizuj podgląd ostatniego strzału
        lastShotTemp = tempL; lastShotG = sharedMaxG; lastShotPeakDB = sharedPeakDB; lastShotAvgDB = sharedAvgDB;
        hasLastShot = true;

        shotsRemaining--;
        if (requireVInput) sysState = SYS_WAITING_FOR_V;
        else if (shotsRemaining > 0) { stateStartTime = millis(); sysState = pauseRequested ? SYS_PAUSED : SYS_WAITING_CADENCE; }
        else { sysState = SYS_IDLE; pauseRequested = false; }
      }
      break;

    case SYS_WAITING_CADENCE:
      if (pauseRequested) sysState = SYS_PAUSED;
      else if (now - stateStartTime >= (unsigned long)currentCadenceMs) sysState = SYS_SHOOTING_PRESS;
      break;

    case SYS_PAUSED:
    case SYS_WAITING_FOR_V:
      break;
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}
