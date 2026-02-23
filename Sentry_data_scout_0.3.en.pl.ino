/**
 * =======================================================================================
 * Project: Sentry Data Scout
 * Author: Maciej Kasperek (vonKrappitz)
 * Version: 0.3
 * [EN]Description: Spyder MR3 marker controller - Web Server + LittleFS + State Machine version
 * [PL]Opis: Kontroler markera Spyder MR3 - Wersja Web Serwer + LittleFS + Maszyna Stanów
 * ---------------------------------------------------------------------------------------
 * [EN]
 * * * CRITICAL HARDWARE NOTES - READ BEFORE CONNECTING!
 * 1. SERVO POWER SUPPLY (DS3218MG and MG945):
 * - DS3218MG is a powerful servo (20kg) that can draw 2-3 Amps when stalled. MG945 adds 1-1.5A.
 * - ABSOLUTELY DO NOT POWER THEM FROM THE 5V (VIN) PIN ON THE ESP32! A voltage drop will reset 
 * the chip (Brownout) or burn the regulator on the NodeMCU/ESP32 board.
 * - SOLUTION: Use an external DC-DC Step-Down converter (UBEC/SBEC) with a minimum 3A 
 * (5A recommended) output at 5V or 6V.
 * - CONNECTION: Connect the red wires from the servos to the positive (+) output of the converter.
 * - CRITICAL (COMMON GROUND): The black/brown wires from the servos, the negative (-) output 
 * of the converter, AND the GND pin on the ESP32 MUST BE CONNECTED TOGETHER. Without a common 
 * ground, the PWM signal will not be interpreted correctly.
 * * 2. TEMPERATURE SENSOR (DS18B20 on pin 4):
 * - 1-Wire module. Requires soldering a 4.7k Ohm pull-up resistor between the DATA pin and 3.3V.
 * * 3. VIBRATION SENSOR (BNO055 on I2C):
 * - SDA (usually 21) and SCL (usually 22) pins.
 * - Requires absolutely rigid, soldered connections with silicone insulation. Standard jumper 
 * wires ("duponts") will disconnect at the first shot due to vibration.
 * * 4. MICROPHONE (MAX4466 on pin 34):
 * - Power with clean 3.3V from ESP32. The built-in potentiometer on the back of the microphone 
 * module allows Gain adjustment. Set to the middle before the first calibration.
 * * * Required libraries: WiFi, WebServer, LittleFS, HTTPClient, ArduinoJson, ESP32Servo, 
 * Adafruit_BNO055, DallasTemperature
 * ---------------------------------------------------------------------------------------
 * [PL]
 * * * ⚠️ KRYTYCZNE UWAGI SPRZĘTOWE (HARDWARE) - PRZECZYTAJ ZANIM PODŁĄCZYSZ!
 * 1. ZASILANIE SERWOMECHANIZMÓW (DS3218MG i MG945):
 * - DS3218MG to potężne serwo (20kg), które przy zablokowaniu (twardy spust) potrafi pociągnąć 
 * nawet 2-3 Ampery. MG945 dorzuca swoje 1-1.5A.
 * - ABSOLUTNIE NIE ZASILAJ ICH Z PINU 5V (VIN) NA ESP32! Spadek napięcia zresetuje układ 
 * (Brownout) lub spali stabilizator na płytce NodeMCU/ESP32.
 * - ROZWIĄZANIE: Użyj zewnętrznej przetwornicy DC-DC Step-Down (tzw. UBEC/SBEC) o wydajności 
 * minimum 3A (zalecane 5A), dającej na wyjściu 5V lub 6V.
 * - PODŁĄCZENIE: Czerwone kable od serw podłącz do plusa (+) wyjścia przetwornicy.
 * - KRYTYCZNE (WSPÓLNA MASA): Czarne/Brązowe kable od serw, minus (-) wyjścia przetwornicy 
 * ORAZ pin GND na ESP32 MUSZĄ BYĆ ZE SOBĄ POŁĄCZONE. Bez połączonej masy sygnał PWM nie 
 * zostanie poprawnie zinterpretowany.
 * * 2. CZUJNIK TEMPERATURY (DS18B20 na pinu 4):
 * - Moduł 1-Wire. Wymaga wlutowania rezystora podciągającego (Pull-up) o wartości 4.7k Ohm 
 * pomiędzy pinem danych (DATA) a zasilaniem 3.3V.
 * * 3. CZUJNIK DRGAŃ (BNO055 na I2C):
 * - Piny SDA (zazwyczaj 21) i SCL (zazwyczaj 22).
 * - Wymagane absolutnie sztywne, lutowane połączenia z izolacją silikonową. Zwykłe kable 
 * zworkowe ("duponty") rozłączą się przy pierwszym strzale od wibracji.
 * * 4. MIKROFON (MAX4466 na pinu 34):
 * - Zasil z czystego 3.3V od ESP32. Wbudowany potencjometr na z tyłu modułu mikrofonu pozwala 
 * na regulację wzmocnienia (Gain). Ustaw na środek przed pierwszą kalibracją.
 * * * Wymagane biblioteki: WiFi, WebServer, LittleFS, HTTPClient, ArduinoJson, ESP32Servo, 
 * Adafruit_BNO055, DallasTemperature
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

/*
 * =======================================================================================
 * [EN] NETWORK CONFIGURATION/[PL] KONFIGURACJA SIECI
 * * =======================================================================================
 */
const char* ssid = "SSID_here";
const char* password = "PASWORD_here";
const String OPENWEATHER_API_KEY = "API_here";
/*
 * [EN] Pins (adapted for ESP32)
 * [PL] Piny (dostosowane do ESP32)
 */
#define SERVO_SPUST_PIN 18 // [EN] Trigger: Yellow/Orange wire DS3218MG / [PL] Spust: Żółty/Pomarańczowy kabel DS3218MG
#define SERVO_SRUBA_PIN 19 // [EN] Power: Yellow/Orange wire MG945 / [PL] Śruba: Żółty/Pomarańczowy kabel MG945
#define DS18B20_PIN     4  // [EN] Reminder: use 4.7k resistor!
/ [PL] Przypomnienie: daj rezystor 4.7k!
#define MIC_PIN         34 // [EN] Analog mic pin (ADC1 pins work with WiFi. Pin 34 is ideal) / [PL] Analogowy pin mikrofonu (Tylko piny ADC1 działają z WiFi! Pin 34 jest idealny)

/*
 * =======================================================================================
 * [EN] SERVO CONFIGURATION
 * Trigger: DS3218MG (Digital, 20kg, metal gears)
 * N2 Screw (Power): MG945 (Analog/Digital, 12kg, metal gears)
 * Default Servo library values (1000-2000us) give only ~90 degrees of rotation on DS3218MG.
 * To get the full torque spectrum and 180-degree range, we extend the pulse to 500-2500us.
 * ---------------------------------------------------------------------------------------
 * [PL] KONFIGURACJA SERWOMECHANIZMÓW
 * Spust (Trigger): DS3218MG (Cyfrowe, 20kg, metalowe zębatki)
 * Śruba N2 (Power): MG945 (Analogowe/Cyfrowe, 12kg, metalowe zębatki)
 * Domyślne wartości biblioteki Servo (1000-2000us) dadzą na DS3218MG tylko ~90 stopni obrotu.
 * Aby mieć pełne spektrum momentu i zakres do 180 stopni, rozszerzamy impuls do 500-2500us.
 * =======================================================================================
 */
#define SERVO_MIN_PULSE_WIDTH 500
#define SERVO_MAX_PULSE_WIDTH 2500

/*
 * =======================================================================================
 * [EN] GLOBAL OBJECTS/[PL] OBIEKTY GLOBALNE
 * * =======================================================================================
 */
WebServer server(80);
Servo serwoSpust;
Servo serwoSruba;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensorsBarrel(&oneWire);
/*
 * =======================================================================================
 * [EN] GLOBAL VARIABLES & SESSION DATA/[PL] ZMIENNE GLOBALNE I DANE SESJI
 * * =======================================================================================
 */
volatile bool isShooting = false;
volatile float sharedMaxG = 0.0; 
volatile float sharedPeakDB = 0.0;
volatile float sharedAvgDB = 0.0;
/*
 * [EN] States for the shooting machine (Ensures timing precision and asynchronous execution)
 * [PL] Stany dla maszyny strzelającej (Zapewnia precyzję czasową i asynchroniczność)
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
volatile int shotsRemaining = 0;
volatile bool pauseRequested = false;
volatile bool requireVInput = false;
unsigned long stateStartTime = 0;

// [EN] Weather / [PL] Pogoda
float currentEnvTemp = 0.0;
float currentEnvHum = 0.0;
float currentEnvPress = 0.0;

/*
 * [EN] Session Configuration (Default GUI values)
 * [PL] Konfiguracja Sesji (Domyślne wartości GUI)
 */
String currentAmmoType = "szorstka";
float currentAmmoWeight = 3.0;
float currentDistance = 5.0; // [m]
int currentCadenceMs = 1000; // [ms]

int totalShotsFired = 0;
/*
 * =======================================================================================
 * [EN] FILE SYSTEM (LittleFS)/[PL] SYSTEM PLIKÓW (LittleFS)
 * * =======================================================================================
 */
void appendLogToCSV(int id, String typ, float waga, float dystans, float tempLufy500, float maxRecoil, float peakDB, float avgDB) {
  File file = LittleFS.open("/logs.csv", FILE_APPEND);
if (!file) return;
  file.printf("%d,%lu,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0.0\n", 
              id, millis(), typ.c_str(), waga, dystans, currentEnvTemp, 
              currentEnvHum, currentEnvPress, tempLufy500, maxRecoil, peakDB, avgDB);
file.close();
}

void updateVelocityInLog(int id, float velocity) {
  File file = LittleFS.open("/logs.csv", FILE_APPEND);
if (file) {
    file.printf("UPDATE_V,%d,%.2f\n", id, velocity);
    file.close();
}
}

/*
 * =======================================================================================
 * [EN] HARDWARE TASKS (CORE 1)/[PL] ZADANIA SPRZĘTOWE (CORE 1)
 * * =======================================================================================
 */
void taskHighSpeedMeasurements(void * pvParameters) {
  for(;;) {
    if (isShooting) {
      sharedMaxG = 0.0;
unsigned long sampleCount = 0;
      double sumSquares = 0;
      int signalMax = 0;
      int signalMin = 4095;
while (isShooting) {
        // [EN] 1. IMU Measurement (BNO055 Recoil) / [PL] 1. Pomiar IMU (Odrzut BNO055)
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
float current_g = sqrt(accel.x()*accel.x() + accel.y()*accel.y() + accel.z()*accel.z());
        if (current_g > sharedMaxG) sharedMaxG = current_g;
// [EN] 2. Microphone measurement (Full ESP32 ADC speed -> ~10-40kHz) / [PL] 2. Pomiar z mikrofonu (Pełna prędkość ADC ESP32 -> ~10-40kHz)
        int sample = analogRead(MIC_PIN);
if (sample > signalMax) signalMax = sample;
        if (sample < signalMin) signalMin = sample;
double volts = ((double)sample - 2048.0) / 2048.0; 
        sumSquares += (volts * volts);
        sampleCount++;
/*
         * [EN]
         * CRITICAL FOR RTOS: 
         * Yield releases CPU time for other tasks with the same priority (1).
* Because of this, the loop() function on the same core can count down the 500ms 
         * elapsed time and change the isShooting flag.
Without this, the system would freeze.
         * ---------------------------------------------------------------------------------------
         * [PL]
         * KRYTYCZNE DLA RTOS: 
         * Yield oddaje czas procesora dla innych tasków o tym samym priorytecie (1).
* Dzięki temu funkcja loop() na tym samym rdzeniu może odliczyć upływ 500ms
         * i zmienić flagę isShooting.
Bez tego - system by się powiesił.
         */
        yield();
}

      // [EN] Microphone calibration (Change this after a dry test with a physical decibel meter) / [PL] Kalibracja mikrofonu (Zmień to po teście na sucho z fizycznym decybelomierzem)
      float base_calibration_db = 110.0;
float peakVolts = ((float)(signalMax - signalMin)) / 4095.0;
      if (peakVolts <= 0.001) peakVolts = 0.001;
sharedPeakDB = 20.0 * log10(peakVolts) + base_calibration_db;
      
      if (sampleCount > 0) {
        double rms = sqrt(sumSquares / sampleCount);
if (rms <= 0.001) rms = 0.001;
        sharedAvgDB = 20.0 * log10(rms) + base_calibration_db;
}
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
}
}

/*
 * =======================================================================================
 * [EN] WEB SERVER - INTERFACE (HTML/JS)/[PL] WEB SERVER - INTERFEJS (HTML/JS)
 * * =======================================================================================
 */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Sentry Data Scout v0.3</title>
  <style>
    body { font-family: Arial, sans-serif; background-color: #121212; color: #e0e0e0; text-align: center; margin:0; padding:10px; }
    h2 { color: #00ff88; margin-bottom: 0px; }
    .author { font-size: 12px; color: #777; margin-top: 2px; margin-bottom: 15px; }
    h3 { border-bottom: 1px solid #444; padding-bottom: 5px; color: #fff; margin-top:0;}
    .card 
{ background-color: #1e1e1e; padding: 15px; border-radius: 8px; margin-bottom: 15px; box-shadow: 0 4px 8px rgba(0,0,0,0.5);
}
    
    button { border: none; color: white; border-radius: 5px; cursor: pointer; margin: 5px;
}
    .btn-small { background-color: #0088ff; padding: 10px 15px; font-size: 14px;
}
    .btn-small:active { background-color: #0055aa; }
    
    .btn-fire { font-size: 22px;
font-weight: bold; width: 100%; padding: 20px; border-radius: 8px; margin-top:10px; transition: background-color 0.3s; }
    .btn-green { background-color: #28a745;
}
    .btn-green:active { background-color: #1e7e34; }
    .btn-red { background-color: #dc3545;
}
    .btn-red:active { background-color: #a71d2a; }
    .btn-orange { background-color: #ff9800; color: #111;
}
    .btn-orange:active { background-color: #e65100; }
    .btn-cancel { background-color: #555; font-size: 14px; padding: 12px;
margin-top: 5px; width: 100%;}
    
    select, input { padding: 8px; font-size: 14px; border-radius: 4px;
border: 1px solid #555; background: #2c2c2c; color: #fff; margin: 3px; }
    select { cursor: pointer;
}
    .input-group { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px;
text-align: left;}
    .input-group label { flex: 1; font-size: 14px;
color: #bbb;}
  </style>
</head>
<body>
  <h2>Sentry Data Scout</h2>
  <div class="author">Autor: Maciej Kasperek (von Krappitz) | MR3 Platform |
wer 0.3</div>
  
  <div class="card">
    <div style="display:flex; justify-content:space-between; align-items:center;">
      <div style="text-align:left;">
        <p style="margin:2px;">Seria: <b id="shots">0</b> strzałów</p>
        <p style="margin:2px; font-size:12px; color:#888;"
id="env">Pogoda: --</p>
      </div>
      <button class="btn-small" onclick="getGPSAndWeather()">📍 Pogoda (GPS)</button>
    </div>
  </div>

  <div class="card">
    <h3>Konfiguracja Ognia</h3>
    
    <div class="input-group">
      <label>Moc (Śruba N2):</label>
      <div style="display:flex; align-items:center;">
        <input type="range" id="p_power" min="0" max="180" value="90" style="width:100px;"
onchange="setPower(this.value)">
        <span id="power_val" style="margin-left: 5px; width: 35px; text-align: right; color: #fff;">90&deg;</span>
      </div>
    </div>
    
    <div class="input-group">
      <label>Rodzaj strzału:</label>
      <select id="shot_mode">
        <option value="1">Pojedynczy (1)</option>
        <option value="3">Seria (3)</option>
        <option value="5">Seria (5)</option>
      </select>
    </div>
    
    <div class="input-group">
 
     <label>Rodzaj kulki:</label>
      <select id="p_type">
        <option value="szorstka">Szorstka</option>
        <option value="gładka">Gładka</option>
      </select>
    </div>

    <div class="input-group">
      <label>Waga [g]:</label>
      <div style="display:flex; gap:5px;">
        <select id="p_weight_sel" onchange="checkCustomWeight()" style="width:70px;">
          <option value="3.0">3g</option>
          <option value="5.0">5g</option>
      
    <option value="9.0">9g</option>
          <option value="inna">Inna</option>
        </select>
        <input type="number" id="p_weight_custom" style="display:none;
width:60px;" placeholder="np 4.5" step="0.1">
      </div>
    </div>

    <div class="input-group">
      <label>Odległość pomiaru V:</label>
      <select id="p_dist">
        <option value="0">0m</option>
        <option value="5">5m</option>
        <option value="10">10m</option>
        <option value="15">15m</option>
        <option value="20">20m</option>
        <option value="25">25m</option>
        <option value="50">50m</option>
    
  </select>
    </div>

    <div class="input-group">
      <label>Odstęp między strzałami [ms]:</label>
      <input type="number" id="p_cadence" value="1000" step="100" style="width:80px;">
    </div>

    <div class="input-group" style="margin-top: 10px;
background: #333; padding: 10px; border-radius: 5px;">
      <label style="color: #00ff88;
font-weight: bold;">Wymagaj wpisania V po każdym strzale:</label>
      <input type="checkbox" id="p_req_v" style="width: 20px;
height: 20px;">
    </div>

    <button id="btn_action" class="btn-fire btn-green" onclick="toggleAction()">▶ START</button>
    <button class="btn-cancel" onclick="stopAction()">⏹ ZAKOŃCZ / ANULUJ</button>
  </div>

  <div class="card">
    <h3>Dopisywanie Prędkości (V)</h3>
    <div class="input-group" style="justify-content:center;
gap:10px;">
      <input type="number" id="v_id" placeholder="ID Strzału" style="width:35%;">
      <input type="number" id="v_val" placeholder="V [m/s]" step="0.1" style="width:35%;">
      <button class="btn-small" onclick="saveVelocity()">✍️ Zapisz</button>
    </div>
    <br>
    <button class="btn-fire btn-green" style="font-size:16px;
padding:15px;" onclick="window.location.href='/logs.csv'">📥 Pobierz Logi CSV</button>
  </div>

  <span id="sys_state" style="display:none;">IDLE</span>

  <script>
    setInterval(updateStatus, 1000); 

    function checkCustomWeight() {
      let sel = document.getElementById('p_weight_sel').value;
      document.getElementById('p_weight_custom').style.display = (sel === 'inna') ? 'block' : 'none';
    }
    
    function setPower(val) {
      document.getElementById('power_val').innerHTML = val + "&deg;";
      fetch(`/api/power?val=${val}`, {method: 'POST'});
    }

    function updateStatus() {
      fetch('/api/status').then(r => r.json()).then(data => {
 
       document.getElementById('shots').innerText = data.totalShots;
        if(data.envPress > 0) {
          document.getElementById('env').innerText = data.envTemp.toFixed(1) + "°C, " + data.envPress.toFixed(0) + "hPa";
        }
        
        let state = data.state;
        document.getElementById('sys_state').innerText = state;
let btn = document.getElementById('btn_action');
        
        if (state === "IDLE") {
            btn.className = "btn-fire btn-green";
btn.style.backgroundColor = "";
            btn.innerText = "▶ START";
        } else if (state === "PAUSED") {
            btn.className = "btn-fire btn-orange";
btn.style.backgroundColor = "";
            btn.innerText = "▶ WZNÓW (PAUZA)";
        } else if (state === "WAITING_V") {
            btn.className = "btn-fire";
btn.style.backgroundColor = "#007bff"; 
            btn.innerText = "⏳ CZEKAM NA V z CHRONO";
            document.getElementById('v_id').value = data.totalShots;
} else {
            btn.className = "btn-fire btn-red";
btn.style.backgroundColor = "";
            btn.innerText = "⏸ PAUZA";
        }
      });
}

    function toggleAction() {
      let state = document.getElementById('sys_state').innerText;
if (state === "IDLE") {
        let t = encodeURIComponent(document.getElementById('p_type').value);
        let selW = document.getElementById('p_weight_sel').value;
let w = (selW === 'inna') ? document.getElementById('p_weight_custom').value : selW;
        if(!w) w = 3.0; 
        let d = document.getElementById('p_dist').value;
let c = document.getElementById('p_cadence').value;
        let reqV = document.getElementById('p_req_v').checked ? 1 : 0;
        let count = document.getElementById('shot_mode').value;
fetch(`/api/params?type=${t}&weight=${w}&dist=${d}&cadence=${c}&reqV=${reqV}`, {method: 'POST'})
          .then(() => fetch(`/api/control?cmd=start&count=${count}`, {method: 'POST'}));
} else if (state === "RUNNING") {
        fetch('/api/control?cmd=pause', {method: 'POST'});
} else if (state === "PAUSED") {
        fetch('/api/control?cmd=resume', {method: 'POST'});
} else if (state === "WAITING_V") {
        saveVelocity();
}
      setTimeout(updateStatus, 200); 
    }

    function stopAction() {
      fetch('/api/control?cmd=stop', {method: 'POST'});
setTimeout(updateStatus, 200);
    }

    function saveVelocity() {
      let id = document.getElementById('v_id').value;
let val = document.getElementById('v_val').value;
      if(!id || !val) { alert("Wypełnij ID i Prędkość!"); return;
}
      fetch(`/api/save_v?id=${id}&v=${val}`, {method: 'POST'})
        .then(r => { 
          alert('Zapisano prędkość!'); 
          document.getElementById('v_id').value=''; 
          document.getElementById('v_val').value=''; 
        });
}

    function getGPSAndWeather() {
      if (navigator.geolocation) {
        document.getElementById('env').innerText = "Szukam satelity...";
navigator.geolocation.getCurrentPosition(position => {
          let lat = position.coords.latitude;
          let lon = position.coords.longitude;
          document.getElementById('env').innerText = "Łączenie z chmurą...";
          fetch(`/api/weather?lat=${lat}&lon=${lon}`, {method: 'POST'})
            .then(r => r.text()).then(t => { alert(t); updateStatus(); });
        }, err => alert("Błąd GPS: " + err.message));
} else {
        alert("Brak wsparcia GPS w przeglądarce.");
}
    }
  </script>
</body>
</html>
)rawliteral";

/*
 * =======================================================================================
 * [EN] WEB SERVER - ENDPOINTS/[PL] WEB SERVER - ENDPOINTY
 * * =======================================================================================
 */
void setupWebEndpoints() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

  server.on("/api/status", HTTP_GET, []() {
    StaticJsonDocument<200> doc;
    doc["envTemp"] = currentEnvTemp;
    doc["envHum"] = currentEnvHum;
    doc["envPress"] = currentEnvPress;
    doc["totalShots"] = totalShotsFired;
    
    if (sysState == SYS_IDLE) doc["state"] = "IDLE";
    else if (sysState == SYS_PAUSED) doc["state"] = "PAUSED";
   
  else if (sysState == SYS_WAITING_FOR_V) doc["state"] = "WAITING_V";
    else doc["state"] = "RUNNING";
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
});

  server.on("/api/control", HTTP_POST, []() {
    if (!server.hasArg("cmd")) {
      server.send(400, "text/plain", "Brak komendy"); // [EN] No command / [PL] Brak komendy
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
        sysState = SYS_SHOOTING_PRESS;
      }
    } else if (cmd == "stop") {
      shotsRemaining = 0;
      pauseRequested = false;
      sysState = SYS_IDLE;
      isShooting = false;
    }
    server.send(200, "text/plain", "OK");
  });
server.on("/api/params", HTTP_POST, []() {
    if (server.hasArg("type")) currentAmmoType = server.arg("type");
    if (server.hasArg("weight")) currentAmmoWeight = server.arg("weight").toFloat();
    if (server.hasArg("dist")) currentDistance = server.arg("dist").toFloat();
    if (server.hasArg("cadence")) currentCadenceMs = server.arg("cadence").toInt();
    if (server.hasArg("reqV")) requireVInput = (server.arg("reqV") == "1");
    server.send(200, "text/plain", "OK");
  });
server.on("/api/power", HTTP_POST, []() {
    if (server.hasArg("val")) {
      int angle = server.arg("val").toInt();
      serwoSruba.write(angle); 
      server.send(200, "text/plain", "Moc zmieniona"); // [EN] Power changed / [PL] Moc zmieniona
    } else {
      server.send(400, "text/plain", "Brak wartosci"); // [EN] No value / [PL] Brak wartości
    }
  });
server.on("/api/save_v", HTTP_POST, []() {
    if (server.hasArg("id") && server.hasArg("v")) {
      updateVelocityInLog(server.arg("id").toInt(), server.arg("v").toFloat());
      
      if (sysState == SYS_WAITING_FOR_V) {
        if (shotsRemaining > 0) {
          sysState = SYS_WAITING_CADENCE;
          stateStartTime = millis(); 
        } else {
          sysState = SYS_IDLE; 
      
  }
      }
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Brak ID"); // [EN] No ID / [PL] Brak ID
    }
  });
server.on("/api/weather", HTTP_POST, []() {
    if (server.hasArg("lat") && server.hasArg("lon")) {
      float lat = server.arg("lat").toFloat();
      float lon = server.arg("lon").toFloat();
      HTTPClient http;
      String url = "https://api.openweathermap.org/data/2.5/weather?lat=" + String(lat, 4) + "&lon=" + String(lon, 4) + "&units=metric&appid=" + OPENWEATHER_API_KEY;
      http.begin(url);
      if (http.GET() == HTTP_CODE_OK) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, http.getString());
        currentEnvTemp 
= doc["main"]["temp"];
        currentEnvHum = doc["main"]["humidity"];
        currentEnvPress = doc["main"]["pressure"];
        server.send(200, "text/plain", "Pogoda zaktualizowana!"); // [EN] Weather updated! / [PL] Pogoda zaktualizowana!
      } else {
        server.send(500, "text/plain", "Błąd API pogodowego"); // [EN] Weather API error / [PL] Błąd API pogodowego
      }
      http.end();
    } else {
      server.send(400, "text/plain", "Brak GPS");
// [EN] No GPS / [PL] Brak GPS
    }
  });
server.on("/logs.csv", HTTP_GET, []() {
    File file = LittleFS.open("/logs.csv", "r");
    if (!file) {
      server.send(404, "text/plain", "Plik logów pusty."); // [EN] Log file empty. / [PL] Plik logów pusty.
      return;
    }
    server.streamFile(file, "text/csv");
    file.close();
  });
}

/*
 * =======================================================================================
 * [EN] NETWORK TASK (CORE 0)/[PL] ZADANIA SIECIOWE (CORE 0)
 * * =======================================================================================
 */
void taskNetworkCore0(void * pvParameters) {
  setupWebEndpoints();
server.begin();
  for(;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}
}

/*
 * =======================================================================================
 * [EN/PL] SETUP & LOOP
 * * =======================================================================================
 */
void setup() {
  Serial.begin(115200);
  
  if (!LittleFS.begin(true)) return;
if (!LittleFS.exists("/logs.csv")) {
    File file = LittleFS.open("/logs.csv", FILE_WRITE);
    file.println("ID,Timestamp,Typ_Kulki,Waga_Kulki,Odleglosc_Pomiaru,Temp_O,Wilgotnosc,Cisnienie,Temp_Lufy_500ms,Max_Odrzut,Peak_dB,Sredni_dB,Zmierzone_V");
    file.close();
}

  // [EN] Critical PWM configuration for powerful servos (DS3218MG / MG945) / [PL] Krytyczna konfiguracja PWM dla mocnych serw (DS3218MG / MG945)
  ESP32PWM::allocateTimer(0);
ESP32PWM::allocateTimer(1);
  serwoSpust.setPeriodHertz(50); // [EN] Standard 50Hz for hobby servos / [PL] Standardowe 50Hz dla serw modelarskich
  serwoSruba.setPeriodHertz(50);
// [EN] Initialization with precise pulse limits ensuring full 180 degrees / [PL] Inicjalizacja z precyzyjnymi limitami impulsów gwarantującymi pełne 180 stopni
  serwoSpust.attach(SERVO_SPUST_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
serwoSruba.attach(SERVO_SRUBA_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
  
  serwoSpust.write(0);
  serwoSruba.write(90); // [EN] Start position of N2 screw / [PL] Pozycja startowa śruby N2
  
  sensorsBarrel.begin();
sensorsBarrel.setResolution(9); 
  sensorsBarrel.setWaitForConversion(false); // [EN] CRITICAL: Asynchronous reading to prevent 94ms loop blocking!
/ [PL] KRYTYCZNE: Odczyt asynchroniczny zapobiegający blokowaniu pętli o 94ms!
  
  bno.begin();
  pinMode(MIC_PIN, INPUT);
  
  WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED) { delay(500); }

  // [EN] Splitting tasks across cores (Core 0 for WiFi, Core 1 for hardware) / [PL] Rozbicie zadań na rdzenie (Core 0 dla WiFi, Core 1 dla sprzętu)
  xTaskCreatePinnedToCore(taskNetworkCore0, "TaskNet0", 8192, NULL, 1, NULL, 0);
/*
   * [EN]
   * CRITICAL (Protection against Task Starvation):
   * Priority changed from 2 to 1!
Equality with loop() task priority - which is also 1.
   * Allows yield() inside taskHighSpeedMeasurements to fairly give CPU time to the loop() 
   * function, enabling equal 500ms countdown during sound analysis.
* ---------------------------------------------------------------------------------------
   * [PL]
   * KRYTYCZNE (Zabezpieczenie przed zagłodzeniem tasku / Task Starvation):
   * Zmiana priorytetu z 2 na 1!
Zrównanie z priorytetem zadania loop() - które również jest 1.
   * Pozwala funkcji yield() wewnątrz taskHighSpeedMeasurements na sprawiedliwe oddawanie czasu
   * procesora do funkcji loop(), umożliwiając równe odmierzanie 500ms podczas analizy dźwięku.
*/
  xTaskCreatePinnedToCore(taskHighSpeedMeasurements, "TaskHiSpeed1", 4096, NULL, 1, NULL, 1); 
}

/*
 * =======================================================================================
 * [EN] MAIN LOOP (CORE 1) - Non-blocking!
Uses state machine.
 * [PL] PĘTLA GŁÓWNA (CORE 1) - Nie blokuje! Używa maszyny stanów.
* =======================================================================================
 */
void loop() {
  unsigned long now = millis();
// [EN] Background reading of slow sensors (request) - to keep fresh data before and after the series / [PL] Odczyt wolnych sensorów w tle (żądanie) - żeby utrzymywać świeże dane przed i po serii
  static unsigned long lastSensorTime = 0;
if (now - lastSensorTime > 2000 && (sysState == SYS_IDLE || sysState == SYS_PAUSED)) {
    sensorsBarrel.requestTemperatures();
lastSensorTime = now;
  }

  // [EN] Asynchronous state machine controlling shot countdown / [PL] Asynchroniczna maszyna stanów sterująca odliczaniem strzałów
  switch (sysState) {
    case SYS_IDLE:
      break;
case SYS_SHOOTING_PRESS:
      isShooting = true;     
      serwoSpust.write(180); 
      sensorsBarrel.requestTemperatures();
// [EN] Request asynchronous measurement upon shot / [PL] Żądanie asynchronicznego pomiaru przy strzale
      stateStartTime = now;
sysState = SYS_SHOOTING_RELEASE;
      break;

    case SYS_SHOOTING_RELEASE:
      if (now - stateStartTime >= 50) { 
        serwoSpust.write(0);
sysState = SYS_SHOOTING_WAIT_500;
      }
      break;
case SYS_SHOOTING_WAIT_500:
      if (now - stateStartTime >= 500) { 
        isShooting = false;
// [EN] Immediate non-blocking read (result from SYS_SHOOTING_PRESS request) / [PL] Odczyt natychmiastowy bez blokowania (wynik zlecenia z SYS_SHOOTING_PRESS)
        float tempLufy500ms = sensorsBarrel.getTempCByIndex(0);
totalShotsFired++;
        appendLogToCSV(totalShotsFired, currentAmmoType, currentAmmoWeight, currentDistance, tempLufy500ms, sharedMaxG, sharedPeakDB, sharedAvgDB);

        shotsRemaining--;
// [EN] UX function for field tests - pauses system waiting for POST command from /api/save_v / [PL] Funkcja UX do badań terenowych - wstrzymuje system oczekując na komendę POST z /api/save_v
        if (requireVInput) {
          sysState = SYS_WAITING_FOR_V;
} else {
          if (shotsRemaining > 0) {
            if (pauseRequested) sysState = SYS_PAUSED;
else sysState = SYS_WAITING_CADENCE;
          } else {
            sysState = SYS_IDLE;
pauseRequested = false;
          }
        }
      }
      break;
case SYS_WAITING_CADENCE:
      if (pauseRequested) {
        sysState = SYS_PAUSED;
} else if (now - stateStartTime >= currentCadenceMs) {
        sysState = SYS_SHOOTING_PRESS;
}
      break;

    case SYS_PAUSED:
    case SYS_WAITING_FOR_V:
      break;
}

  vTaskDelay(10 / portTICK_PERIOD_MS); 
}