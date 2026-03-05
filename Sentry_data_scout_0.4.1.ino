/**
 * =======================================================================================
 * Project: Sentry Data Scout
 * Version: 0.4 SUPER FINAL (Wizards & Weight Select)
 * Opis: Pełna wersja z kalibracją krokową (Wizard), seriami, pomiarem G i dB, pogodą, 
 * mDNS, zapisem kalibracji oraz listą wyboru wagi kulek.
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

// ================== KONFIGURACJA ==================
const char* ssid = "SSID_here";
const char* password = "PASSWORD_here";
const String OPENWEATHER_API_KEY = "API_here";

#define SERVO_SPUST_PIN 18
#define SERVO_SRUBA_PIN 19
#define DS18B20_PIN 4
#define MIC_PIN 34
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

// ================== OBIEKTY ==================
WebServer server(80);
Servo serwoSpust;
Servo serwoSruba;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensorsBarrel(&oneWire);

// ================== KALIBRACJA (zapisuje się do LittleFS) ==================
int cal_spust_min = 0, cal_spust_max = 180;
int cal_sruba_min = 0, cal_sruba_max = 180;

// ================== ZMIENNE GLOBALNE ==================
volatile bool isShooting = false;
volatile float sharedMaxG = 0.0, sharedPeakDB = 0.0, sharedAvgDB = 0.0;

enum SystemState { SYS_IDLE, SYS_SHOOTING_PRESS, SYS_SHOOTING_RELEASE, SYS_SHOOTING_WAIT_500, 
                   SYS_WAITING_CADENCE, SYS_PAUSED, SYS_WAITING_FOR_V };
volatile SystemState sysState = SYS_IDLE;

volatile int shotsRemaining = 0;
volatile bool pauseRequested = false;
volatile bool requireVInput = false;
unsigned long stateStartTime = 0;

float currentEnvTemp = 0.0, currentEnvHum = 0.0, currentEnvPress = 0.0;
String currentAmmoType = "szorstka";
float currentAmmoWeight = 3.2;
float currentDistance = 5.0;
int currentCadenceMs = 1000;
int totalShotsFired = 0;

// ================== LOGOWANIE ==================
void appendLogToCSV(int id, String typ, float waga, float dystans, float tempLufy500, float maxRecoil, float peakDB, float avgDB) {
  File file = LittleFS.open("/logs.csv", FILE_APPEND);
  if (!file) return;
  file.printf("%d,%lu,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0.0\n",
              id, millis(), typ.c_str(), waga, dystans, currentEnvTemp, currentEnvHum, currentEnvPress,
              tempLufy500, maxRecoil, peakDB, avgDB);
  file.close();
}

void updateVelocityInLog(int id, float v) {
  File file = LittleFS.open("/logs.csv", FILE_APPEND);
  if (file) { file.printf("UPDATE_V,%d,%.2f\n", id, v); file.close(); }
}

// ================== ZAPIS / ODCZYT KALIBRACJI ==================
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

// ================== TASK HIGH-SPEED (Core 1) ==================
void taskHighSpeedMeasurements(void * pvParameters) {
  for(;;) {
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

// ================== HTML ==================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Sentry Data Scout v0.4 SUPER FINAL</title>
  <style>
    body {font-family:sans-serif;background:#121212;color:#e0e0e0;text-align:center;padding:10px;margin:0}
    .card {background:#1e1e1e;padding:15px;border-radius:8px;margin-bottom:15px;box-shadow:0 4px 8px rgba(0,0,0,0.5)}
    h2 {color:#00ff88} h3 {color:#00d4ff;border-bottom:1px solid #444;padding-bottom:5px}
    button {border:none;color:white;border-radius:5px;cursor:pointer;margin:5px}
    .btn-fire {font-size:22px;font-weight:bold;width:100%;padding:20px;border-radius:8px;margin-top:10px}
    .btn-green {background:#28a745} .btn-red {background:#dc3545} .btn-orange {background:#ff9800;color:#111}
    .btn-small {background:#0088ff;padding:10px 15px}
    .input-group {display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
  </style>
</head>
<body>
  <h2>Sentry Data Scout v0.4 SUPER FINAL</h2>
  <div class="card"><div style="display:flex;justify-content:space-between">
    <div><p>Seria: <b id="shots">0</b></p><p id="env">Pogoda: --</p></div>
    <button class="btn-small" onclick="getWeather()">📍 Pogoda</button>
  </div></div>

  <div class="card">
    <h3>⚙️ Kalibracja Sprzętowa</h3>
    
    <div style="text-align:left; border-bottom:1px solid #333; padding-bottom:10px; margin-bottom:10px;">
      <div class="input-group"><span style="color:#00ff88; font-weight:bold;">Spust:</span><b id="s_val">0°</b></div>
      <div id="wizard_spust">
        <input type="range" id="s_live" min="0" max="180" value="0" style="width:100%" oninput="moveLive('spust',this.value)">
        <p style="font-size:12px; color:#aaa; margin:5px 0;">1. Przesuwaj aż nastąpi strzał, potem kliknij ZAPISZ STRZAŁ.<br>2. Cofaj aż do luzu, potem kliknij ZAPISZ LUZ.</p>
        <div style="display:flex; gap:5px;">
          <button class="btn-small btn-red" style="flex:1" onclick="zapiszKrokSpustu('max')">🎯 ZAPISZ STRZAŁ</button>
          <button class="btn-small btn-green" style="flex:1" onclick="zapiszKrokSpustu('min')">😌 ZAPISZ LUZ</button>
        </div>
      </div>
      <button id="btn_rekal_spust" class="btn-small" style="display:none; width:100%; background:#555;" onclick="pokazWizard('spust')">🔄 Rekalibruj Spust</button>
    </div>

    <div style="text-align:left;">
      <div class="input-group"><span style="color:#00ff88; font-weight:bold;">Śruba N2:</span><b id="p_val">90°</b></div>
      <div id="wizard_sruba">
        <input type="range" id="p_live" min="0" max="180" value="90" style="width:100%" oninput="moveLive('sruba',this.value)">
        <p style="font-size:12px; color:#aaa; margin:5px 0;">1. Ustaw śrubę fizycznie, załóż serwo i kliknij ZAPISZ ZERO.<br>2. Przesuń suwak na żądaną max moc i kliknij ZAPISZ MAX.</p>
        <div style="display:flex; gap:5px;">
          <button class="btn-small" style="flex:1; background:#888;" onclick="zapiszKrokSruby('min')">0️⃣ ZAPISZ ZERO</button>
          <button class="btn-small btn-orange" style="flex:1" onclick="zapiszKrokSruby('max')">🔥 ZAPISZ MAX</button>
        </div>
      </div>
      <button id="btn_rekal_sruba" class="btn-small" style="display:none; width:100%; background:#555;" onclick="pokazWizard('sruba')">🔄 Rekalibruj Śrubę N2</button>
    </div>
  </div>

  <div class="card">
    <h3>🎯 Kontrola Ognia</h3>
    <div class="input-group"><label>Moc (0-100%):</label><input type="range" id="p_power" min="0" max="100" value="50" onchange="setPower(this.value)"></div>
    <div class="input-group"><label>Tryb:</label><select id="shot_mode"><option value="1">1</option><option value="3">3</option><option value="5">5</option></select></div>
    <div class="input-group"><label>Rodzaj kulki:</label><select id="p_type"><option value="szorstka">Szorstka</option><option value="gładka">Gładka</option></select></div>
    
    <div class="input-group">
      <label>Waga [g]:</label>
      <div style="display:flex; gap:5px;">
        <select id="p_weight_sel" onchange="checkCustomWeight()" style="width:70px;">
          <option value="3.0">3.0g</option>
          <option value="3.2" selected>3.2g</option>
          <option value="5.0">5.0g</option>
          <option value="inna">Inna</option>
        </select>
        <input type="number" id="p_weight_custom" style="display:none; width:60px;" placeholder="np 4.5" step="0.1">
      </div>
    </div>

    <div class="input-group"><label>Dystans [m]:</label><input type="number" id="p_dist" value="5" style="width:70px"></div>
    <div class="input-group"><label>Odstęp [ms]:</label><input type="number" id="p_cadence" value="1000" style="width:80px"></div>
    <div class="input-group"><label>Wymagaj V:</label><input type="checkbox" id="p_req_v"></div>
    <button id="btn_action" class="btn-fire btn-green" onclick="toggleAction()">▶ START</button>
    <button class="btn-small" style="width:100%;margin-top:8px;background:#555" onclick="stopAction()">⏹ STOP</button>
  </div>

  <div class="card">
    <h3>Dopisywanie V</h3>
    <div style="display:flex;gap:5px">
      <input id="v_id" placeholder="ID" style="width:30%">
      <input id="v_val" placeholder="V [m/s]" style="width:40%">
      <button class="btn-small" onclick="saveV()">✍️</button>
    </div>
    <button class="btn-fire btn-green" style="font-size:14px;padding:10px;margin-top:10px" onclick="location.href='/logs.csv'">📥 Pobierz CSV</button>
  </div>

  <script>
    setInterval(updateStatus, 1000);
    function moveLive(t,v){ document.getElementById(t==='spust'?'s_val':'p_val').innerText=v+'°'; fetch(`/api/live?target=${t}&val=${v}`); }
    function saveCal(t,m){ let v=document.getElementById(t==='spust'?'s_live':'p_live').value; fetch(`/api/set_cal?target=${t}&mode=${m}&val=${v}`); }
    function setPower(v){ fetch(`/api/power?val=${v}`); }
    function getWeather(){ navigator.geolocation.getCurrentPosition(p=>fetch(`/api/weather?lat=${p.coords.latitude}&lon=${p.coords.longitude}`)); }

    function checkCustomWeight() {
      let sel = document.getElementById('p_weight_sel').value;
      document.getElementById('p_weight_custom').style.display = (sel === 'inna') ? 'block' : 'none';
    }

    function updateStatus(){
      fetch('/api/status').then(r=>r.json()).then(d=>{
        document.getElementById('shots').innerText=d.totalShots;
        if(d.envPress>0) document.getElementById('env').innerText=d.envTemp.toFixed(1)+'°C, '+d.envPress+'hPa';
        let btn=document.getElementById('btn_action');
        if(d.state==='IDLE'){ btn.className='btn-fire btn-green'; btn.innerText='▶ START'; }
        else if(d.state==='PAUSED'){ btn.className='btn-fire btn-orange'; btn.innerText='▶ WZNÓW'; }
        else if(d.state==='WAITING_V'){ btn.className='btn-fire btn-orange'; btn.innerText='⏳ WPISZ V'; document.getElementById('v_id').value=d.totalShots; }
        else { btn.className='btn-fire btn-red'; btn.innerText='⏸ PAUZA'; }
      });
    }

    function toggleAction(){
      fetch('/api/status').then(r=>r.json()).then(d=>{
        if(d.state==='IDLE'){
          let selW = document.getElementById('p_weight_sel').value;
          let w = (selW === 'inna') ? document.getElementById('p_weight_custom').value : selW;
          if(!w) w = 3.2;

          let params = `type=${document.getElementById('p_type').value}&weight=${w}&dist=${document.getElementById('p_dist').value}&cadence=${document.getElementById('p_cadence').value}&reqV=${document.getElementById('p_req_v').checked?1:0}`;
          fetch(`/api/params?${params}`).then(()=>fetch(`/api/control?cmd=start&count=${document.getElementById('shot_mode').value}`));
        } else if(d.state==='PAUSED') fetch('/api/control?cmd=resume');
        else if(d.state==='WAITING_V') saveV();
        else fetch('/api/control?cmd=pause');
      });
    }
    function stopAction(){ fetch('/api/control?cmd=stop'); }
    function saveV(){ 
      fetch(`/api/save_v?id=${document.getElementById('v_id').value}&v=${document.getElementById('v_val').value}`)
        .then(()=>{ document.getElementById('v_val').value=''; });
    }

    // Zmienne stanów dla Wizardów
    let spustStan = { maxZapisany: false, minZapisany: false };
    let srubaStan = { minZapisany: false, maxZapisany: false };

    function zapiszKrokSpustu(krok) {
      saveCal('spust', krok);
      if(krok === 'max') spustStan.maxZapisany = true;
      if(krok === 'min') spustStan.minZapisany = true;
      if(spustStan.maxZapisany && spustStan.minZapisany) {
        document.getElementById('wizard_spust').style.display = 'none';
        document.getElementById('btn_rekal_spust').style.display = 'block';
      }
    }

    function zapiszKrokSruby(krok) {
      saveCal('sruba', krok);
      if(krok === 'min') srubaStan.minZapisany = true;
      if(krok === 'max') srubaStan.maxZapisany = true;
      if(srubaStan.minZapisany && srubaStan.maxZapisany) {
        document.getElementById('wizard_sruba').style.display = 'none';
        document.getElementById('btn_rekal_sruba').style.display = 'block';
      }
    }

    function pokazWizard(target) {
      if(target === 'spust') {
        document.getElementById('wizard_spust').style.display = 'block';
        document.getElementById('btn_rekal_spust').style.display = 'none';
        spustStan = { maxZapisany: false, minZapisany: false };
      } else {
        document.getElementById('wizard_sruba').style.display = 'block';
        document.getElementById('btn_rekal_sruba').style.display = 'none';
        srubaStan = { minZapisany: false, maxZapisany: false };
      }
    }
  </script>
</body></html>
)rawliteral";

// ================== ENDPOINTY ==================
void setupWebEndpoints() {
  server.on("/", HTTP_GET, [](){ server.send(200, "text/html", index_html); });

  server.on("/api/status", HTTP_GET, [](){
    StaticJsonDocument<256> doc;
    doc["envTemp"] = currentEnvTemp; doc["envPress"] = currentEnvPress; doc["totalShots"] = totalShotsFired;
    if(sysState==SYS_IDLE) doc["state"]="IDLE";
    else if(sysState==SYS_PAUSED) doc["state"]="PAUSED";
    else if(sysState==SYS_WAITING_FOR_V) doc["state"]="WAITING_V";
    else doc["state"]="RUNNING";
    String r; serializeJson(doc, r); server.send(200, "application/json", r);
  });

  server.on("/api/live", HTTP_GET, [](){
    String t = server.arg("target"); int v = server.arg("val").toInt();
    if(t=="spust") serwoSpust.write(v); else serwoSruba.write(v);
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/set_cal", HTTP_GET, [](){
    String t = server.arg("target"), m = server.arg("mode"); 
    int v = constrain(server.arg("val").toInt(), 0, 180); // Zabezpieczenie przed wartościami z kosmosu
    if(t=="spust"){ if(m=="min") cal_spust_min=v; else cal_spust_max=v; }
    else { if(m=="min") cal_sruba_min=v; else cal_sruba_max=v; }
    saveCalibration();
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/power", HTTP_GET, [](){
    int p = server.arg("val").toInt();
    serwoSruba.write(map(p, 0, 100, cal_sruba_min, cal_sruba_max));
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/params", HTTP_GET, [](){
    if(server.hasArg("type")) currentAmmoType = server.arg("type");
    if(server.hasArg("weight")) currentAmmoWeight = server.arg("weight").toFloat();
    if(server.hasArg("dist")) currentDistance = server.arg("dist").toFloat();
    if(server.hasArg("cadence")) currentCadenceMs = server.arg("cadence").toInt();
    if(server.hasArg("reqV")) requireVInput = (server.arg("reqV")=="1");
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/control", HTTP_GET, [](){
    String cmd = server.arg("cmd");
    if(cmd=="start"){ shotsRemaining = server.arg("count").toInt(); sysState = SYS_SHOOTING_PRESS; pauseRequested = false; }
    else if(cmd=="pause") pauseRequested = true;
    else if(cmd=="resume"){ pauseRequested = false; if(sysState==SYS_PAUSED){ stateStartTime=millis(); sysState=SYS_WAITING_CADENCE; } }
    else if(cmd=="stop"){ shotsRemaining=0; sysState=SYS_IDLE; isShooting=false; }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/save_v", HTTP_GET, [](){
    updateVelocityInLog(server.arg("id").toInt(), server.arg("v").toFloat());
    if(sysState==SYS_WAITING_FOR_V){
      if(shotsRemaining>0){ stateStartTime=millis(); sysState=SYS_WAITING_CADENCE; }
      else sysState=SYS_IDLE;
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/weather", HTTP_GET, [](){
    HTTPClient http;
    String url = "https://api.openweathermap.org/data/2.5/weather?lat=" + server.arg("lat") + "&lon=" + server.arg("lon") + "&units=metric&appid=" + OPENWEATHER_API_KEY;
    http.begin(url);
    if(http.GET()==HTTP_CODE_OK){
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, http.getString());
      currentEnvTemp = doc["main"]["temp"];
      currentEnvHum = doc["main"]["humidity"];
      currentEnvPress = doc["main"]["pressure"];
    }
    http.end();
    server.send(200, "text/plain", "OK");
  });

  server.on("/logs.csv", HTTP_GET, [](){
    File f = LittleFS.open("/logs.csv", "r");
    if(f){ server.streamFile(f, "text/csv"); f.close(); }
  });
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  LittleFS.begin(true);

  if(!LittleFS.exists("/logs.csv")){
    File f = LittleFS.open("/logs.csv", FILE_WRITE);
    f.println("ID,Timestamp,Typ,Waga,Dystans,TempO,Wilg,Cisnienie,TempLufy,MaxG,PeakdB,AvgdB,V");
    f.close();
  }

  loadCalibration();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  serwoSpust.attach(SERVO_SPUST_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  serwoSruba.attach(SERVO_SRUBA_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  serwoSpust.write(cal_spust_min);
  serwoSruba.write(map(50, 0, 100, cal_sruba_min, cal_sruba_max));

  sensorsBarrel.begin(); sensorsBarrel.setResolution(9); sensorsBarrel.setWaitForConversion(false);

  bno.begin(); pinMode(MIC_PIN, INPUT);

  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) delay(500);

  MDNS.begin("sentry");
  setupWebEndpoints();
  server.begin();

  xTaskCreatePinnedToCore([](void*){for(;;){server.handleClient(); vTaskDelay(10/portTICK_PERIOD_MS);}}, "Net", 8192, NULL, 1, NULL, 0);

  xTaskCreatePinnedToCore(taskHighSpeedMeasurements, "Meas", 4096, NULL, 1, NULL, 1);

  Serial.println("Sentry Data Scout v0.4 SUPER FINAL GOTOWY → http://sentry.local");
}

// ================== LOOP (Core 1) ==================
void loop() {
  unsigned long now = millis();

  static unsigned long lastSensor = 0;
  if (now - lastSensor > 2000 && sysState == SYS_IDLE) {
    sensorsBarrel.requestTemperatures();
    lastSensor = now;
  }

  switch (sysState) {
    case SYS_SHOOTING_PRESS:
      isShooting = true;
      serwoSpust.write(cal_spust_max);
      sensorsBarrel.requestTemperatures();

      stateStartTime = now;
      sysState = SYS_SHOOTING_RELEASE;
      break;

    case SYS_SHOOTING_RELEASE:
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
        appendLogToCSV(totalShotsFired, currentAmmoType, currentAmmoWeight, currentDistance, tempL, sharedMaxG, sharedPeakDB, sharedAvgDB);
        
        shotsRemaining--;

        if (requireVInput) {
          sysState = SYS_WAITING_FOR_V;

        } else if (shotsRemaining > 0) {
          stateStartTime = millis();

          sysState = pauseRequested ? SYS_PAUSED : SYS_WAITING_CADENCE;
        } else {
          sysState = SYS_IDLE;

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