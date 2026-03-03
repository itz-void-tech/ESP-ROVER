#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ================= PINS & CONFIG =================
#define I2C_SDA 8
#define I2C_SCL 9
#define GPS_RX_PIN 17 
#define GPS_TX_PIN 18 

#define MPU_ADDR 0x68
#define HMC_ADDR 0x1E
#define QMC_ADDR 0x0D

// === CAR PINS ===
#define TRIG_PIN 4
#define ECHO_PIN 5
#define BUZZER_PIN 6
#define ENA 10
#define IN1 11
#define IN2 12
#define IN3 13
#define IN4 14
#define ENB 15

const int REQUIRED_UNIQUE_SAMPLES = 800; 

// ================= GLOBALS =================
WebServer server(80);
Preferences preferences;
TinyGPSPlus gps;

bool mpuOk = false;
bool magOk = false;
uint8_t magAddr = 0x00;
String magType = "NONE";
bool isHMC = false;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float heading = 0.0;
float declination = 0.0; // Adjust for your city

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;

bool isCalibratingMag = false;
int magCalSamplesCollected = 0;
int16_t lastRawX, lastRawY, lastRawZ; 
float magMinX = 32767, magMaxX = -32768;
float magMinY = 32767, magMaxY = -32768;
float magMinZ = 32767, magMaxZ = -32768;

unsigned long lastConsoleUpdate = 0;

// === CAR GLOBALS ===
int motorSpeed = 178; 
char currentDir = 'S';
bool isFallen = false;
bool isTilted = false;
bool isCrashed = false;
bool isRepelling = false;
bool userHornActive = false;
unsigned long lastObstacleCheck = 0;
unsigned long crashTime = 0;

// ================= WEB DASHBOARD HTML/CSS/JS =================
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Advanced Telemetry & Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { background-color: #0d1117; color: #00ff00; font-family: 'Courier New', Courier, monospace; margin: 10px; display: flex; flex-direction: column; align-items: center;}
    h1 { color: #58a6ff; border-bottom: 1px solid #30363d; padding-bottom: 10px; width: 100%; text-align: center; font-size: 1.2em;}
    .container { max-width: 600px; width: 100%; }
    .card { background: #161b22; border: 1px solid #30363d; border-radius: 6px; padding: 15px; margin-bottom: 15px; }
    button { background: #238636; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; font-weight: bold; margin-right: 10px; width: 100%; margin-bottom: 10px;}
    button:hover { background: #2ea043; }
    .btn-danger { background: #da3633; }
    .btn-danger:hover { background: #f85149; }
    button:disabled { background: #444; cursor: not_allowed; }
    table { width: 100%; text-align: left; font-size: 0.8em; }
    th, td { padding: 3px; }
    
    #progress-container { width: 100%; background-color: #30363d; border-radius: 5px; margin: 10px 0; display: none; }
    #progress-bar { width: 0%; height: 20px; background-color: #238636; text-align: center; line-height: 20px; color: white; border-radius: 5px; transition: width 0.1s; }
    #instructions { color: #8b949e; font-size: 0.85em; display: none; border-left: 3px solid #f2cc60; padding-left: 10px; margin-bottom: 10px;}

    #compass-svg { width: 150px; height: 150px; display: block; margin: 10px auto; }
    .compass-ring { stroke: #30363d; stroke-width: 2; fill: none; }
    .compass-degree-mark { stroke: #30363d; stroke-width: 1; }
    .compass-text { fill: #8b949e; font-size: 14px; text-anchor: middle; font-family: sans-serif;}
    .compass-text-main { fill: #ffffff; font-weight: bold; font-size: 18px;}
    #compass-needle { fill: #da3633; transition: transform 0.2s ease-out; transform-origin: 75px 75px; }
    #heading-val { font-size: 2em; color: #58a6ff; text-align: center; }

    .drive-btn { background: #444; padding: 20px 5px; margin: 2px; }
    .drive-btn:active { background: #58a6ff; }
    #car-status { color: #00ff00; text-align: center; font-weight: bold; margin-bottom: 10px; font-size: 1.1em; transition: 0.2s;}
    input[type=range] { width: 90%; margin: 15px 0; }
    .speed-label { font-size: 1.1em; font-weight: bold; color: #58a6ff; }
  </style>
</head>
<body>
  <h1>[ ESP32-S3 ] TELEMETRY & CONTROL</h1>
  
  <div class="container">
    <div class="card">
      <h3>MAGNETOMETER CALIBRATION</h3>
      <div id="instructions">
        <strong>ACTION REQUIRED:</strong><br>
        1. Keep sensor FLAT for 2 seconds.<br>
        2. Rotate in a slow Figure-8 in the air.<br>
        3. Tilt diagonally while rotating.
      </div>
      <div id="progress-container"><div id="progress-bar">0%</div></div>
      <p id="cal-status" style="color:#00ff00; text-align:center;">Status: IDLE</p>
      <button id="btn-start" onclick="startCalibration()">Start Real Calibration</button>
      <button class="btn-danger" id="btn-reset" onclick="fetch('/reset_cal')">Reset to Defaults</button>
    </div>

    <div class="card">
      <h3>CAR CONTROLS</h3>
      <div id="car-status">SYSTEM READY</div>
      <div style="display:grid; grid-template-columns: 1fr 1fr 1fr; gap: 5px; text-align:center;">
         <div></div>
         <button class="drive-btn" onmousedown="drive('F')" onmouseup="drive('S')" ontouchstart="drive('F')" ontouchend="drive('S')">FWD</button>
         <div></div>
         
         <button class="drive-btn" onmousedown="drive('L')" onmouseup="drive('S')" ontouchstart="drive('L')" ontouchend="drive('S')">LEFT</button>
         <button class="drive-btn btn-danger" onmousedown="horn(1)" onmouseup="horn(0)" ontouchstart="horn(1)" ontouchend="horn(0)">HORN</button>
         <button class="drive-btn" onmousedown="drive('R')" onmouseup="drive('S')" ontouchstart="drive('R')" ontouchend="drive('S')">RIGHT</button>
         
         <div></div>
         <button class="drive-btn" onmousedown="drive('B')" onmouseup="drive('S')" ontouchstart="drive('B')" ontouchend="drive('S')">REV</button>
         <div></div>
      </div>
      
      <div style="text-align: center; margin-top: 20px; border-top: 1px solid #30363d; padding-top: 15px;">
        <span class="speed-label">SPEED: <span id="speed-val">70</span>%</span><br>
        <input type="range" id="speed-slider" min="0" max="100" value="70" 
               oninput="document.getElementById('speed-val').innerText = this.value" 
               onchange="updateSpeed(this.value)">
      </div>
    </div>

    <div class="card">
      <div id="heading-val">0.0&deg;</div>
      <svg id="compass-svg" viewBox="0 0 150 150">
        <circle class="compass-ring" cx="75" cy="75" r="70"/>
        <line class="compass-degree-mark" x1="75" y1="5" x2="75" y2="15" /> <line class="compass-degree-mark" x1="145" y1="75" x2="135" y2="75" /> <line class="compass-degree-mark" x1="75" y1="145" x2="75" y2="135" /> <line class="compass-degree-mark" x1="5" y1="75" x2="15" y2="75" /> <text x="75" y="30" class="compass-text compass-text-main">N</text>
        <text x="130" y="80" class="compass-text">E</text> <text x="75" y="135" class="compass-text">S</text> <text x="20" y="80" class="compass-text">W</text>
        <g id="compass-needle">
          <polygon points="75,10 85,75 75,90 65,75" />
          <circle cx="75" cy="75" r="5" fill="#58a6ff"/>
        </g>
      </svg>
    </div>

    <div class="card">
      <h3>SYSTEM DATA</h3>
      <div id="data">Loading...</div>
    </div>
  </div>

  <script>
    function startCalibration() {
      document.getElementById('instructions').style.display = 'block';
      document.getElementById('progress-container').style.display = 'block';
      document.getElementById('btn-start').disabled = true;
      document.getElementById('btn-reset').disabled = true;
      fetch('/start_cal');
    }

    function drive(dir) { fetch('/move?dir=' + dir); }
    function horn(state) { fetch('/horn?s=' + state); }
    function updateSpeed(val) { fetch('/speed?v=' + val); }

    setInterval(() => {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('compass-needle').style.transform = `rotate(${data.heading}deg)`;
          document.getElementById('heading-val').innerHTML = `${data.heading.toFixed(1)}&deg;`;

          let statusEl = document.getElementById('car-status');
          if (data.crashed) {
            statusEl.innerHTML = "<span style='color:red; font-size:1.3em;'>🚨 CRASH DETECTED 🚨</span>";
          } else if (data.fallen) {
            statusEl.innerHTML = "<span style='color:red'>WARNING: VEHICLE OVERTURNED!</span>";
          } else if (data.repelling) {
            statusEl.innerHTML = "<span style='color:#ff00ff'>REPULSION: OBJECT TOO CLOSE!</span>";
          } else if (data.tilted) {
            statusEl.innerHTML = "<span style='color:orange'>WARNING: HIGH TILT ANGLE</span>";
          } else {
            statusEl.innerHTML = "SYSTEM NOMINAL";
          }

          if(data.calibrating) {
            document.getElementById('cal-status').innerText = `Status: COLLECTING DATA...`;
            document.getElementById('cal-status').style.color = "#f2cc60";
            document.getElementById('progress-bar').style.width = `${data.cal_pct}%`;
            document.getElementById('progress-bar').innerText = `${data.cal_pct}%`;
          } else {
            document.getElementById('cal-status').innerText = "Status: IDLE (Offsets active)";
            document.getElementById('cal-status').style.color = "#00ff00";
          }

          document.getElementById('data').innerHTML = `
            <table>
              <tr><th>Mag Sensor</th><td>${data.magOk ? 'OK ('+data.magType+')' : '<span style="color:red">FAIL</span>'}</td></tr>
              <tr><th>Mag Offsets</th><td>X:${data.offX}, Y:${data.offY}, Z:${data.offZ}</td></tr>
              <tr><th>Accel (X,Y,Z)</th><td>${data.aX.toFixed(2)}, ${data.aY.toFixed(2)}, ${data.aZ.toFixed(2)}</td></tr>
              <tr><th>Obstacle Dist</th><td>${data.obsDist} cm</td></tr>
            </table>
          `;
        });
    }, 200);
  </script>
</body>
</html>
)=====";

// ================= FUNCTIONS =================

void moveCar(char dir) {
  if (isCrashed || isFallen) {
    analogWrite(ENA, 0); analogWrite(ENB, 0); 
    return;
  }
  if (isRepelling && dir != 'B') {
    return; 
  }

  currentDir = dir;
  switch (dir) {
    case 'F': 
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, motorSpeed); analogWrite(ENB, motorSpeed);
      break;
    case 'B': 
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      analogWrite(ENA, motorSpeed); analogWrite(ENB, motorSpeed);
      break;
    case 'L': 
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      analogWrite(ENA, motorSpeed); analogWrite(ENB, motorSpeed);
      break;
    case 'R': 
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, motorSpeed); analogWrite(ENB, motorSpeed);
      break;
    case 'S': 
    default:
      analogWrite(ENA, 0); analogWrite(ENB, 0);
      break;
  }
}

long getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if(duration == 0) return -1; 
  return duration * 0.034 / 2;
}

void runCarSafetyLogic() {
  if (!mpuOk) return;

  // 1. Crash Detection
  if (abs(accX) > 2.5 || abs(accY) > 2.5) {
    if (!isCrashed) {
      isCrashed = true;
      crashTime = millis();
      moveCar('S');
    }
  }
  if (isCrashed && millis() - crashTime > 3000) {
    isCrashed = false;
  }

  // 2. Tilt & Fall Detection
  if (!isCrashed) {
    if (abs(accZ) < 0.3) { 
      isFallen = true; isTilted = false; moveCar('S'); 
    } else if (abs(accX) > 0.6 || abs(accY) > 0.6) {
      isTilted = true; isFallen = false;
    } else {
      isFallen = false; isTilted = false;
    }
  }

  // 3. Obstacle Repulsion
  if (millis() - lastObstacleCheck > 100) {
    long dist = getUltrasonicDistance();
    if (dist > 0 && dist < 10 && !isFallen && !isCrashed) {
      if (!isRepelling) isRepelling = true;
      moveCar('B'); 
    } else {
      if (isRepelling) {
        isRepelling = false;
        moveCar('S'); 
      }
    }
    lastObstacleCheck = millis();
  }

  // 4. Buzzer Manager
  if (userHornActive) {
    tone(BUZZER_PIN, 1000); 
  } else if (isCrashed) {
    tone(BUZZER_PIN, 2500); 
  } else if (isFallen) {
    if ((millis() / 250) % 2 == 0) tone(BUZZER_PIN, 1500); 
    else noTone(BUZZER_PIN);
  } else if (isRepelling) {
    if ((millis() / 100) % 2 == 0) tone(BUZZER_PIN, 500); 
    else noTone(BUZZER_PIN);
  } else {
    noTone(BUZZER_PIN); 
  }
}

// === I2C SENSOR INITIALIZATION AND READING ===

void scanI2C() {
  Serial.println("================================================");
  Serial.println("[ SYSTEM BOOT ] ESP32-S3 ONLINE");
  Serial.print("[I2C SCAN] Found: ");
  bool foundAny = false;
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) { Serial.print("0x68 (MPU) "); foundAny = true; }
  
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission() == 0) { Serial.print("0x0D (QMC) "); foundAny = true; }
  else {
    Wire.beginTransmission(HMC_ADDR);
    if (Wire.endTransmission() == 0) { Serial.print("0x1E (HMC) "); foundAny = true; }
  }

  if (!foundAny) Serial.print("NONE");
  Serial.println();
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); 
  if (Wire.endTransmission() == 0) { mpuOk = true; Serial.println("[MPU] OK"); } 
}

void initMag() {
  // Try QMC5883L
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission() == 0) {
    magOk = true; magAddr = QMC_ADDR; magType = "QMC5883L"; isHMC = false;
    Wire.beginTransmission(magAddr); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission(); 
    Wire.beginTransmission(magAddr); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
    Serial.println("[MAG] OK (QMC5883L)");
    return;
  }
  
  // Try HMC5883L
  Wire.beginTransmission(HMC_ADDR);
  if (Wire.endTransmission() == 0) {
    magOk = true; magAddr = HMC_ADDR; magType = "HMC5883L"; isHMC = true;
    Wire.beginTransmission(magAddr); Wire.write(0x02); Wire.write(0x00); Wire.endTransmission(); 
    Serial.println("[MAG] OK (HMC5883L)");
    return;
  }
  Serial.println("[MAG] FAIL - Check Wiring!");
}

void loadCalibration() {
  preferences.begin("mag_cal", true); 
  magOffsetX = preferences.getFloat("offX", 0.0);
  magOffsetY = preferences.getFloat("offY", 0.0);
  magOffsetZ = preferences.getFloat("offZ", 0.0);
  preferences.end();
}

void saveCalibration() {
  preferences.begin("mag_cal", false); 
  preferences.putFloat("offX", magOffsetX);
  preferences.putFloat("offY", magOffsetY);
  preferences.putFloat("offZ", magOffsetZ);
  preferences.end();
}

void readMPU() {
  if (!mpuOk) return;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  accX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  accY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  accZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
}

void readMag() {
  if (!magOk) return;
  int16_t rawX, rawY, rawZ;
  
  if (isHMC) {
    Wire.beginTransmission(magAddr); Wire.write(0x03); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)magAddr, (uint8_t)6, (uint8_t)true);
    rawX = (Wire.read() << 8) | Wire.read(); rawZ = (Wire.read() << 8) | Wire.read(); rawY = (Wire.read() << 8) | Wire.read();
  } else {
    Wire.beginTransmission(magAddr); Wire.write(0x00); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)magAddr, (uint8_t)6, (uint8_t)true);
    rawX = Wire.read() | (Wire.read() << 8); rawY = Wire.read() | (Wire.read() << 8); rawZ = Wire.read() | (Wire.read() << 8);
  }

  if (isCalibratingMag) {
    if (abs(rawX - lastRawX) > 15 || abs(rawY - lastRawY) > 15 || abs(rawZ - lastRawZ) > 15) {
        if (rawX < magMinX) magMinX = rawX; if (rawX > magMaxX) magMaxX = rawX;
        if (rawY < magMinY) magMinY = rawY; if (rawY > magMaxY) magMaxY = rawY;
        if (rawZ < magMinZ) magMinZ = rawZ; if (rawZ > magMaxZ) magMaxZ = rawZ;
        magCalSamplesCollected++;
        lastRawX = rawX; lastRawY = rawY; lastRawZ = rawZ;
    }
    
    if (magCalSamplesCollected >= REQUIRED_UNIQUE_SAMPLES) {
      isCalibratingMag = false;
      magOffsetX = (magMaxX + magMinX) / 2.0;
      magOffsetY = (magMaxY + magMinY) / 2.0;
      magOffsetZ = (magMaxZ + magMinZ) / 2.0;
      saveCalibration();
      Serial.println("[MAG] Calibration Saved.");
    }
  }

  magX = rawX - magOffsetX; 
  magY = rawY - magOffsetY; 
  magZ = rawZ - magOffsetZ;
}

void calculateHeading() {
  if (!magOk) return;
  heading = atan2(magY, magX) * 180.0 / M_PI;
  heading += declination;
  if (heading < 0) heading += 360;
  if (heading > 360) heading -= 360;
}

// ================= WEB SERVER LOGIC =================

void handleWebRequests() {
  server.on("/", []() { server.send(200, "text/html", INDEX_HTML); });

  server.on("/data", []() {
    int progress = 0;
    if (isCalibratingMag) progress = (int)(((float)magCalSamplesCollected / REQUIRED_UNIQUE_SAMPLES) * 100);

    long currentDist = getUltrasonicDistance();

    String json = "{";
    json += "\"mpuOk\":" + String(mpuOk ? "true" : "false") + ",";
    json += "\"magOk\":" + String(magOk ? "true" : "false") + ",";
    json += "\"magType\":\"" + magType + "\",";
    json += "\"heading\":" + String(heading, 1) + ",";
    json += "\"aX\":" + String(accX, 2) + ",\"aY\":" + String(accY, 2) + ",\"aZ\":" + String(accZ, 2) + ",";
    json += "\"offX\":" + String(magOffsetX, 0) + ",\"offY\":" + String(magOffsetY, 0) + ",\"offZ\":" + String(magOffsetZ, 0) + ",";
    json += "\"calibrating\":" + String(isCalibratingMag ? "true" : "false") + ",";
    json += "\"cal_pct\":" + String(progress) + ",";
    json += "\"fallen\":" + String(isFallen ? "true" : "false") + ",";
    json += "\"tilted\":" + String(isTilted ? "true" : "false") + ",";
    json += "\"crashed\":" + String(isCrashed ? "true" : "false") + ",";
    json += "\"repelling\":" + String(isRepelling ? "true" : "false") + ",";
    json += "\"obsDist\":" + String(currentDist);
    json += "}";
    
    server.send(200, "application/json", json);
  });

  server.on("/move", []() {
    moveCar(server.arg("dir")[0]);
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/speed", []() {
    int percentage = server.arg("v").toInt();
    motorSpeed = map(percentage, 0, 100, 0, 255);
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/horn", []() {
    if (server.arg("s") == "1") userHornActive = true;
    else userHornActive = false;
    server.send(200, "text/plain", "OK");
  });

  server.on("/start_cal", []() {
    magMinX = 32767; magMaxX = -32768; magMinY = 32767; magMaxY = -32768; magMinZ = 32767; magMaxZ = -32768;
    magCalSamplesCollected = 0;
    lastRawX = 0; lastRawY = 0; lastRawZ = 0;
    isCalibratingMag = true; 
    server.send(200, "text/plain", "OK");
  });

  server.on("/reset_cal", []() {
    magOffsetX = 0; magOffsetY = 0; magOffsetZ = 0;
    saveCalibration();
    server.send(200, "text/plain", "OK");
  });

  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Wire.begin(I2C_SDA, I2C_SCL);
  
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  scanI2C();
  initMPU(); 
  initMag(); 
  loadCalibration();
  
  WiFi.softAP("ESP32_TELEMETRY", "12345678");
  Serial.print("[WIFI] Access Point Started. IP: "); Serial.println(WiFi.softAPIP());
  
  handleWebRequests();
}

void loop() {
  readMPU();
  readMag(); // THIS IS WHAT WAS MISSING!
  calculateHeading();
  
  runCarSafetyLogic(); 
  
  server.handleClient();
}
