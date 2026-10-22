#define ENABLE_WIFI_DEBUG true

#if ENABLE_WIFI_DEBUG
#include <WiFi.h>
#include <WebServer.h>
const char* AP_SSID = "RobotDebug";
const char* AP_PASS = "12345678";
WebServer server(80);
#endif

const int PIN_BATTERY = 0;
const int PIN_MOTOR_L_IN1 = 1;
const int PIN_MOTOR_L_IN2 = 2;
const int PIN_MOTOR_R_IN3 = 3;
const int PIN_MOTOR_R_IN4 = 4;
const int PIN_SWITCH = 5;
const int PIN_SENS_1 = 6;
const int PIN_SENS_2 = 7;
const int PIN_SENS_3 = 10;
const int PIN_SENS_4 = 20;
const int PIN_SENS_5 = 21;
const int PIN_LED_RED = 8;

float Kp = 35.0;
float Ki = 0.0;
float Kd = 15.0;

int baseSpeed = 130;
int maxSpeed = 255;

float error = 0;
float lastError = 0;
float integral = 0;

unsigned long lastBatCheck = 0;
const float batThreshold = 3.2;
float calibFactor = 0.89;

int debugRawBat = 0;
float debugVoltage = 0.0;
int dbgS1 = 0, dbgS2 = 0, dbgS3 = 0, dbgS4 = 0, dbgS5 = 0;
int dbgSpeedL = 0, dbgSpeedR = 0;
float dbgCorr = 0.0;

#if ENABLE_WIFI_DEBUG
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: monospace; padding: 10px; background: #121212; color: #00ff00; }
textarea { width: 100%; height: 350px; background: #000; color: #00ff00; border: 1px solid #333; font-size: 12px; }
h2 { color: #fff; margin-bottom: 5px; }
.row { margin-bottom: 8px; font-size: 14px; }
b { color: #fff; }
</style>
<script>
setInterval(() => {
  fetch('/data').then(r => r.json()).then(d => {
    document.getElementById('bat').innerText = d.v.toFixed(2) + ' V (Raw: ' + d.raw + ')';
    document.getElementById('sens').innerText = '[' + d.s.join(' ') + ']';
    document.getElementById('err').innerText = d.err;
    document.getElementById('mot').innerText = 'L: ' + d.ml + ' | R: ' + d.mr;
    
    let logMsg = `Err: ${d.err} | PID: ${d.corr.toFixed(1)} | L: ${d.ml} R: ${d.mr} | Sens: [${d.s.join('')}]\n`;
    let cons = document.getElementById('console');
    cons.value = logMsg + cons.value.substring(0, 3000);
  });
}, 500);
</script>
</head><body>
<h2>Robot Console</h2>
<div class="row"><b>Bat:</b> <span id="bat">-</span></div>
<div class="row"><b>Sensors:</b> <span id="sens">-</span></div>
<div class="row"><b>Error:</b> <span id="err">-</span></div>
<div class="row"><b>Motors:</b> <span id="mot">-</span></div>
<textarea id="console" readonly></textarea>
</body></html>
)rawliteral";

void handleRoot() {
    server.send(200, "text/html", index_html);
}

void handleData() {
    String json = "{";
    json += "\"v\":" + String(debugVoltage, 2) + ",";
    json += "\"raw\":" + String(debugRawBat) + ",";
    json += "\"err\":" + String(error) + ",";
    json += "\"s\":[" + String(dbgS1) + "," + String(dbgS2) + "," + String(dbgS3) + "," + String(dbgS4) + "," + String(dbgS5) + "],";
    json += "\"ml\":" + String(dbgSpeedL) + ",";
    json += "\"mr\":" + String(dbgSpeedR) + ",";
    json += "\"corr\":" + String(dbgCorr);
    json += "}";
    server.send(200, "application/json", json);
}
#endif

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_BATTERY, INPUT);
    analogReadResolution(12);
    
    pinMode(PIN_LED_RED, OUTPUT);
    digitalWrite(PIN_LED_RED, LOW);
    
    pinMode(PIN_SWITCH, INPUT_PULLUP);
    
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN3, OUTPUT);
    pinMode(PIN_MOTOR_R_IN4, OUTPUT);
    
    pinMode(PIN_SENS_1, INPUT);
    pinMode(PIN_SENS_2, INPUT);
    pinMode(PIN_SENS_3, INPUT);
    pinMode(PIN_SENS_4, INPUT);
    pinMode(PIN_SENS_5, INPUT);

#if ENABLE_WIFI_DEBUG
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.begin();
#endif
    
    delay(1000);
}

void loop() {
#if ENABLE_WIFI_DEBUG
    server.handleClient();
#endif

    checkBattery();
    readSensorsAndCalculateError();
    runPID();
}

void checkBattery() {
    if (millis() - lastBatCheck < 1000) return;
    lastBatCheck = millis();
    
    int rawValue = 0;
    for (int i = 0; i < 10; i++) {
        rawValue += analogRead(PIN_BATTERY);
        delay(2);
    }
    rawValue /= 10;
    
    float pinVoltage = (rawValue / 4095.0) * 3.3;
    float batteryVoltage = pinVoltage * 2.0 * calibFactor;
    
    debugRawBat = rawValue;
    debugVoltage = batteryVoltage;
    
    if (batteryVoltage < batThreshold) {
        digitalWrite(PIN_LED_RED, HIGH);
    } else {
        digitalWrite(PIN_LED_RED, LOW);
    }
}

void setMotors(int speedL, int speedR) {
    if (speedL >= 0) {
        analogWrite(PIN_MOTOR_L_IN1, speedL);
        analogWrite(PIN_MOTOR_L_IN2, 0);
    } else {
        analogWrite(PIN_MOTOR_L_IN1, 0);
        analogWrite(PIN_MOTOR_L_IN2, -speedL);
    }
    
    if (speedR >= 0) {
        analogWrite(PIN_MOTOR_R_IN3, speedR);
        analogWrite(PIN_MOTOR_R_IN4, 0);
    } else {
        analogWrite(PIN_MOTOR_R_IN3, 0);
        analogWrite(PIN_MOTOR_R_IN4, -speedR);
    }
}

void readSensorsAndCalculateError() {
    int invertLogic = (digitalRead(PIN_SWITCH) == LOW) ? 1 : 0;
    
    dbgS1 = digitalRead(PIN_SENS_1) ^ invertLogic;
    dbgS2 = digitalRead(PIN_SENS_2) ^ invertLogic;
    dbgS3 = digitalRead(PIN_SENS_3) ^ invertLogic;
    dbgS4 = digitalRead(PIN_SENS_4) ^ invertLogic;
    dbgS5 = digitalRead(PIN_SENS_5) ^ invertLogic;
    
    if      (dbgS1 == 0 && dbgS2 == 0 && dbgS3 == 0 && dbgS4 == 0 && dbgS5 == 1) error = 4;
    else if (dbgS1 == 0 && dbgS2 == 0 && dbgS3 == 0 && dbgS4 == 1 && dbgS5 == 1) error = 3;
    else if (dbgS1 == 0 && dbgS2 == 0 && dbgS3 == 0 && dbgS4 == 1 && dbgS5 == 0) error = 2;
    else if (dbgS1 == 0 && dbgS2 == 0 && dbgS3 == 1 && dbgS4 == 1 && dbgS5 == 0) error = 1;
    else if (dbgS1 == 0 && dbgS2 == 0 && dbgS3 == 1 && dbgS4 == 0 && dbgS5 == 0) error = 0;
    else if (dbgS1 == 0 && dbgS2 == 1 && dbgS3 == 1 && dbgS4 == 0 && dbgS5 == 0) error = -1;
    else if (dbgS1 == 0 && dbgS2 == 1 && dbgS3 == 0 && dbgS4 == 0 && dbgS5 == 0) error = -2;
    else if (dbgS1 == 1 && dbgS2 == 1 && dbgS3 == 0 && dbgS4 == 0 && dbgS5 == 0) error = -3;
    else if (dbgS1 == 1 && dbgS2 == 0 && dbgS3 == 0 && dbgS4 == 0 && dbgS5 == 0) error = -4;
    else if (dbgS1 == 0 && dbgS2 == 0 && dbgS3 == 0 && dbgS4 == 0 && dbgS5 == 0) {
        if (lastError > 0) error = 5;
        else if (lastError < 0) error = -5;
    }
}

void runPID() {
    float P = error;
    integral += error;
    float D = error - lastError;
    
    float correction = (Kp * P) + (Ki * integral) + (Kd * D);
    lastError = error;
    dbgCorr = correction;
    
    int speedL = baseSpeed + correction;
    int speedR = baseSpeed - correction;
    
    dbgSpeedL = constrain(speedL, -maxSpeed, maxSpeed);
    dbgSpeedR = constrain(speedR, -maxSpeed, maxSpeed);
    
    setMotors(dbgSpeedL, dbgSpeedR);
}