#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================================================================
// ZuriMobile - pin map (see the wiring reference for the full diagram)
// ===================================================================
// L298N motor driver
#define PIN_ENA   32   // left motor  - PWM speed
#define PIN_IN1   33   // left motor  - direction
#define PIN_IN2   25   // left motor  - direction
#define PIN_IN3   26   // right motor - direction
#define PIN_IN4   27   // right motor - direction
#define PIN_ENB   14   // right motor - PWM speed

// Right side - HC-SR04 ultrasonic, fixed forward-facing (no servo/pan anymore)
#define PIN_TRIG      4
#define PIN_ECHO      34   // input-only pin, fed through a 5V->3.3V divider

// Left side - digital IR obstacle sensor, fixed forward-facing
// Most cheap IR obstacle modules pull the output LOW when they see an obstacle.
#define PIN_IR_LEFT   13   // reuses the pin freed up by removing the radar servo

// OLED (I2C)
#define PIN_SDA   21
#define PIN_SCL   22
#define OLED_W    128
#define OLED_H    64
#define OLED_ADDR 0x3C

// Passive buzzer - needs tone()'s oscillating signal, not a plain digitalWrite
#define PIN_BUZZER 18

// ===================================================================
// Tunables
// ===================================================================
const int MOTOR_SPEED_DEFAULT = 255;   // 0-255, manual driving speed (full speed)
const int MOTOR_SPEED_AUTO    = 255;   // 0-255, self-driving cruise speed (full speed)
const int SAFE_DISTANCE_CM    = 15;     // reverse below this, resume forward once clear again on both sensors
const unsigned long SONAR_TIMEOUT_US  = 25000UL; // ~4m max range
const unsigned long SONAR_MIN_INTERVAL_MS = 60;  // min gap between HC-SR04 pings - shorter causes ghost-echo readings
const unsigned long CMD_TIMEOUT_MS    = 800;     // auto-stop if the phone goes quiet mid-drive

// Auto-mode evasion: back off straight, then turn away from whichever side
// tripped, then resume forward. MAX_REVERSE_MS caps a single backing-up
// attempt - there's no rear sensor, so reversing has to stay bounded rather
// than continuing indefinitely until "clear" (repeating the whole cycle a
// few times to escape a tight space is fine; one unbounded reverse is not).
const unsigned long MAX_REVERSE_MS = 1200;
const unsigned long EVADE_TURN_MS  = 500;

// Sonar calibration: how long each phase (quiet / under motor load) samples for
const unsigned long CALIBRATION_SAMPLE_MS = 1000;

// If a wheel ever spins the wrong way for its command, flip its sign here -
// no need to go hunting through the motor logic below.
const int LEFT_MOTOR_INVERT  = +1;   // +1 = normal, -1 = physically reversed
const int RIGHT_MOTOR_INVERT = -1;   // +1 = normal, -1 = physically reversed
                                     // left/right differ: the two sides are mounted mirror-imaged

// WiFi access point - connect your phone to this network, then browse to
// http://192.168.4.1  (or http://zurimobile.local on iOS / mDNS-aware Android)
const char* AP_SSID = "ZuriMobile";
const char* AP_PASS = "zurimobile";  // WPA2 needs >= 8 chars

WebServer server(80);
DNSServer dnsServer;   // answers every lookup with our IP so phones pop the "Sign in" captive portal
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

enum AppMode { MODE_MENU, MODE_MANUAL, MODE_AUTO };
enum MoveState { MV_STOP, MV_FORWARD, MV_BACKWARD, MV_LEFT, MV_RIGHT };
enum AutoState { AUTO_FORWARD, AUTO_REVERSE, AUTO_TURN };

AppMode mode = MODE_MENU;
MoveState moveState = MV_STOP;
int manualSpeed = MOTOR_SPEED_DEFAULT;
int autoSpeed   = MOTOR_SPEED_AUTO;     // adjustable from the web slider while in self-driving
long lastRightDistance = -1;
bool lastLeftBlocked = false;

AutoState autoState = AUTO_FORWARD;
unsigned long autoPhaseStart = 0;
bool obstacleWasOnLeft = true;         // which side triggered the current reverse - turn away from it after backing off

// Sonar noise filtering - default is a conservative guess, replaced with a
// measured value once /calibrate has run (see calibrateSonar() below).
long sonarNoiseMarginCm = 25;
bool sonarCalibrated = false;

unsigned long lastEyeUpdate = 0;
unsigned long lastCmdTime = 0;
MoveState lastDrawnMove = MV_STOP;

// ===================================================================
// Buzzer - short status chirps. tone(..., durationMs) is non-blocking on
// ESP32 (it schedules its own stop), so these are safe to call from HTTP
// handlers without stalling the web server.
// ===================================================================
void beep(unsigned int freqHz, unsigned long durationMs) {
  tone(PIN_BUZZER, freqHz, durationMs);
}

// ===================================================================
// Motor control
// ===================================================================
void setLeftMotor(int speed) {              // speed: -255..255
  speed *= LEFT_MOTOR_INVERT;
  speed = constrain(speed, -255, 255);
  digitalWrite(PIN_IN1, speed >= 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, speed >= 0 ? LOW : HIGH);
  analogWrite(PIN_ENA, abs(speed));
}

void setRightMotor(int speed) {              // speed: -255..255
  speed *= RIGHT_MOTOR_INVERT;
  speed = constrain(speed, -255, 255);
  digitalWrite(PIN_IN3, speed >= 0 ? HIGH : LOW);
  digitalWrite(PIN_IN4, speed >= 0 ? LOW : HIGH);
  analogWrite(PIN_ENB, abs(speed));
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  moveState = MV_STOP;
}

void driveForward(int speed) {
  setLeftMotor(speed);
  setRightMotor(speed);
  moveState = MV_FORWARD;
}

void driveBackward(int speed) {
  setLeftMotor(-speed);
  setRightMotor(-speed);
  moveState = MV_BACKWARD;
}

void turnLeft(int speed) {                   // left wheel stopped, right wheel full power
  setLeftMotor(0);
  setRightMotor(speed);
  moveState = MV_LEFT;
}

void turnRight(int speed) {                  // right wheel stopped, left wheel full power
  setLeftMotor(speed);
  setRightMotor(0);
  moveState = MV_RIGHT;
}

// ===================================================================
// Sensors - right ultrasonic (HC-SR04), left IR obstacle sensor
// ===================================================================

// One raw ping, honoring the HC-SR04's minimum re-trigger spacing (datasheet
// recommends 60ms+ between pings, or the next trigger picks up leftover
// acoustic reverberation from the last one and returns garbage). No noise
// filtering here - calibrateSonar() needs the unfiltered value to measure
// how noisy the sensor actually is.
long pingSonarOnce() {
  static unsigned long lastPingTime = 0;
  static long lastRaw = -1;
  unsigned long now = millis();
  if (lastPingTime != 0 && now - lastPingTime < SONAR_MIN_INTERVAL_MS) {
    return lastRaw;
  }
  lastPingTime = now;

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, SONAR_TIMEOUT_US);
  Serial.printf("Sonar raw: echo pulse = %lu us (0 = timed out / no echo seen)\n", duration);

  lastRaw = (duration == 0) ? -1 : (long)(duration / 58);
  return lastRaw;
}

// Filtered reading used by auto-drive logic. A single sample that jumps more
// than sonarNoiseMarginCm from the last accepted value is held back and only
// trusted once the same jump repeats on the next ping - that's enough to
// reject the one-off reverberation/EMI glitches motor current induces on
// battery power, without meaningfully delaying a real distance change.
// sonarNoiseMarginCm starts at a guessed default and gets replaced with a
// measured value by calibrateSonar() (see /calibrate).
long readDistanceCm() {
  long raw = pingSonarOnce();
  long sample = (raw < 0) ? 999 : raw;

  static long accepted = 999;
  static long pendingValue = -1;
  static int pendingCount = 0;

  if (labs(sample - accepted) <= sonarNoiseMarginCm) {
    accepted = sample;
    pendingCount = 0;
  } else if (sample == pendingValue) {
    if (++pendingCount >= 2) {      // same big jump twice in a row - trust it
      accepted = sample;
      pendingCount = 0;
    }
  } else {
    pendingValue = sample;
    pendingCount = 1;
  }

  return accepted;
}

bool readLeftBlocked() {
  return digitalRead(PIN_IR_LEFT) == LOW;    // active-low: LOW = obstacle seen
}

// ===================================================================
// Sonar calibration - measures how noisy the ultrasonic reading gets once
// the motors are actually drawing current (battery-powered EMI/voltage sag
// couples into the ECHO line and doesn't show up when powered from clean
// USB). Sets sonarNoiseMarginCm from what's actually measured instead of a
// guessed constant.
// ===================================================================
struct SonarCalResult {
  bool ok;
  long quietAvg, quietMin, quietMax;
  long loadAvg, loadMin, loadMax;
  long marginCm;
};

SonarCalResult calibrateSonar() {
  SonarCalResult r = {false, -1, -1, -1, -1, -1, -1, sonarNoiseMarginCm};

  AppMode prevMode = mode;
  mode = MODE_MENU;
  stopMotors();
  delay(150);

  // Phase 1: quiet baseline, motors off.
  long qSum = 0, qMin = 99999, qMax = -1; int qN = 0;
  unsigned long start = millis();
  while (millis() - start < CALIBRATION_SAMPLE_MS) {
    long d = pingSonarOnce();
    if (d > 0) {
      qSum += d; qN++;
      qMin = min(qMin, d);
      qMax = max(qMax, d);
    }
    delay(10);
  }

  // Phase 2: motors running (wheels should be off the ground / car held
  // still) so the ping sees whatever electrical noise real driving causes.
  Serial.println(F("Calibration: motors running for 1s - wheels should be OFF THE GROUND."));
  driveForward(MOTOR_SPEED_AUTO);
  long lSum = 0, lMin = 99999, lMax = -1; int lN = 0;
  start = millis();
  while (millis() - start < CALIBRATION_SAMPLE_MS) {
    long d = pingSonarOnce();
    if (d > 0) {
      lSum += d; lN++;
      lMin = min(lMin, d);
      lMax = max(lMax, d);
    }
    delay(10);
  }
  stopMotors();

  r.ok = qN > 0 && lN > 0;
  if (r.ok) {
    r.quietAvg = qSum / qN; r.quietMin = qMin; r.quietMax = qMax;
    r.loadAvg  = lSum / lN; r.loadMin  = lMin; r.loadMax  = lMax;

    long worstJump = max(labs(r.loadMax - r.quietAvg), labs(r.loadMin - r.quietAvg));
    r.marginCm = worstJump + 5;   // small pad above the worst observed jump
    sonarNoiseMarginCm = r.marginCm;
    sonarCalibrated = true;
  }

  Serial.println(F("=== Sonar calibration ==="));
  Serial.printf("Quiet (motors off):     avg=%ld min=%ld max=%ld cm\n", r.quietAvg, r.quietMin, r.quietMax);
  Serial.printf("Under load (motors on): avg=%ld min=%ld max=%ld cm\n", r.loadAvg, r.loadMin, r.loadMax);
  Serial.printf("-> noise margin set to %ld cm\n", r.marginCm);
  if (r.marginCm > 50) {
    Serial.println(F("!! That's a large margin - software filtering can only do so much here."));
    Serial.println(F("!! Add decoupling capacitors at the sensor/driver power input before trusting this while driving."));
  }

  mode = prevMode;
  return r;
}

// ===================================================================
// OLED eyes
// ===================================================================
void drawEyes(MoveState state, bool blink = false) {
  display.clearDisplay();

  const int eyeR = 20;
  const int lY = 32, rY = 32;
  const int lX = 40, rX = 88;

  if (blink) {
    display.fillRoundRect(lX - eyeR, lY - 3, eyeR * 2, 6, 3, SSD1306_WHITE);
    display.fillRoundRect(rX - eyeR, rY - 3, eyeR * 2, 6, 3, SSD1306_WHITE);
    display.display();
    return;
  }

  int offX = 0, offY = 0;
  switch (state) {
    case MV_FORWARD:  offY = -6; break;
    case MV_BACKWARD: offY =  6; break;
    case MV_LEFT:      offX = -8; break;
    case MV_RIGHT:     offX =  8; break;
    default: break;                          // MV_STOP -> centered
  }

  display.fillCircle(lX, lY, eyeR, SSD1306_WHITE);
  display.fillCircle(rX, rY, eyeR, SSD1306_WHITE);
  display.fillCircle(lX + offX, lY + offY, eyeR / 2, SSD1306_BLACK);
  display.fillCircle(rX + offX, rY + offY, eyeR / 2, SSD1306_BLACK);

  display.display();
}

void showEyes(MoveState s) {                 // draw immediately and reset the blink timer
  drawEyes(s, false);
  lastDrawnMove = s;
  lastEyeUpdate = millis();
}

void updateEyes() {
  if (moveState != lastDrawnMove) {
    showEyes(moveState);                     // direction changed - follow it right away
    return;
  }

  unsigned long now = millis();
  if (now - lastEyeUpdate < 4000) return;
  lastEyeUpdate = now;
  drawEyes(moveState, true);                 // quick blink, only while holding still
  delay(120);
  drawEyes(moveState, false);
}

// ===================================================================
// Mode / manual command handling (shared by all web endpoints)
// ===================================================================
void enterMode(AppMode m) {
  mode = m;
  stopMotors();
  autoState = AUTO_FORWARD;
  if (m == MODE_MANUAL) beep(1200, 100);
  else if (m == MODE_AUTO) beep(2000, 100);
  Serial.printf("Mode -> %d\n", (int)m);
}

void handleManualCommand(char c) {
  switch (c) {
    case 'F': case 'f': driveForward(manualSpeed); break;
    case 'B': case 'b': driveBackward(manualSpeed); break;
    case 'L': case 'l': turnLeft(manualSpeed); break;
    case 'R': case 'r': turnRight(manualSpeed); break;
    case 'S': case 's': stopMotors(); break;
    default: break;
  }
}

// ===================================================================
// Self-driving loop - called every loop() iteration.
// Clear path -> drive forward. Either sensor trips -> back straight off
// (capped, since there's no rear sensor) then turn away from whichever
// side triggered it, then resume forward on the new heading so the car
// evades the obstacle instead of driving straight back into it.
// ===================================================================
void autoModeStep() {
  long rightD = readDistanceCm();
  if (rightD < 0) rightD = 999;            // no echo - treat as clear
  lastRightDistance = rightD;

  bool obstacleRight = rightD < SAFE_DISTANCE_CM;
  bool obstacleLeft  = readLeftBlocked();
  lastLeftBlocked = obstacleLeft;

  unsigned long now = millis();

  if (autoState == AUTO_REVERSE) {
    driveBackward(autoSpeed);
    bool clear = !obstacleRight && !obstacleLeft;
    bool reverseTimedOut = now - autoPhaseStart >= MAX_REVERSE_MS;
    if (clear || reverseTimedOut) {
      autoState = AUTO_TURN;
      autoPhaseStart = now;
    }
    return;
  }

  if (autoState == AUTO_TURN) {
    // Away from whichever side triggered the reverse, not a live re-check -
    // by now we may have backed clear of both sensors, so there's nothing
    // left to react to; the direction has to come from what got us here.
    if (obstacleWasOnLeft) turnRight(autoSpeed);
    else turnLeft(autoSpeed);
    if (now - autoPhaseStart >= EVADE_TURN_MS) {
      autoState = AUTO_FORWARD;
    }
    return;
  }

  // AUTO_FORWARD
  if (obstacleRight || obstacleLeft) {
    obstacleWasOnLeft = obstacleLeft;  // if both trip at once, treat as left -> turn right
    autoState = AUTO_REVERSE;
    autoPhaseStart = now;
    driveBackward(autoSpeed);
    return;
  }

  driveForward(autoSpeed);
}

// ===================================================================
// Web UI
// ===================================================================
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>ZuriMobile</title>
<style>
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  body {
    margin:0; padding:20px 16px 32px; background:#12141a; color:#e8e8e2;
    font-family:-apple-system,"Segoe UI",Roboto,sans-serif;
    display:flex; flex-direction:column; align-items:center; gap:18px;
    touch-action:manipulation; -webkit-user-select:none; user-select:none;
  }
  h1 { margin:0; font-size:1.3rem; letter-spacing:0.02em; }
  .status {
    font-size:0.82rem; color:#9a9a90; font-family:ui-monospace,Consolas,monospace;
    text-align:center; line-height:1.6;
  }
  .status b { color:#e0a34e; }
  .modes { display:flex; gap:8px; width:100%; max-width:340px; }
  .modes button { flex:1; padding:12px 6px; font-size:0.88rem; }
  button {
    background:#1c2028; color:#e8e8e2; border:1px solid #333a44;
    border-radius:8px; padding:14px; font-size:1rem; font-weight:600;
    touch-action:none;
  }
  button.active { background:#e0a34e; color:#12141a; border-color:#e0a34e; }
  .dpad {
    display:grid; grid-template-columns:repeat(3,74px); grid-template-rows:repeat(3,74px);
    gap:8px; margin-top:4px;
  }
  .dpad button { width:74px; height:74px; font-size:1.5rem; padding:0; }
  #stopBtn { background:#7a2222; border-color:#7a2222; }
  .speedRow { width:100%; max-width:340px; display:flex; align-items:center; gap:10px; font-size:0.85rem; color:#9a9a90; }
  input[type=range] { flex:1; }
</style>
</head>
<body>
  <h1>ZuriMobile</h1>
  <div class="status">mode <b id="stMode">-</b> &nbsp; move <b id="stMove">-</b> &nbsp; R <b id="stDist">-</b> cm &nbsp; L <b id="stIr">-</b></div>

  <div class="modes">
    <button id="btnManual" onclick="setMode('manual')">Manual</button>
    <button id="btnAuto" onclick="setMode('auto')">Self-Driving</button>
    <button onclick="setMode('menu')">Stop / Menu</button>
  </div>

  <div class="dpad">
    <div></div>
    <button data-c="F">&#8593;</button>
    <div></div>
    <button data-c="L">&#8592;</button>
    <button id="stopBtn" onclick="drive('S')">&#9632;</button>
    <button data-c="R">&#8594;</button>
    <div></div>
    <button data-c="B">&#8595;</button>
    <div></div>
  </div>

  <div class="speedRow">
    <span>speed</span>
    <input type="range" id="speed" min="90" max="255" value="255" oninput="setSpeed(this.value)">
  </div>

  <button id="calBtn" onclick="calibrate()">Calibrate Sonar</button>
  <div class="status" id="calResult"></div>

<script>
function setMode(m){ fetch('/mode?m=' + m); }
function drive(c){ fetch('/cmd?c=' + c); }
function setSpeed(v){ fetch('/speed?v=' + v); }

// Slider follows whichever mode is active - don't fight the user while they drag it.
var speedEl = document.getElementById('speed');
var draggingSpeed = false;
speedEl.addEventListener('pointerdown', function(){ draggingSpeed = true; });
speedEl.addEventListener('pointerup', function(){ draggingSpeed = false; });
speedEl.addEventListener('pointercancel', function(){ draggingSpeed = false; });

function calibrate(){
  if (!confirm('Prop the drive wheels off the ground (or hold the car firmly) first - the motors will run for about a second. Continue?')) return;
  var btn = document.getElementById('calBtn');
  var out = document.getElementById('calResult');
  btn.disabled = true;
  out.textContent = 'Calibrating... (motors will run briefly)';
  fetch('/calibrate').then(function(r){ return r.json(); }).then(function(d){
    btn.disabled = false;
    if (!d.ok) { out.textContent = 'Calibration failed - no valid sonar readings.'; return; }
    out.textContent = 'quiet ' + d.quietAvg + 'cm (' + d.quietMin + '-' + d.quietMax +
      ') | under load ' + d.loadMin + '-' + d.loadMax + 'cm | margin set to ' + d.marginCm + 'cm';
  }).catch(function(){
    btn.disabled = false;
    out.textContent = 'Calibration request failed.';
  });
}

var holdTimer = null;
document.querySelectorAll('.dpad button[data-c]').forEach(function(btn){
  var c = btn.getAttribute('data-c');
  function start(e){
    if (e) e.preventDefault();
    drive(c);
    clearInterval(holdTimer);
    holdTimer = setInterval(function(){ drive(c); }, 300);  // keep-alive while held
  }
  function stop(){
    clearInterval(holdTimer);
    drive('S');
  }
  btn.addEventListener('pointerdown', start);
  btn.addEventListener('pointerup', stop);
  btn.addEventListener('pointercancel', stop);
  btn.addEventListener('pointerleave', stop);
});

function poll(){
  fetch('/status').then(function(r){ return r.json(); }).then(function(d){
    document.getElementById('stMode').textContent = d.mode;
    document.getElementById('stMove').textContent = d.move;
    document.getElementById('stDist').textContent = d.dist;
    document.getElementById('stIr').textContent = d.irLeft ? 'blocked' : 'clear';
    if (!draggingSpeed) speedEl.value = d.speed;
    document.getElementById('btnManual').className = d.mode === 'manual' ? 'active' : '';
    document.getElementById('btnAuto').className = d.mode === 'auto' ? 'active' : '';
  }).catch(function(){});
}
setInterval(poll, 600);
poll();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", PAGE_HTML);
}

// Captive portal: phones probe URLs like /generate_204 (Android) or
// /hotspot-detect.html (iOS) to check for internet. Redirecting every unknown
// URL to the controller makes the OS show "Sign in to network", which opens the UI.
void handleNotFound() {
  server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void handleMode() {
  if (server.hasArg("m")) {
    String m = server.arg("m");
    if (m == "manual") enterMode(MODE_MANUAL);
    else if (m == "auto") enterMode(MODE_AUTO);
    else enterMode(MODE_MENU);
  }
  server.send(200, "text/plain", "OK");
}

void handleCmd() {
  if (server.hasArg("c")) {
    if (mode != MODE_MANUAL) enterMode(MODE_MANUAL);  // an arrow press always claims manual control
    handleManualCommand(server.arg("c")[0]);
    lastCmdTime = millis();
  }
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("v")) {
    int v = constrain(server.arg("v").toInt(), 60, 255);
    if (mode == MODE_AUTO) autoSpeed = v;       // slider adjusts whichever mode is active
    else manualSpeed = v;
  }
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  const char* modeStr = mode == MODE_MANUAL ? "manual" : mode == MODE_AUTO ? "auto" : "menu";
  const char* moveStr = moveState == MV_FORWARD ? "fwd" : moveState == MV_BACKWARD ? "back"
                        : moveState == MV_LEFT ? "left" : moveState == MV_RIGHT ? "right" : "stop";
  String json = "{\"mode\":\"" + String(modeStr) + "\",\"move\":\"" + String(moveStr) +
                "\",\"dist\":" + String(lastRightDistance) +
                ",\"irLeft\":" + String(lastLeftBlocked ? "true" : "false") +
                ",\"calibrated\":" + String(sonarCalibrated ? "true" : "false") +
                ",\"noiseMarginCm\":" + String(sonarNoiseMarginCm) +
                ",\"speed\":" + String(mode == MODE_AUTO ? autoSpeed : manualSpeed) + "}";
  server.send(200, "application/json", json);
}

void handleCalibrate() {
  SonarCalResult r = calibrateSonar();
  String json = "{\"ok\":" + String(r.ok ? "true" : "false") +
                ",\"quietAvg\":" + String(r.quietAvg) +
                ",\"quietMin\":" + String(r.quietMin) +
                ",\"quietMax\":" + String(r.quietMax) +
                ",\"loadAvg\":" + String(r.loadAvg) +
                ",\"loadMin\":" + String(r.loadMin) +
                ",\"loadMax\":" + String(r.loadMax) +
                ",\"marginCm\":" + String(r.marginCm) + "}";
  server.send(200, "application/json", json);
}

void onWiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    beep(2200, 120);    // a phone joined the AP
  }
}

// ===================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR_LEFT, INPUT_PULLUP);
  stopMotors();

  beep(1000, 100); delay(150);          // power-on chirp
  beep(1500, 150);

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 init failed"));
  }
  display.setRotation(0);
  drawEyes(MV_STOP);

  WiFi.onEvent(onWiFiEvent);
  WiFi.softAP(AP_SSID, AP_PASS);
  MDNS.begin("zurimobile");
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/cmd", handleCmd);
  server.on("/speed", handleSpeed);
  server.on("/status", handleStatus);
  server.on("/calibrate", handleCalibrate);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.print(F("ZuriMobile AP up. Connect to WiFi '"));
  Serial.print(AP_SSID);
  Serial.println(F("' then browse to http://192.168.4.1"));
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  if (mode == MODE_MANUAL && moveState != MV_STOP && millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    stopMotors();                              // phone went quiet mid-drive - fail safe
  }

  if (mode == MODE_AUTO) {
    autoModeStep();
  }

  updateEyes();
}
