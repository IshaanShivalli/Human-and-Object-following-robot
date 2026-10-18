#include <AFMotor.h>
#include <SoftwareSerial.h>

const int PIN_ARD_RX = 10;
const int PIN_ARD_TX = 9;

SoftwareSerial espSerial(PIN_ARD_RX, PIN_ARD_TX);

AF_DCMotor motorFL(1);
AF_DCMotor motorRL(2);
AF_DCMotor motorFR(3);
AF_DCMotor motorRR(4);

int g_speed     = 190;
int g_turnSpeed = 175;

const unsigned long CMD_TIMEOUT_MS = 500;

unsigned long lastCmdMs  = 0;
bool          motorsLive = false;

const bool USE_SONAR = true;
const int  TRIG_PIN = A0, ECHO_PIN = A1;
const int  STOP_CM  = 25;
int g_dist = 0;
unsigned long lastPingMs = 0;
bool movingForward = false;

int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long us = pulseIn(ECHO_PIN, HIGH, 12000);
  if (us == 0) return 0;
  int cm = (int)(us / 58);
  return (cm >= 2 && cm <= 400) ? cm : 0;
}

bool blockedAhead() { return USE_SONAR && g_dist > 0 && g_dist < STOP_CM; }

void driveLeft(int speed) {
  uint8_t spd = constrain(abs(speed), 0, 255);
  uint8_t dir = (speed > 0) ? FORWARD : (speed < 0) ? BACKWARD : 0;
  motorFL.setSpeed(spd); motorRL.setSpeed(spd);
  if (dir) { motorFL.run(dir); motorRL.run(dir); }
  else     { motorFL.run(RELEASE); motorRL.run(RELEASE); }
}

void driveRight(int speed) {
  uint8_t spd = constrain(abs(speed), 0, 255);
  uint8_t dir = (speed > 0) ? FORWARD : (speed < 0) ? BACKWARD : 0;
  motorFR.setSpeed(spd); motorRR.setSpeed(spd);
  if (dir) { motorFR.run(dir); motorRR.run(dir); }
  else     { motorFR.run(RELEASE); motorRR.run(RELEASE); }
}

void stopMotors() {
  movingForward = false;
  motorFL.run(RELEASE); motorRL.run(RELEASE);
  motorFR.run(RELEASE); motorRR.run(RELEASE);
  motorsLive = false;
}

void doForward()   { driveLeft( g_speed);             driveRight( g_speed);             motorsLive = true; movingForward = true; }
void doBackward()  { driveLeft(-g_speed);             driveRight(-g_speed);             motorsLive = true; }
void doPivotL()    { driveLeft(-g_turnSpeed);         driveRight( g_turnSpeed);         motorsLive = true; }
void doPivotR()    { driveLeft( g_turnSpeed);         driveRight(-g_turnSpeed);         motorsLive = true; }
void doArcFL()     { driveLeft( g_speed * 55 / 100); driveRight( g_speed);             motorsLive = true; }
void doArcFR()     { driveLeft( g_speed);             driveRight( g_speed * 55 / 100); motorsLive = true; }
void doArcBL()     { driveLeft(-g_speed * 55 / 100); driveRight(-g_speed);             motorsLive = true; }
void doArcBR()     { driveLeft(-g_speed);             driveRight(-g_speed * 55 / 100); motorsLive = true; }

void handleCommand(const String& raw) {
  String cmd = raw;
  cmd.trim();
  if (cmd.length() == 0) return;

  int numStart = -1;
  for (int i = 0; i < (int)cmd.length(); i++) {
    if (isDigit(cmd[i])) { numStart = i; break; }
  }
  if (numStart > 0) {
    int spd = cmd.substring(numStart).toInt();
    if (spd > 0 && spd <= 255) g_speed = g_turnSpeed = spd;
    cmd = cmd.substring(0, numStart);
  }

  lastCmdMs = millis();

  Serial.print(F("CMD: "));
  Serial.print(cmd);
  Serial.print(F(" SPD: "));
  Serial.println(g_speed);

  bool wantsForward = (cmd == "F" || cmd == "FL" || cmd == "FR");
  if (wantsForward && blockedAhead()) {
    stopMotors();
    Serial.print(F("BLOCKED at ")); Serial.print(g_dist); Serial.println(F(" cm"));
    return;
  }

  if      (cmd == "F")  doForward();
  else if (cmd == "B")  doBackward();
  else if (cmd == "L")  doPivotL();
  else if (cmd == "R")  doPivotR();
  else if (cmd == "FL") doArcFL();
  else if (cmd == "FR") doArcFR();
  else if (cmd == "BL") doArcBL();
  else if (cmd == "BR") doArcBR();
  else if (cmd == "S")  stopMotors();
}

void setup() {
  motorFL.setSpeed(g_speed); motorRL.setSpeed(g_speed);
  motorFR.setSpeed(g_speed); motorRR.setSpeed(g_speed);
  stopMotors();

  Serial.begin(115200);
  Serial.println(F("motor_controller ready"));

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  espSerial.begin(57600);
}

void loop() {
  if (espSerial.available()) {
    String line = espSerial.readStringUntil('\n');
    handleCommand(line);
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  if (USE_SONAR && millis() - lastPingMs >= 60) {
    lastPingMs = millis();
    g_dist = readDistanceCm();
    if (movingForward && blockedAhead()) stopMotors();
  }

  if (motorsLive && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    stopMotors();
  }
}