#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Servo.h>

// Pins
const int pinMainPower = 2;
const int pinCoilSplit = 3;
const int pinUV        = 4;
const int servoPin     = 5;

// Servo
Servo lidServo;

// States
bool mainPowerState = LOW;
bool coilSplitState = LOW;
bool uvState        = LOW;
bool servoOpen      = false;

// =====================================================

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

// ===================== SETTINGS =====================
#define CAN_BITRATE 1000000
const uint32_t SERIAL_BAUD = 115200;

// ===================== CamMotor =====================
const uint8_t CamMotor_DEV_ID = 0x01;
const uint8_t GDZ_CMD_SET_POS = 0xC2;

const float CamMotor_OPEN_RAD  = -4.0f;
const float CamMotor_CLOSE_RAD =  4.0f;

const float CamMotor_SPEED = 0.35f;
const float SEND_INTERVAL = 0.04f;

// Command: -1 / 0 / 1
int CamMotor_cmd = 0;

float CamMotor_pos = 0.0f;

float last_send = 0.0f;
float last_update = 0.0f;

String serialLine;

// ===================== HELPERS =====================
float nowSeconds() {
  return millis() * 0.001f;
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int32_t CamMotor_pos_to_raw(float rad) {
  float deg = rad * 57.2957795f;
  return (int32_t)roundf(deg * 100.0f);
}

// ===================== CAN SEND =====================
void send_can(uint16_t id, const uint8_t *data, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;
  msg.flags.extended = 0;

  for (int i = 0; i < len; i++) msg.buf[i] = data[i];

  Can0.write(msg);
}

void send_pos(int32_t val) {
  uint32_t v = (uint32_t)val;

  uint8_t data[5];
  data[0] = GDZ_CMD_SET_POS;
  data[1] = v & 0xFF;
  data[2] = (v >> 8) & 0xFF;
  data[3] = (v >> 16) & 0xFF;
  data[4] = (v >> 24) & 0xFF;

  send_can(CamMotor_DEV_ID, data, 5);
  send_can(0x100 | CamMotor_DEV_ID, data, 5);
}

// ===================== CamMotor LOGIC =====================
void CamMotor_update(float now_s) {
  if (last_update == 0.0f) last_update = now_s;

  float dt = now_s - last_update;
  last_update = now_s;

  if (dt > 0.05f) dt = 0.05f;

  if (CamMotor_cmd < 0) {
    CamMotor_pos += CamMotor_SPEED * dt;
  }
  else if (CamMotor_cmd > 0) {
    CamMotor_pos -= CamMotor_SPEED * dt;
  }

  CamMotor_pos = clampf(CamMotor_pos, CamMotor_OPEN_RAD, CamMotor_CLOSE_RAD);

  if ((now_s - last_send) >= SEND_INTERVAL) {
    last_send = now_s;
    send_pos(CamMotor_pos_to_raw(CamMotor_pos));
  }
}

// ===================== SERIAL =====================
// Handles BOTH menu + numeric input
void handle_serial() {
  while (Serial.available()) {
    char c = Serial.read();
/*a--> Actuator Up __________ b--> Actuator Down ______ 'w'--> Actuator Stop
 * d --> Drill On ___________p--> Drill Off
 * t--> Servo Up_____________z--> Servo Down
 */
    if (c == 'a' || c == 'b' || c == 'w' || c == 'd' || c == 'p' || c == 't' || c == 'z') { 
      Serial2.println(c);
    }

    // --- MENU COMMANDS (single char) ---
    if (c == 'm' || c == 'c' || c == 'u' || c == 's' || c == 'h') {
      processCommand(c);
      continue;
    }

    // --- LINE INPUT FOR MOTOR ---
    if (c == '\n' || c == '\r') {
      serialLine.trim();

      if (serialLine.length() > 0) {
        int v = serialLine.toInt();

        if (v > 0) CamMotor_cmd = 1;
        else if (v < 0) CamMotor_cmd = -1;
        else CamMotor_cmd = 0;

        Serial.print("CMD = ");
        Serial.println(CamMotor_cmd);
      }

      serialLine = "";
    }
    else {
      serialLine += c;

      if (serialLine.length() > 20) {
        serialLine = "";
      }
    }
  }
}

// =====================================================

void printMenu() {
  Serial.println("\n========== CONTROL PANEL ==========");
  Serial.println("m : Toggle Main Power");
  Serial.println("c : Toggle Coil Split");
  Serial.println("u : Toggle UV Light");
  Serial.println("s : Toggle Servo (Lid)");
  Serial.println("h : Show Menu Again");
  Serial.println("===================================");
  printStatus();
}

void printStatus() {
  Serial.println("\n------ CURRENT STATUS ------");

  Serial.print("Main Power : ");
  Serial.println(mainPowerState ? "ON" : "OFF");

  Serial.print("Coil Split : ");
  Serial.println(coilSplitState ? "ON" : "OFF");

  Serial.print("UV Light   : ");
  Serial.println(uvState ? "ON" : "OFF");

  Serial.print("Servo Lid  : ");
  Serial.println(servoOpen ? "OPEN" : "CLOSED");

  Serial.println("----------------------------\n");
}

// =====================================================

void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);
  delay(1000);

  Can0.begin();
  Can0.setBaudRate(CAN_BITRATE);
  Can0.enableFIFO();

  Serial.println("Single CamMotor Ready");
  Serial.println("Send: -1 / 0 / 1");

  pinMode(pinMainPower, OUTPUT);
  pinMode(pinCoilSplit, OUTPUT);
  pinMode(pinUV, OUTPUT);

  digitalWrite(pinMainPower, LOW);
  digitalWrite(pinCoilSplit, LOW);
  digitalWrite(pinUV, LOW);

  lidServo.attach(servoPin);
  lidServo.write(20);

  Serial.println("Teensy B READY");
  printMenu();
}

// =====================================================

void loop() {

  float now_s = nowSeconds();

  handle_serial();
  CamMotor_update(now_s);
}

// =====================================================

void processCommand(char cmd) {

  switch (cmd) {

    case 'm':
      handleMainPower();
      break;

    case 'c':
      handleCoilSplit();
      break;

    case 'u':
      handleUV();
      break;

    case 's':
      handleServoControl();
      break;

    case 'h':
      printMenu();
      return;

    default:
      return;
  }

  printMenu();
}

// =====================================================

void handleMainPower() {
  mainPowerState = !mainPowerState;
  digitalWrite(pinMainPower, mainPowerState);
}

void handleCoilSplit() {

  if (mainPowerState == HIGH) {
    Serial.println("Blocked: Coil Split disabled while Main Power is ON");
    return;
  }

  coilSplitState = !coilSplitState;
  digitalWrite(pinCoilSplit, coilSplitState);
}

void handleUV() {
  uvState = !uvState;
  digitalWrite(pinUV, uvState);
}

void handleServoControl() {
  servoOpen = !servoOpen;
  lidServo.write(servoOpen ? 160 : 20);
}
