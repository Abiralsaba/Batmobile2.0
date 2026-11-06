#include <Arduino.h>
#include <ESP32Servo.h>

// ===================== UART (Teensy 4.0 Connection) =====================
static const int TEENSY_RX_PIN = 26; // Receives commands FROM Teensy TX1
static const int TEENSY_TX_PIN = 27; // Sends status BACK to Teensy RX1

// ===================== SERVO (Mud Cache Lid) =====================
static const int SERVO_PIN = 14;
static const int SERVO_CLOSED_DEG = 10; // 10 instead of 0 to prevent binding
static const int SERVO_OPEN_DEG = 140;  // 140 instead of 150 to prevent binding
static const int SERVO_SPEED_MS = 15;   // Speed: Milliseconds per degree

Servo cacheServo;
int currentServoPos = SERVO_CLOSED_DEG;
int targetServoPos = SERVO_CLOSED_DEG;
uint32_t lastServoUpdate = 0;

// ===================== ESC (BLDC Auger Motor) =====================
static const int ESC_PIN = 16;
Servo augerESC;

static const int ESC_NEUTRAL = 1500; // Stop
static const int ESC_MAX = 1000;     // Full Forward (100% Speed)

// ===================== SABERTOOTH (Linear Actuator) =====================
#define Sabertooth1 129
#define st1 Serial1
static const int SABER_TX_PIN = 25; // TX pin for Sabertooth

// ===================== SABERTOOTH FUNCTIONS =====================
void stcommand(byte address, int speed, byte motor) {
  byte checksum = (address + motor + speed) & 0x7F; // 7-bit checksum safety
  st1.write(address);
  st1.write(motor);
  st1.write(speed);
  st1.write(checksum);
}

void drive(int address, int motorNo, int speed) {
  if (motorNo == 1) {
    if (speed > 0) stcommand(address, speed, 0);       // Down
    else if (speed < 0) stcommand(address, -speed, 1); // Up
    else stcommand(address, 0, 0);                     // Stop
  } 
}

void killSabertooth() {
  drive(Sabertooth1, 1, 0);
}

// ===================== SETUP =====================
void setup() {
  // Main USB Serial for local debugging (optional)
  Serial.begin(115200);

  // 1. Init Serial2 for FULL two-way communication with the Teensy
  Serial2.begin(115200, SERIAL_8N1, TEENSY_RX_PIN, TEENSY_TX_PIN);

  // 2. Init Sabertooth on Serial1 (TX = 25, RX ignored = -1)
  st1.begin(9600, SERIAL_8N1, -1, SABER_TX_PIN);
  killSabertooth();

  // 3. Servo Init (Mud Cache)
  cacheServo.setPeriodHertz(50);
  cacheServo.attach(SERVO_PIN, 500, 2400); // Extended sweep range
  cacheServo.write(SERVO_CLOSED_DEG);

  // 4. ESC Init (Auger Drill)
  augerESC.setPeriodHertz(50);
  augerESC.attach(ESC_PIN, 1000, 2000);
  
  // Arming the ESC (Sending Neutral Signal)
  Serial.println("Arming ESC... Please wait 3 seconds.");
  augerESC.writeMicroseconds(ESC_NEUTRAL);
  delay(3000); 

  // Send ready message to the Teensy (which forwards it to the PC)
  Serial2.println("\n=== ESP32 RIG SYSTEM READY ===");
  Serial2.println("Commands:");
  Serial2.println("  'a' -> Actuator UP");
  Serial2.println("  'b' -> Actuator DOWN");
  Serial2.println("  'w' -> Actuator HOLD (Stop)");
  Serial2.println("  'd' -> Drill ON (Full Speed)");
  Serial2.println("  'p' -> Drill OFF");
  Serial2.println("  't' -> Servo FORWARD (Open)");
  Serial2.println("  'z' -> Servo BACKWARD (Close)\n");
}

// ===================== LOOP =====================
void loop() {

  // --- SERIAL COMMAND LISTENER (From Teensy) ---
  if (Serial2.available() > 0) {
    char cmd = Serial2.read();

    // ACTUATOR COMMANDS
    if (cmd == 'a') {
      Serial2.println("ESP32: Actuator MOVING UP");
      drive(Sabertooth1, 1, -127);
    } 
    else if (cmd == 'b') {
      Serial2.println("ESP32: Actuator MOVING DOWN");
      drive(Sabertooth1, 1, 127);
    } 
    else if (cmd == 'w') {
      Serial2.println("ESP32: Actuator HOLDING POSITION");
      drive(Sabertooth1, 1, 0);
    }

    // DRILL COMMANDS
    else if (cmd == 'd') {
      Serial2.println("ESP32: Drill ON (Full Speed)");
      augerESC.writeMicroseconds(ESC_MAX);
    } 
    else if (cmd == 'p') {
      Serial2.println("ESP32: Drill OFF");
      augerESC.writeMicroseconds(ESC_NEUTRAL);
    }

    // SERVO COMMANDS
    else if (cmd == 't') {
      Serial2.println("ESP32: Mud Cache OPENING");
      targetServoPos = SERVO_OPEN_DEG;
    } 
    else if (cmd == 'z') {
      Serial2.println("ESP32: Mud Cache CLOSING");
      targetServoPos = SERVO_CLOSED_DEG;
    }
  }

  // --- SMOOTH SERVO SWEEP ENGINE (Non-Blocking) ---
  if (currentServoPos != targetServoPos) {
    if (millis() - lastServoUpdate >= SERVO_SPEED_MS) {
      lastServoUpdate = millis();
      
      if (currentServoPos < targetServoPos) {
        currentServoPos++; // Move towards open
      } else {
        currentServoPos--; // Move towards closed
      }
      
      // Update the physical servo position
      cacheServo.write(currentServoPos);
    }
  }

  delay(15); // Loop stabilization delay
}