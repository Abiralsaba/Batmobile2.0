/*
 * Batmobile EVA glove — three-finger FOLLOW latch
 * Board: ESP32-S3 Super Mini
 * Sensors: thumb GPIO 1, index GPIO 2, middle GPIO 3
 * Serial: 115200 baud, newline-delimited FLEX_JSON_V2
 *
 * Safety behavior:
 *   - The glove must first be seen unbent after boot (arming).
 *   - A stable three-finger bend emits FOLLOW_START once.
 *   - Holding the bend emits state samples but no repeated command event.
 *   - Stably unbending any of the three fingers emits FOLLOW_STOP once.
 */

#include <Arduino.h>

constexpr uint8_t SENSOR_COUNT = 3;
constexpr uint8_t FLEX_PINS[SENSOR_COUNT] = {1, 2, 3};

// Calibration supplied from the tested glove, in millivolts.
constexpr int FLAT_MV[SENSOR_COUNT] = {785, 780, 959};
constexpr int BENT_MV[SENSOR_COUNT] = {670, 611, 750};

constexpr int MIN_USEFUL_RANGE_MV = 50;
constexpr int ENGAGE_PERCENT = 60;
constexpr int RELEASE_PERCENT = 35;
constexpr uint8_t READ_SAMPLES = 4;
constexpr uint8_t STABLE_SAMPLES = 3;
constexpr uint32_t REPORT_INTERVAL_MS = 100;

uint32_t sequenceNumber = 0;
bool armed = false;
bool followActive = false;
uint8_t engageCount = 0;
uint8_t releaseCount = 0;

int readSensorMV(uint8_t pin) {
  uint32_t total = 0;
  for (uint8_t sample = 0; sample < READ_SAMPLES; ++sample) {
    total += analogReadMilliVolts(pin);
    delay(2);
  }
  return static_cast<int>(total / READ_SAMPLES);
}

int bendPercent(int rawMV, int flatMV, int bentMV) {
  const int range = bentMV - flatMV;
  if (abs(range) < MIN_USEFUL_RANGE_MV) {
    return 0;
  }
  const float percentage = 100.0f * (rawMV - flatMV) / range;
  return constrain(static_cast<int>(roundf(percentage)), 0, 100);
}

void printPacket(const int raw[SENSOR_COUNT], const int bend[SENSOR_COUNT],
                 const char *event) {
  Serial.print("{\"seq\":");
  Serial.print(sequenceNumber++);
  Serial.print(",\"timestamp_ms\":");
  Serial.print(millis());
  Serial.print(",\"flex_raw\":[");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i) Serial.print(',');
    Serial.print(raw[i]);
  }
  Serial.print("],\"bend_pct\":[");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i) Serial.print(',');
    Serial.print(bend[i]);
  }
  Serial.print("],\"gesture\":\"FOLLOW\",\"active\":");
  Serial.print(followActive ? "true" : "false");
  Serial.print(",\"event\":");
  if (event == nullptr) {
    Serial.print("null");
  } else {
    Serial.print('"');
    Serial.print(event);
    Serial.print('"');
  }
  Serial.println('}');
}

void setup() {
  Serial.begin(115200);
  const uint32_t waitStarted = millis();
  while (!Serial && millis() - waitStarted < 2000) {
    delay(10);
  }
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

void loop() {
  int raw[SENSOR_COUNT];
  int bend[SENSOR_COUNT];
  bool allBent = true;
  bool anyReleased = false;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    raw[i] = readSensorMV(FLEX_PINS[i]);
    bend[i] = bendPercent(raw[i], FLAT_MV[i], BENT_MV[i]);
    allBent = allBent && bend[i] >= ENGAGE_PERCENT;
    anyReleased = anyReleased || bend[i] <= RELEASE_PERCENT;
  }

  const char *event = nullptr;
  if (!armed) {
    if (anyReleased) {
      releaseCount++;
      if (releaseCount >= STABLE_SAMPLES) {
        armed = true;
        releaseCount = 0;
      }
    } else {
      releaseCount = 0;
    }
  } else if (!followActive) {
    engageCount = allBent ? engageCount + 1 : 0;
    if (engageCount >= STABLE_SAMPLES) {
      followActive = true;
      engageCount = 0;
      event = "FOLLOW_START";
    }
  } else {
    releaseCount = anyReleased ? releaseCount + 1 : 0;
    if (releaseCount >= STABLE_SAMPLES) {
      followActive = false;
      releaseCount = 0;
      event = "FOLLOW_STOP";
    }
  }

  printPacket(raw, bend, event);
  delay(REPORT_INTERVAL_MS);
}
