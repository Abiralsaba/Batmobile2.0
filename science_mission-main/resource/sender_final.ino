#include <Wire.h>
#include <DFRobot_SCD4X.h>
#include "DFRobot_MultiGasSensor.h"
#include <AltSoftSerial.h>

// I2C Addresses
#define GAS_SENSOR_1_ADDR 0x74
#define GAS_SENSOR_2_ADDR 0x75
#define GAS_SENSOR_3_ADDR 0x77
#define SCD4X_ADDR        0x62

// Analog MQ-4 Configuration
const int mq4Pin = A0;
const float Vcc = 5.0;
const float RL = 10.0;
const float Ro = 4.4;

// RS-485 NPK Sensor Configuration
#define RE 6
#define DE 7

// Modbus commands for NPK sensor
const byte temp[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA}; // Temperature
const byte mois[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}; // Moisture
const byte econ[] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xCA}; // Conductivity
const byte ph[] = {0x01, 0x03, 0x00, 0x03, 0x00, 0x01, 0x74, 0x0A};   // pH
const byte nitro[] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xC5, 0xCB}; // Nitrogen
const byte phos[] = {0x01, 0x03, 0x00, 0x05, 0x00, 0x01, 0x94, 0x0B};  // Phosphorus
const byte pota[] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0B};  // Potassium

byte values[7]; // Buffer for NPK sensor response
AltSoftSerial mod;

// Sensor Objects
DFRobot_SCD4X scd4x(&Wire, SCD4X_ADDR);
DFRobot_GAS_I2C gasSensor1(&Wire, GAS_SENSOR_1_ADDR);
DFRobot_GAS_I2C gasSensor2(&Wire, GAS_SENSOR_2_ADDR);
DFRobot_GAS_I2C gasSensor3(&Wire, GAS_SENSOR_3_ADDR);

// Sensor variables
float soilMoisture = 0.0, soilTemp = 0.0, conductivity = 0.0, soilPH = 0.0;
int nitrogenVal = 0, phosphorusVal = 0, potassiumVal = 0;
float co2_ppm = 0.0, temperature = 0.0, humidity = 0.0;
float gas1_ppm = 0.0, gas2_ppm = 0.0, gas3_ppm = 0.0, ch4_ppm = 0.0;
String gas1_type = "", gas2_type = "", gas3_type = "";

void setup() {
  Serial.begin(9600);
  delay(100);
  Wire.begin();

  // Initialize NPK sensor serial
  mod.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  // Initialize SCD4X
  unsigned long start = millis();
  while (!scd4x.begin()) {
    delay(1000);
    if (millis() - start > 5000) break;
  }

  scd4x.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
  delay(500);
  scd4x.setTempComp(4.0);
  scd4x.setSensorAltitude(540);
  scd4x.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);

  // Initialize Multi-Gas Sensors
  DFRobot_GAS_I2C* gasSensors[] = { &gasSensor1, &gasSensor2, &gasSensor3 };
  for (int i = 0; i < 3; i++) {
    start = millis();
    while (!gasSensors[i]->begin()) {
      delay(1000);
      if (millis() - start > 5000) break;
    }
    gasSensors[i]->changeAcquireMode(gasSensors[i]->PASSIVITY);
    gasSensors[i]->setTempCompensation(gasSensors[i]->ON);
    
    // Get gas types
    if (i == 0) gas1_type = gasSensors[i]->queryGasType();
    else if (i == 1) gas2_type = gasSensors[i]->queryGasType();
    else if (i == 2) gas3_type = gasSensors[i]->queryGasType();
  }
}

float readSensor(const byte *command, byte responseSize, int scaleFactor = 1) {
  mod.flushInput();
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);
  
  for (uint8_t i = 0; i < 8; i++) {
    mod.write(command[i]);
  }
  mod.flush();
  
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  
  unsigned long start = millis();
  while (mod.available() < responseSize && millis() - start < 200);
  
  for (byte i = 0; i < responseSize; i++) {
    values[i] = mod.read();
  }

  return (values[3] << 8 | values[4]) / (float)scaleFactor;
}

void loop() {
  delay(2000); // Main loop delay

  // --- Read SCD4X (CO2, Temp, Humidity) ---
  if (scd4x.getDataReadyStatus()) {
    DFRobot_SCD4X::sSensorMeasurement_t data;
    scd4x.readMeasurement(&data);
    co2_ppm = data.CO2ppm;
    temperature = data.temp;
    humidity = data.humidity;
  }

  // --- Read Multi-Gas Sensors ---
  gas1_ppm = gasSensor1.readGasConcentrationPPM();
  gas2_ppm = gasSensor2.readGasConcentrationPPM();
  gas3_ppm = gasSensor3.readGasConcentrationPPM();

  // --- Read MQ-4 Methane Sensor ---
  int adcValue = analogRead(mq4Pin);
  float voltage = adcValue * (Vcc / 1023.0);
  float Rs = ((Vcc - voltage) / voltage) * RL;
  float ratio = Rs / Ro;
  float ppm_log = (log10(ratio) - 0.518) / -0.318;
  ch4_ppm = pow(10, ppm_log);

  // --- Read NPK Soil Sensor ---
  soilMoisture = readSensor(mois, 7, 10);    // Scaling: 0.1%
  soilTemp = readSensor(temp, 7, 10);        // Scaling: 0.1°C
  conductivity = readSensor(econ, 7, 1);     // Scaling: None
  soilPH = readSensor(ph, 7, 10);           // Scaling: 0.1
  nitrogenVal = readSensor(nitro, 7, 1);     // Scaling: None
  phosphorusVal = readSensor(phos, 7, 1);    // Scaling: None
  potassiumVal = readSensor(pota, 7, 1);     // Scaling: None

  // // Environmental data
  // Serial.print(temperature);   Serial.print(",");
  // Serial.print(humidity);      Serial.print(",");
  // Serial.print(co2_ppm);       Serial.print(",");
  
  // // Gas sensor data
  // Serial.print(gas1_ppm);      Serial.print(",");
  // Serial.print(gas2_ppm);      Serial.print(",");
  // Serial.print(gas3_ppm);      Serial.print(",");
  // Serial.print(ch4_ppm);       Serial.print(",");
  
  // // Soil data
  // Serial.print(soilTemp);      Serial.print(",");
  // Serial.print(soilMoisture);  Serial.print(",");
  // Serial.print(conductivity);  Serial.print(",");
  // Serial.print(soilPH);        Serial.print(",");
  // Serial.print(nitrogenVal);   Serial.print(",");
  // Serial.print(phosphorusVal); Serial.print(",");
  // Serial.print(potassiumVal);  Serial.println();

  Serial.print("{");
    Serial.print("\"airTemp\":"); Serial.print(temperature); Serial.print(",");
    Serial.print("\"airHumidity\":"); Serial.print(humidity); Serial.print(",");
    Serial.print("\"co2\":"); Serial.print(co2_ppm); Serial.print(",");
    Serial.print("\"gas1\":"); Serial.print(gas1_ppm); Serial.print(",");
    Serial.print("\"gas2\":"); Serial.print(gas2_ppm); Serial.print(",");
    Serial.print("\"gas3\":"); Serial.print(gas3_ppm); Serial.print(",");
    Serial.print("\"ch4\":"); Serial.print(ch4_ppm); Serial.print(",");

    Serial.print("\"soilTemp\":"); Serial.print(soilTemp); Serial.print(",");
    Serial.print("\"soilMoisture\":"); Serial.print(soilMoisture); Serial.print(",");
    Serial.print("\"soilEC\":"); Serial.print(conductivity); Serial.print(",");
    Serial.print("\"soilPH\":"); Serial.print(soilPH); Serial.print(",");
    Serial.print("\"soilN\":"); Serial.print(nitrogenVal); Serial.print(",");
    Serial.print("\"soilP\":"); Serial.print(phosphorusVal); Serial.print(",");
    Serial.print("\"soilK\":"); Serial.print(potassiumVal);
  Serial.println("}");


}
