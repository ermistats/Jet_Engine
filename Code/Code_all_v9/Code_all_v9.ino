#include <HX711_ADC.h>
#include <DFRobot_MAX31855.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <math.h>


const int HX711_dout = 4;
const int HX711_sck = 5;
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
float loadCellValue = 0;
bool newDataReady = false;


DFRobot_MAX31855 max31855(&Wire, 0x10);
float temperatureC = 0;
float lastTemperatureC = 0;


const int hydrogenPin = A0;
int hydrogenAnalog = 0;
float hydrogenPPM = 0;


#define RL 10000.0
#define VCC 3.3
#define ADC_RESOLUTION 4095.0
float R0 = 10000.0;


Servo esc1, esc2;
const int esc1Pin = 18; 
const int esc2Pin = 19; 
int throttlePercent1 = 0;
int throttlePercent2 = 0;


const int ESC_MIN_PULSE = 1000;
const int ESC_MAX_PULSE = 2000;


unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500;  // ms

void setup() {
  Serial.begin(9600);

  LoadCell.begin();
  bool _tare = true;
  LoadCell.start(2000, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("HX711 timeout. Check wiring.");
    while (1);
  } else {
    float storedCal = 1.0;
    EEPROM.get(calVal_eepromAdress, storedCal);
    LoadCell.setCalFactor(storedCal);
    Serial.print("Load Cell ready. Calibration factor: ");
    Serial.println(storedCal);
  }
  while (!LoadCell.update());


  max31855.begin();


  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc1.attach(esc1Pin, ESC_MIN_PULSE, ESC_MAX_PULSE);
  esc2.attach(esc2Pin, ESC_MIN_PULSE, ESC_MAX_PULSE);
  esc1.writeMicroseconds(ESC_MIN_PULSE);
  esc2.writeMicroseconds(ESC_MIN_PULSE);
  Serial.println("ESCs armed.");
  setThrottlePercent(1, 0);
  setThrottlePercent(2, 0);
}

void loop() {
  // Load Cell
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    loadCellValue = LoadCell.getData();
    newDataReady = false;
  }

  // Temperature
  float temp = max31855.readCelsius();
  if (temp != 0) {
    temperatureC = temp;
    lastTemperatureC = temp;
  } else {
    temperatureC = lastTemperatureC;
  }

  // Hydrogen
  hydrogenAnalog = analogRead(hydrogenPin);
  hydrogenPPM = getHydrogenPPM(hydrogenAnalog);

  // Serial Plotter Output
  if (millis() - lastPrintTime > printInterval) {
    Serial.print("Hydrogen:");
    Serial.print(hydrogenPPM);
    Serial.print("\tKg:");
    Serial.print(loadCellValue);
    Serial.print("\t");
    Serial.print(temperatureC);
    Serial.print("°C");
    Serial.print("\tThrottle1:");
    Serial.print(throttlePercent1);
    Serial.print("\tThrottle2:");
    Serial.println(throttlePercent2);
    lastPrintTime = millis();
  }

  if (Serial.available() > 0) {
    char inByte = Serial.read();

    if (inByte == 't') {
      LoadCell.tareNoDelay();
      Serial.println("Taring...");
    } else if (inByte == 'c') {
      Serial.println("Send new calibration factor:");
      while (Serial.available() == 0);
      float newCal = Serial.parseFloat();
      if (newCal != 0) {
        LoadCell.setCalFactor(newCal);
        EEPROM.put(calVal_eepromAdress, newCal);
        Serial.print("New calibration factor saved: ");
        Serial.println(newCal);
      }
    } else if (inByte == 'r') {
      calibrate();
    } else if (inByte == 'm') {
      while (Serial.available() == 0);
      int motorID = Serial.parseInt();
      while (Serial.available() == 0);
      int percent = Serial.parseInt();
      setThrottlePercent(motorID, percent);
    }
  }

  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete.");
  }
}

float getHydrogenPPM(int analogValue) {
  float Vout = (analogValue / ADC_RESOLUTION) * VCC;
  float Rs = (VCC - Vout) * RL / Vout;
  float ratio = Rs / R0;
  float ppm = pow(10, (-0.45 * log10(ratio) + 0.72));
  return ppm;
}

void calibrate() {
  Serial.println("*** Calibration Mode ***");
  Serial.println("Step 1: Remove any weight from the load cell.");
  Serial.println("Send 't' to tare (set zero baseline).\n");

  bool tared = false;
  while (!tared) {
    LoadCell.update();
    if (Serial.available()) {
      if (Serial.read() == 't') {
        LoadCell.tareNoDelay();
        while (!LoadCell.getTareStatus()) LoadCell.update();
        Serial.println("Tare complete.\n");
        tared = true;
      }
    }
  }

  Serial.println("Step 2: Place a known weight on the scale.");
  Serial.println("Send the exact weight in grams (e.g., 100.0):\n");

  float known_mass = 0;
  bool gotWeight = false;
  while (!gotWeight) {
    LoadCell.update();
    if (Serial.available()) {
      known_mass = Serial.parseFloat();
      if (known_mass > 0) {
        gotWeight = true;
        Serial.print("Known weight received: ");
        Serial.println(known_mass);
      }
    }
  }

  LoadCell.refreshDataSet();
  float newCal = LoadCell.getNewCalibration(known_mass);
  LoadCell.setCalFactor(newCal);

  Serial.print("New calibration factor: ");
  Serial.println(newCal);
  Serial.println("Save this to EEPROM? Send 'y' to save, 'n' to skip.\n");

  bool saveDone = false;
  while (!saveDone) {
    if (Serial.available()) {
      char inByte = Serial.read();
      if (inByte == 'y') {
        EEPROM.put(calVal_eepromAdress, newCal);
        Serial.println("Calibration factor saved to EEPROM.\n");
        saveDone = true;
      } else if (inByte == 'n') {
        Serial.println("Calibration not saved.\n");
        saveDone = true;
      }
    }
  }

  Serial.println("*** Calibration Complete ***\n");
}


void setThrottlePercent(int motorID, int percent) {
  if (percent < 0) return;
  if (percent > 100) percent = 100;

  int pulse = map(percent, 0, 100, ESC_MIN_PULSE, ESC_MAX_PULSE);

  if (motorID == 1) {
    esc1.writeMicroseconds(pulse);
    throttlePercent1 = percent;
  } else if (motorID == 2) {
    esc2.writeMicroseconds(pulse);
    throttlePercent2 = percent;
  }

  Serial.print("Motor ");
  Serial.print(motorID);
  Serial.print(" throttle set to ");
  Serial.print(percent);
  Serial.print("% (");
  Serial.print(pulse);
  Serial.println(" µs)");
}
