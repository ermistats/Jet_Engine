#include <Wire.h>
#include <DFRobot_MAX31855.h>
#include "HX711.h"
#include <Servo.h>


DFRobot_MAX31855 max31855;


const int   gasPin = A0;
const float Vref   = 5.0;          // Use 3.3 if ADC reference is 3.3V
const float RL     = 10.0;         // kΩ
const float R0     = 2.0;          // kΩ (your calibrated clean-air)
const float a      = 116.6020682;  // replace per your MQ model
const float b      = -2.769034857;


#define RELAY0_PIN 7
#define RELAY1_PIN 9
#define ACTIVE_LOW0 1            // 1: LOW = ON, 0: HIGH = ON
#define ACTIVE_LOW1 1
bool isOn0 = false;
bool isOn1 = false;

inline void setRelay0(bool on){
  digitalWrite(RELAY0_PIN, (ACTIVE_LOW0 ? (on?LOW:HIGH) : (on?HIGH:LOW)));
}
inline void setRelay1(bool on){
  digitalWrite(RELAY1_PIN, (ACTIVE_LOW1 ? (on?LOW:HIGH) : (on?HIGH:LOW)));
}


#define HX_DT_PIN  2
#define HX_SCK_PIN 3
HX711 scale;

enum LCState { LC_STREAM, LC_CAL_WAIT_WEIGHT };
LCState lcState = LC_STREAM;

bool waitReady(uint32_t timeout_ms){
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms){
    if (scale.is_ready()) return true;
    delay(1);
  }
  return false;
}


Servo esc1, esc2;

const int ESC1_PIN = 10;  // EDF1
const int ESC2_PIN = 11;  // EDF2

const int MIN_US = 1000;     
const int MAX_US = 2000;    

const int   RAMP_STEP_US = 6;     
const unsigned long STEP_MS = 20;  

int  esc1_target_us = MIN_US, esc2_target_us = MIN_US;
int  esc1_current_us = MIN_US, esc2_current_us = MIN_US;
unsigned long lastEscStep = 0;

int pctToUs(float pct) {
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return (int)(MIN_US + (pct / 100.0f) * (MAX_US - MIN_US));
}
float usToPct(int us) {
  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;
  return 100.0f * (us - MIN_US) / (MAX_US - MIN_US);
}
inline void writeESCs(int us1, int us2){
  esc1.writeMicroseconds(us1);
  esc2.writeMicroseconds(us2);
}
void escRampUpdate(){
  unsigned long now = millis();
  if (now - lastEscStep < STEP_MS) return;
  lastEscStep = now;

  // EDF1
  if (esc1_current_us < esc1_target_us) {
    esc1_current_us += RAMP_STEP_US;
    if (esc1_current_us > esc1_target_us) esc1_current_us = esc1_target_us;
  } else if (esc1_current_us > esc1_target_us) {
    esc1_current_us -= RAMP_STEP_US;
    if (esc1_current_us < esc1_target_us) esc1_current_us = esc1_target_us;
  }

  // EDF2
  if (esc2_current_us < esc2_target_us) {
    esc2_current_us += RAMP_STEP_US;
    if (esc2_current_us > esc2_target_us) esc2_current_us = esc2_target_us;
  } else if (esc2_current_us > esc2_target_us) {
    esc2_current_us -= RAMP_STEP_US;
    if (esc2_current_us < esc2_target_us) esc2_current_us = esc2_target_us;
  }

  writeESCs(esc1_current_us, esc2_current_us);
}


const int sensorPin = 4;   
const int buzzerPin = 6;   

#define LIQUID_ACTIVE_HIGH 0  
#define BUZZER_ACTIVE_HIGH 1  
#define USE_INPUT_PULLUP 1    

unsigned long AUTO_TIMEOUT_SEC = 30; 

enum BzMode { AUTO_MODE, MANUAL_ON, MANUAL_OFF };
void applyBuzzer(bool on);
void enterManual(BzMode m);
// ----------------------------------------------------------------

BzMode bzMode = AUTO_MODE;
bool buzzerOn = false;
unsigned long manualSince = 0;

void applyBuzzer(bool on) {
  buzzerOn = on;
  if (BUZZER_ACTIVE_HIGH) digitalWrite(buzzerPin, on ? HIGH : LOW);
  else                    digitalWrite(buzzerPin, on ? LOW  : HIGH);
}
void enterManual(BzMode m) {
  bzMode = m;
  applyBuzzer(m == MANUAL_ON);
  manualSince = millis();
}


void handleCommand(const String& raw){
  String cmd = raw; cmd.trim();
  if (!cmd.length()) return;
  String up = cmd; up.toUpperCase();


  if (up.startsWith("UALL")) {
    long us = up.substring(4).toInt();
    us = constrain(us, MIN_US, MAX_US);
    esc1_target_us = esc2_target_us = (int)us;
    Serial.print("ACK:UALL="); Serial.println(us);
    return;
  }
  if (up.startsWith("U1")) {
    long us = up.substring(2).toInt();
    us = constrain(us, MIN_US, MAX_US);
    esc1_target_us = (int)us;
    Serial.print("ACK:U1="); Serial.println(us);
    return;
  }
  if (up.startsWith("U2")) {
    long us = up.substring(2).toInt();
    us = constrain(us, MIN_US, MAX_US);
    esc2_target_us = (int)us;
    Serial.print("ACK:U2="); Serial.println(us);
    return;
  }


  if (up.startsWith("ALL")) {
    float pct = up.substring(3).toFloat();
    pct = constrain(pct, 0, 100);
    int us = pctToUs(pct);
    esc1_target_us = esc2_target_us = us;
    Serial.print("ACK:ALL="); Serial.print(pct,1); Serial.print("% ("); Serial.print(us); Serial.println("us)");
    return;
  }
  if (up.startsWith("EDF1")) {
    float pct = up.substring(4).toFloat();
    pct = constrain(pct, 0, 100);
    esc1_target_us = pctToUs(pct);
    Serial.print("ACK:EDF1="); Serial.print(pct,1); Serial.print("% ("); Serial.print(esc1_target_us); Serial.println("us)");
    return;
  }
  if (up.startsWith("EDF2")) {
    float pct = up.substring(4).toFloat();
    pct = constrain(pct, 0, 100);
    esc2_target_us = pctToUs(pct);
    Serial.print("ACK:EDF2="); Serial.print(pct,1); Serial.print("% ("); Serial.print(esc2_target_us); Serial.println("us)");
    return;
  }


  if (up == "ON")  { isOn0 = true;  setRelay0(true);  Serial.println("ACK:ON");  return; }
  if (up == "OFF") { isOn0 = false; setRelay0(false); Serial.println("ACK:OFF"); return; }
  if (up == "ON1") { isOn1 = true;  setRelay1(true);  Serial.println("ACK:ON1"); return; }
  if (up == "OFF1"){ isOn1 = false; setRelay1(false); Serial.println("ACK:OFF1");return; }


  if (up == "T"){
    if (waitReady(2000)) scale.tare();
    Serial.println("TARED:ENTER_KNOWN_GRAMS");
    lcState = LC_CAL_WAIT_WEIGHT;
    return;
  }
  if (lcState == LC_CAL_WAIT_WEIGHT){
    float known_g = cmd.toFloat();
    if (known_g > 0.0f){
      if (!waitReady(2000)){
        Serial.println("ERR:HX711_NOT_READY");
      } else {
        long net = scale.get_value(20);   
        if (net == 0){
          Serial.println("ERR:CAL_ZERO_NET");
        } else {
          float cal = (float)net / known_g;  
          scale.set_scale(cal);
          float test = scale.get_units(10);
          if (test < 0){ cal = -cal; scale.set_scale(cal); test = scale.get_units(10); }
          Serial.print("ACK:CAL="); Serial.println(cal, 6);
          Serial.print("CHECK_G="); Serial.println(test, 2);
        }
      }
      lcState = LC_STREAM; 
    } else {
      Serial.println("ERR:ENTER_POSITIVE_GRAMS");
    }
    return;
  }


  if (up == "SOUNDON")  { enterManual(MANUAL_ON);  Serial.println("ACK:SOUNDON");  return; }
  if (up == "SOUNDOFF") { enterManual(MANUAL_OFF); Serial.println("ACK:SOUNDOFF"); return; }
  if (up == "AUTO")     { bzMode = AUTO_MODE;      Serial.println("ACK:AUTO");     return; }

  if (up.startsWith("AUTOAFTER")) {
    int sp = up.indexOf(' ');
    if (sp > 0) {
      long secs = up.substring(sp + 1).toInt();
      if (secs >= 0) {
        AUTO_TIMEOUT_SEC = (unsigned long)secs;
        if (bzMode != AUTO_MODE) manualSince = millis();
        Serial.print("ACK:AUTOAFTER="); Serial.print(secs); Serial.println("s");
        return;
      }
    }
    Serial.println("ERR:USE AUTOAFTER <seconds>");
    return;
  }
  if (up == "NOTIMEOUT") {
    AUTO_TIMEOUT_SEC = 0;
    Serial.println("ACK:NOTIMEOUT");
    return;
  }


  if (lcState == LC_STREAM) {
    float pct = cmd.toFloat();
    if (pct >= 0 || cmd == "0") {
      pct = constrain(pct, 0, 100);
      int us = pctToUs(pct);
      esc1_target_us = esc2_target_us = us;
      Serial.print("ACK:ALL="); Serial.print(pct,1); Serial.print("% ("); Serial.print(us); Serial.println("us)");
    }
  }
}


void setup(){
  Serial.begin(9600);


  Wire.begin();
  max31855.begin();


  pinMode(RELAY0_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  setRelay0(false);
  setRelay1(false);


  scale.begin(HX_DT_PIN, HX_SCK_PIN);
  scale.set_scale(1.0);
  scale.tare();


  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);
  writeESCs(MIN_US, MIN_US);  
  delay(6000);                
  esc1_current_us = esc1_target_us = MIN_US;
  esc2_current_us = esc2_target_us = MIN_US;
  writeESCs(esc1_current_us, esc2_current_us);


#if USE_INPUT_PULLUP
  pinMode(sensorPin, INPUT_PULLUP);
#else
  pinMode(sensorPin, INPUT);
#endif
  pinMode(buzzerPin, OUTPUT);
  applyBuzzer(false);

  Serial.println("READY");
  Serial.println("Commands: ON/OFF, ON1/OFF1, T(+grams), EDF1 <pct>, EDF2 <pct>, ALL <pct>, U1 <us>, U2 <us>, UALL <us>, SOUNDON, SOUNDOFF, AUTO, AUTOAFTER <sec>, NOTIMEOUT");
}


void loop(){

  if (Serial.available() > 0){
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }


  float tempC = max31855.readCelsius();

  int adcValue = analogRead(gasPin);
  float Vout = (adcValue / 1023.0) * Vref;
  float RS = (Vout > 0.001f) ? ((Vref - Vout) * RL / Vout) : 1e9;
  float ratio = RS / R0;
  float ppm = a * pow(ratio, b);

  float grams = 0.0f;
  if (lcState == LC_STREAM && scale.is_ready()){
    grams = scale.get_units(5);
  }


  escRampUpdate();


  int raw = digitalRead(sensorPin);
  bool liquidDetected = (raw == (LIQUID_ACTIVE_HIGH ? HIGH : LOW));
  if (bzMode == AUTO_MODE) {
    applyBuzzer(liquidDetected);
  } else if (AUTO_TIMEOUT_SEC > 0) {
    if (millis() - manualSince >= AUTO_TIMEOUT_SEC * 1000UL) {
      bzMode = AUTO_MODE;
      Serial.println("ACK:AUTO_TIMEOUT");
    }
  }


  if (lcState == LC_STREAM){

    Serial.print(tempC, 2);  Serial.print(", ");
    Serial.print(ppm, 3);    Serial.print(", ");
    Serial.print(isOn0 ? "ON" : "OFF"); Serial.print(", ");
    Serial.print(isOn1 ? "ON" : "OFF"); Serial.print(", ");
    Serial.print(grams, 2);  Serial.print(", ");

    Serial.print(usToPct(esc1_current_us), 1); Serial.print(", ");
    Serial.print(esc1_current_us);             Serial.print(", ");
    Serial.print(usToPct(esc2_current_us), 1); Serial.print(", ");
    Serial.print(esc2_current_us);             Serial.print(", ");

    Serial.print(liquidDetected ? "YES" : "NO"); Serial.print(", ");
    Serial.println(buzzerOn ? "ON" : "OFF");
  }

  delay(200); 
}
