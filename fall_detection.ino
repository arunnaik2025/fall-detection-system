#include <Arduino.h>
#include <SoftwareSerial.h>

// User tuning (more sensitive)
const float LOW_G_THRESHOLD = 0.80f;
const uint16_t LOW_G_MIN_MS = 80;
const float HIGH_G_THRESHOLD = 1.30f;
const uint16_t IMPACT_WINDOW_MS = 1200;
const uint32_t ALERT_COOLDOWN_MS = 5000;

// GSM config
SoftwareSerial sim900a(7, 8);
const char PHONE_NUMBER[] = "+918197212768";

// ADXL335 pins
const uint8_t PIN_X = A0;
const uint8_t PIN_Y = A1;
const uint8_t PIN_Z = A2;

// Electrical model
const float ADC_REF_VOLT = 5.00f;
const float SENSOR_VCC = 3.30f;
const float SENSITIVITY_V_PER_G = SENSOR_VCC / 10.0f;

// Calibration & read settings
const uint16_t CALIB_SAMPLES = 300;
const uint16_t READ_AVG_SAMPLES = 3;

// State variables
float biasVx = 0, biasVy = 0, biasVz = 0;
bool lowGArmed = false;
uint32_t lowGStartMs = 0;
uint32_t lastAlertMs = 0;

// Helpers
float adcToVolts(int adc) {
  return (adc * ADC_REF_VOLT) / 1023.0f;
}

void readAveragedVolts(float &vx, float &vy, float &vz) {
  long sx = 0, sy = 0, sz = 0;
  for (uint16_t i = 0; i < READ_AVG_SAMPLES; ++i) {
    sx += analogRead(PIN_X);
    sy += analogRead(PIN_Y);
    sz += analogRead(PIN_Z);
    delayMicroseconds(350);
  }
  vx = adcToVolts((int)(sx / (float)READ_AVG_SAMPLES));
  vy = adcToVolts((int)(sy / (float)READ_AVG_SAMPLES));
  vz = adcToVolts((int)(sz / (float)READ_AVG_SAMPLES));
}

void calibrateBiases() {
  Serial.println("Calibrating... keep the board still ~1.5 s");
  long sx = 0, sy = 0, sz = 0;
  for (uint16_t i = 0; i < CALIB_SAMPLES; ++i) {
    sx += analogRead(PIN_X);
    sy += analogRead(PIN_Y);
    sz += analogRead(PIN_Z);
    delay(5);
  }
  biasVx = adcToVolts((int)(sx / (float)CALIB_SAMPLES));
  biasVy = adcToVolts((int)(sy / (float)CALIB_SAMPLES));
  biasVz = adcToVolts((int)(sz / (float)CALIB_SAMPLES));

  Serial.print("Bias V (X,Y,Z) = ");
  Serial.print(biasVx, 3); Serial.print(", ");
  Serial.print(biasVy, 3); Serial.print(", ");
  Serial.println(biasVz, 3);
  Serial.println("Calibration done.");
}

// GSM helpers
void simReadDump(unsigned long ms = 500) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (sim900a.available()) Serial.write(sim900a.read());
  }
}

bool sendSMS(const char* number, const char* msg) {
  Serial.println("Sending SMS...");

  sim900a.println("AT");
  if (!sim900a.find("OK")) simReadDump(500);

  sim900a.println("ATE0");
  if (!sim900a.find("OK")) simReadDump(500);

  sim900a.println("AT+CMGF=1");
  if (!sim900a.find("OK")) simReadDump(500);

  sim900a.println("AT+CSCS=\"GSM\"");
  if (!sim900a.find("OK")) simReadDump(500);

  sim900a.print("AT+CMGS=\"");
  sim900a.print(number);
  sim900a.println("\"");

  if (!sim900a.find(">")) {
    Serial.println("CMGS failed");
    return false;
  }

  sim900a.print(msg);
  sim900a.write(26);

  if (!sim900a.find("+CMGS:")) simReadDump(2000);
  if (!sim900a.find("OK")) simReadDump(2000);

  Serial.println("SMS sent OK.");
  return true;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Fall Detection + GSM SMS");

  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_Z, INPUT);

  delay(150);
  calibrateBiases();

  sim900a.begin(9600);
  Serial.println("SIM900A started");
}

void loop() {
  float vx, vy, vz;
  readAveragedVolts(vx, vy, vz);

  float gx = (vx - biasVx) / SENSITIVITY_V_PER_G;
  float gy = (vy - biasVy) / SENSITIVITY_V_PER_G;
  float gz = (vz - biasVz) / SENSITIVITY_V_PER_G;
  float amag = sqrt(gx*gx + gy*gy + gz*gz);

  Serial.print("gX="); Serial.print(gx, 2);
  Serial.print(" gY="); Serial.print(gy, 2);
  Serial.print(" gZ="); Serial.println(gz, 2);

  const uint32_t now = millis();

  if (amag < LOW_G_THRESHOLD) {
    if (!lowGArmed) {
      lowGArmed = true;
      lowGStartMs = now;
    }
  } else {
    if (lowGArmed && (now - lowGStartMs) < LOW_G_MIN_MS) {
      lowGArmed = false;
    }
  }

  static bool impactWindowOpen = false;
  static uint32_t impactWindowStart = 0;

  if (lowGArmed && (now - lowGStartMs) >= LOW_G_MIN_MS && !impactWindowOpen) {
    impactWindowOpen = true;
    impactWindowStart = now;
    lowGArmed = false;
    Serial.println("Low-g event detected");
  }

  if (impactWindowOpen) {
    if (amag >= HIGH_G_THRESHOLD) {
      if (now - lastAlertMs >= ALERT_COOLDOWN_MS) {
        lastAlertMs = now;

        Serial.println("FALL DETECTED");

        char smsText[120];
        snprintf(smsText, sizeof(smsText),
                 "FALL DETECTED\nGX=%.2f GY=%.2f GZ=%.2f",
                 gx, gy, gz);

        sendSMS(PHONE_NUMBER, smsText);
      }
      impactWindowOpen = false;
    }
    if (now - impactWindowStart > IMPACT_WINDOW_MS) {
      impactWindowOpen = false;
    }
  }

  if (sim900a.available()) simReadDump(150);

  delay(40);
}