/* FOR ONLY BOX-D HARDWARE */

#include <Wire.h>
#include <INA226_WE.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>
#include "Adafruit_MAX31855.h"

/* -------------------------------- Ultrasonics (HC-SR04) Configuration -------------------------------- */
/*
  #include <NewPing.h>
  #define TRIGGER_PIN  5  // Arduino pin tied to trigger pin on the ultrasonic sensor.
  #define ECHO_PIN     4  // Arduino pin tied to echo pin on the ultrasonic sensor.
  #define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
  #define us_roundtrip_mm 5.7
  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
*/

/* -------------------------------- OneWire (DS18B20) Configuration -------------------------------- */
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// Update the addresses below according to your OneWire Addresses, check "DallasTemperature" library example (ex. oneWireSearch).
DeviceAddress D0_thermometer = { 0x28, 0x4E, 0x12, 0xFF, 0x0C, 0x00, 0x00, 0xD7 };
DeviceAddress D1_thermometer = { 0x28, 0xBC, 0x54, 0xFE, 0x0C, 0x00, 0x00, 0x31 };
DeviceAddress D2_thermometer = { 0x28, 0xA6, 0xF9, 0x71, 0x15, 0x21, 0x01, 0xA6 };
DeviceAddress D3_thermometer = { 0x28, 0x88, 0x78, 0x38, 0x11, 0x20, 0x06, 0xAF };
float D0_temp, D1_temp, D2_temp, D3_temp;


/* -------------------------------- DHT22 Configuration -------------------------------- */
#define DHTPIN 3
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float h, t;
//unsigned long lastDHTreading = 0;
//int DHT_delay_ms = 300;
// ^ not yet uploaded ^

/* -------------------------------- 2x INA226 (Voltmeters) / INA219 (low-Amp) Configuration -------------------------------- */
#define I2C_ADDRESS 0x40
#define I2C_ADDRESS_TRIG 0x44
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
INA226_WE ina226_trig = INA226_WE(I2C_ADDRESS_TRIG);
Adafruit_INA219 ina219(0x41);
float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0;
float trig_shuntVoltage_mV = 0.0;
float trig_loadVoltage_V = 0.0;
float trig_busVoltage_V = 0.0;

float trig_offset_V = 0;
float load_offset_V = -1.1 / 2;

/* -------------------------------- RTC & SD Card Configuration -------------------------------- */
RTC_DS3231 rtc;
DateTime now;
File myFile, myFile2, myFile3, myFile4;
String filename_general = "genD.txt";
String filename_max = "maxD.txt";
String filename_triggerVolt = "trig_Df.txt";
String filename_log = "log_D.txt";
const byte chipSelect = 53; // same as SS

/* -------------------------------- MPPT Parameters -------------------------------- */
const byte switch_pin = 8;
const byte relay_pin = 9;
float power_mW = 0;
float P_max = 0;
byte k = 0;
unsigned int j = 0;
String buff = "";
unsigned int overload = 0;
byte found_peak_cycles = 0;

/* DO NOT CHANGE THESE VALUES */
bool high = false;
bool discharging = false;
bool firsttime = true;
bool rested = true;
/* DO NOT CHANGE THESE VALUES */

const bool use_hall_effect = true;
const byte trigger_threshold = 0;
const byte limit_volt = 7;
const byte limit_k = 50;
const byte peaks_before_reboot = 10;
//const byte dischargeTime_before_reboot = 2;  // 2 mins

/* -------------------------------- MPPT - Hall effect -------------------------------- */
const int ammeter_pin = A15;
byte sensitive = 100;   // for 20A Range
float offset = 2486.5;

/* -------------------------------- MPPT - MovAvg for Hall effect (no longer used) --------------------------------
  const byte numReadings = 1;
  float readings[numReadings];      // the readings from the analog input
  byte readIndex = 0;              // the index of the current reading
  float total = 0;                  // the running total
  float average = 0;                // the average
  //float current = 0;
*/

/* -------------------------------- Thermocouple Amplifiers (MAX31855) -------------------------------- */
#define MAXCS1   3
#define MAXCS2   4
#define MAXCS3   5
#define MAXCS4   6
#define MAXCS5   7
#define MAXCS6   8

Adafruit_MAX31855 thermocouple1(MAXCS1);
Adafruit_MAX31855 thermocouple2(MAXCS2);
Adafruit_MAX31855 thermocouple3(MAXCS3);
Adafruit_MAX31855 thermocouple4(MAXCS4);
Adafruit_MAX31855 thermocouple5(MAXCS5);
Adafruit_MAX31855 thermocouple6(MAXCS6);
double Temp1, Temp2, Temp3, Temp4, Temp5, Temp6;
bool use_thermocouple = false;

void setup() {
  /* ========================== Initialization ========================== */
  pinMode(switch_pin, INPUT_PULLUP);
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW);

  Serial.begin(9600);
  Wire.begin();

  /* ========================== OneWire (DS18B20) ========================== */
  sensors.begin();
  sensors.setResolution(D0_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(D1_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(D2_thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(D3_thermometer, TEMPERATURE_PRECISION);
  /* ========================== DHT22 ========================== */
  dht.begin();
  /* ========================== INA226 ========================== */
  ina226.init();
  Serial.println(F("INA226 Voltage Sensor - Initialized"));
  ina226.waitUntilConversionCompleted();

  ina226_trig.init();
  Serial.println(F("INA226_trig Voltage Sensor - Initialized"));
  ina226_trig.waitUntilConversionCompleted();
  /* ========================== INA219 ========================== */
  if (!use_hall_effect) {
    if (!ina219.begin()) {
      Serial.println(F("Failed to find INA219 chip"));
      while (1) {
        delay(10);
      }
    }
  }
  /* ========================== RTC ========================== */
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    while (1) delay(10);
  }
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, let's set the time!"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  /* ========================== SD Card ========================== */
  Serial.print(F("Initializing SD card..."));
  pinMode(SS, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println(F("initialization done."));
  /* ========================== MovAvg (no longer used) ========================== */
  /*
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
    }*/

  /* ========================== Thermocouples ========================== */
  if (use_thermocouple) {
    if (!thermocouple1.begin()) {
      Serial.println(F("ERROR Thermocouple1"));
      while (1) delay(10);
    }
    if (!thermocouple2.begin()) {
      Serial.println(F("ERROR Thermocouple2"));
      while (1) delay(10);
    }
    if (!thermocouple3.begin()) {
      Serial.println(F("ERROR Thermocouple3"));
      while (1) delay(10);
    }
    if (!thermocouple4.begin()) {
      Serial.println(F("ERROR Thermocouple4"));
      while (1) delay(10);
    }
    if (!thermocouple5.begin()) {
      Serial.println(F("ERROR Thermocouple5"));
      while (1) delay(10);
    }
    if (!thermocouple6.begin()) {
      Serial.println(F("ERROR Thermocouple6"));
      while (1) delay(10);
    }
  }
}

/* =================================== MPPT OPERATION =================================== */
void loop() {
  read_trigger_voltage();

  while (trig_loadVoltage_V * 2 < trigger_threshold) {
    if (!rested) digitalWrite(relay_pin, LOW); // open circuit
    rested = true;
    delay(5000); // idle for 5 seconds
    read_trigger_voltage();
    SD_card_triggerVolt();
    Serial.print(F("Waiting for open-circuit voltage to reach threshold, current volt: "));
    Serial.print(trig_loadVoltage_V * 2);
    Serial.println(F(" V, current distance: "));

    if (trig_loadVoltage_V * 2 >= trigger_threshold) break;
  }


  while (true) {
    //Serial.println(String(millis() - checkpoint));
    //checkpoint = millis();
    if (digitalRead(switch_pin) == HIGH && (!high || rested)) {
      Serial.println(F("START"));
      discharging = false;
      buff = "";
      high = true;
      firsttime = true;
      rested = false;
      //start_of_cycle = millis();
      digitalWrite(relay_pin, HIGH);
    }
    else if (digitalRead(switch_pin) == LOW && high) {
      digitalWrite(relay_pin, LOW);
      buff = "";
      high = false;
      j = 0;
    }

    ina226.readAndClearFlags();
    shuntVoltage_mV = ina226.getShuntVoltage_mV();
    busVoltage_V = ina226.getBusVoltage_V();
    loadVoltage_V  = busVoltage_V + (shuntVoltage_mV / 1000) + load_offset_V;

    if (!use_hall_effect) current_mA = ina219.getCurrent_mA();
    else if (use_hall_effect) current_mA = getCurrentAvg() * 1000;

    power_mW = (loadVoltage_V * 2) * current_mA;

    if (high && !discharging) {
      if (firsttime) {
        buff = "-9;0\n";
        buff += String(loadVoltage_V * 2) + ";" + String(current_mA, 1) + "\n";
        firsttime = false;
        j++;
        P_max = 0;
        k = 0;
        overload = 0;
      }
      else if (j < 250) {
        buff += String(loadVoltage_V * 2) + ";" + String(current_mA, 1) + "\n";
        j++;
      }
      else if (j == 250) {
        byte newline_index = buff.indexOf("\n", 5);
        buff.remove(5, newline_index + 1 - 5);
        buff.concat(String(loadVoltage_V * 2) + ";" + String(current_mA, 1) + "\n");
        overload++;
        if (overload == 300 * 10) {
          digitalWrite(relay_pin, LOW);
          Serial.println(F("Cannot find MPPT peaks within 3000 rows ... Discharge & Reboot ..."));
          SD_card_log();
          delay(120000);
          Serial.println(F("REBOOT\n"));
          software_Reboot();
        }
      }
      //Serial.println(loadVoltage_V * 2);
      if (loadVoltage_V * 2 >= limit_volt) {
        if (power_mW > P_max) {
          P_max = power_mW;
          k = 0;
          continue;
        }
        else {
          if (power_mW < 0.8 * P_max) {
            k++;
            if (k > limit_k) {
              digitalWrite(relay_pin, LOW);
              read_thermocouple(use_thermocouple);
              sensors.requestTemperatures();
              D0_temp = read_surface_temperature(D0_thermometer);
              D1_temp = read_surface_temperature(D1_thermometer);
              D2_temp = read_surface_temperature(D2_thermometer);
              D3_temp = read_surface_temperature(D3_thermometer);
              read_DHT();
              now = rtc.now();
              SD_card_timePowerAmbTempAtMAX();
              buff.concat(String(now.year()) + ";"
                          + String(now.month()) + ";"
                          + String(now.day()) + ";"
                          + String(now.hour()) + ";"
                          + String(now.minute()) + ";"
                          + String(now.second()));
              SD_card_allAtOnce();
              P_max = 0;
              k = 0;
              j = 0;
              discharging = true;
              firsttime = true;
              found_peak_cycles++;
              Serial.println(overload);
              Serial.println(found_peak_cycles);
              continue;
            }
            else continue;
          }
          else continue;
        }
      }
      else continue;
    }
    else if (high && discharging) {
      delay(200);
      if (found_peak_cycles == peaks_before_reboot) {
        Serial.println(F("Numbers of peaks reach threshold ... Discharge & Reboot ..."));
        delay(120000);
        Serial.println(F("REBOOT\n"));
        software_Reboot();
      }
      Serial.println(String(loadVoltage_V * 2) + ";" + String(current_mA, 1));
      if (loadVoltage_V * 2 < limit_volt) {
        read_trigger_voltage();
        SD_card_triggerVolt();
        if (trig_loadVoltage_V * 2 >= trigger_threshold) {
          Serial.println(F("START"));
          discharging = false;
          digitalWrite(relay_pin, HIGH);
          continue;
        }
        else {
          rested = false;
          break;
        }
      }
    }
    else if (!high) {
      read_trigger_voltage();
      SD_card_triggerVolt();  // <-- already includes DHT22 and Time readings!
      bool printOut = false;
      if (printOut) {
        Serial.println(String(now.year()) + ";"
                       + String(now.month()) + ";"
                       + String(now.day()) + ";"
                       + String(now.hour()) + ";"
                       + String(now.minute()) + ";"
                       + String(now.second()) + ";" + " "
                       + String(trig_loadVoltage_V * 2) + ";"
                       + String(loadVoltage_V * 2) + ";" + " "
                       + String(current_mA) + ";"
                       + String(t) + ";"
                       + String(h) + ";"
                       + String(D0_temp) + ";"
                       + String(D1_temp) + ";"
                       + String(D2_temp) + ";"
                       + String(D3_temp) + ";"
                       + String(Temp1) + ";"
                       + String(Temp2) + ";"
                       + String(Temp3) + ";"
                       + String(Temp4) + ";"
                       + String(Temp5) + ";"
                       + String(Temp6));
      }
      delay(100);
      if (trig_loadVoltage_V * 2 < trigger_threshold) {
        rested = false;
        break;
      }
    }
  }
}

/* =================================== FUNCTION ZONE =================================== */

float getCurrentAvg() {
  int count = 4;
  float sum = 0;
  for (int i = 0; i < count; i++) {
    sum += getCurrent();
    //delay(1);
  }
  float val = sum / count;
  return val;
}

float getCurrent() {
  int a = analogRead(ammeter_pin);
  float v = (a / 1024.0) * 5000;
  float c = (v - offset) / sensitive;
  return c;
}

void SD_card_timePowerAmbTempAtMAX() {
  myFile2 = SD.open(filename_max, FILE_WRITE);
  if (myFile2) {
    String entire_line = String(now.year()) + ";"
                         + String(now.month()) + ";"
                         + String(now.day()) + ";"
                         + String(now.hour()) + ";"
                         + String(now.minute()) + ";"
                         + String(now.second()) + ";"
                         + String(P_max) + ";"
                         + String(t) + ";"
                         + String(h) + ";"
                         + String(D0_temp) + ";"
                         + String(D1_temp) + ";"
                         + String(D2_temp) + ";"
                         + String(D3_temp) + ";"
                         + String(Temp1) + ";"
                         + String(Temp2) + ";"
                         + String(Temp3) + ";"
                         + String(Temp4) + ";"
                         + String(Temp5) + ";"
                         + String(Temp6);
    myFile2.println(entire_line);
    myFile2.close();
    Serial.println(entire_line);
  } else Serial.println("error opening" + filename_max);
}

void read_DHT() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
}

float read_surface_temperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    //Serial.println("Error: Could not read temperature data");
    return -1.0;
  }
  return tempC;
}

void SD_card_allAtOnce() {
  myFile = SD.open(filename_general, FILE_WRITE);
  if (myFile) {
    myFile.println(buff);
    myFile.close();
    //Serial.println(buff);
  } else Serial.println("error opening" + filename_general);
}

void software_Reboot()
{
  wdt_enable(WDTO_15MS);
  delay(100);
}

void read_trigger_voltage() {
  ina226_trig.readAndClearFlags();
  trig_shuntVoltage_mV = ina226_trig.getShuntVoltage_mV();
  trig_busVoltage_V = ina226_trig.getBusVoltage_V();
  trig_loadVoltage_V  = trig_busVoltage_V + (trig_shuntVoltage_mV / 1000) + trig_offset_V;
}

void SD_card_triggerVolt() {
  read_thermocouple(use_thermocouple);
  sensors.requestTemperatures();
  D0_temp = read_surface_temperature(D0_thermometer);
  D1_temp = read_surface_temperature(D1_thermometer);
  D2_temp = read_surface_temperature(D2_thermometer);
  D3_temp = read_surface_temperature(D3_thermometer);
  read_DHT();
  now = rtc.now();
  myFile3 = SD.open(filename_triggerVolt, FILE_WRITE);
  if (myFile3) {
    myFile3.println(String(now.year()) + ";"
                    + String(now.month()) + ";"
                    + String(now.day()) + ";"
                    + String(now.hour()) + ";"
                    + String(now.minute()) + ";"
                    + String(now.second()) + ";"
                    + String(trig_loadVoltage_V * 2) + ";"
                    + String(t) + ";"
                    + String(h) + ";"
                    + String(D0_temp) + ";"
                    + String(D1_temp) + ";"
                    + String(D2_temp) + ";"
                    + String(D3_temp) + ";"
                    + String(Temp1) + ";"
                    + String(Temp2) + ";"
                    + String(Temp3) + ";"
                    + String(Temp4) + ";"
                    + String(Temp5) + ";"
                    + String(Temp6));
    myFile3.close();
  } else Serial.println("error opening" + filename_triggerVolt);
}

void SD_card_log() {
  now = rtc.now();
  buff.concat(String(now.year()) + ";"
              + String(now.month()) + ";"
              + String(now.day()) + ";"
              + String(now.hour()) + ";"
              + String(now.minute()) + ";"
              + String(now.second()));
  myFile4 = SD.open(filename_log, FILE_WRITE);
  if (myFile4) {
    myFile4.println(buff);
    myFile4.close();
  } else Serial.println("error opening" + filename_log);
}

void read_thermocouple(bool use) {
  if (use) {
    Temp1 = thermocouple1.readCelsius();
    Temp2 = thermocouple2.readCelsius();
    Temp3 = thermocouple3.readCelsius();
    Temp4 = thermocouple4.readCelsius();
    Temp5 = thermocouple5.readCelsius();
    Temp6 = thermocouple6.readCelsius();
    //double Intemp1 = thermocouple1.readInternal();
    //double Intemp2 = thermocouple2.readInternal();
    //double Intemp3 = thermocouple3.readInternal();
    //double Intemp4 = thermocouple4.readInternal();
    //double Intemp5 = thermocouple5.readInternal();
    //double Intemp6 = thermocouple6.readInternal();
  }
  else {
    Temp1 = -1.0;
    Temp2 = -1.0;
    Temp3 = -1.0;
    Temp4 = -1.0;
    Temp5 = -1.0;
    Temp6 = -1.0;
  }

}
/* =================================== End of this file =================================== */
