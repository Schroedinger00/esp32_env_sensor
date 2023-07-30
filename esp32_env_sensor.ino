// Bibliotheken
//#include <Arduino.h>
#include <ThingSpeak.h>
#include <WiFi.h>
#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "SparkFunHTU21D.h"
#include "Adafruit_SGP30.h"
#include "TaskScheduler.h"

// Defines für den BMP280
//#define BMP_SCK 13
//#define BMP_MISO 12  // an 3.3v
//#define BMP_MOSI 11
//#define BMP_CS 10  // an 3.3v

//eigene mapping funktion für Voltage
float map2(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

// WiFi Einstellungen
const char* ssid = "YOUR SSID";                      //  your network SSID (name)
const char* pass = "YOUR WIFI PASSWORD";  // your network password
//int status = WL_IDLE_STATUS;
WiFiClient client;

// Mini Timer Einstellungen
const unsigned long period = 30000;  // the value is a number of milliseconds, ie 1 second
unsigned long currentMillis;
unsigned long previousMillis = 0;

// Mini Timer2 Einstellungen
const unsigned long period2 = 1000;  // the value is a number of milliseconds, ie 1 second
unsigned long previousMillis2 = 0;

// Mini Timer3 Einstellungen
unsigned long currentMillis3;
const unsigned long period3 = 60000;  // the value is a number of milliseconds, ie 1 second
unsigned long previousMillis3 = 0;

// Mini Timer4 Einstellungen
const unsigned long period4 = 1000;  // the value is a number of milliseconds, ie 1 second
unsigned long previousMillis4 = 0;

// Time to sleep (Deepsleep) (in seconds):
const long long sleepTimeS = 3600; // 60 min Deepsleep

// ThinsSpeak Einstellungen
unsigned long myChannelNumber = THINGSPEAK CHANNEL NUMBER;
const char *myWriteAPIKey = "THINGSPEAK API KEY";

// Arrays und Globale Variablen für die Durchschnitsberechnung
float tempReading[10];
float humidReading[10];
float batReading[10];
float pReading[10];

float tvocReading[10];
float eco2Reading[10];
float rawH2Reading[10];
float rawETHReading[10];

float averageHumid = -50;
float averagebat = -50;
float averageTemp = -50;
float averagep = -50;

float averagetvoc = -1;
float averageeco2 = -1;
float averagerawH2 = -1;
float averagerawETH = -1;

float htu_t_check = -50;
float htu_rh_check = -50;

float sgp_temp = 0;
float sgp_humid = 0;

int counter = 0;

// Initialisierung der Sensoren Library
Adafruit_BMP280 bmp;
Adafruit_SGP30 sgp;
HTU21D Humidity;

// TaskScheduler für Cooperatives Multitasking
Scheduler ts;

void temp_avg_cb();
void humid_avg_cb();
void p_avg_cb();
void co_avg_cb();
void bat_avg_cb();
void ausgabe_cb();

Task temp_avg(100 * TASK_MILLISECOND, TASK_FOREVER, &temp_avg_cb, &ts, true);
Task humid_avg(100 * TASK_MILLISECOND, TASK_FOREVER, &humid_avg_cb, &ts, true);
Task p_avg(100 * TASK_MILLISECOND, TASK_FOREVER, &p_avg_cb, &ts, true);
Task co_avg(500 * TASK_MILLISECOND, TASK_FOREVER, &co_avg_cb, &ts, true);
Task bat_avg(100 * TASK_MILLISECOND, TASK_FOREVER, &bat_avg_cb, &ts, true);
Task ausgabe(50 * TASK_MILLISECOND, TASK_FOREVER, &ausgabe_cb, &ts, true);

// Main Programm Code
void setup() {
  delay(1000);  // Wartezeit damit der ESP anständig hochfährt
  WiFi.begin(ssid, pass);
  Serial.begin(115200);  // Serial Start
  Serial.println("\nESP32 Started! Running Routines...");
  while (WiFi.status() != WL_CONNECTED)  // Run this while WiFi is not connected yet
  {
    currentMillis = millis();
    if (currentMillis - previousMillis2 >= period2)  // Check if 30 sec have passed
    {
      previousMillis2 = currentMillis;  // Reset the 30 sec Timer or it will stay stuck
      Serial.println(". . .");
      if (currentMillis - previousMillis >= period)  // Check if 30 sec have passed
      {
        previousMillis = currentMillis;     // Reset the 30 sec Timer or it will stay stuck
        if (WiFi.status() != WL_CONNECTED)  // Last Check if WiFi is still not connected after 30 sec
        {
          Serial.println("No WiFi Connection Established after 30 Seconds, going to Deepsleep now to Conserve Energy!");
          Serial.println("ESP32 GOING INTO DEEPSLEEP NOW FOR 60 MIN!");
          esp_sleep_enable_timer_wakeup(1000000 * sleepTimeS); // Sleep Time Set
          esp_deep_sleep_start(); // Start Deep Sleep
        }
      }
    }
  }

  Serial.println("WiFi connected");
  Wire.setClock(400000);   // Setze I2C Geschwindigkeit
  delay(200);
  Wire.begin();      // I2C Initialisierung
  delay(200);
  Humidity.begin();  // HTU21D Initialisierung
  delay(200);
  bmp.begin();       // BMP280 Initialisierung
  delay(200);
  //sgp.begin();
  sgp.begin(&Wire, true);       // SGP30 Initialisierung
  delay(200);
  //sgp.IAQinit();     // SGP30 Initialisierung
  delay(200);
  ThingSpeak.begin(client);  // ThingSpeak Initialisierung
  Serial.println("15 sec Wartezeit damit alle Sensoren bereit sind");
  delay(15000);       // Wartezeit damit Sensoren ready sind und vor allem der SGP30 aufgeheizt ist
  //if (! sgp.begin()){
  //  Serial.println("Could not find a valid SGP30 sensor, check wiring or try a different address!");
  //}
  if (sgp.IAQmeasure()) {
      Serial.print("TVOC: "); Serial.print(sgp.TVOC); Serial.println(" ppb");
      Serial.print("eCO2: "); Serial.print(sgp.eCO2); Serial.println(" ppm");
  } else {
    Serial.println("SGP30 Test Measurement Failed\t");
  }
  if (sgp.IAQmeasureRaw()) {
    Serial.print("Raw H2: "); Serial.print(sgp.rawH2); Serial.println(" ");
    Serial.print("Raw Ethanol: "); Serial.print(sgp.rawEthanol); Serial.println(" ");
  } else {
    Serial.println("SGP30 Raw Test Measurement Failed\t");
  }
  if (bmp.begin()) {
    Serial.print("Luftdruck in hPa: "); Serial.print((bmp.readPressure() / 100)); Serial.println(" hPa");
  } else {
    Serial.println("BMP280 Test Measurement Failed\t");
  }
  htu_t_check = Humidity.readTemperature();
  htu_rh_check = Humidity.readHumidity();
  if (htu_t_check > -50 && htu_rh_check > -50) {
    Serial.print("Temperatur in °C: "); Serial.print(htu_t_check); Serial.println(" ");
    Serial.print("Luftfeuchte in %RH: "); Serial.print(htu_rh_check); Serial.println(" ");
  } else {
    Serial.println("HTU21D Test Measurement failed");
  }
  sgp_temp = Humidity.readTemperature();
  sgp_humid = Humidity.readHumidity();
  sgp.setHumidity(getAbsoluteHumidity(sgp_temp, sgp_humid));
  for (int i = 0; i < 30; i++) { // runs 30 times
  //Automatische Kalibrierung alle 30 messungen, wird bei jedem kompletten neustart einmal ausgeführt
        counter++;
        sgp.IAQmeasure();
        sgp.IAQmeasureRaw();
        //sgp.TVOC;
        //sgp.eCO2;
        //sgp.rawH2;
        //sgp.rawEthanol;
        Serial.print("Run Number for Calibration: ");
        Serial.print(counter);
        Serial.println("");
        delay(1000);
  if (counter == 30) {
    counter = 0;
    uint16_t TVOC_base, eCO2_base;
    if (sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
        //sgp.getIAQBaseline(&eCO2_base, &TVOC_base);
        Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
        Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
      } else {
        Serial.println("Failed to get baseline readings");
      }
  }
  }
  htu_t_check = Humidity.readTemperature();
  htu_rh_check = Humidity.readHumidity();
  if (sgp.IAQmeasure() && sgp.IAQmeasureRaw() && bmp.begin() && htu_t_check > -50 && htu_rh_check > -50){
  Serial.print("Temperatur in °C: "); Serial.print(htu_t_check); Serial.println(" ");
  Serial.print("Luftfeuchte in %RH: "); Serial.print(htu_rh_check); Serial.println(" ");
  Serial.print("Luftdruck in hPa: "); Serial.print((bmp.readPressure() / 100)); Serial.println(" hPa");
  Serial.print("TVOC: "); Serial.print(sgp.TVOC); Serial.println(" ppb");
  Serial.print("eCO2: "); Serial.print(sgp.eCO2); Serial.println(" ppm");
  Serial.print("Raw H2: "); Serial.print(sgp.rawH2); Serial.println(" ");
  Serial.print("Raw Ethanol: "); Serial.print(sgp.rawEthanol); Serial.println(" ");
  Serial.println("All Sensors seem to be Functional and Ready, Proceeding with Tasks");
  } else {
    Serial.println("Sensor Issue Detected!");
    Serial.println("Stopping Routines and putting ESP32/Sensors into Deep Sleep for Energy Conservation, next try in 60 minutes.");
    sgp.softReset();
    Wire.beginTransmission(0x00); // General Call mode -> writing to address 0 creates a 0x00 first byte
    Wire.write(0x06);
    Wire.endTransmission();
    Serial.println("ESP32 GOING INTO DEEPSLEEP NOW FOR 60 MIN!");
    esp_sleep_enable_timer_wakeup(1000000 * sleepTimeS); // Sleep Time Set
    esp_deep_sleep_start(); // Start Deep Sleep
  }
  Serial.println("All Startup Setup Routines have been done! Starting Main Tasks...");
}

void loop() {
  ts.execute();  // Task scheduler ausführung
}

// avg temp
void temp_avg_cb() {
  for (int i = 0; i < 10; i++) {
    tempReading[i] = Humidity.readTemperature();
  };
  float tempTotal = 0;
  for (int i = 0; i < 10; i++) {
    tempTotal += tempReading[i];
  };
  averageTemp = (tempTotal / 10);
}

// avg humid
void humid_avg_cb() {
  for (int i = 0; i < 10; i++) {
    humidReading[i] = Humidity.readHumidity();
  };
  float humidTotal = 0;
  for (int i = 0; i < 10; i++) {
    humidTotal += humidReading[i];
  };
  averageHumid = (humidTotal / 10);
}

// avg Pressure
void p_avg_cb() {
  for (int i = 0; i < 10; i++) {
    pReading[i] = (bmp.readPressure() / 100);
  };
  float pTotal = 0;
  for (int i = 0; i < 10; i++) {
    pTotal += pReading[i];
  };
  averagep = (pTotal / 10);
}

//avg CO2 etc.
void co_avg_cb(){
  sgp_temp = Humidity.readTemperature();
  sgp_humid = Humidity.readHumidity();
  sgp.setHumidity(getAbsoluteHumidity(sgp_temp, sgp_humid));

  sgp.IAQmeasure();     //muss jedes mal aufgerufen um einen neuen Sensor Wert zu messen
  sgp.IAQmeasureRaw();  //muss jedes mal aufgerufen um einen neuen Sensor Wert zu messen

  //avg TVOC
  for (int i = 0; i < 10; i++) {
    tvocReading[i] = sgp.TVOC;
  };
  float tvocTotal = 0;
  for (int i = 0; i < 10; i++) {
    tvocTotal += tvocReading[i];
  };
  averagetvoc = (tvocTotal / 10);

  //avg eCO2
  for (int i2 = 0; i2 < 10; i2++) {
    eco2Reading[i2] = sgp.eCO2;
  };
  float eco2Total = 0;
  for (int i2 = 0; i2 < 10; i2++) {
    eco2Total += eco2Reading[i2];
  };
  averageeco2 = (eco2Total / 10);

  //avg rawH2
  for (int i3 = 0; i3 < 10; i3++) {
    rawH2Reading[i3] = sgp.rawH2;
  };
  float rawH2Total = 0;
  for (int i3 = 0; i3 < 10; i3++) {
    rawH2Total += rawH2Reading[i3];
  };
  averagerawH2 = (rawH2Total / 10);

  //avg rawETH
  for (int i4 = 0; i4 < 10; i4++) {
    rawETHReading[i4] = sgp.rawEthanol;
  };
  float rawETHTotal = 0;
  for (int i4 = 0; i4 < 10; i4++) {
    rawETHTotal += rawETHReading[i4];
  };
  averagerawETH = (rawETHTotal / 10);

  //SGP30 Sensor zurücksetzen sobald die messungen abgeschlossen und werte gesetzt worden sind, hierdurch wird das Integrierte heizelement zur Stromersparnis abgeschaltet (Sleep Mode)
  if (averagetvoc > -50 && averageeco2 > -50 && averagerawH2 > -50 && averagerawETH > -50) {
          sgp.softReset();
          Wire.beginTransmission(0x00); // General Call mode -> writing to address 0 creates a 0x00 first byte
          Wire.write(0x06);
          Wire.endTransmission();
        }
}


//Funktion die für die Batteriespannung die custom mapping funktion benutzt
void bat_avg_cb() {
  // avg bat volt
  float voltage = (((map2(analogRead(35), 0, 4095, 0.00, 3.30)) + 0.11)*2); //offset da ADC des ESP32 nicht linear ist und wir nur mit halber auflösung messen um im genauesten bereich (mittig) des ADC zu agieren.
  for (int i = 0; i < 10; i++) {
    batReading[i] = voltage;
  };
  float batTotal = 0;
  for (int i = 0; i < 10; i++) {
    batTotal += batReading[i];
  };
  averagebat = (batTotal / 10);
}

//Ausgabe Funktion
void ausgabe_cb() {
  currentMillis3 = millis(); // Für 60 sec timer
  // Führe Checks aus, damit die Werte erst dann als JSON gesetzt werden, wenn diese Fertig berechnet worden sind
  if (averageTemp > -50 && averageHumid > -50 && averagep > -50 && averagetvoc > -50 && averageeco2 > -50 && averagerawH2 > -50 && averagerawETH > -50 && averagebat > -50) {
    Serial.println("Temperatur in °C: ");
    Serial.println(averageTemp);
    ThingSpeak.setField(1, averageTemp);
    Serial.println("Luftfeuchte in %RH: ");
    Serial.println(averageHumid);
    ThingSpeak.setField(2, averageHumid);
    Serial.println("Druck in hPa: ");
    Serial.println(averagep);
    ThingSpeak.setField(3, averagep);
    Serial.println("TVOC Wert in ppb: ");
    Serial.println(averagetvoc);
    ThingSpeak.setField(4, averagetvoc);
    Serial.println("eCO2 Wert in ppm: ");
    Serial.println(averageeco2);
    ThingSpeak.setField(5, averageeco2);
    Serial.println("Roh H2 Wert: ");
    Serial.println(averagerawH2);
    ThingSpeak.setField(6, averagerawH2);
    Serial.println("Roh Ethanol Wert: ");
    Serial.println(averagerawETH);
    ThingSpeak.setField(7, averagerawETH);
    Serial.println("Batteriespannung in Volt: ");
    Serial.println(averagebat);
    ThingSpeak.setField(8, averagebat);
    sgp.softReset(); //SGP30 Sensor zurücksetzen
    Wire.beginTransmission(0x00); // General Call mode -> writing to address 0 creates a 0x00 first byte
    Wire.write(0x06);
    Wire.endTransmission();
    Serial.println("Data Verified, all Fields have been Set and are Ready to be sent to ThingSpeak!");
    Serial.println("Sending data to Thingspeak now!");
    // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
    // pieces of information in a channel.  Here, we write all the Set fields.
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    // ThingSpeak will only accept updates every 15 seconds.
    Serial.println("ESP32 GOING INTO DEEPSLEEP NOW FOR 60 MIN!");
    esp_sleep_enable_timer_wakeup(1000000 * sleepTimeS); // Sleep Time Set
    esp_deep_sleep_start(); // Start Deep Sleep
}  else if (currentMillis3 - previousMillis3 >= period3) { // Check if 60 sec have passed
        previousMillis3 = currentMillis3;     // Reset the 60 sec Timer or it will stay stuck
        Serial.println("Not able to get Proper Data from Calculations after 60 Seconds, putting ESP32 and Sensors into Deep Sleep!");
        Serial.println("ESP32 GOING INTO DEEPSLEEP NOW FOR 60 MIN!");
        esp_sleep_enable_timer_wakeup(1000000 * sleepTimeS); // Sleep Time Set
        esp_deep_sleep_start(); // Start Deep Sleep
        }
}
