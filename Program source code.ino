/*
ECE445 final project code for Team 6: Health Hero
Wearable heart rate sensor with bluetooth connectivity.
*/

// HR sensor
#include <Wire.h> // include libraies for hr sensor
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;
// init vars for heart rate capture and calulation
const byte RATE_SIZE = 4; 
byte rates[RATE_SIZE]; 
byte rateSpot = 0;
long lastBeat = 0; 
float beatsPerMinute;
int beatAvg;

// bluetooth
#include <Arduino.h> // include libraies for BTE and lcd
#include <U8x8lib.h>
#include <SPI.h>
#include "SdFat.h"
#include <ArduinoBLE.h>
SdFat SD;
File BPM;
#define REPORTING_PERIOD_MS 1000
U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
// init vars for BTE communication
uint32_t tsLastReport = 0;
const int chipSelect = 10;
unsigned char c;

BLEService heartRateService("180D");
BLEUnsignedCharCharacteristic hR("2A37", BLERead | BLENotify);

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); 
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("Heart Rate"); // set name and AS for BTE connection
  BLE.setAdvertisedService(heartRateService);
  heartRateService.addCharacteristic(hR);
  BLE.addService(heartRateService);

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  u8x8.begin(); // start lcd screen
  SD.begin(chipSelect);
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Initializing...");

  // init sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");
  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x0A); 
  particleSensor.setPulseAmplitudeGreen(0); 
}

// constant loop for continuouse updating
void loop(){  
  BLEDevice central = BLE.central();
  long irValue = particleSensor.getIR();

  // if there is a heartbeat present, calculate the BPM
  if (checkForBeat(irValue) == true){ 
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    
    // only use reading in acceptable range
    if (beatsPerMinute < 200 && beatsPerMinute > 50){ 
      rates[rateSpot++] = (byte)beatsPerMinute; 
      rateSpot %= RATE_SIZE; 

      // average the readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];

      beatAvg /= RATE_SIZE;
    }
  }
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    // display "PLACE FINGER" text on lcd if finger is not present
    if (irValue < 50000){
      u8x8.clearDisplay();
      u8x8.setFont(u8x8_font_lucasarts_scumm_subtitle_o_2x2_f);
      u8x8.setCursor(0, 0);
      u8x8.print("PLACE");
      u8x8.setCursor(0, 2);
      u8x8.print("FINGER");
      tsLastReport = millis();
    }
    // display the average BPM when finger is on sensor
    else{
      u8x8.clearDisplay();
      u8x8.setFont(u8x8_font_courB18_2x3_f);
      u8x8.setCursor(0, 1);
      u8x8.print(beatAvg);
      u8x8.print(" BPM"); 
      tsLastReport = millis();
    }
  }
  // display IR, bpm, and average bpm in the serial output
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  // sent average bpm to phone via BTE
  if (central){
    hR.writeValue(beatAvg);
  }
  Serial.println();
}

