/*

Sensor
Author: Sven Thomas

*/

//INCLUDES
#include <SPI.h>
#include <RH_RF95.h>
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

// DECLARATIONS
#define RFM95_CS 8      // RFMxx pin assignment
#define RFM95_RST 4     // RFMxx pin assignment
#define RFM95_INT 3     // RFMxx pin assignment
#define RF95_FREQ 434.0 // Change to 434.0 or other frequency, must match RX's freq!
#define LED 13          // Blinky on loop
#define VBATPIN A7      // M0 battery pin (used to measure batt level)

uint8_t stationID = 10;                     // Station ID to which this sensor belongs
uint8_t sensorID = 101;                     // Sensor ID of this sensor
RH_RF95 rf95(RFM95_CS, RFM95_INT);          // Singleton instance of the radio driver
uint8_t loopCnt = 0;                        // Loop count for toggling the sensor heater
Adafruit_SHT31 sht31 = Adafruit_SHT31();    // Load the sensor driver

// Uncomment if using the TMP36 analog sensor
// int sensorPin = 0; // The analog pin the TMP36's Vout (sense) pin is connected to
//                    // the resolution is 10 mV / degree centigrade with a
//                    // 500 mV offset to allow for negative temperatures

// SETUP
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n#### BEGIN SETUP ####\n");

  // PINs
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // SHT31 Sensor Init
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("  --> ERROR! Couldn't find SHT31. Halting. <--");
    while (1) delay(1);
  }
  sht31.heater(false);        // The sensor has a heater to clear off condensation
                              // that is disabled for the sauna. Note that enabling
                              // the heater causes the temp to report +2C hotter.
  Serial.println("  SHT31 initialized (heater is off).");

  // Radio Reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the radio
  while (!rf95.init()) {
    Serial.println("  --> ERROR! LoRa radio init failed. Halting. <--");
    Serial.println("  Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    Serial.println("  --> ERROR! Setting LoRa frequency failed. Halting. <--");
    while (1);
  }
  Serial.print("  LoRa freq: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);   // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
                                // the default transmitter power is 13dBm, using PA_BOOST. If you are using RFM95/96/97/98
                                // modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm.
  Serial.println("  LoRa Init Complete.");

  Serial.println("\n#### END SETUP ####\n");
}


// MAIN LOOP
void loop()
{
  // Blink the LED to indicate things are still working
  digitalWrite(LED, HIGH);
  delay(150);
  digitalWrite(LED, LOW);
  delay(150);

  Serial.println("Waiting for request...");

  // Wait for incoming request from the base station
  // Incoming Protocol: stationID, sensorID
  if (rf95.waitAvailableTimeout(10000))  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];     // Prepare a buffer to hold the incoming packet
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {               // If there's an incoming request
      Serial.println("Transmission recieved! Processing...");
      Serial.print("\n   LoRa RX:  "); Serial.print(buf[0]); Serial.print(", "); Serial.println(buf[1]);

      // Only take readings if the request is valid
      if ((uint8_t)buf[0] == stationID && (uint8_t)buf[1] == sensorID) {
        Serial.print("   Valid request from station "); Serial.print((uint8_t)buf[0]); Serial.println(". Proceeding...");

        // Temperature
        // (DIGITAL SENSOR SHT31)
        uint8_t temperatureC = sht31.readTemperature();
        if (! isnan(temperatureC)) {  // check if 'is not a number'
          Serial.print("   Temp:     "); Serial.print(temperatureC); Serial.println("c");
        } else {
          Serial.println("   --> WARNING: Failed to read temperature! <--");
        }

        // ANALOG SENSOR (TMP36)
        // int reading = analogRead(sensorPin);              // Get the voltage reading from the temp sensor
        // float voltage = reading * 3.3;                    // Convert to voltage (3.3v)
        // voltage /= 1024.0;
        // uint8_t temperatureC = (voltage - 0.5) * 100 ;    // Convert from 10 mv per degree w/500 mV offset to degrees ((voltage - 500mV) times 100)
        // Serial.print("   Temp:     "); Serial.print(temperatureC); Serial.println("c");

        // Humidity
        uint8_t humidity = sht31.readHumidity();
        if (! isnan(humidity)) {  // check if 'is not a number'
          Serial.print("   Humidity: "); Serial.print(humidity); Serial.println("%");
        } else {
          Serial.println("   --> WARNING: Failed to read humidity! <--");
        }

        // Battery
        // float measuredvbat = analogRead(VBATPIN);
        // measuredvbat      *= 2;          // Multiply by 2
        // measuredvbat      *= 3.3;        // Multiply by 3.3V (reference voltage)
        // uint8_t sensorBatt = map(measuredvbat, 3400, 4200, 0, 100);
        // Serial.print("   Batt:     "); Serial.print(sensorBatt); Serial.println("%");
        Serial.print("   Batt:     "); Serial.println("Not installed/Set to 100%");
        uint8_t sensorBatt = 100;

        // Send the reply
        uint8_t packet[] = {stationID, sensorID, sensorBatt, temperatureC, humidity};
        rf95.send(packet, sizeof(packet));
        rf95.waitPacketSent();
        Serial.print("   LoRa TX:  "); Serial.print(packet[0]); Serial.print(", ");
                                      Serial.print(packet[1]); Serial.print(", ");
                                      Serial.print(packet[2]); Serial.print(", ");
                                      Serial.print(packet[3]); Serial.print(", ");
                                      Serial.print(packet[4]); Serial.println("\n");
      } else {
        Serial.println("\nInvalid request ignored.\n");
      }
    }
  } else {
    Serial.println("Request timeout.\n");
  }
}
