/*

Display Station
Author: Sven Thomas

*/

//INCLUDES
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <SPI.h>
#include <RH_RF95.h>

// DECLARATIONS
#define RFM95_CS 8      // RFMxx pin assignments
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 434.0 // Change to 434.0 or other frequency, must match RX's freq!
#define LED 13          // Diagnostic LED
#define VBATPIN A7      // M0 battery pin (used to measure local batt level)
#define SCR_PWR A2      // Pin for display toggle

// Sensor IDs and Names. These are paired by index so that we can get the name
// of the current sensor and know what kind of data it's reporting. Sensor names
// are a maxed at 4 characters to fit on the LCD display.
uint8_t sensID[]   = {101};
char sensName[][5] = {"SANA"};
uint8_t stnID = 10;                                 // Station ID to ensure it listens only to sensors paired with it.
RH_RF95 rf95(RFM95_CS, RFM95_INT);                  // Singleton instance of the radio driver
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();   // Create display object
char msg[5]  = "";                                  // Universal msg var to show message
uint8_t scr_pwr = 1;                                // Display power toggle var
unsigned long prevExec = 0;                         // Last execution of the main codeblock
unsigned long curExec = millis();
bool data_disp = 0;


// SETUP
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n#### BEGIN SETUP ####\n");

  // PINs
  pinMode(LED, OUTPUT);
  pinMode(11, OUTPUT);          // Buzzer
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(SCR_PWR, INPUT_PULLUP);

  // LED init & test
  alpha4.begin(0x70);           // Init (set the address)
  alpha4.writeDigitRaw(3, 0x0); // Test
  alpha4.writeDigitRaw(0, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(0, 0x0);
  alpha4.writeDigitRaw(1, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(1, 0x0);
  alpha4.writeDigitRaw(2, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(2, 0x0);
  alpha4.writeDigitRaw(3, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  clr_display();
  Serial.println("  LCD Setup Complete.");

  // Radio Reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the radio
  while (!rf95.init()) {
    Serial.println("  ERROR: LoRa radio init failed! Halting.");
    Serial.println("  Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info.");
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    Serial.println("  ERROR: setFrequency failed! Halting.");
    while (1);
  }
  Serial.print("  Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);   // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
                                // the default transmitter power is 13dBm, using PA_BOOST. If you are using RFM95/96/97/98
                                // modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm.
  Serial.println("  LoRa Init Complete.");

  Serial.println("\n#### END SETUP ####\n");
}

// FUNCTIONS
void set_display(char charDispBuff[5]) {
  alpha4.writeDigitAscii(0, charDispBuff[0]);
  alpha4.writeDigitAscii(1, charDispBuff[1]);
  alpha4.writeDigitAscii(2, charDispBuff[2]);
  alpha4.writeDigitAscii(3, charDispBuff[3]);
  alpha4.writeDisplay();
}

void clr_display() {
  alpha4.clear();
  alpha4.writeDisplay();
}

// Update the display based on warning type.
// Both types show the message. Type 2 adds beeps!
void warn_display(int WarnType, char errMsg1[5], char errMsg2[5], char errMsg3[5]) {
  if ((WarnType == 1) || (WarnType == 2)) {
    for (int y = 0; y < 2; y++) {  // 2 showings
      clr_display();
      delay(1666);
      set_display(errMsg1);
      delay(1666);
      set_display(errMsg2);
      delay(1666);
      set_display(errMsg3);
      delay(1666);
      if (WarnType == 2) {
        for (int z = 0; z < 3; z++) {  // 3 beeps
          digitalWrite(11, HIGH);
          delay(400);
          digitalWrite(11, LOW);
        }
      }
    }
    clr_display();
    delay(1666);
    set_display(msg);   // Go back to showing the current message
  }
}


// MAIN LOOP
void loop()
{

  // Detect screen pwr button.
  if ( digitalRead(SCR_PWR) == LOW ) {
    // If we're turning the screen on...
    if (scr_pwr == 0) {
      prevExec = millis();
      scr_pwr = 1;
      data_disp = 0;

      // Show the init sequence to acknowledge
      alpha4.begin(0x70);
      alpha4.writeDigitRaw(3, 0x0);
      alpha4.writeDigitRaw(0, 0xFFFF);
      alpha4.writeDisplay();
      delay(200);
      alpha4.writeDigitRaw(0, 0x0);
      alpha4.writeDigitRaw(1, 0xFFFF);
      alpha4.writeDisplay();
      delay(200);
      alpha4.writeDigitRaw(1, 0x0);
      alpha4.writeDigitRaw(2, 0xFFFF);
      alpha4.writeDisplay();
      delay(200);
      alpha4.writeDigitRaw(2, 0x0);
      alpha4.writeDigitRaw(3, 0xFFFF);
      alpha4.writeDisplay();
      delay(200);
      clr_display();
      delay(1000);
    }
    // If we're turning screen off...
    if ((scr_pwr == 1) && (digitalRead(SCR_PWR) == LOW)) {
      scr_pwr = 0;
      alpha4.begin(0x70);
      alpha4.writeDigitRaw(0, 0xFFFF);
      alpha4.writeDigitRaw(1, 0xFFFF);
      alpha4.writeDigitRaw(2, 0xFFFF);
      alpha4.writeDigitRaw(3, 0xFFFF);
      alpha4.writeDisplay();
      delay(1500);
      clr_display();
      delay(2000);
    }
  }

  if (scr_pwr == 1) {

    // Execution timer. Used to schedule temp vs. humidity display.
    curExec = millis();
    if (curExec - prevExec <= 49999) {
      // set_display("0000");  // Dev only. Shows something when there's nothing to show.
      data_disp = 0;        // Temp
    }
    if (curExec - prevExec >= 50000) {
      // set_display("1111");  // Dev only. Shows something when there's nothing to show.
      data_disp = 1;        // Humidity
    }
    if (curExec - prevExec >= 60000) {
      prevExec = curExec;
    }

    // Work with each sensor one-by-one
    for (int i = 0; i < sizeof(sensID); i++) {

      // Send a request to the sensor
      Serial.print("\nRequesting from sensor:   "); Serial.println(sensID[i]);
      uint8_t packet[] = {stnID, sensID[i]};      // Construct the request packet. Since the only thing
                                                  // the station sends is a request, anything coming
                                                  // from it is assumed by all sensors to be a request.
                                                  // Therefore, the packet contains only the requesting
                                                  // station ID and the sensor ID being queried. At this
                                                  // time there is no need for additional security/privacy.
      rf95.send(packet, sizeof(packet));     // Transmit the packet
      rf95.waitPacketSent();                 // Wait for it to finish sending

      // Wait for a reply
      // Incoming Protocol: stnID, sensID, Battery, Data1, Data2
      if (rf95.waitAvailableTimeout(1000)) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];       // Construct a recieve buffer
        uint8_t len = sizeof(buf);
        if (rf95.recv(buf, &len)) {

          // If the station or sensor IDs do not validate then try again.
          if (((uint8_t)buf[0] != stnID) || ((uint8_t)buf[1] != (uint8_t)sensID[i])) {
            Serial.println("WARNING: Wrong sensor replied! Retrying...\n");
            i--;
            continue;
          }

          // Clear the msg VAR from the last loop.
          sprintf(msg, "%s", "    ");

          // Station Battery
          float measuredvbat = analogRead(VBATPIN);
          measuredvbat *= 2;          // Multiply by 2
          measuredvbat *= 3.3;        // Multiply by 3.3V (reference voltage)
          uint8_t stnBatt = map(measuredvbat, 3400, 4200, 0, 100);

          // Data Processing
          int16_t data[5] = {rf95.lastRssi(), buf[2], buf[3], buf[4], stnBatt}; // RSSI, Battery, Data1, Data2, stnBatt
          for (int x = 0; x < sizeof(data); x++) {

            // Signal Strength (err cond)
            if (x == 0) {
              Serial.print("  RSSI: "); Serial.println(data[x]);
              if (data[x] <= -125) {
                Serial.println("    --> Warning: Sauna signal is weak! <--");
                warn_display(1, (char*)"SANA", (char*)"SIG ", (char*)"WEAK");
              }
            }

            // Sens Battery (err cond)
            // if (x == 1) {
            //   Serial.print("  Sensor Batt: "); Serial.print(data[x]); Serial.println("%");
            //   if (data[x] <= 5) {
            //     Serial.println("    --> Warning: Sauna battery is low! <--");
            //     warn_display(2, (char*)"SANA", (char*)"BATT", (char*)"LOW ");
            //   }
            // }

            // Data 1 (disp, err cond)
            if ((x == 2) && (data_disp == 0) && (scr_pwr == 1))  {
              sprintf(msg, "%ic  ", data[x]);
              set_display(msg);
              Serial.print("  Data1: "); Serial.println(msg);
              if ((data[x] >= 87) && (data[x] <= 150)) {
                Serial.println("    --> Warning: Sauna temp is high! <--");
                warn_display(2, (char*)"SANA", (char*)"TEMP", (char*)"WARN");
              }
              if (data[x] >= 151) {
                Serial.println("    --> Warning: Sauna sensor failure! <--");
                warn_display(2, (char*)"SANA", (char*)"SENS", (char*)"FAIL");
              }
            }

            // Data 2 (disp)
            if ((x == 3) && (data_disp == 1) && (scr_pwr == 1)) {
              sprintf(msg, "h%i  ", data[x]);
              set_display(msg);
              Serial.print("  Data2: "); Serial.print(data[x]); Serial.println("%");
            }

            // Station Battery (err cond)
            // if (x == 4) {
            //   Serial.print("  Station Batt: "); Serial.print(data[x]); Serial.println("%");
            //   if (data[x] <= 5) {
            //     Serial.println("    --> Warning: Station battery is low! <--");
            //     warn_display(2, (char*)"STN ", (char*)"BATT", (char*)"LOW ");
            //   }
            // }
            
          } // Data Processing
        } // Packet received
      } // Wait for reply
    } // Sensor loop
  } // Display pwr condition
} // Main loop
