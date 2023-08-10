# LoRa Remote Sensor Display

An extensible, LoRa based remote monitor system powered by the [Adafruit Feather M0 RFM96 LoRa](https://www.adafruit.com/product/3179) microcontroller. The project is comprised of two parts: a Display Station and any number of Sensors.

## Display Station

Built using the Adafruit [Feather M0 RFM96 LoRa](https://www.adafruit.com/product/3179) and the [0.54" Quad Alphanumeric FeatherWing Display](https://www.adafruit.com/product/3127), the display station maintains an internal list of all known sensors and cycles between them, making requests and displaying results as it goes.

### Key Features

- Time-based cycling between sensors. Supports simple characters for things like humidity, temp, etc.
- Signal strength detection with warning and indicators for weak signal strength.
- Warning thresholds for high temps.
- Piezo beeper for alarm conditions.
- Power button for turning the display on/off.
- 3D print design is on [Tinkercad](https://www.tinkercad.com/things/hti6Ns5rOIe).

### Usage

This is not beginner project and will require manual editing to get working. Here are some tips to get started:

- Plan ahead. Make a list of every sensor the display should report and assign it a four-character name and a number. Enter this information into the sketch:

```
    uint8_t sensID[]   = {101};  
    char sensName[][5] = {"SANA"};
```

- Get familiar with the datagram format (`stnID, sensID, Batt, Data1, Data2`) and decide which field will contain the primary and secondary values (make a note of this for the sensors). Presently, the sketch assumes the use of two data points per sensor. If your sensor(s) won't report a second value then update the display queue to omit this information:

```
    // Data 2 (disp)  
    if ((x == 3) && (data_disp == 1) && (scr_pwr == 1)) {  
      sprintf(msg, "h%i  ", data[x]);  
      set_display(msg);  
      Serial.print("  Data2: "); Serial.print(data[x]); Serial.println("%");  
    }
```

- Update the warning/error conditions for each data point. This is fully customizable to differentiate between sensors (since each sensor has a unique ID) and/or data fields. Presently, the sketch assumes only one temperature sensor:

```
    if ((data[x] >= 87) && (data[x] <= 150)) {  
      Serial.println("    --> Warning: Sauna temp is high! <--");  
      warn_display(2, (char*)"SANA", (char*)"TEMP", (char*)"WARN");  
    }  
    if (data[x] >= 151) { // Melted or disconnected  
      Serial.println("    --> Warning: Sauna sensor failure! <--");  
      warn_display(2, (char*)"SANA", (char*)"SENS", (char*)"FAIL");  
    }
```

### Security

Given the remote deployment of this project and the non-sensitive data it reports, security is not a top priority. However, basic protection against unintentional interference is provided through the use of unique, static and manually configured IDs for the Display Station and Sensors. This allows the Display Station to validate replies and ignore anything it isn't expecting. Altogether, this has proven sufficient protection against 3rd party nearby packet radios on the 433mhz band.

## Sensor(s)

Sensors are simple devices that listen for a request from the Display Station and, if valid, take a reading and report back. Currently, the sketch is designed for a single SHT31 digital temp/humidity sensor with support for reporting battery status (and more), but may be customized with any compatible sensor.

### Key Features

- Ultra-minimal design, easily customizable with any Arduino compatible sensor.
- Two data fields allow reporting of up to two sensors.
- Built-in support for battery reporting and analog temp sensor.
- Display Station validation / replies only to valid requests from its associated Display Station.

### Usage

This is where the information the Display Station is expecting gets collected, assembled and transmitted. Therefore, just like the Display Station each sensor needs to be customized for its unique hardware and decisions applied regarding datagram fields.

- Hardware: decide on the sensor type and get the sensor reporting accurate values in the Arduino serial terminal.

```
    // Load the sensor driver  
    Adafruit_SHT31 sht31 = Adafruit_SHT31();  

    // Take a reading  
    uint8_t temperatureC = sht31.readTemperature();  
```

- Protocol: if the sensor only produces a single value (and/or there is only one sensor), then place that value into the first data field in the datagram. If there is a second value, place it in the second field:

```
    // Send the reply  
    uint8_t packet[] = {stationID, sensorID, sensorBatt, temperatureC, humidity};
```

- Features: does the unit use a battery? If not, then leave the section disabled:

```
    // Battery  
    // float measuredvbat = analogRead(VBATPIN);  
    // measuredvbat      *= 2;          // Multiply by 2  
    // measuredvbat      *= 3.3;        // Multiply by 3.3V (reference voltage)  
    // uint8_t sensorBatt = map(measuredvbat, 3400, 4200, 0, 100);  
    // Serial.print("   Batt:     "); Serial.print(sensorBatt); Serial.println("%");  
    Serial.print("   Batt:     "); Serial.println("Not installed/Set to 100%");  
    uint8_t sensorBatt = 100;
```

- Once the Sensor and Display Station are complete, they should communicate and the display will show the relevant values.
