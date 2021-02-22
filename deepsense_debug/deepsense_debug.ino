// BME 280 Chip Id: 0x58 / Address: 0x76 (if SDO=0)

#include <Adafruit_BMP280.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// I2C Interface
Adafruit_BMP280 bme(0x76);

// We can now create our Software Serial object after including the GPS library
SoftwareSerial mySerial(3, 2);

// And finally attach our Serial object pins to our GPS module
Adafruit_GPS GPS(&mySerial);

float ground_pressure = 1013.0;

void printGyroDetails(void) {
    sensor_t sensor;
    gyro.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    0x");
    Serial.println(sensor.sensor_id, HEX);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" rad/s");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" rad/s");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" rad/s");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void printAccelMagDetails(void) {
    sensor_t accel, mag;
    accelmag.getSensor(&accel, &mag);
    Serial.println("------------------------------------");
    Serial.println("ACCELEROMETER");
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(accel.name);
    Serial.print("Driver Ver:   ");
    Serial.println(accel.version);
    Serial.print("Unique ID:    0x");
    Serial.println(accel.sensor_id, HEX);
    Serial.print("Min Delay:    ");
    Serial.print(accel.min_delay);
    Serial.println(" s");
    Serial.print("Max Value:    ");
    Serial.print(accel.max_value, 4);
    Serial.println(" m/s^2");
    Serial.print("Min Value:    ");
    Serial.print(accel.min_value, 4);
    Serial.println(" m/s^2");
    Serial.print("Resolution:   ");
    Serial.print(accel.resolution, 8);
    Serial.println(" m/s^2");
    Serial.println("------------------------------------");
    Serial.println("");
    Serial.println("------------------------------------");
    Serial.println("MAGNETOMETER");
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(mag.name);
    Serial.print("Driver Ver:   ");
    Serial.println(mag.version);
    Serial.print("Unique ID:    0x");
    Serial.println(mag.sensor_id, HEX);
    Serial.print("Min Delay:    ");
    Serial.print(accel.min_delay);
    Serial.println(" s");
    Serial.print("Max Value:    ");
    Serial.print(mag.max_value);
    Serial.println(" uT");
    Serial.print("Min Value:    ");
    Serial.print(mag.min_value);
    Serial.println(" uT");
    Serial.print("Resolution:   ");
    Serial.print(mag.resolution);
    Serial.println(" uT");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void printBMPData(float ground_pressure) {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() /
                 100);  // 100 Pa = 1 millibar (Pa = newton per square meter)
    Serial.print("Approx Altitude = ");
    Serial.print(bme.readAltitude(
        ground_pressure));  // This should be lined up (atmospheric pressure at
                            // sea level is 1013 millibars);
}

void setup(void) {
    Serial.begin(115200);

    /* Wait for the Serial Monitor */
    while (!Serial) {
        delay(1);
    }

    if (!bme.begin()) {
        Serial.println("Error! No BMP Sensor Detected!!!");
        while (1)
            ;
    }

    if (!gyro.begin()) {
        /* There was a problem detecting the FXAS21002C ... check your
         * connections
         */
        Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
        while (1)
            ;
    }

    /* Display some basic information on this sensor */
    printGyroDetails();

    if (!accelmag.begin(ACCEL_RANGE_4G)) {
        /* There was a problem detecting the FXOS8700 ... check your connections
         */
        Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
        while (1)
            ;
    }

    /* Display some basic information on this sensor */
    printAccelMagDetails();

    GPS.begin(9600);
    // These lines configure the GPS Module
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Sets output to only RMC
                                                   // and GGA sentences
    GPS.sendCommand(
        PMTK_SET_NMEA_UPDATE_1HZ);  // Sets the output to 1/second. If you want
                                    // you can go higher/lower
    GPS.sendCommand(PGCMD_ANTENNA);  // Can report if antenna is connected or
                                     // not
}

void printGyroData(void) {
    /* Get a new sensor event */
    sensors_event_t event;
    gyro.getEvent(&event);

    /* Display the results (speed is measured in rad/s) */
    Serial.print("X: ");
    Serial.print(event.gyro.x);
    Serial.print("  ");
    Serial.print("Y: ");
    Serial.print(event.gyro.y);
    Serial.print("  ");
    Serial.print("Z: ");
    Serial.print(event.gyro.z);
    Serial.print("  ");
    Serial.println("rad/s ");
}

void printAccelMagData(void) {
    sensors_event_t aevent, mevent;

    /* Get a new sensor event */
    accelmag.getEvent(&aevent, &mevent);

    /* Display the accel results (acceleration is measured in m/s^2) */
    Serial.print("A ");
    Serial.print("X: ");
    Serial.print(aevent.acceleration.x, 4);
    Serial.print("  ");
    Serial.print("Y: ");
    Serial.print(aevent.acceleration.y, 4);
    Serial.print("  ");
    Serial.print("Z: ");
    Serial.print(aevent.acceleration.z, 4);
    Serial.print("  ");
    Serial.println("m/s^2");

    /* Display the mag results (mag data is in uTesla) */
    Serial.print("M ");
    Serial.print("X: ");
    Serial.print(mevent.magnetic.x, 1);
    Serial.print("  ");
    Serial.print("Y: ");
    Serial.print(mevent.magnetic.y, 1);
    Serial.print("  ");
    Serial.print("Z: ");
    Serial.print(mevent.magnetic.z, 1);
    Serial.print("  ");
    Serial.println("uT");
    Serial.println("");
}

void printGPSData(void) {
    // Now we will start our GPS module, parse (break into parts) the Last NMEA
    // sentence
    GPS.parse(GPS.lastNMEA());  // This is going to parse the last NMEA sentence
                                // the Arduino has received, breaking it down
                                // into its constituent parts.
    GPS.newNMEAreceived();  // This will return a boolean TRUE/FALSE depending
                            // on the case.
    // Print the current date/time/etc
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC);
    Serial.print('/');
    Serial.print(GPS.month, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(" quality: ");
    Serial.println((int)GPS.fixquality);
    // If GPS module has a fix, line by line prints the GPS information
    if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4);
        Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4);
        Serial.println(GPS.lon);
        Serial.print("Location (in degrees, works with Google Maps): ");
        Serial.print(GPS.latitudeDegrees, 4);
        Serial.print(", ");
        Serial.println(GPS.longitudeDegrees, 4);
        Serial.print("Speed (knots): ");
        Serial.println(GPS.speed);
        Serial.print("Angle: ");
        Serial.println(GPS.angle);
        Serial.print("Altitude: ");
        Serial.println(GPS.altitude);
        Serial.print("Satellites: ");
        Serial.println((int)GPS.satellites);
    }
}

void loop(void) {
    printBMPData(ground_pressure);
    printGyroData();
    printAccelMagData();
    delay(500);
}
