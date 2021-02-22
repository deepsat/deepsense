#include <Adafruit_BMP280.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define ground_pressure 1013.0;
#define lora_delay = 200;

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

/* rtcbot schema

writeFormat="< 3f 3f 3f 3f 3B H 3B ? B 5f B I"
writeKeys=[
    "temperature", "pressure", "altitude",
    "gyro_x", "gyro_y", "gyro_z",
    "accel_x", "accel_y", "accel_z",
    "mag_x", "mag_y", "mag_z",
    "hour", "minute", "second",
    "milisecond",
    "day", "month", "year",
    "gps_fix",
    "gps_fix_quality",
    "latitude", "longitude", "speed", "angle", "gps_altitude",
    "gps_num_satellites".
    "photos_taken"
]
*/
// will use same payload with lora and raspi
typedef struct __attribute__((packed)) {
    // BMP (K, hPa, m)
    float temperature, pressure, altitude;

    // Gyro (rad/s)
    float gyro_x, gyro_y, gyro_z;

    // Accel (m/s2)
    float accel_x, accel_y, accel_z;

    // Mag (uT)
    float mag_x, mag_y, mag_z;

    // GPS
    uint8_t hour, minute, second;
    uint16_t milisecond;
    uint8_t day, month, year;
    bool gps_fix;
    uint8_t gps_fix_quality;
    // deg, deg, knots, deg, m
    float latitude, longitude, speed, angle, gps_altitude;
    uint8_t gps_num_satellites;

    // rpi
    uint32_t photos_taken;
} toPi_t;

/* rtcbot schema

writeFormat="< I"
writeKeys=[
    "photos_taken"
]
*/

typedef struct __attribute__((packed)) {
    uint32_t photos_taken;
} fromPi_t;

toPi_t toPi;
fromPi_t fromPi;

void setup(void) {
    Serial1.begin(115200);

    GPS.begin(9600);
    // These lines configure the GPS Module
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Sets output to only RMC
                                                   // and GGA sentences
    GPS.sendCommand(
        PMTK_SET_NMEA_UPDATE_1HZ);   // Sets the output to 1/second. If you want
                                     // you can go higher/lower
    GPS.sendCommand(PGCMD_ANTENNA);  // Can report if antenna is connected or
                                     // not
}

void readBPMData() {
    toPi.temperature = bme.readTemperature();
    toPi.pressure = bme.readPressure() / 100;
    toPi.altitude = bme.readAltitude(ground_pressure);
}

void readGPSData() {
    GPS.parse(GPS.lastNMEA());

    toPi.hour = GPS.hour;
    toPi.minute = GPS.minute;
    toPi.second = GPS.second;
    toPi.milisecond = GPS.miliseconds;
    toPi.day = GPS.day;
    toPi.month = GPS.month;
    toPi.year = GPS.year;
    toPi.gps_fix = GPS.fix;
    toPi.gps_fix_quality = GPS.fixquality;
    if (GPS.fix) {
        toPi.latitude = GPS.latitudeDegrees;
        toPi.longitude = GPS.longitudeDegrees;
        toPi.speed = GPS.speed;
        toPi.angle = GPS.angle;
        toPi.gps_altitude = GPS.altitude;
        toPi.gps_num_satellites = GPS.satellites;
    }
}

void readAccelMagData() {
    sensors_event_t aevent, mevent;
    accelmag.getEvent(&aevent, &mevent);

    toPi.accel_x = aevent.acceleration.x;
    toPi.accel_y = aevent.acceleration.y;
    toPi.accel_z = aevent.acceleration.z;

    toPi.mag_x = mevent.magnetic.x;
    toPi.mag_y = mevent.magnetic.y;
    toPi.mag_z = mevent.magnetic.z;
}

unsigned long last_lora;

void loop() {
    Serial1.readBytes((char*)&fromPi, sizeof(fromPi));
    toPi.photos_taken = fromPi.photos_taken;
    readGPSData();
    readBPMData();
    readAccelMagData();
    Serial1.write((char*)&toPi, sizeof(toPi));
    if (millis() - last_lora >= lora_delay) {
        last_lora = millis();
        // send toPi via lora
    }
}