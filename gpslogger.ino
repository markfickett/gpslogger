/**
 * Fetch GPS data over serial and log it to an SD card.
 *
 * This uses TinyGPS to decode NMEA (RMS and GGA) sentences from a LS20031 GPS
 * module, and then uses the the SD library to write GPX formatted data to a
 * micro SD card.
 *
 * NMEA explanations: http://www.gpsinformation.org/dale/nmea.htm#position
 * GPS getting-started guide: https://www.sparkfun.com/tutorials/176
 */

#include <SoftwareSerial.h>
#include <SD.h>
#include <TinyGPS.h>

#define PIN_RX_FROM_GPS 3
#define PIN_TX_TO_GPS 4
#define PIN_SD_CHIP_SELECT 10
#define PIN_SPI_CHIP_SELECT_REQUIRED 10
// The SD library requires, for SPI communication:
// 11 MOSI (master/Arduino output, slave/SD input)
// 12 MISO (master input, slave output)
// 13 CLK (clock)

TinyGPS gps;
SoftwareSerial nss(PIN_RX_FROM_GPS, PIN_TX_TO_GPS);

Sd2Card card;
SdVolume volume;

bool gpxFileStarted;
char gpxFileName[32];

struct GpsSample {
  float lat_deg_millionths,
        lon_deg_millionths;
  float altitude_cm;

  int satellites;
  int hdop_hundredths;
  unsigned long fix_age_ms;

  float speed_mps;
  float course_deg_hundredths;

  unsigned long datetime_fix_age_ms;
  int year;
  byte month,
       day,
       hour,
       minute,
       second,
       hundredths;
};

void setup() {
  Serial.begin(115200);
  setUpSd();
  setUpGps();
  gpxFileStarted = false;
}

void setUpSd() {
  if (PIN_SD_CHIP_SELECT != PIN_SPI_CHIP_SELECT_REQUIRED) {
    pinMode(PIN_SPI_CHIP_SELECT_REQUIRED, OUTPUT);
  }

  if (!card.init(SPI_QUARTER_SPEED, PIN_SD_CHIP_SELECT)) {
    Serial.println(F("SD card initialization failed. Check card/wiring."));
    return;
  }

  if (!volume.init(card)) {
    Serial.println(F("Could not find FAT16/32 partition."));
    return;
  }
  Serial.print(F("Found FAT"));
  Serial.print(volume.fatType(), DEC);
  Serial.print(F(" volume, size is "));
  Serial.print(
      (volume.blocksPerCluster() * volume.clusterCount() * 512)
      / (1024 * 1024));
  Serial.println(F(" Mbytes."));
}

void setUpGps() {
  nss.begin(57600);
}

void loop() {
  unsigned long start = millis();
  // Every second we print an update.
  while (millis() - start < 1000) {
    readFromGpsSerial();
  }
  struct GpsSample sample = getGpsSample(gps);
  if (sample.fix_age_ms == TinyGPS::GPS_INVALID_AGE) {
    Serial.println(F("Waiting for location fix."));
  } else if (sample.fix_age_ms == TinyGPS::GPS_INVALID_AGE) {
    Serial.println(F("Waiting for datetime fix."));
  } else {
    if (!gpxFileStarted) {
      sprintf(
          gpxFileName,
          "%02d%02d%02d-%02d%02d%02d.gpx",
          sample.year,
          sample.month,
          sample.day,
          sample.hour,
          sample.minute,
          sample.second);
      Serial.print(F("Starting log file "));
      Serial.println(gpxFileName);
      startGpxFileOnSd(gpxFileName, sample);
      gpxFileStarted = true;
    }
    writeGpxSampleToSd(gpxFileName, sample);
  }
}

static bool readFromGpsSerial() {
  while (nss.available()) {
    gps.encode(nss.read());
  }
}

static void startGpxFileOnSd(
    const char* gpxFileName,
    const struct GpsSample &sample) {
}

static void writeGpxSampleToSd(
    const char* gpxFileName,
    const struct GpsSample &sample) {
  /*
  SdFile root;
  TinyGPS::GPS_INVALID_SATELLITES
  TinyGPS::GPS_INVALID_HDOP
  TinyGPS::GPS_INVALID_F_ANGLE
  TinyGPS::GPS_INVALID_F_ANGLE
  TinyGPS::GPS_INVALID_F_ALTITUDE
  TinyGPS::GPS_INVALID_F_SPEED
  TinyGPS::GPS_INVALID_F_ANGLE
  */
}

static struct GpsSample getGpsSample(TinyGPS &gps) {
  struct GpsSample sample;

  gps.f_get_position(
      &sample.lat_deg_millionths,
      &sample.lon_deg_millionths,
      &sample.fix_age_ms);
  sample.altitude_cm = gps.f_altitude();

  sample.satellites = gps.satellites();
  sample.hdop_hundredths = gps.hdop();

  sample.course_deg_hundredths = gps.f_course();
  sample.speed_mps = gps.f_speed_mps();

  gps.crack_datetime(
      &sample.year,
      &sample.month,
      &sample.day,
      &sample.hour,
      &sample.minute,
      &sample.second,
      &sample.hundredths,
      &sample.datetime_fix_age_ms);

  return sample;
}
