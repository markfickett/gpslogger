/**
 * Fetch GPS data over serial and log it to an SD card.
 *
 * This uses TinyGPS to decode NMEA (RMS and GGA) sentences from a LS20031 GPS
 * module, and then uses the the SD library to write GPX formatted data to a
 * micro SD card.
 *
 * NMEA explanations: http://www.gpsinformation.org/dale/nmea.htm#position
 * GPS getting-started guide: https://www.sparkfun.com/tutorials/176
 * GPX tags http://wiki.openstreetmap.org/wiki/GPX
 *          http://www.topografix.com/gpx_manual.asp
 * Similar project http://forum.arduino.cc/index.php?topic=199019.15
 */

#include <SoftwareSerial.h>
#include <SdFat.h>
#include <TinyGPS.h>

#define SAMPLE_INTERVAL_MS 1000

#define PIN_STATUS_LED 13

#define PIN_RX_FROM_GPS 3
#define PIN_TX_TO_GPS 4
#define PIN_SD_CHIP_SELECT 10
#define PIN_SPI_CHIP_SELECT_REQUIRED 10
// The SD library requires, for SPI communication:
// 11 MOSI (master/Arduino output, slave/SD input)
// 12 MISO (master input, slave output)
// 13 CLK (clock)

// Seek to fileSize + this position before writing track points.
#define SEEK_TRKPT_BACKWARDS -24
#define GPX_EPILOGUE "\t</trkseg></trk>\n</gpx>\n"

TinyGPS gps;
SoftwareSerial nss(PIN_RX_FROM_GPS, PIN_TX_TO_GPS);

SdFat sd;
SdFile gpxFile;

char buf[32];

struct GpsSample {
  float lat_deg,
        lon_deg;
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
struct GpsSample sample;

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);
  Serial.begin(115200);
  setUpSd();
  setUpGps();
  digitalWrite(PIN_STATUS_LED, LOW);
}

void loop() {
  readFromGpsUntilSampleTime();
  fillGpsSample(gps);
  writeGpxSampleToSd();

  // TODO: Exit condition. Voltage drop from power being switched off?
  if (false) {
    gpxFile.close();
    Serial.println(F("Exiting."));
    exit(0);
  }
}

void setUpSd() {
  if (PIN_SD_CHIP_SELECT != PIN_SPI_CHIP_SELECT_REQUIRED) {
    pinMode(PIN_SPI_CHIP_SELECT_REQUIRED, OUTPUT);
  }

  if (!sd.begin(PIN_SD_CHIP_SELECT, SPI_QUARTER_SPEED)) {
    sd.initErrorHalt();
  }
}

void setUpGps() {
  nss.begin(57600);

  while (true) {
    readFromGpsUntilSampleTime();
    fillGpsSample(gps);
    if (sample.fix_age_ms == TinyGPS::GPS_INVALID_AGE) {
      Serial.println(F("Waiting for location fix."));
    } else if (sample.fix_age_ms == TinyGPS::GPS_INVALID_AGE) {
      Serial.println(F("Waiting for datetime fix."));
    } else {
      startGpxFileOnSdNoSync();
      break;
    }
  }
}

static void readFromGpsUntilSampleTime() {
  unsigned long start = millis();
  // Process a sample from the GPS every second.
  while (millis() - start < SAMPLE_INTERVAL_MS) {
    readFromGpsSerial();
  }
}

static bool readFromGpsSerial() {
  while (nss.available()) {
    gps.encode(nss.read());
  }
}

static void startGpxFileOnSdNoSync() {
  sprintf(
      buf,
      "%02d%02d%02d",
      sample.year,
      sample.month,
      sample.day);
  if (!sd.exists(buf)) {
    if (!sd.mkdir(buf)) {
      sd.errorHalt("Creating log directory for today failed.");
    }
  }
  sprintf(
      buf,
      "%02d%02d%02d/%02d%02d%02d.gpx",
      sample.year,
      sample.month,
      sample.day,
      sample.hour,
      sample.minute,
      sample.second);
  Serial.print(F("Starting log file "));
  Serial.println(buf);
  if (sd.exists(buf)) {
    Serial.println(F("warning: log file already exists, overwriting."));
  }
  if (!gpxFile.open(buf, O_CREAT | O_WRITE)) {
    sd.errorHalt("Opening new log file failed.");
  }

  gpxFile.print(F(
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<gpx version=\"1.0\">\n"
    "\t<trk><trkseg>\n"));
  gpxFile.print(F(GPX_EPILOGUE));
}

static void writeGpxSampleToSd() {
  gpxFile.seekSet(gpxFile.fileSize() + SEEK_TRKPT_BACKWARDS);
  gpxFile.print(F("\t\t<trkpt "));

  gpxFile.print(F("lat=\""));
  gpxFile.print(sample.lat_deg);
  gpxFile.print(F("\" lon=\""));
  gpxFile.print(sample.lon_deg);
  gpxFile.print(F("\">"));

  /*
  float altitude_cm;

  int satellites;
  int hdop_hundredths;
  unsigned long fix_age_ms;

  float speed_mps;
  float course_deg_hundredths;

  SdFile root;
  TinyGPS::GPS_INVALID_SATELLITES
  TinyGPS::GPS_INVALID_HDOP
  TinyGPS::GPS_INVALID_F_ANGLE
  TinyGPS::GPS_INVALID_F_ANGLE
  TinyGPS::GPS_INVALID_F_ALTITUDE
  TinyGPS::GPS_INVALID_F_SPEED
  TinyGPS::GPS_INVALID_F_ANGLE
  */
  gpxFile.print(F("</trkpt>\n"));

  gpxFile.print(F(GPX_EPILOGUE));

  digitalWrite(PIN_STATUS_LED, HIGH);
  if (!gpxFile.sync() || gpxFile.getWriteError()) {
    Serial.println(F("SD sync/write error."));
  }
  digitalWrite(PIN_STATUS_LED, LOW);
}

static void fillGpsSample(TinyGPS &gps) {
  gps.f_get_position(
      &sample.lat_deg,
      &sample.lon_deg,
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
}
