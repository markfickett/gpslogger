/**
 * Fetch GPS data over serial and log it to an SD card.
 *
 * This uses TinyGPS to decode NMEA (RMS and GGA) sentences from a LS20031 GPS
 * module, and then uses the the SD library to write GPX formatted data to a
 * micro SD card.
 *
 * Recommended configuration for the LS20031 GPS module:
 *   // FULL COLD RESTART (clears any bad almanac data)
 *   Serial.println("$PMTK104*37");
 *   // GGA + RMC (all that is used by TinyGPS), 1Hz
 *   Serial.println("$PMTK314,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
 *   // Reduce serial output rate 57600 => 14400 baud, since SoftwareSerial
 *   // on an 8MHz Arduino Pro Mini can't keep up (though an Uno can).
 *   Serial.println("$PMTK251,14400*29");
 * Also edit SoftwareSerial.h to have a larger buffer (default is 64):
 *   #define _SS_MAX_RX_BUFF 256
 * On MacOS, it's in Contents/Resources/Java/libraries/SoftwareSerial/.
 *
 * Voltage and power-off detection is designed for an Arduino Pro Mini 3.3v
 * powered from at least 3.8v (3x NiMH AAs), with a 680uF 12+v capacitor across
 * the raw voltage supply, and a 560KOhm/120KOhm voltage divider to A3. The
 * setup (with GPS and SD writing) draws around 42mA.
 *
 * GPS getting-started guide: https://www.sparkfun.com/tutorials/176
 * NMEA explanations: http://www.gpsinformation.org/dale/nmea.htm#position
 * NMEA checksum calculator: http://www.hhhh.org/wiml/proj/nmeaxor.html
 * GPX tags http://wiki.openstreetmap.org/wiki/GPX
 *          http://www.topografix.com/gpx_manual.asp
 * Similar project http://forum.arduino.cc/index.php?topic=199019.15
 */

#include <SoftwareSerial.h>
#include <SdFat.h>
#include <TinyGPS.h>

#define SAMPLE_INTERVAL_MS 1000

#define PIN_STATUS_LED 13

#define PIN_RX_FROM_GPS 2
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
#define LATLON_PREC 6

#define PIN_BATTERY_DIVIDED_VOLTAGE A3
#define VOLTAGE_ANALOG_CONSTANT 53.71
#define CUTOFF_DV -0.1
#define CUTOFF_VOLTAGE 3.4

TinyGPS gps;
SoftwareSerial nss(PIN_RX_FROM_GPS, PIN_TX_TO_GPS);

SdFat sd;
SdFile gpxFile;
SdFile voltageFile;

// General-purpose text buffer used in formatting.
char buf[32];

struct GpsSample {
  float lat_deg,
        lon_deg;
  float altitude_m;

  int satellites;
  int hdop_hundredths;

  // How many ms, according to millis(), since the last position data was read
  // from the GPS.
  unsigned long fix_age_ms;

  float speed_mps;
  float course_deg;

  // How many ms, according to millis(), since the last datetime data was read
  // from the GPS.
  unsigned long datetime_fix_age_ms;

  int year;
  byte month,
       day,
       hour,
       minute,
       second,
       hundredths;
};
// The latest sample read from the GPS.
struct GpsSample sample;

// The previously sampled / recorded voltage.
float lastVoltage;
byte lastRecordedVoltageMinute;

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);
  lastVoltage = 0;
  lastRecordedVoltageMinute = 61;
  Serial.begin(115200);
  setUpSd();
  getFirstGpsSample();
  startFilesOnSdNoSync();
  digitalWrite(PIN_STATUS_LED, LOW);
}

void loop() {
  readFromGpsUntilSampleTime();
  fillGpsSample(gps);
  if (sample.fix_age_ms <= SAMPLE_INTERVAL_MS) {
    // TODO: Write whenever there is new data (trust GPS is set at 1Hz).
    writeGpxSampleToSd();
  }

  if (!recordVoltageAndReportSafe()) {
    gpxFile.close();
    voltageFile.print(F("detected power-off"));
    voltageFile.close();
    Serial.print(F("Exiting, last voltage was "));
    Serial.println(lastVoltage);
    // TODO: Resume in case of a false alarm.
    while(true) {
      //digitalWrite(PIN_STATUS_LED, HIGH);
      delay(50);
      //digitalWrite(PIN_STATUS_LED, LOW);
      delay(500);
    }
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

void getFirstGpsSample() {
  nss.begin(14400);

  while (true) {
    readFromGpsUntilSampleTime();
    fillGpsSample(gps);
    if (sample.fix_age_ms == TinyGPS::GPS_INVALID_AGE) {
      Serial.println(F("Waiting for location fix."));
    } else if (sample.fix_age_ms == TinyGPS::GPS_INVALID_AGE) {
      Serial.println(F("Waiting for datetime fix."));
    } else {
      Serial.println(F("Got GPS fix."));
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

static void startFilesOnSdNoSync() {
  // directory
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

  // SdFat will silently die if given a filename longer than "8.3"
  // (8 characters, a dot, and 3 file-extension characters).

  // GPX log
  openTimestampedFile(".gpx", gpxFile);
  gpxFile.print(F(
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<gpx version=\"1.0\">\n"
    "\t<trk><trkseg>\n"));
  gpxFile.print(F(GPX_EPILOGUE));

  // input (battery) voltage log
  openTimestampedFile("v.csv", voltageFile);
  voltageFile.print(F("datetime,voltage\n"));
}

static void openTimestampedFile(const char *shortSuffix, SdFile &file) {
  sprintf(
      buf,
      "%02d%02d%02d/%02d%02d%02d%s",
      sample.year,
      sample.month,
      sample.day,
      sample.hour,
      sample.minute,
      sample.second,
      shortSuffix);
  Serial.print(F("Starting file "));
  Serial.println(buf);
  if (sd.exists(buf)) {
    Serial.println(F("warning: already exists, overwriting."));
  }
  if (!file.open(buf, O_CREAT | O_WRITE)) {
    sd.errorHalt();
  }
}

static void writeFloat(float v, SdFile &file, int precision) {
  obufstream ob(buf, sizeof(buf));
  ob << setprecision(precision) << v;
  file.print(buf);
}

static void writeFormattedSampleDatetime(SdFile &file) {
  sprintf(
      buf,
      "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
      sample.year,
      sample.month,
      sample.day,
      sample.hour,
      sample.minute,
      sample.second,
      sample.hundredths);
  file.print(buf);
}

static void writeGpxSampleToSd() {
  gpxFile.seekSet(gpxFile.fileSize() + SEEK_TRKPT_BACKWARDS);
  gpxFile.print(F("\t\t<trkpt "));

  gpxFile.print(F("lat=\""));
  writeFloat(sample.lat_deg, gpxFile, LATLON_PREC);
  gpxFile.print(F("\" lon=\""));
  writeFloat(sample.lon_deg, gpxFile, LATLON_PREC);
  gpxFile.print(F("\">"));

  gpxFile.print(F("<time>"));
  writeFormattedSampleDatetime(gpxFile);
  gpxFile.print(F("</time>"));

  if (sample.altitude_m != TinyGPS::GPS_INVALID_F_ALTITUDE) {
    gpxFile.print(F("<ele>")); // meters
    writeFloat(sample.altitude_m, gpxFile, 2 /* centimeter precision */);
    gpxFile.print(F("</ele>"));
  }

  if (sample.speed_mps != TinyGPS::GPS_INVALID_F_SPEED) {
    gpxFile.print(F("<speed>"));
    writeFloat(sample.speed_mps, gpxFile, 1);
    gpxFile.print(F("</speed>"));
  }
  if (sample.course_deg != TinyGPS::GPS_INVALID_F_ANGLE) {
    gpxFile.print(F("<course>"));
    writeFloat(sample.course_deg, gpxFile, 1);
    gpxFile.print(F("</course>"));
  }

  if (sample.satellites != TinyGPS::GPS_INVALID_SATELLITES) {
    gpxFile.print(F("<sat>"));
    gpxFile.print(sample.satellites);
    gpxFile.print(F("</sat>"));
  }
  if (sample.hdop_hundredths != TinyGPS::GPS_INVALID_HDOP) {
    gpxFile.print(F("<hdop>"));
    writeFloat(sample.hdop_hundredths / 100.0, gpxFile, 2);
    gpxFile.print(F("</hdop>"));
  }

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
  sample.altitude_m = gps.f_altitude();

  sample.satellites = gps.satellites();
  sample.hdop_hundredths = gps.hdop();

  sample.course_deg = gps.f_course();
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

static bool recordVoltageAndReportSafe() {
  float voltage =
      analogRead(PIN_BATTERY_DIVIDED_VOLTAGE) / VOLTAGE_ANALOG_CONSTANT;
  if (voltage == 0.0) {
    return true; // Regulated (USB) supply voltage.
  }

  if (lastVoltage != voltage || lastRecordedVoltageMinute != sample.minute) {
    writeFormattedSampleDatetime(voltageFile);
    voltageFile.print(F(","));
    writeFloat(voltage, voltageFile, 2);
    voltageFile.print(F("\n"));
    lastRecordedVoltageMinute = sample.minute;

    digitalWrite(PIN_STATUS_LED, HIGH);
    if (!voltageFile.sync() || voltageFile.getWriteError()) {
      Serial.println(F("SD sync/write error."));
    }
    digitalWrite(PIN_STATUS_LED, LOW);
  }

  float dv = voltage - lastVoltage;
  lastVoltage = voltage;
  return voltage > CUTOFF_VOLTAGE && dv > CUTOFF_DV;
}
