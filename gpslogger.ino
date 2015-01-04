/**
 * Fetch GPS data over serial and log it to an SD card. See README for more.
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

#define DEFAULT_GPS_BAUD 57600
#define GPS_BAUD 14400

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

// The previously sampled / recorded voltages.
float lastVoltage;
float lastRecordedVoltage;
byte lastRecordedVoltageMinute;

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);
  lastVoltage = 0;
  lastRecordedVoltage = 0;
  lastRecordedVoltageMinute = 61;
  Serial.begin(115200);
  setUpSd();
  resetGpsConfig();
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
  recordVoltage();
}

void setUpSd() {
  if (PIN_SD_CHIP_SELECT != PIN_SPI_CHIP_SELECT_REQUIRED) {
    pinMode(PIN_SPI_CHIP_SELECT_REQUIRED, OUTPUT);
  }

  if (!sd.begin(PIN_SD_CHIP_SELECT, SPI_QUARTER_SPEED)) {
    sd.initErrorHalt();
  }
}

/**
 * Redoes GPS configuration, assuming it has factory-default settings.
 *
 * Although this should only have to be done once, the GPS module sometimes
 * drops these customizations (possibly due to power brown-out).
 */
void resetGpsConfig() {
  digitalWrite(PIN_STATUS_LED, HIGH);
  nss.begin(DEFAULT_GPS_BAUD);
  nss.println("$PMTK314,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  nss.println("$PMTK251,14400*29");
  nss.flush();
  nss.end();
  digitalWrite(PIN_STATUS_LED, LOW);

  nss.begin(GPS_BAUD);
}

void getFirstGpsSample() {
  nss.begin(GPS_BAUD);

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
    getVoltageMaybeExit();
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
  getVoltageMaybeExit();

  gpxFile.print(F("<time>"));
  writeFormattedSampleDatetime(gpxFile);
  gpxFile.print(F("</time>"));

  if (sample.altitude_m != TinyGPS::GPS_INVALID_F_ALTITUDE) {
    gpxFile.print(F("<ele>")); // meters
    writeFloat(sample.altitude_m, gpxFile, 2 /* centimeter precision */);
    gpxFile.print(F("</ele>"));
  }
  getVoltageMaybeExit();

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
  getVoltageMaybeExit();

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
  getVoltageMaybeExit();

  gpxFile.print(F("</trkpt>\n"));

  gpxFile.print(F(GPX_EPILOGUE));

  getVoltageMaybeExit();
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

static float getVoltageMaybeExit() {
  // TODO: Do safety calculation in int to be faster?
  float voltage =
      analogRead(PIN_BATTERY_DIVIDED_VOLTAGE) / VOLTAGE_ANALOG_CONSTANT;
  float dv = voltage - lastVoltage;
  // Regulated (USB) supply voltage is approximately 0 on the battery vin,
  // otherwise abort if vin is low or falling.
  if (voltage > 2.0
      && (voltage < CUTOFF_VOLTAGE || dv < CUTOFF_DV)) {
    gpxFile.close();
    voltageFile.print(F("x"));
    voltageFile.close();
    digitalWrite(PIN_STATUS_LED, HIGH);
    Serial.print(F("Exiting, final voltage was "));
    Serial.println(voltage);
    Serial.flush();
    exit(0);
    // TODO: Resume in case of a false alarm.
  }
  lastVoltage = voltage;
  return voltage;
}

static bool recordVoltage() {
  float voltage = getVoltageMaybeExit();
  if (voltage != lastRecordedVoltage
      || lastRecordedVoltageMinute != sample.minute) {
    writeFormattedSampleDatetime(voltageFile);
    voltageFile.print(F(","));
    writeFloat(voltage, voltageFile, 2);
    voltageFile.print(F("\n"));
    lastRecordedVoltageMinute = sample.minute;
    lastRecordedVoltage = voltage;

    digitalWrite(PIN_STATUS_LED, HIGH);
    if (!voltageFile.sync() || voltageFile.getWriteError()) {
      Serial.println(F("SD sync/write error."));
    }
    digitalWrite(PIN_STATUS_LED, LOW);
  }

}
