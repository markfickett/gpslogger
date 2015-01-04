Arduino GPS Logger
==================

Fetch GPS data over serial and log it to an SD card.

This uses TinyGPS to decode NMEA (RMS and GGA) sentences from a [LS20031 GPS module](https://www.sparkfun.com/products/8975), and then uses the the SD library to write GPX formatted data to a [micro SD card](https://www.sparkfun.com/products/544).

This is a functional logger, and a good lesson about GPS modules, NMEA, and SD/FAT from Arduino. It is probably not the most efficient GPS logger (see "improvements").

![assembled logger](https://farm8.staticflickr.com/7580/16198442305_999b0dc2b0_z.jpg)

[more photos on flickr](https://www.flickr.com/photos/markfickett/sets/72157650108735212)

Setup
-----

Recommended configuration for the LS20031 GPS module:

    // FULL COLD RESTART (clears any bad almanac data)
    Serial.println("$PMTK104*37");
    // GGA + RMC (all that is used by TinyGPS), 1Hz
    Serial.println("$PMTK314,0,5,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
    // Reduce serial output rate 57600 => 14400 baud, since SoftwareSerial
    // on an 8MHz Arduino Pro Mini can't keep up (though an Uno can).
    Serial.println("$PMTK251,14400*29");

Since the module sometimes loses its configuration, the Arduino startup code always sends the latter two configuration commands.

Also edit SoftwareSerial.h to have a larger buffer (default is 64):

    #define _SS_MAX_RX_BUFF 256

On MacOS, it's in Arduino.app in Contents/Resources/Java/libraries/SoftwareSerial/.

Voltage Detection
-----------------

Voltage and power-off detection is designed for an Arduino Pro Mini 3.3v
powered from at least 3.8v (3x NiMH AAs), with a 680uF 12+v capacitor across
the raw voltage supply, and a 560KOhm/120KOhm voltage divider to A3. The
setup (with GPS and SD writing) draws around 42mA.

Improvements
------------

* [The MTK3339 from Adafruit](http://www.adafruit.com/product/746) looks better for this application: smaller, lower power (20mA), built-in datalogging, 5v safe but accepts 3.3v power.
* Power-off detection is unreliable; a shutdown switch (perhaps a capacitive touch sensor on the casing) would be safer.

Resources
---------

* [SparkFun GPS getting-started guide](https://www.sparkfun.com/tutorials/176)
* [NMEA explanations](http://www.gpsinformation.org/dale/nmea.htm#position)
* [NMEA checksum calculator](http://www.hhhh.org/wiml/proj/nmeaxor.html)
* GPX tags specification on [OpenStreetMap](http://wiki.openstreetmap.org/wiki/GPX) or [Topografix](http://www.topografix.com/gpx_manual.asp)
* [Similar project on Arduino forums](http://forum.arduino.cc/index.php?topic=199019.15)
