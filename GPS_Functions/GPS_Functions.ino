#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

long unsigned int timer = 0;

HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO false
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()
{
    //Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
    Serial.begin(115200);
    Wire.begin();

    setupGPS();
}

void loop()
{
    parseGPS();

    // if millis() or timer wraps around, we'll just reset it - GPS
    if (timer > millis())
        timer = millis();

    // approximately every 2 seconds or so, print out the current stats - GPS
    if (millis() - timer > 2000)
    {
        timer = millis(); // reset the timer

        //SD Card
        datalog = SD.open(filename, FILE_WRITE); //starts writing on the SD Card

        datalogGPS();

        datalog.println();
        datalog.close();
        delay(100);

        //This is stuff printed on the Serial Monitor
        serialGPS();

        Serial.println();

        delay(1000);
    }
}

void setupGPS()
{
    //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);

    //this line turns on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

    // the nice thing about this code is you can have a timer0 interrupt go off
    // every 1 millisecond, and read data from the GPS for you. that makes the
    // loop code a heck of a lot easier!
    useInterrupt(true);

    delay(1000);
}

void parseGPS()
{
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived())
    {

        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return;                     // we can fail to parse a sentence in which case we should just wait for another
    }
}

void datalogGPS()
{
    datalog.print(GPS.hour, DEC);
    datalog.print(':');
    datalog.print(GPS.minute, DEC);
    datalog.print(':');
    datalog.print(GPS.seconds, DEC);
    datalog.print(",   ");
    datalog.print(GPS.latitude, 4);
    datalog.print(",  ");
    datalog.print(GPS.longitude, 4);
    datalog.print(", ");
    datalog.print(GPS.altitude);
    datalog.print(",");
    datalog.print((int)GPS.satellites);
}

void serialGPS()
{
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    Serial.print(GPS.seconds, DEC);
    Serial.print(",   ");
    Serial.print(GPS.latitude, 4);
    Serial.print(",   ");
    Serial.print(GPS.longitude, 4);
    Serial.print(",   ");
    Serial.print(GPS.altitude);
    Serial.print(",      ");
    Serial.print((int)GPS.satellites);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect)
{
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
#ifdef UDR0
    if (GPSECHO)
        if (c)
            UDR0 = c;
            // writing direct to UDR0 is much much faster than Serial.print
            // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v)
{
    if (v)
    {
        // Timer0 is already used for millis() - we'll just interrupt somewhere
        // in the middle and call the "Compare A" function above
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
        usingInterrupt = true;
    }
    else
    {
        // do not call the interrupt function COMPA anymore
        TIMSK0 &= ~_BV(OCIE0A);
        usingInterrupt = false;
    }
}