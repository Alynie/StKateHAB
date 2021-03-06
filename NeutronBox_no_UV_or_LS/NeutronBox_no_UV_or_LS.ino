
/*
  //If you're using an Adafruit Ultimate GPS Breakout:
  //Connect the GPS Power pin to 5V
  //Connect the GPS Ground pin to ground
  //If using hardware serial - required for Arduino Mega:
  //Connect the GPS TX (transmit) pin to Arduino RX1 (Serial1)
  //Connect the GPS RX (receive) pin to matching TX1 (Serial1)
  //Pin 8 is for Geiger (was on Pin 7)
*/
/*last edited by Melissa on 6/11/19 removed UV and LS*/

//Libraries
#include <RTClib.h>
#include <Wire.h> //This is for the RTC
#include <SD.h>   // SD library
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <OneWire.h> //this is for temp sensor

//RTC
//For the older Adafruit dataloggers, use DS1307 RTC
//begin RTC variables
RTC_PCF8523 rtc; //RTC module

//SD
File datalog;
char filename[] = "LOGGER00.csv";

const int chipSelect = 10; // specific to Adafruit datashield
//end SD initialize variables

//begin Geiger initialize variables
long unsigned int timer = 0;
long unsigned int Logtime = 5000; // Logging time in milliseconds
long unsigned int LocalTime = 0;
long unsigned int LoopLog = 150;
long int counter1 = 0; // Local Counter for Geiger counter sensor 1 hits
boolean hit1 = false;  // Tells if sensor 1 was low during the current while loop. 1 if true, 0 if false
//end Geiger initalize variables

//GPS
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

  //GPS
  //Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Wire.begin();

  //RTC
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (!rtc.initialized())
  {
    Serial.println("Resetting the Clock!");
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // UNCOMMENT THIS LINE TO SET TIME TO MATCH COMPUTER
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    rtc.adjust(DateTime(2021, 6, 14, 5, 36, 0));
    //while (1);
  }

  //GPS

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

  //SD Card
  Serial.print("Initializing SD Card...");
  pinMode(10, OUTPUT);

  if (!SD.begin(10)) // for Mega and Adafruit datashield
  {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("Card initialized.");
  Serial.print("Creating File...");

  // Make a new file each time the arduino is powered
  for (uint8_t i = 0; i < 100; i++)
  {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (!SD.exists(filename))
    {
      // only open a new file if it doesn't exist
      datalog = SD.open(filename, FILE_WRITE);
      break;
    }
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  if (!datalog)
  {
    Serial.println("Couldn't Create File");
    delay(1000);
    return;
  }

  // Print Header
  String Header = "Date, Time, GPS Time,  Lat,  Lon,  GPS Altitude, # Satelites, Counter";

  datalog = SD.open(filename, FILE_WRITE);
  datalog.println(Header);
  Serial.println(Header);

  datalog.close();
}

//outside of void setup?
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

void loop()
{
  //RTC
  DateTime now = rtc.now();

  //GPS
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {

    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                       // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it - GPS
  if (timer > millis())
    timer = millis();

  // approximately every 2 seconds or so, print out the current stats - GPS
  if (millis() - timer > 2000)
  {
    timer = millis(); // reset the timer

    // Begin Geiger loop
    timer = millis();

    while ((millis() - timer) < Logtime)
    {
      LocalTime = micros();
      int sensor1 = digitalRead(8); // Read in the pin for sensor 1. Duplicate for multiple geiger counters

      if (sensor1 == LOW)
      {
        counter1++;
        hit1 = true;
      }

      hit1 = false;

      while ((micros() - LocalTime) < LoopLog)
      {
        delayMicroseconds(5); //slow code down if needed, to let Geiger counters reset.  This will need to be changed once logging has been added
      }
    }

    //SD Card
    datalog = SD.open(filename, FILE_WRITE); //starts writing on the SD Card
    datalog.print(now.month(), DEC);
    datalog.print('/');
    datalog.print(now.day(), DEC);
    datalog.print('/');
    datalog.print(now.year(), DEC);
    datalog.print(", ");
    datalog.print(now.hour(), DEC);
    datalog.print(':');
    datalog.print(now.minute(), DEC);
    datalog.print(':');
    datalog.print(now.second(), DEC);
    datalog.print(", ");
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
    datalog.print(",");
    datalog.print(counter1);
    datalog.println();
    datalog.close();
    delay(100);

    //This is stuff printed on the Serial Monitor
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.print(", ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(",   ");
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
    Serial.print(",   ");
    Serial.print(counter1);
    Serial.println();

    delay(1000);
  }
}
