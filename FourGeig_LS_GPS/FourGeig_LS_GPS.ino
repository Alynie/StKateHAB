//Libraries
#include <RTClib.h>
#include <Wire.h> //This is for the RTC
#include <SD.h> // This is a special library for use with older SD board and mega, for breakout SDs
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//LS
#define ONE_WIRE_BUS 8
float tempC;
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress temp;

int tempcontrolpin = 24; //the pin that connected to the transistor base
float hightempset = 13; //heater off temperature
float lowtempset = 10; //heater on temperature


//RTC
//For the older Adafruit dataloggers, use DS1307 RTC
//begin RTC variables
RTC_PCF8523 rtc; //RTC module

//SD
File datalog;
char filename[] = "LOGGER00.csv";

const int chipSelect = 10; // specific to Adafruit datashield
//end SD initialize variables

//GPS
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);
long unsigned int timer = 0;
long unsigned int Logtime = 5000; //logging time in milliseconds
long unsigned int LocalTime = 0;
long unsigned int LoopLog = 150;
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup(void) {
  pinMode(tempcontrolpin, OUTPUT);  //heater
  
  //GPS
  //Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Wire.begin();
  rtc.begin();

  //RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (!rtc.begin()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
   //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //UNCOMMENT THIS LINE TO SET TIME TO MATCH COMPUTER
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //while (1);
  }

  //GPS

  //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  //this line turns on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  //SD Card
  Serial.print("Initializing SD Card...");

  pinMode(SS, OUTPUT);

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
    if (! SD.exists(filename))
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
  String Header =  "Date, Time, GPS Time,  Lat,  Lon,  GPS Altitude, # Satelites, Temp(C)";

  datalog = SD.open(filename, FILE_WRITE);
  datalog.println(Header);
  Serial.println(Header);

  datalog.close();

  //LS
  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  oneWire.reset_search();
  if (!oneWire.search(temp)) Serial.println("Unable to find address for Device"); 

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(temp, 12);

 // if (!sensors.getAddress(temp, 0)) Serial.println("Unable to find address for Device 0"); 
  Serial.print("Device 1 Address: ");
  printAddress(temp);
  Serial.println();
 
  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(temp), DEC); 
  Serial.println();
}

//outside of void setup
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  //Serial.print("Temp C: ");
  //Serial.print(tempC);
  //Serial.print(" Temp F: ");
  //Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}


void loop() {
// call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
  tempC=sensors.getTempC(temp);
  
  // It responds almost immediately. Let's print out the data
  //printTemperature(temp); // Use a simple function to print out the data

  //RTC
  DateTime now = rtc.now();
  //LS
  delay(100);

  //Check if life support is needed
  if (tempC <= lowtempset) {
    digitalWrite(tempcontrolpin, HIGH);   // turn the heater on (HIGH is the voltage level)
    delay(250);
  }

  else if (tempC >= hightempset) {
    digitalWrite(tempcontrolpin, LOW);   // turn the heater off (LOW is the voltage level)
    delay(250);
  }

  else if (lowtempset < tempC && tempC < hightempset && (digitalRead(tempcontrolpin) == LOW)) {
    digitalWrite(tempcontrolpin, LOW);
    delay(250);
  }

  else if (lowtempset < tempC && tempC < hightempset && (digitalRead(tempcontrolpin) == HIGH)) {
    digitalWrite(tempcontrolpin, HIGH);
    delay(250);
  }

  else {
  }
  //GPS
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it - GPS
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats - GPS
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer


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
    datalog.print(  GPS.hour, DEC);
    datalog.print(':');
    datalog.print(  GPS.minute, DEC);
    datalog.print(':');
    datalog.print(  GPS.seconds, DEC);
    datalog.print(",   ");
    datalog.print(  GPS.latitude, 4);
    datalog.print(",  ");
    datalog.print(  GPS.longitude, 4);
    datalog.print(", ");
    datalog.print(  GPS.altitude);
    datalog.print(",");
    datalog.print((int)  GPS.satellites);
    datalog.print(",");
    datalog.print(tempC);
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
    Serial.print(tempC);
    Serial.println();

    delay(1000);

}
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  }
