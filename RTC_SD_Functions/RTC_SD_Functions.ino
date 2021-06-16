#include <RTClib.h>
#include <Wire.h>
#include <SD.h> // This is a special library for use with older SD board and mega, for breakout SDs
#include <SPI.h>

//For the older Adafruit dataloggers, use DS1307 RTC
RTC_PCF8523 rtc; //RTC module

File datalog;
char filename[] = "LOGGER00.csv";

const int chipSelect = 10; // specific to Adafruit datashield

void setup()
{
    Serial.begin(115200);

    setupRTC();

    // define Header for csv and serial here
    setupSD("Date, Time, GPS Time,  Lat,  Lon,  GPS Altitude, # Satelites, Temp(C)");
}

void loop()
{
    DateTime now = rtc.now();

    if (timer > millis())
        timer = millis();

    // approximately every 2 seconds or so, print out the current stats - GPS
    if (millis() - timer > 2000)
    {
        timer = millis(); // reset the timer

        datalog = SD.open(filename, FILE_WRITE); //starts writing on the SD Card

        datalogRTC(now);

        datalog.println();
        datalog.close();
        delay(100);

        serialRTC(now);
        Serial.println();

        delay(1000);
    }
}

void setupRTC()
{
    Wire.begin();
    rtc.begin();
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        while (1);
    }
}

void setRTCAuto()
{
    if (!rtc.begin())
    {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void setRTCManual(DateTime date)
{
    if (!rtc.begin())
    {
        rtc.adjust(date);
    }
}

void setupSD(String Header)
{
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

    datalog = SD.open(filename, FILE_WRITE);
    datalog.println(Header);
    Serial.println(Header);

    datalog.close();
}

void datalogRTC(DateTime now)
{
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
}

void serialRTC(DateTime now)
{
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
}
