#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 8
float tempC;
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress temp;

int tempcontrolpin = 24; //the pin that connected to the transistor base
float hightempset = 13;  //heater off temperature
float lowtempset = 10;   //heater on temperature

void setup()
{
    Serial.begin(115200);

    setupLS();
}

void loop()
{
    checkLS();

    if (timer > millis())
        timer = millis();

    // approximately every 2 seconds or so, print out the current stats - GPS
    if (millis() - timer > 2000)
    {
        timer = millis(); // reset the timer

        //SD Card
        datalog = SD.open(filename, FILE_WRITE); //starts writing on the SD Card

        datalogLS();

        datalog.println();
        datalog.close();
        delay(100);

        //This is stuff printed on the Serial Monitor
        serialLS();
        Serial.println();

        delay(1000);
    }
}

void setupLS()
{
    pinMode(tempcontrolpin, OUTPUT);
    Serial.print("Locating temp sensor...");
    sensors.begin();
    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" device.");

    oneWire.reset_search();
    if (!oneWire.search(temp))
        Serial.println("Unable to find address for Device");

    // set the resolution to 9 or 12 bit (Each Dallas/Maxim device is capable of several different resolutions)
    sensors.setResolution(temp, 12);

    Serial.print("Temp Sensor Address: ");
    printAddress(temp);
    Serial.println();

    Serial.print("Temp Sensor Resolution: ");
    Serial.print(sensors.getResolution(temp), DEC);
    Serial.println();
}

void printAddress(DeviceAddress deviceAddress)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (deviceAddress[i] < 16)
            Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
    }
}

void checkLS()
{
    sensors.requestTemperatures(); // Send the command to get temperatures
    tempC = sensors.getTempC(temp);

    if (tempC <= lowtempset)
    {
        digitalWrite(tempcontrolpin, HIGH); // turn the heater on (HIGH is the voltage level)
    }

    else if (tempC >= hightempset)
    {
        digitalWrite(tempcontrolpin, LOW); // turn the heater off (LOW is the voltage level)
    }

    else if (lowtempset < tempC && tempC < hightempset && (digitalRead(tempcontrolpin) == LOW))
    {
        digitalWrite(tempcontrolpin, LOW); // keep the heater off if temp is between thresholds and already off
    }

    else if (lowtempset < tempC && tempC < hightempset && (digitalRead(tempcontrolpin) == HIGH))
    {
        digitalWrite(tempcontrolpin, HIGH); // keep the heater on if temp is between thresholds and already on
    }
    else
    {
        // in the event that none of these conditions are met, turn off heater until condition is met
        digitalWrite(tempcontrolpin, LOW);
    }
    delay(250);
}

void datalogLS()
{
    datalog.print(tempC);
}

void serialLS()
{
    Serial.print(tempC);
}