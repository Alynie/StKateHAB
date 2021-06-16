#include <SD.h>

float psi1;
float psi2;
int pressurePin1 = A0;
int pressurePin2 = A1;

void setup()
{
    setupPressure(pressurePin1);
    setupPressure(pressurePin2);
    Serial.begin(115200);
}

void loop()
{
    psi1 = readPSI(pressurePin1);
    psi2 = readPSI(pressurePin2);

    datalog = SD.open(filename, FILE_WRITE); //starts writing on the SD Card

    datalogPressure(psi1);
    datalog.print(", ");
    datalogPressure(psi2);

    datalog.println();

    datalog.close();
    delay(100);

    serialPressure(psi1);
    Serial.print(",  ");
    serialPressure(psi2);

    Serial.println();
}

void setupPressure(int pin)
{
    pinMode(pin, INPUT);
}

float readPSI(int pin)
{
    float pressureSensor = analogRead(pin);                // Read the analog pin
    float pressureSensorV = pressureSensor * (5.0 / 1024); // Convert the digital number to voltage
    return (pressureSensorV - (0.1 * 5.0)) / (4.0 / 15.0); // Convert the voltage to proper units
}

void datalogPressure(float psiValue)
{
    datalog.print(psiValue, 2);
}

void serialPressure(float psiValue)
{
    Serial.print(psiValue, 2);
}