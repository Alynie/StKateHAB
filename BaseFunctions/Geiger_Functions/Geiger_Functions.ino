#include <Wire.h> //This is for the RTC

//begin Geiger initialize variables
int geigerPins[4] = [ 4, 5, 6, 7 ];
long unsigned int timer = 0;
long unsigned int Logtime = 5000; // Logging time in milliseconds
long unsigned int LocalTime = 0;
long unsigned int LoopLog = 150;
long int counter1 = 0;           // Local Counter for Geiger counter sensor 1 hits
long int counter2 = 0;           // Local Counter for Geiger counter sensor 2 (on top of 1) hits
long int counter3 = 0;           // Local Counter for Geiger counter sensor 3 (on side of 1) hits
long int counter4 = 0;           // Local Counter for Geiger counter sensor 4
long int coincidencecount12 = 0; // Counter for coincident hits between sensors 1 and 2
long int coincidencecount13 = 0; // Counter for coincident hits between sensors 1 and 3
long int coincidencecount14 = 0;
long int coincidencecount23 = 0; // Counter for coincident hits between sensors 2 and 3
long int coincidencecount24 = 0;
long int coincidencecount34 = 0;
long int coincidencecount123 = 0; // Counter for coincident hits between sensors 1,2, and 3. These hits do not add to other coincidence counters
long int coincidencecount124 = 0;
long int coincidencecount134 = 0;
long int coincidencecount234 = 0;
long int coincidencecount1234 = 0;
boolean hit1 = false; // Tells if sensor 1 was low during the current while loop. 1 if true, 0 if false
boolean hit2 = false; // Tells if sensor 2 was law during the current while loop. 1 if true, 0 if false
boolean hit3 = false; // Tells if sensor 3 was low during the current while loop. 1 if true, 0 if false
boolean hit4 = false;
int sensor1;
int sensor2;
int sensor3;
int sensor4;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    readGeigers();

    datalog = SD.open(filename, FILE_WRITE); //starts writing on the SD Card

    datalogGeigers();

    datalog.println();
    datalog.close();
    delay(100);

    //This is stuff printed on the Serial Monitor
    serialGeigers();

    Serial.println();

    resetCounters();

    delay(300);
}

void readGeigers()
{
    timer = millis();
    while ((millis() - timer) < Logtime)
    {
        LocalTime = micros();
        int sensor1 = digitalRead(geigerPins[0]); // Read in the pin for sensor 1. Duplicate for multiple geiger counters
        int sensor2 = digitalRead(geigerPins[1]); // Read in the pin for sensor 2. Duplicate for multiple geiger counters
        int sensor3 = digitalRead(geigerPins[2]); // Read in the pin for sensor 3. Duplicate for multiple geiger counters
        int sensor4 = digitalRead(geigerPins[3]);

        if (sensor1 == LOW)
        {
            counter1++;
            hit1 = true;
        }
        if (sensor2 == LOW)
        {
            counter2++;
            hit2 = true;
        }
        if (sensor3 == LOW)
        {
            counter3++;
            hit3 = true;
        }
        if (sensor4 == LOW)
        {
            counter4++;
            hit4 = true;
        }

        if (hit1 == 1 && hit2 == 1 && hit3 == 1 && hit4 == 1) // These if statements increment the coincidence counters
        {
            coincidencecount1234++;
        }
        if (hit1 == 1 && hit2 == 1)
        {
            coincidencecount12++;
        }

        if (hit1 == 1 && hit3 == 1)
        {
            coincidencecount13++;
        }
        if (hit1 == 1 && hit4 == 1)
        {
            coincidencecount14++;
        }

        if (hit2 == 1 && hit3 == 1)
        {
            coincidencecount23++;
        }
        if (hit2 == 1 && hit4 == 1)
        {
            coincidencecount24++;
        }
        if (hit3 == 1 && hit4 == 1)
        {
            coincidencecount34++;
        }
        if (hit1 == 1 && hit2 == 1 && hit3 == 1)
        {
            coincidencecount123++;
        }
        if (hit1 == 1 && hit2 == 1 && hit4 == 1)
        {
            coincidencecount124++;
        }
        if (hit1 == 1 && hit3 == 1 && hit4 == 1)
        {
            coincidencecount134++;
        }
        if (hit2 == 1 && hit3 == 1 && hit4 == 1)
        {
            coincidencecount234++;
        }

        hit1 = false;
        hit2 = false;
        hit3 = false;
        hit4 = false;
    }

    while ((micros() - LocalTime) < LoopLog)
    {
        delayMicroseconds(5); //slow code down if needed, to let Geiger counters reset.  This will need to be changed once logging has been added
    }
}

void datalogGeigers()
{
    datalog.print(counter1);
    datalog.print(", ");
    datalog.print(counter2);
    datalog.print(", ");
    datalog.print(counter3);
    datalog.print(", ");
    datalog.print(counter4);
    datalog.print(", ");
    datalog.print(coincidencecount12);
    datalog.print(", ");
    datalog.print(coincidencecount13);
    datalog.print(", ");
    datalog.print(coincidencecount14);
    datalog.print(", ");
    datalog.print(coincidencecount23);
    datalog.print(", ");
    datalog.print(coincidencecount24);
    datalog.print(", ");
    datalog.print(coincidencecount34);
    datalog.print(", ");
    datalog.print(coincidencecount123);
    datalog.print(", ");
    datalog.print(coincidencecount124);
    datalog.print(", ");
    datalog.print(coincidencecount134);
    datalog.print(", ");
    datalog.print(coincidencecount234);
    datalog.print(", ");
    datalog.print(coincidencecount1234);
}

void serialGeigers()
{
    Serial.print(counter1);
    Serial.print(", ");
    Serial.print(counter2);
    Serial.print(", ");
    Serial.print(counter3);
    Serial.print(", ");
    Serial.print(counter4);
    Serial.print(", ");
    Serial.print(coincidencecount12);
    Serial.print(", ");
    Serial.print(coincidencecount13);
    Serial.print(", ");
    Serial.print(coincidencecount14);
    Serial.print(", ");
    Serial.print(coincidencecount23);
    Serial.print(", ");
    Serial.print(coincidencecount24);
    Serial.print(", ");
    Serial.print(coincidencecount34);
    Serial.print(", ");
    Serial.print(coincidencecount123);
    Serial.print(", ");
    Serial.print(coincidencecount124);
    Serial.print(", ");
    Serial.print(coincidencecount134);
    Serial.print(", ");
    Serial.print(coincidencecount234);
    Serial.print(", ");
    Serial.print(coincidencecount1234);
}

void resetCounters()
{
    counter1 = 0;
    counter2 = 0;
    counter3 = 0;
    counter4 = 0;
}