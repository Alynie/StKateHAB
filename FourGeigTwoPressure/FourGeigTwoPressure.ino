/*Code for geiger bunkbed with 4 RM-80s and pressure sensors
 * Written by Claire and Judy on 01.15.2019
 */

//begin SD initialzie variables/library
#include <SD.h> 


//begin RTC libraries and variables
#include <Wire.h> //This is for the RTC
#include <RTClib.h> //RTC library can be downloaded from Adafruit.com
//For the older Adafruit dataloggers, use DS1307 RTC

RTC_PCF8523 rtc; //RTC module

//resume SD stuff - set up to create files
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
long int counter2 = 0; // Local Counter for Geiger counter sensor 2 (on top of 1) hits
long int counter3 = 0; // Local Counter for Geiger counter sensor 3 (on side of 1) hits
long int counter4 = 0; // Local Counter for Geiger counter sensor 4 
long int coincidencecount12 = 0; // Counter for coincident hits between sensors 1 and 2
long int coincidencecount13 = 0; // Counter for coincident hits between sensors 1 and 3
long int coincidencecount14 = 0;
long int coincidencecount23 = 0; // Counter for coincident hits between sensors 2 and 3
long int coincidencecount24 = 0;
long int coincidencecount34 = 0;
long int coincidencecount123 = 0; // Counter for coincident htis between sensors 1,2, and 3. These hits do not add to other coincidence counters
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
//end Geiger initalize variables 


//Pressure
float pressureSensor;
float pressureSensorV;
float pressureSensor2;
float pressureSensor2V;
float psi1;
float psi2;

void setup() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);


// connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200); 
  
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (!rtc.begin()) {
    Serial.println("RTC clock is being reset. Please recomment and rerun!");
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //UNCOMMENT THIS LINE TO SET TIME TO MATCH COMPUTER
    //This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

//Initialize SD Card
  Serial.print("Initializing SD Card...");

  pinMode(SS, OUTPUT);

if(!SD.begin(10)) // for Mega and Adafruit datashield
  {
    Serial.println("Card failed, or not present");
    //digitalWrite(led, HIGH);
    //delay(1000);
    return;
  }
  Serial.println("Card initialized.");  
  Serial.print("Creating File...");

  // Make a new file each time the arduino is powered
  for (uint8_t i = 0; i < 100; i++) 
  {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) 
    {
      // only open a new file if it doesn't exist
      datalog = SD.open(filename, FILE_WRITE); 
      break;
    }  
}

Serial.print("Logging to: ");
  Serial.println(filename);

  if(!datalog)
  {  
    Serial.println("Couldn't Create File"); 
    delay(1000);
    Serial.println("Couldn't Create File");
    return;
  }

 // Print Header
  String Header =  "Date, Time, S1 Hit Count, S2 Hit Count, S3 Hit Count, S4 Hit Count, Coin 1&2, Coin 1&3, Coin 1&4, Coin 2&3, Coin 2&4, Coin 3&4, Coin 1&2&3, Coin 1&2&4, Coin 1&3&4, Coin 2&3&4, Coin 1&2&3&4, P(1), P(2)";

  datalog = SD.open(filename, FILE_WRITE);
  datalog.println(Header);
  Serial.println(Header); 
 
  datalog.close(); 
}


void loop() {
  
  //RTC stuff
  DateTime now = rtc.now();
  
  // Begin Geiger loop
  timer = millis();

while((millis()-timer) < Logtime)
  {
    LocalTime = micros();
    int sensor1 = digitalRead(4); // Read in the pin for sensor 1. Duplicate for multiple geiger counters
    int sensor2 = digitalRead(5); // Read in the pin for sensor 2. Duplicate for multiple geiger counters
    int sensor3 = digitalRead(6); // Read in the pin for sensor 3. Duplicate for multiple geiger counters
    int sensor4 = digitalRead(7);
        
    if(sensor1==LOW)
    {
      counter1++;
      hit1 = true;
       
    }
    if(sensor2==LOW)
    {
      counter2++;
      hit2 = true;
       
    }
    if(sensor3==LOW)
    {
      counter3++;
      hit3 = true;
       
    }
    if(sensor4==LOW)
    {
      counter4++;
      hit4 = true;
    }
  
    if(hit1==1 && hit2==1 && hit3==1 && hit4==1) // These if statements increment the coincidence counters
    {
      coincidencecount1234++;
    }
   if(hit1==1 && hit2==1)
    {
      coincidencecount12++;
    }
    
    if(hit1==1 && hit3==1)
    {
      coincidencecount13++;
    }
     if(hit1==1 && hit4==1)
    {
      coincidencecount14++;
    }
    
    if(hit2==1 && hit3==1)
    {
      coincidencecount23++;
    }
    if (hit2==1 && hit4==1)
    {
      coincidencecount24++;
    }
    if (hit3==1 && hit4==1)
    {
      coincidencecount34++;
    }
    if (hit1==1 && hit2==1 && hit3==1)
    {
      coincidencecount123++;
    }
    if (hit1==1 && hit2==1 && hit4==1) 
    {
      coincidencecount124++;
    }
    if (hit1==1 && hit3==1 && hit4==1)
    {
      coincidencecount134++;
    }
    if (hit2==1 && hit3==1 && hit4==1)
    {
      coincidencecount234++;
    }
  
  
      hit1=false;
      hit2=false;
      hit3=false;
      hit4=false;
  }
      
   while((micros()-LocalTime)<LoopLog)
    {
      delayMicroseconds(5);  //slow code down if needed, to let Geiger counters reset.  This will need to be changed once logging has been added
    }    

    //Pressure
    // Pressure Sensor Honeywell SSCSANN015PAAA5
    pressureSensor = analogRead(A0); // Read the analog pin
    pressureSensorV = pressureSensor * (5.0 / 1024); // Convert the digital number to voltage
    psi1 = (pressureSensorV - (0.1 * 5.0)) / (4.0 / 15.0); // Convert the voltage to proper units

    // Pressure Sensor Honeywell SSCMRNT030PAAA3
    pressureSensor2 = analogRead(A1); // Read the analog pin
    pressureSensor2V = pressureSensor2 * (5.0 / 1024); // Convert the digital number to voltage
    psi2 = (pressureSensor2V - (0.1 * 5.0)) / (4.0 / 15.0); // Convert the voltage to proper units

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
  datalog.print(", ");
  datalog.print(  psi1, 2);
  datalog.print(",");
  datalog.print(  psi2, 2);
  datalog.print(",  ");
  
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
  Serial.print(", ");
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
  Serial.print(", ");
  Serial.print(psi1, 2);
  Serial.print(",  ");
  Serial.print(psi2, 2);
  Serial.print(",  ");
    
  Serial.println();

  // Reset short-term counters but not cumulative counters 
  counter1=0;
  counter2=0;
  counter3=0;
  counter4=0;
  
  delay(300);
  }// end loop()
