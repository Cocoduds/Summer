// Libraries...
#include <SPI.h>
#include <Adafruit_Sensor.h>  
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
#include <Wire.h>
// Core graphics library
// Time libraries
#include <Time.h>
#include <Adafruit_I2CDevice.h>
// #include <Arduino.h>
#include <SdFat.h>                            // SD card & FAT filesystem library
// #include "FsDateTime.h"
#include <Adafruit_AS7341.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>


//---------- SPECTROMETER ----------
Adafruit_AS7341 as7341;
#define AS7341_ASTEP 299
#define AS7341_ATIME 14
#define AS7341_GAIN AS7341_GAIN_512X


//---------- PULSE OX ----------
int resPin = 9;
int mfioPin = 7;
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin); 
bioData body; 



//---------- SD CARD ----------
#define SD_CS 10

// define a file object
SdFat                SD;         // SD card filesystem
File myFile;
char filename[14];

// Create a CSV file name in 8.3 format. I will encode a file sequence
// number in the last three characters. For example: pulse000.csv
char csv_filename[13] = "pulsexxx.csv";
uint32_t index_next_filename;


int timer;
#define size 100
uint16_t readings[12];



void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  analogReadResolution(12);    // set ADC precision to 12 bits.

  //---------- SPECTROMETER ----------
  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }
  as7341.setATIME(AS7341_ATIME);
  as7341.setASTEP(AS7341_ASTEP); //should be 73ms per read
  as7341.setGain(AS7341_GAIN );



  //---------- PULSE OX ----------
  Wire.begin();
  int result = bioHub.begin();
  if (result == 0) //Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!");
 
  Serial.println("Configuring Sensor...."); 
  int error = bioHub.configBpm(MODE_TWO); 
  if(error == 0){ 
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }
  // took out buffer

  delay(250);
  //---------- SD ----------
  if(!SD.begin(SD_CS, SD_SCK_MHZ(10))) { // Breakouts require 10 MHz limit due to longer wires
    Serial.println(F("SD begin() failed"));
    for(;;); // Fatal error, do not continue
  }

  myFile.open("pulsebig.csv", FILE_WRITE);

  if(myFile){
    myFile.println("time,415,445,480,515,555,590,630,680,Clear,NIR,Heartrate,Oxygen,R,Confidence");
  myFile.close();
  }
  timer = micros();
}


void loop() {
  // save the data THANKS GEORGE FOR FILENAMING
  // find an unused file name
  // I'll use this flag in the loop searching for the first unused filename.
  bool have_a_name = false;

  // start at 0, and loop up to 1000.
  long index_filename;
  long index_start = 0;

  for (index_filename = index_start; index_filename <= 1000; index_filename++) 
  {
    if (have_a_name) break;

    index_next_filename = index_filename;

    // Now create a filename for this value of index_next_filename,
    // which is a global variable. name is in csv_filename.

    // get the least significant hexadecimal digit:
    long test1 = index_next_filename;
    int digit7 = test1 % 10;

    // keep going...
    test1 = (test1 - digit7) / 10;
    int digit6 = test1 % 10;

    test1 = (test1 - digit6) / 10;
    int digit5 = test1 % 10;

    // now convert each digit to ASCII characters. Recall that
    // the ASCII character '0' is 48, '1' is 49,... '9' is 57.

    csv_filename[7] = digit7 + 48;
    csv_filename[6] = digit6 + 48;
    csv_filename[5] = digit5 + 48;

    //////////////////////////////////////////////////////
    // now that we've constructed a filename, see if it already exists.

    if (!SD.exists(csv_filename)) {
      have_a_name = true;
      // Serial.println(F(">>>     file doesn't exist, so that is for us!")); 
    } else {
      // Serial.println(F(">>>     file exists, so keep going.")); 
    }
  }
  Serial.println(csv_filename);
  myFile.open("pulsebig.csv", FILE_WRITE);

  if(myFile){
    // myFile.println("time,415,445,480,515,555,590,630,680,Clear,NIR,Heartrate,Oxygen,R,Confidence");
    as7341.setLEDCurrent(8); // mA
    as7341.enableLED(true);
    // timer = micros();
    for (int i = 0; i<size; i++){
      if (!as7341.readAllChannels(readings)){
        Serial.println("Error reading all channels!");
        return;
      }
      body = bioHub.readBpm();
      myFile.print(micros()-timer);
      myFile.print(",");
      myFile.print(readings[0]);
      myFile.print(",");
      myFile.print(readings[1]);
      myFile.print(",");
      myFile.print(readings[2]);
      myFile.print(",");
      myFile.print(readings[3]);
      myFile.print(",");
      myFile.print(readings[6]);
      myFile.print(",");
      myFile.print(readings[7]);
      myFile.print(",");
      myFile.print(readings[8]);
      myFile.print(",");
      myFile.print(readings[9]);
      myFile.print(",");
      myFile.print(readings[10]);
      myFile.print(",");
      myFile.print(readings[11]);
      myFile.print(",");
      myFile.print(body.heartRate);
      myFile.print(",");
      myFile.print(body.oxygen);
      myFile.print(",");
      myFile.print(body.rValue);
      myFile.print(",");
      myFile.println(body.confidence);
    }
    myFile.println(micros()-timer);
    Serial.print("time = ");
    Serial.println(micros()-timer);
    myFile.close();
    as7341.enableLED(false);
  }
  else{
    Serial.println("Error saving data");
  }
  

  Serial.print("ADC0/F1 415nm : ");
  Serial.println(readings[0]);
  Serial.print("ADC1/F2 445nm : ");
  Serial.println(readings[1]);
  Serial.print("ADC2/F3 480nm : ");
  Serial.println(readings[2]);
  Serial.print("ADC3/F4 515nm : ");
  Serial.println(readings[3]);
  Serial.print("ADC0/F5 555nm : ");

  /* 
  // we skip the first set of duplicate clear/NIR readings
  Serial.print("ADC4/Clear-");
  Serial.println(readings[4]);
  Serial.print("ADC5/NIR-");
  Serial.println(readings[5]);
  */
  
  Serial.println(readings[6]);
  Serial.print("ADC1/F6 590nm : ");
  Serial.println(readings[7]);
  Serial.print("ADC2/F7 630nm : ");
  Serial.println(readings[8]);
  Serial.print("ADC3/F8 680nm : ");
  Serial.println(readings[9]);
  Serial.print("ADC4/Clear    : ");
  Serial.println(readings[10]);
  Serial.print("ADC5/NIR      : ");
  Serial.println(readings[11]);

  Serial.print("Heartrate: ");
  Serial.println(body.heartRate); 
  Serial.print("Confidence: ");
  Serial.println(body.confidence); 
  Serial.print("Oxygen: ");
  Serial.println(body.oxygen); 
  Serial.print("Status: ");
  Serial.println(body.status); 
  Serial.print("Extended Status: ");
  Serial.println(body.extStatus); 
  Serial.print("Blood Oxygen R value: ");
  Serial.println(body.rValue); 


  Serial.println();
  delay(1000);
}