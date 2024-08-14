// Libraries...
#include <SPI.h>
#include <Adafruit_Sensor.h>  
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
#include <Wire.h>
// Core graphics library
#include <Adafruit_GFX.h>    
#include <Adafruit_ST7789.h>                  // Hardware-specific library for TFT display
#include <Adafruit_ImageReader.h>             // Image-reading functions
// Time libraries
#include <Time.h>
#include <Adafruit_I2CDevice.h>
// #include <Arduino.h>
#include <SdFat.h>                            // SD card & FAT filesystem library
// #include "FsDateTime.h"
#include <Adafruit_AS7341.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>


#define debug_print false;

//---------- SparkFun Pulse Oximeter, etc. ----------
// Reset pin, MFIO pin
int MAX30101resPin = 9;
int MAX30101mfioPin = 7;

// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(MAX30101resPin, MAX30101mfioPin); 

bioData body;  

// This holds specific information on your heartrate and blood oxygen levels. 
// body.heartrate  - Heartrate
// body.confidence - Confidence in the heartrate value
// body.oxygen     - Blood oxygen level
// body.status     - Has a finger been sensed?


//---------- on-board BluetoothLE ----------
// I will try not to use this for now
// CS: I don't know what pin, actually.
#define BLU_CS 8


//---------- TFT Display ----------
// TFT select pin
#define SPI_TFT_CS  12 
// TFT display/command pin
#define SPI_D_slash_C  11 
// RESET pin (reset on LOW)
#define TFT_RESET 13 
// backlight pin, can be PWM.
#define BACKLIGHT 5
// PWM value (0 is off, 255 is full on) for backlight pin
#define TFT_BACKLIGHT_PWM 128
#define TFT_BACKLIGHT_PWM_MAX 255

// width and height
#define TFTWIDTH  240
#define TFTHEIGHT 135

// define TFT colors that are roughly the same as these bands
// see https://www.en.silicann.com/blog/post/wavelength-color/
/*
#define AS7341_F1_TFT_COLOR 0X6100E9
#define AS7341_F2_TFT_COLOR 0X001AFF
#define AS7341_F3_TFT_COLOR 0X00CCFF
// #define AS7341_F4_TFT_COLOR 0X12FF00
// #define AS7341_F5_TFT_COLOR 0XA4FF00
#define AS7341_F4_TFT_COLOR 0X098000
// #define AS7341_F5_TFT_COLOR 0X528000
#define AS7341_F5_TFT_COLOR 0X203000
#define AS7341_F6_TFT_COLOR 0XFFD800
#define AS7341_F7_TFT_COLOR 0XFF3B00
#define AS7341_F8_TFT_COLOR 0XFF0000
#define AS7341_NIR_TFT_COLOR 0XAE0E0E
*/

#define AS7341_F1_TFT_COLOR ST77XX_WHITE
#define AS7341_F2_TFT_COLOR ST77XX_YELLOW
#define AS7341_F3_TFT_COLOR ST77XX_ORANGE
#define AS7341_F4_TFT_COLOR ST77XX_RED
#define AS7341_F5_TFT_COLOR ST77XX_GREEN
#define AS7341_F6_TFT_COLOR ST77XX_CYAN
#define AS7341_F7_TFT_COLOR ST77XX_YELLOW
#define AS7341_F8_TFT_COLOR ST77XX_ORANGE
#define AS7341_NIR_TFT_COLOR ST77XX_RED

// circular buffers to hold the readings
#define max_buffer_readings 100  // KEEP THIS FIXED AT 100!
#define max_buffer_channels 12
#define index_NIR 11
#define index_680 9

float AS7341_NIR_680_ratio[max_buffer_readings];
float tft_ratio_array[max_buffer_readings];
int last_tft_ratio_array_entry = 0; 

uint16_t current_y_pixel = 0;
uint16_t current_x_pixel = 0;

uint32_t tft_loads = 0;

Adafruit_ST7789 tft = Adafruit_ST7789(SPI_TFT_CS, SPI_D_slash_C, TFT_RESET);


//---------- SD CARD ----------
#define SPI_SD_CS 10

// define a file object
SdFat                SD;         // SD card filesystem
Adafruit_ImageReader reader(SD); // Image-reader object, pass in SD filesys
File myFile;
char filename[14];

// Create a CSV file name in 8.3 format. I will encode a file sequence
// number in the last three characters. For example: pulse000.csv
char csv_filename[13] = "pulsexxx.csv";

// when we first get into loop--
uint32_t times_into_loop = 0;


//---------- AS7341 visible light + near IR 10-band spectrometer ----------
// see https://adafruit.github.io/Adafruit_AS7341/html/class_adafruit___a_s7341.htm
// for parameter definitions.

// ADG436 control lines as I've wired things: 
// HIGH enables S1A, S2A, disables S1B, S2B;
// LOW disables S1A, S2A, enables S1B, S2B.

// two of the device parameters (see manufacturer's docs)
// original values: requires 612 milliseconds per all-channels read.
// #define AS7341_ASTEP 999
// #define AS7341_ATIME 100
// the following give 149 msec per all-channels read.
// #define AS7341_ASTEP 599
// #define AS7341_ATIME 29
// The following give 73 milliseconds per all-channels read.
#define AS7341_ASTEP 299
#define AS7341_ATIME 14

/*
allowed gain values:
  AS7341_GAIN_0_5X
  AS7341_GAIN_1X
  AS7341_GAIN_2X
  AS7341_GAIN_4X
  AS7341_GAIN_8X
  AS7341_GAIN_16X
  AS7341_GAIN_32X
  AS7341_GAIN_64X
  AS7341_GAIN_128X
  AS7341_GAIN_256X
  AS7341_GAIN_512X
*/
#define AS7341_GAIN AS7341_GAIN_512X

// a comment on this from 
// https://newscrewdriver.com/2023/01/23/notes-on-as7341-integration-time/
// Integration time follows the formula: (ATIME+1)*(ASTEP+1)*2.78 microseconds. 
// so ATIME of 100 and ASTEP of 999 ought to give an integration time of
// (101) * (1000) * 2.78 = 280.78 milliseconds. Two ADCs, so twice this, 
// or about 560 msec.

Adafruit_AS7341 as7341_spectrometer;
uint16_t as7341_spectrometer_readings[12];

int as7341_LIGHT_CURRENT;   // current (mA) to AS7341's headlight


//---------- RTC ----------
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


//---------- LED ----------
#define LED_OFF LOW
#define LED_ON HIGH

#define RED_LED_PIN 2   // LEDs: red is just for informing the user of stuff.

//---------- SETUP ----------
uint32_t timer;
void setup(){
  // light up the serial monitor and wait for it to be ready.
  timer = millis();
  Serial.begin(115200);
  while (!Serial && millis() - timer < 5000) {}

  Serial.println("\n*************** nRF95480 pulse ox test ***************");
  Serial.print("This file is " );
  Serial.println(__FILE__);
  Serial.print("Date and time of compilation: ");
  Serial.print(__DATE__);
  Serial.print(", ");
  Serial.println(__TIME__);

  pinMode(RED_LED_PIN, OUTPUT);   // set the LED-illuminating pin to be a digital output.

  digitalWrite(RED_LED_PIN, LED_OFF);   // make sure the LED is off.

  analogReadResolution(12);    // set ADC precision to 12 bits.

  // set various SPI device chip select lines to HIGH to make sure the devices aren't active right now.
  pinMode(SPI_TFT_CS, OUTPUT);
  digitalWrite(SPI_TFT_CS, HIGH);
  pinMode(SPI_SD_CS, OUTPUT);
  digitalWrite(SPI_SD_CS, HIGH);


  //---------- PULSE OX ----------
  Wire.begin();
  int result = bioHub.begin();
  if (result == 0) {
    Serial.println("Pulse of sensor started. Put your finger on it, please.");
  } else{
    Serial.println("Could not communicate with the pulse ox so bail out");
    while(1){}
  }

  Serial.println("Configuring pulse ox sensor...."); 

  // Configuring just the BPM settings. 
  // int error = bioHub.configBpm(MODE_ONE);
  int error = bioHub.configBpm(MODE_TWO);  // VERBOSE
  if(error == 0){
    Serial.println("Sensor configured.");
  } else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }
  //removed buffer bit


  //---------- RTC ----------
  if(! rtc.begin()){
    Serial.println("\nCouldn't find RTC so quit!");
    Serial.flush();
    while(1){}
  }
  if(rtc.lostPower()){      // When time needs to be set on a new device, or after a power loss, sets the RTC to the date & time this sketch was compiled
    Serial.println("\nRTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else{
    Serial.print("\nRTC has remained powered since last adjustment, so do not reset it.\nCurrent date and time: ");
  }

  //left out datetime and temp for now


  //---------- SD/TFT ----------
  setup_SD_TFT();


  //---------- Spectrometer ----------
  Serial.println("\nNow test the AS7341 spectrometer");
  Serial.print("We are running with ATIME = ");
  Serial.print(AS7341_ATIME);
  Serial.print(", ASTEP = ");
  Serial.print(AS7341_ASTEP);
  Serial.print(", and GAIN = ");
  Serial.println(AS7341_GAIN);

  int return_code = test_AS7341();

  if(return_code != 0)
  {
    Serial.print("AS7341 problem. return code is ");
    Serial.println(return_code);
  } else {
    Serial.println("AS7341s seem OK");
  }

  Serial.println("Now do an AS7341 speed test.");
  uint16_t datum;

  if (!as7341_spectrometer.readAllChannels(as7341_spectrometer_readings))
  {
    Serial.println("Error reading all as7341_spectrometer channels!");
    while(1) {}
  }

  datum = as7341_spectrometer.readChannel(AS7341_ADC_CHANNEL_3);
  Serial.print("meatball attempt first: ");
  Serial.println(datum);

  int max_to_do = 100;
  timer = millis();

  for(int itimes = 0; itimes < max_to_do; itimes++)
  {
    as7341_spectrometer.readAllChannels(as7341_spectrometer_readings);
    datum = as7341_spectrometer.readChannel(AS7341_ADC_CHANNEL_3);
  }

  Serial.print("meatball attempt last: ");
  Serial.println(datum);

  Serial.print("Just read AS7341 ");
  Serial.print(max_to_do);
  Serial.print(" times in ");
  Serial.print(millis()-timer);
  Serial.println(" milliseconds");


}

void loop(){
  delay(5000);
}


void setup_SD_TFT(){
  Serial.println("\n>>>>> Inside setup_SD_TFT()...");

  pinMode(TFT_RESET, OUTPUT);     // Initialize screen after resetting the TFT breakout by pulling the reset line low.
  digitalWrite(TFT_RESET, LOW);
  delay(100);
  digitalWrite(TFT_RESET, HIGH);
  delay(100);

  tft.init(135, 240);       // use if plastic film protecting the screen has black tab
  analogWrite(BACKLIGHT, TFT_BACKLIGHT_PWM_MAX);    // set up the backlight

  tft.setRotation(tft.getRotation()+1); // rotate the screen.

  Serial.print(F("Now initialize the SD file system..."));
  if(!SD.begin(SPI_SD_CS, SD_SCK_MHZ(10))){// Adafruit says breakouts require 10 MHz limit due to longer wires
    Serial.println(F("SD begin() failed so fall into a hole."));
    while(1){} 
  }
  Serial.println(F(" file system initialization went OK!"));

  load_TFT_display();
}

void load_TFT_display(){
  #define new_character_height 8
  #define new_x_origin 0
  #define new_x_middle 64
  #define new_x_rightside 127
  #define new_y_origin 0
  #define new_screen_background 0X1863    //very dark gray
  #define new_title_color ST77XX_YELLOW
  #define new_pixel_color ST77XX_ORANGE
  #define new_spacer_pixels 1
  tft_loads++;

  tft.fillScreen(new_screen_background);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  current_y_pixel = new_y_origin;

  //---------- SPECTROMETER VALUES ----------

  // title
  tft.setTextColor(new_title_color);
  tft.setCursor(new_x_origin + 1, current_y_pixel);
  tft.println("Skin color 2-8 + NIR");

  // values
  // need to move down the screen...
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  // ADC0/F1 415nm.. don't display this becaus we only want eight values total
  // tft.setTextColor(AS7341_F1_TFT_COLOR);
  // tft.print(as7341_spectrometer_readings[1]);
  // tft.print(" ");

  // ADC1/F2 445nm
  tft.setTextColor(AS7341_F2_TFT_COLOR);
  tft.print(as7341_spectrometer_readings[1]);
  tft.print(" ");

  // ADC2/F3 480nm
  tft.setTextColor(AS7341_F3_TFT_COLOR);
  tft.print(as7341_spectrometer_readings[2]);
  tft.print(" ");

  // ADC3/F4 515nm
  tft.setTextColor(AS7341_F4_TFT_COLOR);
  tft.print(as7341_spectrometer_readings[3]);
  tft.print(" ");

  // ADC0/F5 555nm
  tft.setTextColor(AS7341_F5_TFT_COLOR);
  tft.println(as7341_spectrometer_readings[6]);

  // we've just started a new line.
  current_y_pixel = current_y_pixel + new_character_height;
  tft.setCursor(new_x_origin, current_y_pixel);

  // ADC1/F6 590nm
  tft.setTextColor(AS7341_F6_TFT_COLOR);
  tft.print(as7341_spectrometer_readings[7]);
  tft.print(" ");

  // ADC2/F7 630nm
  tft.setTextColor(AS7341_F7_TFT_COLOR);
  tft.print(as7341_spectrometer_readings[8]);
  tft.print(" ");

  // ADC3/F8 680nm
  tft.setTextColor(AS7341_F8_TFT_COLOR);
  tft.print(as7341_spectrometer_readings[9]);
  tft.print(" ");

  // ADC5/NIR near infrared
  tft.setTextColor(AS7341_NIR_TFT_COLOR);
  tft.println(as7341_spectrometer_readings[11]);


  ////////////////////////////////////////////
  // horizontal line, flanked by blank lines
  ////////////////////////////////////////////

  // move down the screen
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);
  tft.drawFastHLine(new_x_origin, current_y_pixel, tft.width(), new_title_color);
  current_y_pixel = current_y_pixel + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  ////////////////////////////////////////////
  // time and date or filename: alternate between these two.
  ////////////////////////////////////////////
  DateTime now = rtc.now();

  // move down the screen
  current_y_pixel = current_y_pixel + new_spacer_pixels;
  tft.setCursor(new_x_origin + 1, current_y_pixel);

  if(tft_loads % 2 == 0)
  {
    tft.print(now.year(), DEC);
    tft.print('/');
    tft.print(now.month(), DEC);
    tft.print('/');
    tft.print(now.day(), DEC);
    tft.print(" ");
    tft.print(now.hour(), DEC);
    tft.print(':');
    if(now.minute() < 10) {tft.print("0");}
    tft.print(now.minute(), DEC);
    tft.print(':');
    if(now.second() < 10) {tft.print("0");}
    tft.print(now.second(), DEC);
    tft.println();
  } else {
    tft.println(csv_filename);
  }

  ////////////////////////////////////////////
  // horizontal line, flanked by blank lines
  ////////////////////////////////////////////

  // move down the screen
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  tft.drawFastHLine(new_x_origin, current_y_pixel, tft.width(), new_title_color);
  current_y_pixel = current_y_pixel + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  ////////////////////////////////////////////
  // LED currents
  ////////////////////////////////////////////

  tft.setTextColor(new_title_color);
  current_y_pixel = current_y_pixel + new_spacer_pixels;
  tft.setCursor(new_x_origin + 1, current_y_pixel);
  tft.println("NIR, white LED currnt");
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin + 1, current_y_pixel);

  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  ////////////////////////////////////////////
  // horizontal line, flanked by blank lines
  ////////////////////////////////////////////

  tft.setCursor(new_x_origin, current_y_pixel);
  tft.drawFastHLine(new_x_origin, current_y_pixel, tft.width(), new_title_color);
  current_y_pixel = current_y_pixel + new_spacer_pixels;
 
  ////////////////////////////////////////////
  // breakout boards' light currents
  ////////////////////////////////////////////

  tft.setTextColor(new_title_color);
  current_y_pixel = current_y_pixel + new_spacer_pixels;
  tft.setCursor(new_x_origin + 1, current_y_pixel);
  tft.println("AS7341 light");
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin + 1, current_y_pixel);

  tft.print(as7341_LIGHT_CURRENT);
  tft.print(" mA ");

  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;

  ////////////////////////////////////////////
  // horizontal line, flanked by blank lines
  ////////////////////////////////////////////

  tft.setCursor(new_x_origin, current_y_pixel);
  tft.drawFastHLine(new_x_origin, current_y_pixel, tft.width(), new_title_color);
  int top_of_divider_line = current_y_pixel;
  current_y_pixel = current_y_pixel + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  ////////////////////////////////////////////
  // number of AS7341 reads so far
  ////////////////////////////////////////////

  tft.setTextColor(new_title_color);
  current_y_pixel = current_y_pixel + new_spacer_pixels;
  tft.setCursor(new_x_origin + 1, current_y_pixel);
  tft.print("# reads");
  tft.setCursor(new_x_middle + 2, current_y_pixel);
  tft.println("NIR/680nm");

  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin + 1, current_y_pixel);
  tft.println(times_into_loop);

  float ratio_to_display;
  if(as7341_spectrometer_readings[9] > 0)
  {
    ratio_to_display = float(as7341_spectrometer_readings[index_NIR]) / 
    float(as7341_spectrometer_readings[index_680]);
  } else {
    ratio_to_display = 0;
  }
  tft.setCursor(new_x_middle + 2, current_y_pixel);
  tft.println(ratio_to_display, 3);
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;

  //---------- horizontal line, flanked by blank lines ----------

  tft.setCursor(new_x_origin, current_y_pixel);
  tft.drawFastHLine(new_x_origin, current_y_pixel, tft.width(), new_title_color);
  int bottom_of_divider_line = current_y_pixel;
  current_y_pixel = current_y_pixel + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  ////////////////////////////////////////////
  // vertical divider line
  ////////////////////////////////////////////

  tft.drawFastVLine(new_x_middle, bottom_of_divider_line, 
    (top_of_divider_line - bottom_of_divider_line), new_title_color);

  // value of current_y_pixel is 124 here, so we have 36 left to use.

  ////////////////////////////////////////////
  // make a graph of the NIR / 680 nm ratios.
  ////////////////////////////////////////////

  // let's work up a plot of all 100 NIR / 680 nm ratios. if times_into_loop
  // is 100 or greater, we can list the entire (circular) buffer. If not, then
  // we've only filled AS7341_NIR_680_ratio[0] through 
  // AS7341_NIR_680_ratio[times_into_loop - 1]. 
  // when we've filled the whole buffer, the earliest value is 
  // AS7341_NIR_680_ratio[times_into_loop] and the latest is 
  // AS7341_NIR_680_ratio[times_into_loop - 1].

  int circular_index_first, circular_index_last;
  float biggest_ratio = -100.;
  float smallest_ratio = 100.;
  int index_tft = 0;
  
  // bool dump_stuff = true;
  bool dump_stuff = false;

  if(dump_stuff)
  {
    Serial.print("\n\n>>>>>>>>>>>>>> times into loop = ");
    Serial.println(times_into_loop);
  }

  if(times_into_loop <= max_buffer_readings)
  {
    circular_index_last = times_into_loop - 1;
    circular_index_first = 0;

    for(int moo = circular_index_first; moo <= circular_index_last; moo++)
    {
      if(dump_stuff)
      {
        Serial.print(" "); Serial.print(moo); Serial.print("="); Serial.print(AS7341_NIR_680_ratio[moo]);
        if(moo % 10 == 0) {Serial.print("\n");}
      }
      if(AS7341_NIR_680_ratio[moo] > biggest_ratio) 
        {biggest_ratio = AS7341_NIR_680_ratio[moo];}
      if(AS7341_NIR_680_ratio[moo] < smallest_ratio) 
        {smallest_ratio = AS7341_NIR_680_ratio[moo];}
      tft_ratio_array[index_tft] = AS7341_NIR_680_ratio[moo];
      index_tft++;
    }
      if(dump_stuff)
      {
        Serial.print("\n "); 
        Serial.print("smallest, largest ratios = ");
        Serial.print(smallest_ratio, 3); Serial.print(" ");
        Serial.println(biggest_ratio, 3); 
      }
  } else {
    circular_index_last = (times_into_loop - 1) % 100;
    circular_index_first = circular_index_last + 1;

    for(int moo = circular_index_first; moo < max_buffer_readings; moo++)
    {
      if(dump_stuff)
      {
        Serial.print(" "); Serial.print(moo); Serial.print("="); Serial.print(AS7341_NIR_680_ratio[moo]);
        if(moo % 10 == 0) {Serial.print("\n");}
      }
      if(AS7341_NIR_680_ratio[moo] > biggest_ratio) 
        {biggest_ratio = AS7341_NIR_680_ratio[moo];}
      if(AS7341_NIR_680_ratio[moo] < smallest_ratio) 
        {smallest_ratio = AS7341_NIR_680_ratio[moo];}
      tft_ratio_array[index_tft] = AS7341_NIR_680_ratio[moo];
      index_tft++;
    }

    for(int moo = 0; moo <= circular_index_last; moo++)
    {
      if(dump_stuff)
      {
        Serial.print(" "); Serial.print(moo); Serial.print("="); Serial.print(AS7341_NIR_680_ratio[moo]);
        if(moo % 10 == 0) {Serial.print("\n");}
      }
      if(AS7341_NIR_680_ratio[moo] > biggest_ratio) 
        {biggest_ratio = AS7341_NIR_680_ratio[moo];}
      if(AS7341_NIR_680_ratio[moo] < smallest_ratio) 
        {smallest_ratio = AS7341_NIR_680_ratio[moo];}
      tft_ratio_array[index_tft] = AS7341_NIR_680_ratio[moo];
      index_tft++;
    }
    if(dump_stuff)
    {
      Serial.print("\n "); 
      Serial.print("smallest, largest ratios = ");
      Serial.print(smallest_ratio, 3); Serial.print(" ");
      Serial.println(biggest_ratio, 3); 
    }
  }

    if(dump_stuff)
  {
    // now see what we've put into the tft plot array.
    Serial.print("tft plot array; number of filled entries = "); Serial.println(index_tft);
    for(int ijklm = 0; ijklm < index_tft; ijklm++)
    {
      Serial.print(ijklm); Serial.print("="); Serial.print(tft_ratio_array[ijklm]);
      Serial.print(" ");
      if((ijklm + 1) % 10 == 0) {Serial.print("\n");}
    }
  }

  ////////////////////////////////////////////
  // put axis lines on the plot
  ////////////////////////////////////////////

  uint16_t ymax = 159;
  uint16_t ymin = 125;

  tft.drawFastVLine(new_x_origin + 11, bottom_of_divider_line, 
    (ymax - ymin) + 1, ST77XX_ORANGE);

  tft.drawFastHLine(new_x_origin + 11, ymax - 1, 100, ST77XX_ORANGE);

  // add an ordinate label
  tft.setRotation(3);
  tft.setCursor(1, 1);
  tft.print("IR/680");
  tft.setRotation(0);

  ////////////////////////////////////////////
  // plot the points
  ////////////////////////////////////////////

  // now plot the points! note that we'll need to turn everything upside down.
  float pixels_per;
  float drange = biggest_ratio - smallest_ratio;
  if(drange <= 0.) {drange = 0.01;} 
  pixels_per = float(ymax - ymin + 1) / drange;

  for(int ijklm = 0; ijklm < index_tft; ijklm++)
  {
    float dy = pixels_per * (tft_ratio_array[ijklm] - smallest_ratio);
    current_y_pixel = floor(ymax - dy);
    current_x_pixel = ijklm + new_x_origin + 10;
    tft.drawPixel(current_x_pixel, current_y_pixel, new_pixel_color); 
    // Serial.print("pixel x, y = "); Serial.print(current_x_pixel);
    // Serial.print(" "); Serial.println(current_y_pixel);
  }
}