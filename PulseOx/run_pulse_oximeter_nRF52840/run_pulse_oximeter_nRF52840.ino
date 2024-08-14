/*******************************************************************************
  This program is run_pulse_oximeter_nRF52840.ino. Run the 
  the circuit pulse_oximeter_nRF52840.

  Use test_pulse_oximeter_nRF52840.ino to test the circuiy; use this code
  to take pulse ox data.
  
  George Gollin
  University of Illinois
  March 2023.
*******************************************************************************/

// Do we want to do BluetoothLE communications? If not, then don't
// call the bluetooth routines.

#define DO_BLUETOOTH false

// which AS7341 channels shall I read? Only ONE of these should be true.
#define DO_F1F4_CLEAR_NIR false
#define DO_F5F8_CLEAR_NIR true
#define DO_ALL_CHANNELS   false

// should I only (after initializing and testing) only read the pulse ox
// AS7341?
#define ONLY_READ_PULSE_OX true

/*
  AS7341_CHANNEL_415nm_F1,
  AS7341_CHANNEL_445nm_F2,
  AS7341_CHANNEL_480nm_F3,
  AS7341_CHANNEL_515nm_F4,
  AS7341_CHANNEL_CLEAR_0,
  AS7341_CHANNEL_NIR_0
*/
/*
  AS7341_CHANNEL_555nm_F5,
  AS7341_CHANNEL_590nm_F6,
  AS7341_CHANNEL_630nm_F7,
  AS7341_CHANNEL_680nm_F8,
  AS7341_CHANNEL_CLEAR,
  AS7341_CHANNEL_NIR
*/

// Libraries...
#include <SPI.h>
#include <Adafruit_Sensor.h>  
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
#include <Wire.h>
// Core graphics library
#include <Adafruit_GFX.h>    
// Hardware-specific library for TFT display
#include <Adafruit_ST7789.h>
// SPI / QSPI flash library
// Time libraries
#include <Time.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
// SD card & FAT filesystem library
#include <SdFat.h>
// #include <SD.h>
// #include "FsDateTime.h"
//#include <Adafruit_AS7341.h>
#include <Adafruit_AS7341.h>
// #include <Adafruit_AS7341_GDG.h>
// SPI / QSPI flash library
#include <Adafruit_SPIFlash.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>

/////////////////// SparkFun Pulse Oximeter, etc. //////////////////////
// SparkFun Pulse Oximeter and Heart Rate Sensor - MAX30101 & MAX32664
// see https://www.sparkfun.com/products/15219

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

/////////////////// on-board BluetoothLE //////////////////////

// CS: I don't know what pin, actually.
#define BLU_CS 8

/////////////// TFT display, including its microSD carrier //////////

// I am using an Adafruit 1.14" 240x135 Color TFT Display 
// with a MicroSD Card carrier. (part ID 4383)

// TFT select pin
#define SPI_TFT_CS  12 
// TFT display/command pin
#define SPI_D_slash_C  11 
#define SPI_DC  11 
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
// KEEP THIS FIXED AT 100!
#define max_buffer_readings 100
#define max_buffer_channels 12
#define index_NIR 11
#define index_680 9


float PULSE_OX_NIR_680_ratio[max_buffer_readings];
float tft_ratio_array[max_buffer_readings];
int last_tft_ratio_array_entry = 0; 

// the device uses a 5-6-5 color encoding scheme: 5 bits of red, 6 bits
// of green, 5 bits of blue.
/*
Some ready-made 16-bit ('565') color settings in Adafruit_ST77xx.h:
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_ORANGE 0xFC00
*/

// green is 0x07E0 =   0b.0000.0111.1110.0000 = 0b.00000.11111.00000
// dark green would be 0b.0000.0001.1110.0000 = 0x01E0

#define GDGDKGREEN 0x01E0

// Height of a size = 1 TFT character, in pixels:
#define TFT_character_height 8

// TFT screen rotation parameters:
//    setRotation(uint8_t rotation);
// with possible argument values of 0, 1, 2, or 3. 
// 0: top of the display is by the SD card carrier
// 1: top of the display is by the header pins
// 2: top of the display is on the opposite side from 
//    the SD card carrier
// 3: top of the display is on the opposite side from 
//    the orientation of parameter value 1.

// current y value as we generate contents to display
uint16_t current_y_pixel = 0;
uint16_t current_x_pixel = 0;

uint32_t tft_loads = 0;

// here's the TFT display.
// Adafruit_ST7735 tft = Adafruit_ST7735(SPI_TFT_CS, SPI_D_slash_C, TFT_RESET);
Adafruit_ST7789 tft = Adafruit_ST7789(SPI_TFT_CS, SPI_D_slash_C, TFT_RESET);

//////////////////// SD stuff ///////////////////

// SD card select "pin" on the TFT display: on the schematic it's 
// net SPI_SD_CS which goes to pin 10.
#define SPI_SD_CS 10

// define a file object
File myFile;
char filename[14];

// here's another, to hold csv data.
File csvFile;

// files are named sequentially. Here's the index number for the next one to
// be written
uint32_t index_next_filename;

// Create a CSV file name in 8.3 format. I will encode a file sequence
// number in the last three characters. For example: flowM000.csv
char csv_filename[13] = "pulsexxx.csv";

// close/reopen the file after this many events:
#define CLOSE_REOPEN_FILE 25

// when we first get into loop--
uint32_t t_start;
uint32_t millis_SD_start;
uint32_t times_into_loop = 0;

// since we're doing hardware SPI, the MOSI, MISO and CLK pins
// are already known to the compiler.

// since we're doing hardware SPI, the MOSI, MISO and CLK pins
// are already known to the compiler.

// define a file object for the TFT.
File myFilePOx;

// other SD card stuff
SdFile root;

// SD card filesystem
SdFat SD;   

// also... device data file for SD.
File PulseFile;

// Create a CSV file name in 8.3 format. I will encode a file sequence
// number in the last four characters. For example: pulse000.csv
char Pulse_filename[13] = "Pulsexxx.csv";

// close/reopen the file after this many events:
#define CLOSE_REOPEN_FILE 25


// some other TFT display parameters 

// PWM value (0 is off, 255 is full on) for backlight pin
#define TFT_BACKLIGHT_PWM 128
#define TFT_BACKLIGHT_PWM_MAX 255

// height of a size = 1 TFT character, in pixels:
#define TFT_character_height 8
// the display can fit 21 characters of this size in one row.
 
// BMP image dimensions
int32_t width  = 0, 
        height = 0;

// pi is used by some of the drawing routines.
float pipi = 3.1415926;

uint32_t millis_start;

/////////// AS7341 visible light + near IR 10-band spectrometer ////////////////

// see https://adafruit.github.io/Adafruit_AS7341/html/class_adafruit___a_s7341.htm
// for parameter definitions.

// I have two of these, with an ADG436 dual analog switch to choose which one 
// gets I2C commands so...

// Feather M0 WiFi pin to pick which one:
#define SWITCH_I2C 9

// ADG436 control lines as I've wired things: 
// HIGH enables S1A, S2A, disables S1B, S2B;
// LOW disables S1A, S2A, enables S1B, S2B.

// AS7341 # 1, selected when ADG436 control line is HIGH
#define DO_SKIN_COLOR HIGH

// AS7341 # 2, selected when ADG436 control line is LOW
#define DO_PULSE_OX LOW

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

#define AS7341_GAIN AS7341_GAIN_512X
// #define AS7341_GAIN AS7341_GAIN_256X
// #define AS7341_GAIN AS7341_GAIN_128X

// a comment on this from 
// https://newscrewdriver.com/2023/01/23/notes-on-as7341-integration-time/
// Integration time follows the formula: (ATIME+1)*(ASTEP+1)*2.78 microseconds. 
// so ATIME of 100 and ASTEP of 999 ought to give an integration time of
// (101) * (1000) * 2.78 = 280.78 milliseconds. Two ADCs, so twice this, 
// or about 560 msec.

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

/* channel references
these are as7341_adc_channel_t constants
  AS7341_ADC_CHANNEL_0,
  AS7341_ADC_CHANNEL_1,
  AS7341_ADC_CHANNEL_2,
  AS7341_ADC_CHANNEL_3,
  AS7341_ADC_CHANNEL_4,
  AS7341_ADC_CHANNEL_5

these are as7341_color_channel_t constants
  AS7341_CHANNEL_415nm_F1,
  AS7341_CHANNEL_445nm_F2,
  AS7341_CHANNEL_480nm_F3,
  AS7341_CHANNEL_515nm_F4,
  AS7341_CHANNEL_CLEAR_0,
  AS7341_CHANNEL_NIR_0,
  AS7341_CHANNEL_555nm_F5,
  AS7341_CHANNEL_590nm_F6,
  AS7341_CHANNEL_630nm_F7,
  AS7341_CHANNEL_680nm_F8,
  AS7341_CHANNEL_CLEAR,
  AS7341_CHANNEL_NIR,
*/

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
// KEEP THIS FIXED AT 100!
#define max_buffer_readings 100
#define max_buffer_channels 12
#define index_NIR 11
#define index_680 9

Adafruit_AS7341 as7341_SKIN_COLOR;
uint16_t as7341_SKIN_COLOR_readings[max_buffer_channels];

Adafruit_AS7341 as7341_PULSE_OX;
uint16_t as7341_PULSE_OX_readings[max_buffer_channels];

uint16_t as7341_SKIN_COLOR_readings_buffer[max_buffer_readings][max_buffer_channels];
uint16_t as7341_PULSE_OX_readings_buffer[max_buffer_readings][max_buffer_channels];


// where in the buffer the last read went
int SKIN_COLOR_index_last_read = -1; 
int PULSE_OX_index_last_read = -1; 

// The device is an AMS-OSRAM AS7341. See p. 16 of the data sheet at 
// data sheet: https://ams.com/en/as7341 for spectral sensitivity.
/*
channel wavelength (nm)  FWHM (or range)
  1           415         26
  2           445         30
  3           480         36
  4           515         39
  5           555         39
  6           590         40
  7           630         50
  8           680         52
  NIR         910      875 - 975
*/
// compare with white LED intensities:
/*
band   wavelength (nm) 
  1           430 
  2           560 
  3           570 
  4           595 
  5           612 
  6           633 
  7           660 
  8           670 
  9           700 (goes to zero ~775)
*/
// the on-breakout "white" light is connected to the sensor's LDR pin.
// It's an Everlight Americas EAHC2835WD6. The intensity is broadband, with
// one peak at 450 nm (with FWHM ~ 25 nm) and a second at 600 nm with FWHM
// extending from abot 510 nm to 660 nm. Illumination goes to zero below 
// 400 nm and above 800 nm.

// breakout board white light currents, mA
int as7341_PULSE_OX_LIGHT_CURRENT = 0;
int as7341_SKIN_COLOR_LIGHT_CURRENT = 0;

/////////////////// iPhone Bluetooth stuff //////////////////////

// first character position in the string specifying the device name
#define name_start 14
// Size of the read buffer for incoming data
#define BUFSIZE 160   
// If set to 'true' enables debug output
#define VERBOSE_MODE  true  

// Circuit pins
#define BLUEFRUIT_SPI_CS               16
#define BLUEFRUIT_SPI_IRQ              0
#define BLUEFRUIT_SPI_RST              10  
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

// carriage return is 0x0A.
#define ASCII_CR 0x0A

// here is the name to give to this particular BluetoothLE device,
// including the AT command that sets the (new) device name.
// Pick one...
#define SETBLUETOOTHNAME  "AT+GAPDEVNAME=_PulseOx_two_sensors"
// #define SETBLUETOOTHNAME  "AT+GAPDEVNAME=_P523_mic1"

char iPhone_incoming[BUFSIZE+1];
int iphone_incoming_index = 0;
char iPhone_last_message[BUFSIZE+1];
int iPhone_last_message_length = 0;

bool start_recording = false;

// hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user-
// selected CS/IRQ/RST

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ,
BLUEFRUIT_SPI_RST);

/////////////////// DS3231 RTC stuff //////////////////////

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//////////////////////////// LED stuff //////////////////////

#define LED_OFF LOW
#define LED_ON HIGH

// LEDs: red is just for informing the user of stuff,
// while white is a quasi-white LED and NIR is a 940
// nm near infrared LED. The red LED is fed (through a 1k 
// resistor) directly from a digital output pin.
// However, the white and NIR LEDs are fed by DAC outputs.
// Pins A3 and A4 read the voltages at the positive LED input pin.
#define RED_LED_PIN 6

// which DAC channel drives which LED
#define NIR_LED_DAC_CHANNEL MCP4728_CHANNEL_A
#define WHITE_LED_DAC_CHANNEL MCP4728_CHANNEL_B

// which ADC channel reads voltage at which LED
#define NIR_LED_ADC_CHANNEL A3
#define WHITE_LED_ADC_CHANNEL A4

// ADC values read for the LED voltages
uint16_t NIR_LED_ADC_VALUE;
uint16_t WHITE_LED_ADC_VALUE;

// ADC voltage references
#define ADC_VREF 3.3
#define ADC_MAX 4096

// series resistor values in the schematic:
#define R_NIR 220
#define R_WHITE 30

// the particular LEDs I am using:
// NIR: Everlight IR333-A, peak intensity at 940 nm, falling to zero below 
// 880 nm and above 1035 nm.
// https://cdn-shop.adafruit.com/datasheets/IR333_A_datasheet.pdf
// white: FLR-50T04-HW7, with 9 different LED junctions. Peak intensities
// have FWHM of about 25 nm with peaks at the following wavelengths:
/*
band   wavelength (nm) 
  1           430 
  2           560 
  3           570 
  4           595 
  5           612 
  6           633 
  7           660 
  8           670 
  9           700 (goes to zero ~775)
*/
// https://www.adafruit.com/datasheets/FLR-50T04-HW7%20(2012.02.22).pdf 

// LED currents, in mA:
float NIR_LED_current;
float white_LED_current;
float skin_color_breakout_LED_current;
float pulse_ox_breakout_LED_current;

float DAC_voltage_NIR;
float LED_voltage_NIR;
float DAC_voltage_WHITE;
float LED_voltage_WHITE;

/////////////////// MCP4728 quad 12-bit DAC //////////////////////

Adafruit_MCP4728 mcp;
// DAC voltage reference
#define DAC_VREF 3.3
#define DAC_MAX 4096

/////////////////// other stuff //////////////////////
#define become_quiet_after -1
int32_t times_into_loop = 0;

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() 
{
  // light up the serial monitor and wait for it to be ready.
  uint32_t t1234 = millis();
  Serial.begin(115200);
  while (!Serial && millis() - t1234 < 5000) {}

  Serial.println("\n*************** Run pulse_oximeter_2024 device ***************");

  Serial.print("This file is " );
  Serial.println(__FILE__);

  Serial.print("Date and time of compilation: ");
  Serial.print(__DATE__);
  Serial.print(", ");
  Serial.println(__TIME__);

  if(DO_F1F4_CLEAR_NIR && !DO_F5F8_CLEAR_NIR && !DO_ALL_CHANNELS)
  {
    Serial.println("We will only look at 415nm, 445nm, 480nm. 515nm, clear, and NIR.");

  } else if(!DO_F1F4_CLEAR_NIR && DO_F5F8_CLEAR_NIR && !DO_ALL_CHANNELS) {
    Serial.println("We will only look at 555nm, 590nm, 630nm. 680nm, clear, and NIR.");

  } else if(!DO_F1F4_CLEAR_NIR && !DO_F5F8_CLEAR_NIR && DO_ALL_CHANNELS) {
    Serial.println("We will look at all wavelength channels");

  } else {
    Serial.println("Error in DO_F1F4_CLEAR_NIR, DO_F5F8_CLEAR_NIR, DO_ALL_CHANNELS switches.");
    Serial.println("bailing out!");
    while(1) {}
  }

  // set the LED-illuminating pin to be a digital output.
  pinMode(RED_LED_PIN, OUTPUT);

  // make sure the LED is off.
  digitalWrite(RED_LED_PIN, LED_OFF);

  // set ADC precision to 12 bits.
  analogReadResolution(12);

  // set various SPI device chip select lines to HIGH to
  // make sure the devices aren't active right now.

  // WiFi on board the Feather M0 WiFi board
  pinMode(WIFI_CS, OUTPUT);
  digitalWrite(WIFI_CS, HIGH);
  // BluetoothLE device
  pinMode(BLUEFRUIT_SPI_CS, OUTPUT);
  digitalWrite(BLUEFRUIT_SPI_CS, HIGH);
  // TFT display and its built-in SD carrier
  pinMode(SPI_TFT_CS, OUTPUT);
  digitalWrite(SPI_TFT_CS, HIGH);
  pinMode(SPI_SD_CS, OUTPUT);
  digitalWrite(SPI_SD_CS, HIGH);

  // also tell the user about the treset button. Pushing the button will
  // ground this pin, which we'll recognize as a reboot request.
  pinMode(RESET_ARDUINO_PIN, INPUT);

  Serial.println(">>> Push the button soldered to the PCB to reboot. <<<");

  ////////////////////////////////////////////////////////////
  // fire up the RTC.
  ////////////////////////////////////////////////////////////

  if (! rtc.begin()) {
    Serial.println("\nCouldn't find RTC so quit!");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) 
  {
    // Serial.println("\nRTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  } else {
    // Serial.print("\nRTC has remained powered since last adjustment, so do not reset it.\nCurrent date and time: ");
  }

  DateTime now = rtc.now();

  Serial.print("DS3231 RTC date, time are ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print("), ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if(now.minute() < 10) {Serial.print("0");}
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if(now.second() < 10) {Serial.print("0");}
  Serial.print(now.second(), DEC);

  Serial.print(". temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.print("*C or ");
  Serial.print(1.8 * rtc.getTemperature() + 32.);
  Serial.println("*F");

  ///////////////////// setup and initialize SD and TFT /////////////////////

  setup_SD_TFT();

  ///////////////////// display a bitmap /////////////////////

  Serial.println("Back from setup_SD_TFT. About to display a bitmap.");

  // now display a colorful bitmap image if it is available on SD.
  // Serial.println("Back in setup after setup_SD_TFT. Display a pretty picture of a rose...");  
  if(SD.exists("roseMS.bmp"))
  {
    // Serial.println("Found roseMS.bmp.");

    // tell the user the dimensions of the image...
    stat_stat = reader.bmpDimensions("/roseMS.bmp", &width, &height);
    // reader.printStatus(stat_stat);
    if(stat_stat == IMAGE_SUCCESS) 
    { 
      /*
      Serial.print(F("Image dimensions: "));
      Serial.print(width);
      Serial.write('x');
      Serial.println(height);
      */
    }

    // Load full-screen BMP file 'roseMS.bmp' at position (0,0) (top left).
    // Notice the 'reader' object performs this, with 'tft' as an argument.
    // Serial.print(F("Loading roseMS.bmp to screen (1)..."));
    stat_stat = reader.drawBMP("/roseMS.bmp", tft, 0, 0);
    // reader.printStatus(stat_stat);   

    // now pause for a bit
    delay(5000);

  } else {
    Serial.println("roseMS.bmp not found.");
  }

  ////////////////////////////////////////////////////////////
  // briefly test the MCP4728 DAC we use to drive the LEDs.
  ////////////////////////////////////////////////////////////

  // Serial.println("\nMCP4728 quad DAC test");

  // Initialize!
  Serial.println("fire up the MCP4728 DAC.");
  if (!mcp.begin()) 
  {
    Serial.println("Failed to find MCP4728 DAC so bail out!");
    while (1) {delay(10);}
  }
  
  // now set the DAC values all the way up.
  int the_value = DAC_MAX - 1;
  set_LED_DACs(the_value, the_value, true);

  // now set the DAC values to zero.
  the_value = 0;
  set_LED_DACs(the_value, the_value, true);

  // now flash the LEDs a few times, but using verbose mode off.
  Serial.println("Use DAC to flash NIR and white LEDs for a few seconds.");
  for(int ijk = 0; ijk < 10; ijk++)
  {
    set_LED_DACs(DAC_MAX - 1, DAC_MAX - 1, false);
    delay(250);
    set_LED_DACs(0, 0, false);
    delay(250);
  }
  // just for fun...
  blink_LED(5000);

  ////////////////////////////////////////////////////////////
  // Initialise the BluetoothLE (BLE) module if we want to...
  ////////////////////////////////////////////////////////////

  if(DO_BLUETOOTH)
  {
    Serial.print(F("\nInitialising the Bluefruit LE module: "));

    if ( !ble.begin(VERBOSE_MODE) )
    {
      error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
    Serial.println( F("OK!") );

    if ( FACTORYRESET_ENABLE )
    {
      /* Perform a factory reset to make sure everything is in a known state */
      Serial.println(F("Performing a factory reset: "));
      if ( ! ble.factoryReset() )
      {
        // error(F("Couldn't factory reset"));
        Serial.println(F("Couldn't factory reset"));
      }
    }

    // Disable command echo from Bluefruit
    ble.echo(false);

    Serial.println("Requesting Bluefruit info:");
    ble.info();

    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
    Serial.println(F("then Enter characters to send to Bluefruit"));
    Serial.println();

    // debug info is a little annoying after this point!
    ble.verbose(false);  

    // Wait for connection to iPhone
    Serial.println("waiting for connection (might hang)...");
    while (! ble.isConnected()) {delay(500);}
    Serial.println("now BLE is connected.");

    // now try changing the name of the BLE device.
    Serial.println("About to change bluetoothLE module name via the command\n   ");
    Serial.println(SETBLUETOOTHNAME);
    ble.println(SETBLUETOOTHNAME);
    Serial.println("Just past the ble.println. \n\nNow blink the LED for a bit.");

    // Set module to DATA mode
    Serial.println( F("Switching Bluetooth to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);

    // see Adafruit_BluefruitLE_SPI.cpp.
    // mode choices are  BLUEFRUIT_MODE_COMMAND and BLUEFRUIT_MODE_DATA
  } else {

    Serial.print(F("\nLet's not do anything with Bluetooth."));
  }

  ////////////////////////////////////////////////////////////
  // Do some AS7341 spectrometer tests.
  ////////////////////////////////////////////////////////////

  Serial.println("\nNow test the two AS7341 spectrometers");
  Serial.print("We are running both with ATIME = ");
  Serial.print(AS7341_ATIME);
  Serial.print(", ASTEP = ");
  Serial.print(AS7341_ASTEP);
  Serial.print(", and GAIN = ");
  Serial.println(AS7341_GAIN);

  // set the analog switch that we use to select which device sees the I2C lines
  pinMode(SWITCH_I2C, OUTPUT);
  // two choices for this pin:
  digitalWrite(SWITCH_I2C, DO_SKIN_COLOR);
  // digitalWrite(SWITCH_I2C, DO_PULSE_OX);

  int return_code = test_AS7341();

  if(return_code != 0)
  {
    Serial.print("AS7341 problem. return code is ");
    Serial.println(return_code);
  } else {
    Serial.println("AS7341s seem OK");
  }

  ////////////////////////////////////////////////////////////
  // read AS7341 channels a few rimes.
  ////////////////////////////////////////////////////////////

  Serial.println("Now do a quick AS7341 speed test.");
  uint32_t tstart, tstop;
  uint16_t datum;

  // select the spectrometer.
  digitalWrite(SWITCH_I2C, DO_PULSE_OX);
  // wait for things to settle
  delay(5);
  
  int max_to_do = 25;
  tstart = millis();

  for(int itimes = 0; itimes < max_to_do; itimes++)
  {
    if(DO_F1F4_CLEAR_NIR)
    {
      as7341_PULSE_OX.readF1F4_Clear_NIRChannels(as7341_PULSE_OX_readings);

    } else if (DO_F5F8_CLEAR_NIR) {
      as7341_PULSE_OX.readF5F8_Clear_NIRChannels(as7341_PULSE_OX_readings);

    } else {
      as7341_PULSE_OX.readAllChannels(as7341_PULSE_OX_readings);
    }
    datum = as7341_PULSE_OX_readings[11];
  }
  tstop = millis();

  Serial.print("Just read AS7341 ");
  Serial.print(max_to_do);
  Serial.print(" times in ");
  Serial.print(tstop - tstart);
  float ms_per_read = float(tstop - tstart) / float(max_to_do);
  Serial.print(" milliseconds. Time per read: ");
  Serial.print(ms_per_read, 3);
  Serial.print(" milliseconds. Last NIR value read was ");
  Serial.println(datum);

  ////////////////////////////////////////////////////////////
  // now load the TFT display with stuff.
  ////////////////////////////////////////////////////////////

  // turn on the breakout lights, do a TFT display, then turn off the lights.
  set_breakout_lights(50, 10, true);
  load_TFT_display();
  set_breakout_lights(0, 0, true);

  ////////// all done with setup! ////////// 
  Serial.println("*****************************\n...all done with setup.");
  if(DO_BLUETOOTH)
  {
    Serial.println("Enter stuff in the BluetoothLE UART module on your smartphone.\n");
  }
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void loop() 
{
  // verbose output for a while?
  bool verbose = (times_into_loop < become_quiet_after);
  times_into_loop++;

  // how often to update the display
  int TFT_how_often;

  // keep track of when we get started.
  if(times_into_loop <= 1) 
  {
    Serial.println("\nJust entered loop for the first time.");
    TFT_how_often = 25;
    t_start = millis();

  // turn on the LEDs to maximum brightness. first argument is NIR LED, second is white LED, third is verbose mode switch.
    int the_NIR_value = DAC_MAX - 1;
    int the_white_value = 0;
    set_LED_DACs(the_NIR_value, the_white_value, false);
    
    // also set the breakout board light on the skin color spectrometer if
    // we plan to read it out. first argument is for skin color, second 
    // for pulse ox.
    if(ONLY_READ_PULSE_OX)
    {
      set_breakout_lights(0, 0, false); 
    } else {
      set_breakout_lights(4, 0, false); 
    }
  }

  if(times_into_loop == 101) 
  {
    Serial.println("\nTime to complete 100 passes through loop: ");
    Serial.print(float(millis() - t_start) / 1000., 3);
    Serial.println(" seconds\n**********************************");
  }

  if(times_into_loop % 250 == 0) 
  {
    Serial.print("...entered loop for the ");
    Serial.print(times_into_loop);
    Serial.println("th time.");
  }

  // read (and perhaps) echo the two spectrometers' data
  reread_AS7341(ONLY_READ_PULSE_OX, verbose);

  // also write the data to an SD file. File is already open.
  write_SD_data();

  // update display from time to time
  if(times_into_loop % TFT_how_often == 0) {load_TFT_display();}

/*
  // turn off the LEDs.
  the_value = 0;
  set_LED_DACs(the_value, the_value, false);
  set_breakout_lights(the_value, the_value, false);
*/

  if(verbose)
  {
    // rereadings stores readings into circular buffers:
    //    as7341_SKIN_COLOR_readings_buffer[0:999][0:11]
    //    as7341_PULSE_OX_readings_buffer[0:999][0:11]
    // with indices SKIN_COLOR_index_last_read and PULSE_OX_index_last_read 
    // let's dump some of them.

    Serial.println("Inside loop. dump data read from AS7341.");

    int indxfrst = SKIN_COLOR_index_last_read - 10;
    if(indxfrst < 0) {indxfrst = 0;}
    if(SKIN_COLOR_index_last_read >= 0)
    {
      for(int indx = indxfrst; indx <= SKIN_COLOR_index_last_read; indx++)
      {
        Serial.print("skin color data, slice ");
        Serial.println(indx);  
        for(int indx2 = 0; indx2 < max_buffer_channels; indx2++)
        {
          Serial.print(indx2);
          Serial.print(" ");
          Serial.println(as7341_SKIN_COLOR_readings_buffer[indx][indx2]);
        }
      }
    }
  }

    if(DO_BLUETOOTH)
  {
    // Check for user input from the serial monitor input field
    // to be sent to the bluetooth module, if I want to do bluetooth.
    char n, inputs[BUFSIZE+1];

    if (Serial.available())
    {
      n = Serial.readBytes(inputs, BUFSIZE);
      // inputs[n] = 0;
      // Send characters to Bluefruit
      Serial.print("Sending to iPhone: ");
      Serial.println(inputs);

      // Send input data to host via Bluefruit
      ble.print(inputs);
    }

    // Echo received-from-iPhone data
    while ( ble.available() )
    {
      int c = ble.read();

      // Serial.print((char)c);

      iPhone_incoming[iphone_incoming_index] = (char)c;
      iphone_incoming_index++;
      if(iphone_incoming_index > BUFSIZE) {iphone_incoming_index = 0;}

      iPhone_last_message[iPhone_last_message_length] = (char)c;
      iPhone_last_message_length++;
      if(iPhone_last_message_length > BUFSIZE) {iPhone_last_message_length = 0;}

      if(c == ASCII_CR)
      {
        Serial.print("Just saw an iPhone message: '");

        // add a null after the message, where the carriage return is.
        iPhone_last_message[iPhone_last_message_length - 1] = '\0';

        Serial.print(iPhone_last_message);
        Serial.println("'");
        iPhone_last_message_length = 0;

        // now let's see if we recognize it.
        char go_message[3] = "go";
        char GO_message[3] = "GO";

        if(strcmp(iPhone_last_message, go_message) == 0 ||
          strcmp(iPhone_last_message, GO_message) == 0)
          {
              Serial.println("got a GO! message from iPhone");
              start_recording = true;
              blink_LED(10000);
          } else {
              Serial.println("unrecognized message from iPhone");
              start_recording = false;
          }
      }
    }
  }
  // OK, that's it for bluetoothing! 
  // that's it!
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup_SD_TFT(void) 
{
  // Serial.println("\n>>>>> Inside setup_SD_TFT()...");

  // set up the backlight line... 
  // see https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/ 
  // analogWrite(BACKLIGHT, TFT_BACKLIGHT_PWM);
  analogWrite(BACKLIGHT, TFT_BACKLIGHT_PWM_MAX);
  
  // Initialize screen after resetting the TFT breakout by pulling the reset line low.
  pinMode(TFT_RESET, OUTPUT);
  digitalWrite(TFT_RESET, LOW);
  delay(10);
  digitalWrite(TFT_RESET, HIGH);
  delay(10);
  
  // if the plastic film protecting the screen has a black tab, use this:
  tft.initR(INITR_BLACKTAB); 
  // other options are INITR_REDTAB and INITR_GREENTAB

  // ******************** ALERT ******************** 
  // I find that my green-tabbed TFTs must be initialized using the 
  // INITR_BLACKTAB paramater! The INITR_GREENTAB parameter does not work
  // properly for them, the symptom being anj inability to display a
  // bitmap file. Very strange.
  // ******************** ALERT ******************** 

  // The Adafruit_ImageReader constructor call (before setup())
  // accepts an uninitialized SdFat or FatFileSystem object. This MUST
  // BE INITIALIZED before using any of the image reader functions!
  // Serial.print(F("Now initialize the SD file system..."));

  // SD card is pretty straightforward, a single call...
  if(!SD.begin(SPI_SD_CS, SD_SCK_MHZ(10))) 
  { 
    // Adafruit says breakouts require 10 MHz limit due to longer wires
    Serial.println(F("SD begin() failed so fall into a hole."));
    while(1) {} 
  }

  Serial.println(F("SD file system initialization went OK."));

  // let's try writing/reading to a file before messing with the BMP images.
  
  // Serial.println("Now remove testPOx.txt if it exists.");
  if(SD.exists("testPOx.txt"))
  {
    SD.remove("testPOx.txt");
    // Serial.println("Found, then removed testPOx.txt");
  } else {
    // Serial.println("testPOx.txt does not exist yet.");
  }

  delay(10);

  // Serial.println("Now test SD file write/read: file name is testPOx.txt.");
  // open the file. 
  myFilePOx = SD.open("testPOx.txt", FILE_WRITE);

  delay(10);

  // if the file opened okay, write to it:
  if (myFilePOx) 
  {
    // Serial.println("Writing to testPOx.txt...");
    myFilePOx.println("testing 1, 2, 3.");
    myFilePOx.println(__FILE__);
    myFilePOx.print(__DATE__);
    myFilePOx.print(", ");
    myFilePOx.println(__TIME__);
    myFilePOx.println("//// end of file writing ////");

    /*
    Serial.println("We tried to write the following to SD: "); 
    Serial.println("testing 1, 2, 3.");
    Serial.println(__FILE__);
    Serial.print(__DATE__);
    Serial.print(", ");
    Serial.println(__TIME__);
    Serial.println("//// end of file writing ////");
    */

    // close the file:
    myFilePOx.close();
    Serial.println("SD file writing test went OK.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("SD trouble: error opening testPOx.txt for writing.");
  }

  // re-open the file for reading:
  // Serial.print("Now read the file. ");
  myFilePOx = SD.open("testPOx.txt");

  if (myFilePOx) 
  {
    // Serial.println("SD test file testPOx.txt contents:");
    // read from the file until there's nothing else in it:
    while (myFilePOx.available()) 
    {
      myFilePOx.read();
      // Serial.write(myFilePOx.read());
    }
    // close the file:
    myFilePOx.close();

  } else {

    // if the file didn't open, print an error:
    Serial.println("error opening testPOx.txt so bail out!");
    while(1) {}
  }

  // now open the data file. Look around for a filename that isn't already 
  // present on the SD card.

  // I'll use this flag in the loop searching for the first unused filename.
  bool have_a_name = false;

  // start at 0, and loop up to 16^5 -1 = 1,048,575.
  long index_filename;
  long index_start = 0;

  for (index_filename = index_start; index_filename <= 1048575L; index_filename++) 
  {
    if (have_a_name) break;

    index_next_filename = index_filename;
    // Now create a filename for this value of index_next_filename,
    // which is a global variable. name is in Pulse_filename.

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

    Pulse_filename[7] = digit7 + 48;
    Pulse_filename[6] = digit6 + 48;
    Pulse_filename[5] = digit5 + 48;

    //////////////////////////////////////////////////////setupPablo()
    // now that we've constructed a filename, see if it already exists.

    // if the file doesn't already exist set a flag and bail out when we
    // reenter the loop.

    // Serial.print(F(">>> about to do an SD.exists for filename ")); 
    // Serial.println(Pulse_filename);

    if (!SD.exists(Pulse_filename)) {
      have_a_name = true;
      // Serial.print(F(">>>     file doesn't exist, so that is for us!")); 
    } else {
      // Serial.print(F(">>>     file exists, so keep going.")); 
    }
  }

  Serial.print("We will write to the SD file ");
  Serial.print(Pulse_filename);
  Serial.print(" and close/reopen it after every ");
  Serial.print(CLOSE_REOPEN_FILE);
  Serial.println(" spectrometer samples.");

  // now open the file. (close it this way: PulseFile.close();)
  PulseFile = SD.open(Pulse_filename, FILE_WRITE);

  // now write a header to the file.
  // line 1:
  PulseFile.print("Pulse Ox data file ");
  PulseFile.print(Pulse_filename);
  PulseFile.print(" code file ");
  PulseFile.println(__FILE__);

  // line 2:
  PulseFile.print("Code date/time of last compilation was ");
  PulseFile.print(__DATE__);
  PulseFile.print(" ");
  PulseFile.print(__TIME__);

  // get the RTC time too
  DateTime now = rtc.now();
  PulseFile.print("  DS3231 RTC date and time are ");
  PulseFile.print(now.year(), DEC);
  PulseFile.print('/');
  PulseFile.print(now.month(), DEC);
  PulseFile.print('/');
  PulseFile.print(now.day(), DEC);
  PulseFile.print(" (");
  PulseFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
  PulseFile.print(")   ");
  PulseFile.print(now.hour(), DEC);
  PulseFile.print(':');
  if(now.minute() < 10) {PulseFile.print("0");}
  PulseFile.print(now.minute(), DEC);
  PulseFile.print(':');
  if(now.second() < 10) {PulseFile.print("0");}
  PulseFile.print(now.second(), DEC);

  PulseFile.print(". temperature: ");
  PulseFile.print(rtc.getTemperature());
  PulseFile.print("*C or ");
  PulseFile.print(1.8 * rtc.getTemperature() + 32.);
  PulseFile.println("*F");

  // line 3:
  PulseFile.print("N,");
  PulseFile.print("t (ms),");
  PulseFile.print("NIR I,");
  PulseFile.print("white I,");
  PulseFile.print("pulse ox light I,");
  PulseFile.print("white light I,");

  PulseFile.print("pulse ox 415,");
  PulseFile.print("pulse ox 445,");
  PulseFile.print("pulse ox 480,");
  PulseFile.print("pulse ox 515,");
  PulseFile.print("pulse ox clear0,");
  PulseFile.print("pulse ox NIR0,");
  PulseFile.print("pulse ox 555,");
  PulseFile.print("pulse ox 590,");
  PulseFile.print("pulse ox 630,");
  PulseFile.print("pulse ox 680,");
  PulseFile.print("pulse ox clear,");
  PulseFile.print("pulse ox NIR,");
  
  PulseFile.print("skin color 415,");
  PulseFile.print("skin color 445,");
  PulseFile.print("skin color 480,");
  PulseFile.print("skin color 515,");
  PulseFile.print("skin color clear0,");
  PulseFile.print("skin color NIR0,");
  PulseFile.print("skin color 555,");
  PulseFile.print("skin color 590,");
  PulseFile.print("skin color 630,");
  PulseFile.print("skin color 680,");
  PulseFile.print("skin color clear,");
  PulseFile.print("skin color NIR,");

  PulseFile.println("magic date");

  Serial.println("Now do a TFT test");

  // Fill screen blue. Not a required step, this just shows that we're
  // successfully communicating with the screen.
  tft.fillScreen(ST7735_BLUE);

  return;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// A small helper
void error(const __FlashStringHelper*err) 
{
  Serial.println(err);
  while (1);
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Blink the red LED
void blink_LED(uint32_t blink_ms)
{
  // blink the LED for the specified number of milliseconds

  uint32_t start_time = millis();

  // make sure the LED is off, initially.
  digitalWrite(RED_LED_PIN, LED_OFF);

  while(millis() - start_time < blink_ms)
  {
    digitalWrite(RED_LED_PIN, LED_ON);
    delay(50);
    digitalWrite(RED_LED_PIN, LED_OFF);
    delay(50);
  }
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void load_TFT_display() 
{
  // put info into the 128 x 160 TFT display.
  // size 1 text characters are 5 pixels wide by 8 high,
  // including tails on lower case g, etc.

  // hard code stuff for the sake of quick code creation (sorry!)

  /* new format for pulse ox display:
  lines 1 - 8: "Skin color 2 - 8 + NIR"
  line 9: blank
  lines 10 - 17: skin color channels 2 - 5
  line 18: blank
  lines 19 - 26: skin color channels 6 - 8 and NIR
  line 27: blank

  lines 28 - 35: "Pulse ox 2 - 8 + NIR"
  line 36: blank
  lines 37 - 44: pulse ox channels 2 - 5
  line 45: blank
  lines 46 - 53: pulse ox channels 6 - 8 and NIR
  line 54: blank

  lines 55 - 62: time, date, e.g. "12:15:31 2024/01/15"
  line 63: blank
*/
  // some definitions here:
  #define new_character_height 8
  #define new_x_origin 0
  #define new_x_middle 64
  #define new_x_rightside 127
  #define new_y_origin 0
  // 5-6-5 RGB. Make this very dark gray: 0b 00011 000011 00011 = 0b 0001 1000 0110 0011 = 0x1863
  #define new_screen_background 0X1863
  // #define new_screen_background 0XFFFFFF
  // #define new_title_color 0X000000
  #define new_title_color ST77XX_YELLOW
  #define new_pixel_color ST77XX_ORANGE
  #define new_spacer_pixels 1

  // number of times this routine has been called
  tft_loads++;

  // background...
  // tft.fillScreen(ST77XX_BLACK);
  // tft.fillScreen(TFT_IVORY);
  // tft.fillScreen(ST77XX_WHITE);
  tft.fillScreen(new_screen_background);

  // no wrapping
  tft.setTextWrap(false);
  // set text size
  tft.setTextSize(1);
  current_y_pixel = new_y_origin;

  ////////////////////////////////////////////
  // skin color AS7341 values
  ////////////////////////////////////////////

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
  // tft.print(as7341_SKIN_COLOR_readings[1]);
  // tft.print(" ");

  // ADC1/F2 445nm
  tft.setTextColor(AS7341_F2_TFT_COLOR);
  tft.print(as7341_SKIN_COLOR_readings[1]);
  tft.print(" ");

  // ADC2/F3 480nm
  tft.setTextColor(AS7341_F3_TFT_COLOR);
  tft.print(as7341_SKIN_COLOR_readings[2]);
  tft.print(" ");

  // ADC3/F4 515nm
  tft.setTextColor(AS7341_F4_TFT_COLOR);
  tft.print(as7341_SKIN_COLOR_readings[3]);
  tft.print(" ");

  // ADC0/F5 555nm
  tft.setTextColor(AS7341_F5_TFT_COLOR);
  tft.println(as7341_SKIN_COLOR_readings[6]);

  // we've just started a new line.
  current_y_pixel = current_y_pixel + new_character_height;
  tft.setCursor(new_x_origin, current_y_pixel);

  // ADC1/F6 590nm
  tft.setTextColor(AS7341_F6_TFT_COLOR);
  tft.print(as7341_SKIN_COLOR_readings[7]);
  tft.print(" ");

  // ADC2/F7 630nm
  tft.setTextColor(AS7341_F7_TFT_COLOR);
  tft.print(as7341_SKIN_COLOR_readings[8]);
  tft.print(" ");

  // ADC3/F8 680nm
  tft.setTextColor(AS7341_F8_TFT_COLOR);
  tft.print(as7341_SKIN_COLOR_readings[9]);
  tft.print(" ");

  // ADC5/NIR near infrared
  tft.setTextColor(AS7341_NIR_TFT_COLOR);
  tft.println(as7341_SKIN_COLOR_readings[11]);

  ////////////////////////////////////////////
  // pulse ox AS7341 values
  ////////////////////////////////////////////

  // move down the screen
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  tft.setTextColor(new_title_color);
  tft.setCursor(new_x_origin + 1, current_y_pixel);
  tft.println("Pulse ox 2-8 + NIR");

  // move down the screen
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin, current_y_pixel);

  // ADC0/F1 415nm
  // tft.setTextColor(AS7341_F1_TFT_COLOR);
  // tft.print(as7341_PULSE_OX_readings[1]);
  // tft.print(" ");

  // ADC1/F2 445nm
  tft.setTextColor(AS7341_F2_TFT_COLOR);
  tft.print(as7341_PULSE_OX_readings[1]);
  tft.print(" ");

  // ADC2/F3 480nm
  tft.setTextColor(AS7341_F3_TFT_COLOR);
  tft.print(as7341_PULSE_OX_readings[2]);
  tft.print(" ");

  // ADC3/F4 515nm
  tft.setTextColor(AS7341_F4_TFT_COLOR);
  tft.print(as7341_PULSE_OX_readings[3]);
  tft.print(" ");

  // ADC0/F5 555nm
  tft.setTextColor(AS7341_F5_TFT_COLOR);
  tft.println(as7341_PULSE_OX_readings[6]);

  // we've just started a new line.
  current_y_pixel = current_y_pixel + new_character_height;
  tft.setCursor(new_x_origin, current_y_pixel);

  // ADC1/F6 590nm
  tft.setTextColor(AS7341_F6_TFT_COLOR);
  tft.print(as7341_PULSE_OX_readings[7]);
  tft.print(" ");

  // ADC2/F7 630nm
  tft.setTextColor(AS7341_F7_TFT_COLOR);
  tft.print(as7341_PULSE_OX_readings[8]);
  tft.print(" ");

  // ADC3/F8 680nm
  tft.setTextColor(AS7341_F8_TFT_COLOR);
  tft.print(as7341_PULSE_OX_readings[9]);
  tft.print(" ");

  // ADC5/NIR near infrared
  tft.setTextColor(AS7341_NIR_TFT_COLOR);
  tft.println(as7341_PULSE_OX_readings[11]);

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
    tft.println(Pulse_filename);
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

  tft.print(NIR_LED_current, 3);
  tft.print(" mA ");
  tft.print(white_LED_current, 3);
  tft.println(" mA");

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
  tft.println("Skin, pulse-ox lights");
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;
  tft.setCursor(new_x_origin + 1, current_y_pixel);

  tft.print(as7341_SKIN_COLOR_LIGHT_CURRENT);
  tft.print(" mA ");
  tft.print(as7341_PULSE_OX_LIGHT_CURRENT);
  tft.println(" mA");

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
  if(as7341_PULSE_OX_readings[9] > 0)
  {
    ratio_to_display = float(as7341_PULSE_OX_readings[index_NIR]) / 
    float(as7341_PULSE_OX_readings[index_680]);
  } else {
    ratio_to_display = 0;
  }
  tft.setCursor(new_x_middle + 2, current_y_pixel);
  tft.println(ratio_to_display, 3);
  current_y_pixel = current_y_pixel + new_character_height + new_spacer_pixels;

  ////////////////////////////////////////////
  // horizontal line, flanked by blank lines
  ////////////////////////////////////////////

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
  // we've only filled PULSE_OX_NIR_680_ratio[0] through 
  // PULSE_OX_NIR_680_ratio[times_into_loop - 1]. 
  // when we've filled the whole buffer, the earliest value is 
  // PULSE_OX_NIR_680_ratio[times_into_loop] and the latest is 
  // PULSE_OX_NIR_680_ratio[times_into_loop - 1].

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
        Serial.print(" "); Serial.print(moo); Serial.print("="); Serial.print(PULSE_OX_NIR_680_ratio[moo]);
        if(moo % 10 == 0) {Serial.print("\n");}
      }
      if(PULSE_OX_NIR_680_ratio[moo] > biggest_ratio) 
        {biggest_ratio = PULSE_OX_NIR_680_ratio[moo];}
      if(PULSE_OX_NIR_680_ratio[moo] < smallest_ratio) 
        {smallest_ratio = PULSE_OX_NIR_680_ratio[moo];}
      tft_ratio_array[index_tft] = PULSE_OX_NIR_680_ratio[moo];
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
        Serial.print(" "); Serial.print(moo); Serial.print("="); Serial.print(PULSE_OX_NIR_680_ratio[moo]);
        if(moo % 10 == 0) {Serial.print("\n");}
      }
      if(PULSE_OX_NIR_680_ratio[moo] > biggest_ratio) 
        {biggest_ratio = PULSE_OX_NIR_680_ratio[moo];}
      if(PULSE_OX_NIR_680_ratio[moo] < smallest_ratio) 
        {smallest_ratio = PULSE_OX_NIR_680_ratio[moo];}
      tft_ratio_array[index_tft] = PULSE_OX_NIR_680_ratio[moo];
      index_tft++;
    }

    for(int moo = 0; moo <= circular_index_last; moo++)
    {
      if(dump_stuff)
      {
        Serial.print(" "); Serial.print(moo); Serial.print("="); Serial.print(PULSE_OX_NIR_680_ratio[moo]);
        if(moo % 10 == 0) {Serial.print("\n");}
      }
      if(PULSE_OX_NIR_680_ratio[moo] > biggest_ratio) 
        {biggest_ratio = PULSE_OX_NIR_680_ratio[moo];}
      if(PULSE_OX_NIR_680_ratio[moo] < smallest_ratio) 
        {smallest_ratio = PULSE_OX_NIR_680_ratio[moo];}
      tft_ratio_array[index_tft] = PULSE_OX_NIR_680_ratio[moo];
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
    (ymax - ymin) + 1, ST7735_ORANGE);

  tft.drawFastHLine(new_x_origin + 11, ymax - 1, 100, ST7735_ORANGE);

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

////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int test_AS7341()
{
  // now setup, then test the two AS7341 light spectrometers.
  Serial.println("\nAS7341 spectrometers test");

  // two choices for the analog switch to choose which decvice ses I2C:
  // digitalWrite(SWITCH_I2C, DO_SKIN_COLOR);
  // or
  // digitalWrite(SWITCH_I2C, DO_PULSE_OX);

  // select the right spectrometer:
  digitalWrite(SWITCH_I2C, DO_SKIN_COLOR);
  // wait for things to settle
  delay(10);
  
  if (!as7341_SKIN_COLOR.begin())
  {
    Serial.println("Could not find as7341_SKIN_COLOR!");
    return -1;
  }
  
  // set some parameters
  as7341_SKIN_COLOR.setATIME(AS7341_ATIME);
  as7341_SKIN_COLOR.setASTEP(AS7341_ASTEP);
  as7341_SKIN_COLOR.setGain(AS7341_GAIN);

  // flash the breakout board's on-board light with different currents
  as7341_SKIN_COLOR.enableLED(true);
  int current_here = 5;
  int di = 5;
  for(int ijk = 0; ijk < 10; ijk++)
  {
    as7341_SKIN_COLOR.setLEDCurrent(current_here);
    as7341_SKIN_COLOR_LIGHT_CURRENT = current_here;
    current_here = current_here + di;
    delay(250);
  }
  // now turn the light off.
  as7341_SKIN_COLOR.enableLED(false);
  as7341_SKIN_COLOR_LIGHT_CURRENT = 0;
  
  // now read the device.

  if(DO_F1F4_CLEAR_NIR)
  {
    as7341_SKIN_COLOR.readF1F4_Clear_NIRChannels(as7341_SKIN_COLOR_readings);

  } else if (DO_F5F8_CLEAR_NIR) {
    as7341_SKIN_COLOR.readF5F8_Clear_NIRChannels(as7341_SKIN_COLOR_readings);

  } else {
    as7341_SKIN_COLOR.readAllChannels(as7341_SKIN_COLOR_readings);
  }

  Serial.println("AS7341 Skin Color spectrometer readings");
  Serial.print("ADC0/F1 415nm : ");
  Serial.println(as7341_SKIN_COLOR_readings[0]);
  Serial.print("ADC1/F2 445nm : ");
  Serial.println(as7341_SKIN_COLOR_readings[1]);
  Serial.print("ADC2/F3 480nm : ");
  Serial.println(as7341_SKIN_COLOR_readings[2]);
  Serial.print("ADC3/F4 515nm : ");
  Serial.println(as7341_SKIN_COLOR_readings[3]);
  Serial.print("ADC0/F5 555nm : ");

  /* 
  // we skip the first set of duplicate clear/NIR readings
  Serial.print("ADC4/Clear-");
  Serial.println(as7341_SKIN_COLOR_readings[4]);
  Serial.print("ADC5/NIR-");
  Serial.println(as7341_SKIN_COLOR_readings[5]);
  */
  
  Serial.println(as7341_SKIN_COLOR_readings[6]);
  Serial.print("ADC1/F6 590nm : ");
  Serial.println(as7341_SKIN_COLOR_readings[7]);
  Serial.print("ADC2/F7 630nm : ");
  Serial.println(as7341_SKIN_COLOR_readings[8]);
  Serial.print("ADC3/F8 680nm : ");
  Serial.println(as7341_SKIN_COLOR_readings[9]);
  Serial.print("ADC4/Clear    : ");
  Serial.println(as7341_SKIN_COLOR_readings[10]);
  Serial.print("ADC5/NIR      : ");
  Serial.println(as7341_SKIN_COLOR_readings[11]);

  // now select the other spectrometer.
  digitalWrite(SWITCH_I2C, DO_PULSE_OX);
  // wait for things to settle
  delay(10);
  
  if (!as7341_PULSE_OX.begin())
  {
    Serial.println("Could not find as7341_PULSE_OX!");
    return -2;
  }
  
  // set some parameters
  as7341_PULSE_OX.setATIME(AS7341_ATIME);
  as7341_PULSE_OX.setASTEP(AS7341_ASTEP);
  as7341_PULSE_OX.setGain(AS7341_GAIN);

  // flash the breakout board's on-board light with different currents
  as7341_PULSE_OX.enableLED(true);
  current_here = 5;
  di = 5;
  for(int ijk = 0; ijk < 10; ijk++)
  {
    as7341_PULSE_OX.setLEDCurrent(current_here);
    as7341_PULSE_OX_LIGHT_CURRENT = current_here;
    current_here = current_here + di;
    delay(250);
  }
  // now turn the light off.
  as7341_PULSE_OX.enableLED(false);
  as7341_PULSE_OX_LIGHT_CURRENT = 0;

  // now read the device.

  if(DO_F1F4_CLEAR_NIR)
  {
    as7341_PULSE_OX.readF1F4_Clear_NIRChannels(as7341_PULSE_OX_readings);

  } else if (DO_F5F8_CLEAR_NIR) {
    as7341_PULSE_OX.readF5F8_Clear_NIRChannels(as7341_PULSE_OX_readings);

  } else {
    as7341_PULSE_OX.readAllChannels(as7341_PULSE_OX_readings);
  }

  Serial.println("AS7341 Pulse Oximeter Color spectrometer readings");
  Serial.print("ADC0/F1 415nm : ");
  Serial.println(as7341_PULSE_OX_readings[0]);
  Serial.print("ADC1/F2 445nm : ");
  Serial.println(as7341_PULSE_OX_readings[1]);
  Serial.print("ADC2/F3 480nm : ");
  Serial.println(as7341_PULSE_OX_readings[2]);
  Serial.print("ADC3/F4 515nm : ");
  Serial.println(as7341_PULSE_OX_readings[3]);
  Serial.print("ADC0/F5 555nm : ");

  /* 
  // we skip the first set of duplicate clear/NIR readings
  Serial.print("ADC4/Clear-");
  Serial.println(as7341_PULSE_OX_readings[4]);
  Serial.print("ADC5/NIR-");
  Serial.println(as7341_PULSE_OX_readings[5]);
  */
  
  Serial.println(as7341_PULSE_OX_readings[6]);
  Serial.print("ADC1/F6 590nm : ");
  Serial.println(as7341_PULSE_OX_readings[7]);
  Serial.print("ADC2/F7 630nm : ");
  Serial.println(as7341_PULSE_OX_readings[8]);
  Serial.print("ADC3/F8 680nm : ");
  Serial.println(as7341_PULSE_OX_readings[9]);
  Serial.print("ADC4/Clear    : ");
  Serial.println(as7341_PULSE_OX_readings[10]);
  Serial.print("ADC5/NIR      : ");
  Serial.println(as7341_PULSE_OX_readings[11]);

  Serial.println("=============================");

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int reread_AS7341(bool OnlyPulseOx, bool verbose)
{
  // now reread and echo (if verbose is true) the two AS7341 light 
  // spectrometers.

  int indx;

  // do I want to read the pulse ox device, but not the skin color device?
  if(!OnlyPulseOx)
  {
    // select the skin color spectrometer:
    digitalWrite(SWITCH_I2C, DO_SKIN_COLOR);
    
    // now read the device.
    if(DO_F1F4_CLEAR_NIR)
    {
      as7341_SKIN_COLOR.readF1F4_Clear_NIRChannels(as7341_SKIN_COLOR_readings);

    } else if (DO_F5F8_CLEAR_NIR) {
      as7341_SKIN_COLOR.readF5F8_Clear_NIRChannels(as7341_SKIN_COLOR_readings);

    } else {
      as7341_SKIN_COLOR.readAllChannels(as7341_SKIN_COLOR_readings);
    }

    // also load data into a circular buffer.
    indx = SKIN_COLOR_index_last_read + 1;
    if(indx >= max_buffer_readings) {indx = 0;}

    for(int indx2 = 0; indx2 < max_buffer_channels; indx2++)
    {
      as7341_SKIN_COLOR_readings_buffer[indx][indx2] = as7341_SKIN_COLOR_readings[indx2];
    }
    SKIN_COLOR_index_last_read = indx;
  }

  // now select the pulse ox spectrometer.
  digitalWrite(SWITCH_I2C, DO_PULSE_OX);

  if(DO_F1F4_CLEAR_NIR)
  {
    as7341_PULSE_OX.readF1F4_Clear_NIRChannels(as7341_PULSE_OX_readings);

  } else if (DO_F5F8_CLEAR_NIR) {
    as7341_PULSE_OX.readF5F8_Clear_NIRChannels(as7341_PULSE_OX_readings);

  } else {
    as7341_PULSE_OX.readAllChannels(as7341_PULSE_OX_readings);
  }  
  
  // also load data into a circular buffer.
  indx = PULSE_OX_index_last_read + 1;
  if(indx >= max_buffer_readings) {indx = 0;}

  for(int indx2 = 0; indx2 < max_buffer_channels; indx2++)
  {
    as7341_PULSE_OX_readings_buffer[indx][indx2] = as7341_PULSE_OX_readings[indx2];
  }
  PULSE_OX_index_last_read = indx;
  // also calculate (and store) the ratio of the 680 nm kand NIR signals.
  if(as7341_PULSE_OX_readings_buffer[indx][index_680] > 0)
  {
    PULSE_OX_NIR_680_ratio[indx] = 
      float(as7341_PULSE_OX_readings_buffer[indx][index_NIR]) /
      float(as7341_PULSE_OX_readings_buffer[indx][index_680]);
  } else {
    PULSE_OX_NIR_680_ratio[indx] = 0.;
  }

  // now dump info if user wants this.
  if(verbose)
  {
    Serial.println("information from skin color as7341, from reread_AS7341");
    // select the skin color spectrometer:
    digitalWrite(SWITCH_I2C, DO_SKIN_COLOR);
    delay(2);

    Serial.print("Current limit (mA) for this device's LED: ");
    Serial.println(as7341_SKIN_COLOR.getLEDCurrent());

    Serial.println("information from pulse ox as7341...");
    // select the skin color spectrometer:
    digitalWrite(SWITCH_I2C, DO_PULSE_OX);
    delay(2);

    Serial.print("Current limit (mA) for this device's LED: ");
    Serial.println(as7341_PULSE_OX.getLEDCurrent());

    Serial.println("AS7341 Skin Color and Pulse Oximeter readings ");

    Serial.print("ADC0/F1 415nm : ");
    int ichan = 0;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("ADC1/F2 445nm : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("ADC2/F3 480nm : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("ADC3/F4 515nm : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("ADC0/F5 555nm : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    /* 
    // we skip the first set of duplicate clear/NIR readings
    Serial.print("ADC4/Clear-");
    Serial.println(as7341_SKIN_COLOR_readings[4]);
    Serial.print("ADC5/NIR-");
    Serial.println(as7341_SKIN_COLOR_readings[5]);
    */
    ichan++;
    ichan++;
    
    Serial.print("ADC1/F6 590nm : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("ADC2/F7 630nm : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("ADC3/F8 680nm : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("ADC4/Clear    : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("ADC5/NIR      : ");
    ichan++;
    Serial.print(as7341_SKIN_COLOR_readings[ichan]);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_readings[ichan]);

    Serial.print("Ratio of NIR / 680 nm: ");
    Serial.println(PULSE_OX_NIR_680_ratio[PULSE_OX_index_last_read], 3);

  }
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int set_LED_DACs(uint16_t NIR_LED_DAC_VALUE, uint16_t WHITE_LED_DAC_VALUE, bool verbose)
{
  // first argument is NIR LED, second is white LED.
  // verbose mode?

  // check that the DAC value is legal.
  if(NIR_LED_DAC_VALUE >= DAC_MAX || WHITE_LED_DAC_VALUE >= DAC_MAX) 
  {
    Serial.print("Illegal DAC value(s): ");
    Serial.print(NIR_LED_DAC_VALUE);
    Serial.print(" ");
    Serial.println(WHITE_LED_DAC_VALUE);
    return -1;
  }

  // so far so good! set DACs and wait a bit.

  mcp.setChannelValue(NIR_LED_DAC_CHANNEL, NIR_LED_DAC_VALUE);
  mcp.setChannelValue(WHITE_LED_DAC_CHANNEL, WHITE_LED_DAC_VALUE);

  NIR_LED_ADC_VALUE = analogRead(NIR_LED_ADC_CHANNEL);
  WHITE_LED_ADC_VALUE = analogRead(WHITE_LED_ADC_CHANNEL);

  LED_voltage_NIR = ADC_VREF * float(NIR_LED_ADC_VALUE) / float(ADC_MAX); 
  DAC_voltage_NIR = NIR_LED_DAC_VALUE * float(DAC_VREF) / float(DAC_MAX);
  LED_voltage_WHITE = ADC_VREF * float(WHITE_LED_ADC_VALUE) / float(ADC_MAX); 
  DAC_voltage_WHITE = WHITE_LED_DAC_VALUE * float(DAC_VREF) / float(DAC_MAX);

  NIR_LED_current = 1000 * (DAC_voltage_NIR - LED_voltage_NIR) / float(R_NIR);
  white_LED_current = 1000 * (DAC_voltage_WHITE - LED_voltage_WHITE) / float(R_WHITE);

  // if we want, now report what we got:

  if(verbose)
  {
    Serial.print("\nJust set NIR and White LED DACs to ");
    Serial.print(NIR_LED_DAC_VALUE);
    Serial.print(" ");
    Serial.println(WHITE_LED_DAC_VALUE);

    Serial.print("Correspondoing voltages are ");                                                                  
    Serial.print(NIR_LED_DAC_VALUE * float(DAC_VREF) / float(DAC_MAX), 3);
    Serial.print(" and  ");
    Serial.print(WHITE_LED_DAC_VALUE * float(DAC_VREF) / float(DAC_MAX), 3);
    Serial.println(" volts.");
    
    Serial.print("NIR and White LEDs are fed through ");
    Serial.print(R_NIR);
    Serial.print(" and ");
    Serial.print(R_WHITE);
    Serial.println(" ohm resistors.");

    Serial.print("NIR LED ADC readback value = ");
    Serial.print(NIR_LED_ADC_VALUE);
    Serial.print(" counts = ");
    Serial.print(LED_voltage_NIR, 3);
    Serial.println(" volts.");
    Serial.print("Current to this LED = ");
    Serial.print(NIR_LED_current);
    Serial.println(" mA.");

    Serial.print("White LED ADC readback value = ");
    Serial.print(WHITE_LED_ADC_VALUE);
    Serial.print(" counts = ");
    Serial.print(LED_voltage_WHITE, 3);
    Serial.println(" volts.");
    Serial.print("Current (mA) to this LED = ");
    Serial.print(white_LED_current);
    Serial.println(" mA.");
  }
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int set_breakout_lights(uint16_t skin_color_light, uint16_t pulse_ox_light, bool verbose)
{
  // set the currents to the lights on the AS7341 breakout boards

  // select the right spectrometer, then wait for things to settle:
  digitalWrite(SWITCH_I2C, DO_SKIN_COLOR);

  if(skin_color_light > 0)
  {
    as7341_SKIN_COLOR.enableLED(true);
    as7341_SKIN_COLOR.setLEDCurrent(skin_color_light);
    delay(5);
    as7341_SKIN_COLOR_LIGHT_CURRENT = as7341_SKIN_COLOR.getLEDCurrent();
  } else {
    as7341_SKIN_COLOR.enableLED(false);
    as7341_SKIN_COLOR_LIGHT_CURRENT = 0;
  }

  // now select the other spectrometer.
  digitalWrite(SWITCH_I2C, DO_PULSE_OX);
  
  if(pulse_ox_light > 0)
  {
    as7341_PULSE_OX.enableLED(true);
    as7341_PULSE_OX.setLEDCurrent(pulse_ox_light);
    delay(5);
    as7341_PULSE_OX_LIGHT_CURRENT = as7341_PULSE_OX.getLEDCurrent();
  } else {
    as7341_PULSE_OX.enableLED(false);
    as7341_PULSE_OX_LIGHT_CURRENT = 0;
  }
  // now echo what we got.

  if(verbose)
  {
    Serial.println("AS7341 Skin Color and Pulse Oximeter breakout light currents (mA)");
    Serial.print(as7341_SKIN_COLOR_LIGHT_CURRENT);
    Serial.print("  ");
    Serial.println(as7341_PULSE_OX_LIGHT_CURRENT);
  }

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int write_SD_data()
{
  // write data to SD, closing and reopening the file periodically.

  // this is a CSV file so...
  // sequence number
  PulseFile.print(times_into_loop); PulseFile.print(",");

  // elapsed time in milliseconds
  PulseFile.print(millis() - t_start); PulseFile.print(",");

  // LED currents
  PulseFile.print(NIR_LED_current, 3); PulseFile.print(",");
  PulseFile.print(white_LED_current, 3); PulseFile.print(",");
  PulseFile.print(as7341_PULSE_OX_LIGHT_CURRENT); PulseFile.print(",");
  PulseFile.print(as7341_SKIN_COLOR_LIGHT_CURRENT); PulseFile.print(",");

  // pulse ox AS7341 data
  for(int indx2 = 0; indx2 < max_buffer_channels; indx2++)
  {
    PulseFile.print(as7341_PULSE_OX_readings[indx2]);  PulseFile.print(",");
  }

  // skin color AS7341 data
  for(int indx2 = 0; indx2 < max_buffer_channels; indx2++)
  {
    PulseFile.print(as7341_SKIN_COLOR_readings[indx2]);  PulseFile.print(",");
  }
  // last entry
  PulseFile.println(9188);

  // we want to close/reopen the file periodically.
  if(times_into_loop % CLOSE_REOPEN_FILE == 0)
  {
    PulseFile.close();
    PulseFile = SD.open(Pulse_filename, FILE_WRITE);
  }

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
