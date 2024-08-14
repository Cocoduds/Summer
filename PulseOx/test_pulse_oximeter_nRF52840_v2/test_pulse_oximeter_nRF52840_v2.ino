/*******************************************************************************
  This program is test_pulse_oximeter_nRF52840.ino. Test hardware (except, 
  so far, the Bluetooth module) in the circuit test_pulse_oximeter_nRF52840.

  Processor is an ItsyBitsy nRF52840 with an onboard BluetoothLE module.

  I use an Adafruit Pro Trinket LiPoly/LiIon Backpack to feed battery power 
  to the device. Note that the nRF's RESET buttton is hidden below the 
  battery packpage breakout. You can reach it with a flat-bladed jeweler's
  screwdriver, inserted under the "Li" in the "LiPoly" silkscreen label. 

  Note that you may need to update the nRF's bootloader. See 
    https://learn.adafruit.com/adafruit-itsybitsy-nrf52840-express/update-bootloader

  Here's what I found in the nRF's INFO_UF2.TXT file, intially, when I
  opened the microcontroller as a disk drive:

      UF2 Bootloader 0.2.13 lib/nrfx (v1.1.0-1-g096e770) lib/tinyusb (legacy-755-g55874813) s140 6.1.1
      Model: Adafruit ItsyBitsy nRF52840 Express
      Board-ID: nRF52840-ItsyBitsy-revA
      Date: Sep 25 2019

  Bootloader version 0.2.13 is too old to work properly.

  Update using the command line approach descriibed by Adafruit. See
  https://learn.adafruit.com/adafruit-itsybitsy-nrf52840-express/update-bootloader-use-command-line

  To avoid interference with the load, you'll have to quit from all
  applications (including the Arduino IDE) that use the serial port.

  Put the necessary files into a folder such as 
    /Users/g-gollin/SuperWoodchuckie2/Physics_education/physics_398dlp/Arduino_code/_bootloader

  On a mac, open terminal, then do a 
    cd /Users/g-gollin/SuperWoodchuckie2/Physics_education/physics_398dlp/Arduino_code/_bootloader

  The bootloader zip package I download from Adafruit is
    itsybitsy_nrf52840_express_bootloader-0.8.2_s140_6.1.1.zip

  Also download
    adafruit-nrfutil--0.5.3.post17-macos.zip
  and extract adafruit-nrfutil to the file adafruit-nrfutil, then do a 
    chmod +x adafruit-nrfutil
  to make it executable.

  Find the nRF device by doing a 
    ls /dev/cu.*
  to find it is
    /dev/cu.usbmodem141101

  Quit out of the Arduino IDE if it's open, then do this (all on one line):
    ./adafruit-nrfutil --verbose dfu serial --package itsybitsy_nrf52840_express_bootloader-0.8.2_s140_6.1.1.zip -p /dev/cu.usbmodem141101 -b 115200 --singlebank --touch 1200

  Messages that show on the terminal window:

    (base) g-gollin@MacBook-Pro-576 _bootloader % ./adafruit-nrfutil --verbose dfu serial --package itsybitsy_nrf52840_express_bootloader-0.8.2_s140_6.1.1.zip -p /dev/cu.usbmodem141101 -b 115200 --singlebank --touch 1200
    Upgrading target on /dev/cu.usbmodem141101 with DFU package /Users/g-gollin/SuperWoodchuckie2/Physics_education/physics_398dlp/Arduino_code/_bootloader/itsybitsy_nrf52840_express_bootloader-0.8.2_s140_6.1.1.zip. Flow control is disabled, Single bank, Touch 1200
    Touched serial port /dev/cu.usbmodem141101
    Opened serial port /dev/cu.usbmodem141101
    Starting DFU upgrade of type 3, SoftDevice size: 151016, bootloader size: 39728, application size: 0
    Sending DFU start packet
    Sending DFU init packet
    Sending firmware file
    ########################################
    ########################################
    ########################################
    ########################################
    ########################################
    ########################################
    ########################################
    ########################################
    ########################################
    #############
    Activating new firmware

    DFU upgrade took 20.7058048248291s
    Device programmed.
    (base) g-gollin@MacBook-Pro-576 _bootloader % 

  Now looking at INFO_UF2.TXT, I find it contains this:

    UF2 Bootloader 0.8.2 lib/nrfx (v2.0.0) lib/tinyusb (0.12.0-145-g9775e7691) lib/uf2 (remotes/origin/configupdate-9-gadbb8c7)
    Model: Adafruit ItsyBitsy nRF52840 Express
    Board-ID: nRF52840-ItsyBitsy-revA
    Date: Jan  6 2024
    SoftDevice: S140 6.1.1

  So bootloader has been updated!

  George Gollin
  University of Illinois
  February 2023.
*******************************************************************************/

// Libraries...
#include <SPI.h>
#include <Adafruit_Sensor.h>  
// #include "Adafruit_BLE.h"
// #include "Adafruit_BluefruitLE_SPI.h"
// #include "Adafruit_BluefruitLE_UART.h"
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
#include <Wire.h>
// Core graphics library
#include <Adafruit_GFX.h>    
// Hardware-specific library for TFT display
// #include <Adafruit_ST7735.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_ImageReader.h> // Image-reading functions
// Time libraries
#include <Time.h>
#include <Adafruit_I2CDevice.h>
// #include <Arduino.h>
// #include <SD.h>
#include <SdFat.h>                // SD card & FAT filesystem library
// #include "FsDateTime.h"
#include <Adafruit_AS7341.h>
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

/////////////// TFT display, perhaps including its microSD carrier //////////

// I am using an Adafruit 1.14" 240x135 Color TFT Display 
// with a MicroSD Card carrier. (part ID 4383)

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
// KEEP THIS FIXED AT 100!
#define max_buffer_readings 100
#define max_buffer_channels 12
#define index_NIR 11
#define index_680 9

float AS7341_NIR_680_ratio[max_buffer_readings];
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
SdFat                SD;         // SD card filesystem
Adafruit_ImageReader reader(SD); // Image-reader object, pass in SD filesys
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

/////////// AS7341 visible light + near IR 10-band spectrometer ////////////////

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

Adafruit_AS7341 as7341_spectrometer;
uint16_t as7341_spectrometer_readings[12];

// current (mA) to AS7341's headlight
int as7341_LIGHT_CURRENT;

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

/////////////////// DS3231 RTC stuff //////////////////////

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//////////////////////////// LED stuff //////////////////////

#define LED_OFF LOW
#define LED_ON HIGH

// LEDs: red is just for informing the user of stuff.
#define RED_LED_PIN 2

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() 
{
  // light up the serial monitor and wait for it to be ready.
  uint32_t t1234 = millis();
  Serial.begin(115200);
  while (!Serial && millis() - t1234 < 5000) {}

  Serial.println("\n*************** nRF95480 pulse ox test ***************");

  Serial.print("This file is " );
  Serial.println(__FILE__);

  Serial.print("Date and time of compilation: ");
  Serial.print(__DATE__);
  Serial.print(", ");
  Serial.println(__TIME__);

  // set the LED-illuminating pin to be a digital output.
  pinMode(RED_LED_PIN, OUTPUT);

  // make sure the LED is off.
  digitalWrite(RED_LED_PIN, LED_OFF);

  // set ADC precision to 12 bits.
  analogReadResolution(12);

  // set various SPI device chip select lines to HIGH to
  // make sure the devices aren't active right now.

  // TFT display and its built-in SD carrier
  pinMode(SPI_TFT_CS, OUTPUT);
  digitalWrite(SPI_TFT_CS, HIGH);
  pinMode(SPI_SD_CS, OUTPUT);
  digitalWrite(SPI_SD_CS, HIGH);

  ///////////////////// setup pulse ox device /////////////////////

  Wire.begin();
  int result = bioHub.begin();
  if (result == 0) 
  {
    Serial.println("Pulse of sensor started. Put your finger on it, please.");
  } else {
    Serial.println("Could not communicate with the pulse ox so bail out");
    while(1){}
  }

  Serial.println("Configuring pulse ox sensor...."); 

  // Configuring just the BPM settings. 
  // int error = bioHub.configBpm(MODE_ONE);
  int error = bioHub.configBpm(MODE_TWO);  
  if(error == 0)
  {
    Serial.println("Sensor configured.");
  } else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }
  // Data lags a bit behind the sensor, if you're finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up. 
  Serial.println("Loading up the buffer with data... wait four seconds please.");
  delay(4000); 

  // now read it a few times.
  for (int iread =1; iread < 11; iread++)
  {
    body = bioHub.readBpm();
    Serial.print("Read # ");
    Serial.print(iread);
    Serial.print(" Heartrate (BPM): ");
    Serial.print(body.heartRate); 
    Serial.print("  Confidence: ");
    Serial.print(body.confidence); 
    Serial.print("  Oxygenation (%): ");
    Serial.print(body.oxygen); 
    Serial.print("  Status: ");
    Serial.println(body.status); 
    delay(250); 
  }
  Serial.println("SparkFun pulse ox all set.");

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
    Serial.println("\nRTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  } else {
    Serial.print("\nRTC has remained powered since last adjustment, so do not reset it.\nCurrent date and time: ");
  }

  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if(now.minute() < 10) {Serial.print("0");}
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if(now.second() < 1) {Serial.print("0");}
  if(now.second() < 10) {Serial.print("0");}
  Serial.print(now.second(), DEC);
  Serial.println();

  Serial.print("elapsed time since midnight 1/1/1970 = ");
  Serial.print(now.unixtime());
  Serial.print(" seconds = ");
  Serial.print(now.unixtime() / 86400L);
  Serial.println(" days");

  // calculate a date which is 7 days, 12 hours, 30 minutes, 6 seconds into the future
  DateTime future (now + TimeSpan(7,12,30,6));

  Serial.print("current time + 7d + 12h + 30m + 6s: ");
  Serial.print(future.year(), DEC);
  Serial.print('/');
  Serial.print(future.month(), DEC);
  Serial.print('/');
  Serial.print(future.day(), DEC);
  Serial.print(' ');
  Serial.print(future.hour(), DEC);
  Serial.print(':');
  Serial.print(future.minute(), DEC);
  Serial.print(':');
  Serial.print(future.second(), DEC);
  Serial.println();

  Serial.print("DS3231 temperature reading: ");
  Serial.print(rtc.getTemperature());
  Serial.print("*C or ");
  Serial.print(1.8 * rtc.getTemperature() + 32.);
  Serial.println("*F");

  ///////////////////// setup and initialize SD and TFT /////////////////////

  setup_SD_TFT();

  ////////////////////////////////////////////////////////////
  // Initialise the BluetoothLE (BLE) module 
  ////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////
  // Do some AS7341 spectrometer tests.
  ////////////////////////////////////////////////////////////

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

  ////////////////////////////////////////////////////////////
  // try reading AS7341 channels, but perhaps not all of them.
  ////////////////////////////////////////////////////////////

  Serial.println("Now do an AS7341 speed test.");
  uint32_t tstart, tstop;
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
  tstart = millis();

  for(int itimes = 0; itimes < max_to_do; itimes++)
  {
    as7341_spectrometer.readAllChannels(as7341_spectrometer_readings);
    datum = as7341_spectrometer.readChannel(AS7341_ADC_CHANNEL_3);
  }
  tstop = millis();

  Serial.print("meatball attempt last: ");
  Serial.println(datum);

  Serial.print("Just read AS7341 ");
  Serial.print(max_to_do);
  Serial.print(" times in ");
  Serial.print(tstop - tstart);
  Serial.println(" milliseconds");

  // Serial.print(F("Loading roseMS.bmp to screen (2)..."));
  // stat_stat = reader.drawBMP("/roseMS.bmp", tft, 0, 0);
  // reader.printStatus(stat_stat);   

  ////////// all done with setup! ////////// 
  Serial.println("*****************************\n...now going into loop.");
  Serial.println("Enter stuff in the UART module on your smartphone.\n");

}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void loop() 
{
  // bump a counter
  times_into_loop++;

  // read pulse ox.
    body = bioHub.readBpm();
    Serial.print("Read # ");
    Serial.print(times_into_loop);
    Serial.print(" Heartrate (BPM): ");
    Serial.print(body.heartRate); 
    Serial.print("  Confidence: ");
    Serial.print(body.confidence); 
    Serial.print("  Oxygenation (%): ");
    Serial.print(body.oxygen); 
    Serial.print("  Status: ");
    Serial.println(body.extStatus); 

  // read and echo the spectrometer.
  reread_AS7341();

  uint16_t datum = as7341_spectrometer.readChannel(AS7341_ADC_CHANNEL_3);

  Serial.print("meatball attempt: ");
  Serial.println(datum);
  load_TFT_display();
  delay(1000); 

  /*

  // Check for user input from the serial monitor input field
  // to ve sent to the bluetooth module
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
  // that's it!
  */
}

////////////////////////////////////////////////////////////////////////////
void setup_SD_TFT(void) 
{
  bool debug_print = true;

  Serial.println("\n>>>>> Inside setup_SD_TFT()...");

  // set up the backlight line... 
  // see https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/ 
  // analogWrite(BACKLIGHT, TFT_BACKLIGHT_PWM);
  analogWrite(BACKLIGHT, TFT_BACKLIGHT_PWM_MAX);
  
  // Initialize screen after resetting the TFT breakout by pulling the reset line low.
  pinMode(TFT_RESET, OUTPUT);
  digitalWrite(TFT_RESET, LOW);
  delay(100);
  digitalWrite(TFT_RESET, HIGH);
  delay(100);
  
  // if the plastic film protecting the screen has a black tab, use this:
  tft.init(135, 240); 

    // set up the backlight line... 
  analogWrite(BACKLIGHT, TFT_BACKLIGHT_PWM_MAX);
  
  // now rotate the screen.
  if(debug_print)
  {
    Serial.print("Initial rotation parameter value is ");
    Serial.println(tft.getRotation());
    Serial.println("Now increment parameter by 1.");
  }
  tft.setRotation(tft.getRotation()+1);
  if(debug_print)
  {
    Serial.print("Now parameter is ");
    Serial.println(tft.getRotation());
    Serial.println("Now write a large block of text.");
  }

  // create a large block of text to display.

  // build the long character array. start with a string...
  //                                    111111111122222222223333333333444444444455555555556
  //                           123456789012345678901234567890123456789012345678901234567890
  String long_string =        "1234567890123456789012345678901234567890";
  long_string = long_string + " Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing";  
  long_string = long_string + " ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed ";
  long_string = long_string + "malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante";
  long_string = long_string + " pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa";
  long_string = long_string + " eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor ";
  long_string = long_string + "imperdiet posuere. ";

  // now build the character array, including space for a null at the end.
  int long_string_len = long_string.length() + 1;
  char long_char[long_string_len];

  // now load the character array from the string.
  long_string.toCharArray(long_char, long_string_len);

  // now print with yellow text on a black screen. allow ytext to wrap around.
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextWrap(true); 
  tft.setTextSize(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  // tft.println(long_string);
  tft.print(long_string);

  if(debug_print)
  {
    // now ask for the cursor position.
    Serial.print("TFT cursor x, y are now ");
    Serial.print(tft.getCursorX());
    Serial.print(", ");
    Serial.println(tft.getCursorY());
    delay(1000);
  }
  Serial.println("done with TFT tests");

  // The Adafruit_ImageReader constructor call (before setup())
  // accepts an uninitialized SdFat or FatFileSystem object. This MUST
  // BE INITIALIZED before using any of the image reader functions!
  Serial.print(F("Now initialize the SD file system..."));

  // SD card is pretty straightforward, a single call...
  if(!SD.begin(SPI_SD_CS, SD_SCK_MHZ(10))) 
  { 
    // Adafruit says breakouts require 10 MHz limit due to longer wires
    Serial.println(F("SD begin() failed so fall into a hole."));
    while(1) {} 
  }

  Serial.println(F(" file system initialization went OK!"));

  // let's try writing/reading to a file before messing with the BMP images.
  
  Serial.println("Now remove testTFT.txt if it exists.");
  if(SD.exists("testTFT.txt"))
  {
    SD.remove("testTFT.txt");
    Serial.println("Found, then removed testTFT.txt");
  } else {
    Serial.println("testTFT.txt does not exist yet.");
  }

  delay(100);

  Serial.println("Now test SD file write/read: file name is testTFT.txt.");
  // open the file. 
  myFile = SD.open("testTFT.txt", FILE_WRITE);

  delay(100);

  // if the file opened okay, write to it:
  if (myFile) 
  {
    Serial.println("Writing to testTFT.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.println(__FILE__);
    myFile.print(__DATE__);
    myFile.print(", ");
    myFile.println(__TIME__);
    myFile.println("//// end of file writing ////");

    Serial.println("We tried to write the following to SD: "); 
    Serial.println("testing 1, 2, 3.");
    Serial.println(__FILE__);
    Serial.print(__DATE__);
    Serial.print(", ");
    Serial.println(__TIME__);
    Serial.println("//// end of file writing ////");
    
    // close the file:
    myFile.close();
    Serial.println("Done with file writing. So far so good.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening testTFT.txt for writing.");
  }

  // re-open the file for reading:
  Serial.print("Now read the file. ");
  myFile = SD.open("testTFT.txt");

  if (myFile) 
  {
    Serial.println("testTFT.txt contents:");
    // read from the file until there's nothing else in it:
    while (myFile.available()) 
    {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
    delay(100);

    Serial.println("\nJust closed the TFT SD file.");

  } else {

    // if the file didn't open, print an error:
    Serial.println("error opening testTFT.txt");
  }

  // Fill screen blue. Not a required step, this just shows that we're
  // successfully communicating with the screen.
  // tft.fillScreen(ST77XX_BLUE);

  /////////////////////


  /////////////////////

  Serial.println(">>> about to call load_TFT_display from inside setup_SD_TFT.");
  load_TFT_display();

  Serial.println(">>>>> Now leaving setup_SD_TFT().\n");

  return;
}

/////////////////////////////////////////////////////////////////////

// Blink the red LED
void blink_LED(uint32_t blink_ms)
{
  // blink the LED for the specofied number of milliseconds

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

////////////////////////////////////////////////////////////////////////////

int test_AS7341()
{
  // now setup, then test the two AS7341 light spectrometers.
  Serial.println("\nAS7341 spectrometer test");

  
  if (!as7341_spectrometer.begin())
  {
    Serial.println("Could not find as7341_spectrometer!");
    return -1;
  }
  
  // set some parameters
  as7341_spectrometer.setATIME(AS7341_ATIME);
  as7341_spectrometer.setASTEP(AS7341_ASTEP);
  as7341_spectrometer.setGain(AS7341_GAIN);

  // now read the device.

  if (!as7341_spectrometer.readAllChannels(as7341_spectrometer_readings))
  {
    Serial.println("Error reading all as7341_spectrometer channels!");
    return -3;
  }

  Serial.println("AS7341 spectrometer readings");
  Serial.print("ADC0/F1 415nm : ");
  Serial.println(as7341_spectrometer_readings[0]);
  Serial.print("ADC1/F2 445nm : ");
  Serial.println(as7341_spectrometer_readings[1]);
  Serial.print("ADC2/F3 480nm : ");
  Serial.println(as7341_spectrometer_readings[2]);
  Serial.print("ADC3/F4 515nm : ");
  Serial.println(as7341_spectrometer_readings[3]);
  Serial.print("ADC0/F5 555nm : ");

  /* 
  // we skip the first set of duplicate clear/NIR readings
  Serial.print("ADC4/Clear-");
  Serial.println(as7341_spectrometer_readings[4]);
  Serial.print("ADC5/NIR-");
  Serial.println(as7341_spectrometer_readings[5]);
  */
  
  Serial.println(as7341_spectrometer_readings[6]);
  Serial.print("ADC1/F6 590nm : ");
  Serial.println(as7341_spectrometer_readings[7]);
  Serial.print("ADC2/F7 630nm : ");
  Serial.println(as7341_spectrometer_readings[8]);
  Serial.print("ADC3/F8 680nm : ");
  Serial.println(as7341_spectrometer_readings[9]);
  Serial.print("ADC4/Clear    : ");
  Serial.println(as7341_spectrometer_readings[10]);
  Serial.print("ADC5/NIR      : ");
  Serial.println(as7341_spectrometer_readings[11]);

  Serial.println("=============================");

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int reread_AS7341()
{
  // now reread and echo the two AS7341 light spectrometers.
  
  // now read the device.
  if (!as7341_spectrometer.readAllChannels(as7341_spectrometer_readings))
  {
    Serial.println("Error reading all as7341_spectrometer channels!");
    return -3;
  }

  // now try messing with the LED on the Adafruit breakout.
  as7341_spectrometer.enableLED(true);
  delay(1000);
  Serial.print("Current limit (mA) for this device's LED: ");
  Serial.println(as7341_spectrometer.getLEDCurrent());
  as7341_LIGHT_CURRENT = as7341_spectrometer.getLEDCurrent();

  Serial.print("Now change the value to 15mA: ");
  as7341_spectrometer.setLEDCurrent(15);
  Serial.print("Current limit (mA) for this device's LED is now ");
  Serial.println(as7341_spectrometer.getLEDCurrent());
  as7341_LIGHT_CURRENT = as7341_spectrometer.getLEDCurrent();

  // delay, then turn off.
  delay(1000);
  as7341_spectrometer.setLEDCurrent(0);
  as7341_spectrometer.enableLED(false);
  Serial.println(as7341_spectrometer.getLEDCurrent());
  as7341_LIGHT_CURRENT = as7341_spectrometer.getLEDCurrent();

  delay(1000);

  Serial.println("=============================");

  return 0;
}