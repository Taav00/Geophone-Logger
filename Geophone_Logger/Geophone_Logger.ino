/*
  Geophone Logger
  By: Taavet Kangur
  Based on some parts on Openlog Artemis by Nathan Seidle and Paul Clark
  Date: August 9th, 2023
  License: MIT. Please see LICENSE.md for more details.

  This firmware runs the OpenLog Artemis. A large variety of system settings can be
  adjusted by connecting at 115200bps.

  The Board should be set to SparkFun ESP32 IoT RedBoard

  
*/

#include <SparkFun_RV8803.h>                              // Click here to get the library: https://github.com/sparkfun/SparkFun_RV-8803_Arduino_Library
#include "SparkFun_ADS1015_Arduino_Library.h"             // Click here to get the library: https://github.com/sparkfun/SparkFun_ADS1015_Arduino_Library
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // Click here to get the library: https://github.com/sparkfun/SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library
#include <CircularBuffer.h>                               // Click here to get the library: https://github.com/rlogiacco/CircularBuffer   by AgileWare

#include <Wire.h>
#include <string.h>
#include "SdFat.h"
#include "SPI.h"

// library to disable watchdog timer
#include "soc/rtc_wdt.h"


// library to use WS2812 LED and parameters of the LED
#include <LiteLED.h>
#define LED_TYPE          LED_STRIP_WS2812
#define LED_TYPE_IS_RGBW  0  // if the LED is an RGBW type, change the 0 to 1
#define LED_BRIGHT        20
#define LED_PIN           2    // LED pins

static const crgb_t RED = 0xff0000;
static const crgb_t GREEN = 0x00ff00;
static const crgb_t BLUE = 0x0000ff;



// I2C settings
// Pins for I2C line 0 with RTC and Lipo monitor connected
#define PIN_SDA_0 21
#define PIN_SCL_0 22

// Pins for I2C line 0 with ADS1015s connected
#define PIN_SDA_1 16
#define PIN_SCL_1 17
#define I2C_CLOCK_SPEED 1000000  //1MHz

// ADS1015 settings
#define GAIN            ADS1015_CONFIG_PGA_16
#define SAMPLING_SPEED  ADS1015_CONFIG_RATE_3300HZ
#define MODE            ADS1015_CONFIG_MODE_CONT

#define PRINT_TIME 0

// Recording parameters
#define SIZE_CHUCK          512 // default size in bytes of blocks in SD card
#define NUMBER_CHUCKS       3 // number of 512 byte chucks we want to be recorded
#define GEOPHONE_DATA_SIZE  NUMBER_CHUCKS * SIZE_CHUCK
#define THRESHOLD           100   // at which the recording of the data will occur
#define RECORDING_MOMENT    GEOPHONE_DATA_SIZE/3  // 1/3 of the data before the threshold and 2/3 after
#define TOO_LOW_BATTERY_VOLTAGE   3.5
#define VOLTAGE_LOGGING_TIME_SEC 3600

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  1680        /* Time ESP32 will go to sleep (in seconds) 1680 = 28 minutes*/
RTC_DATA_ATTR int bootCount = 0;

// SD card settings and configuration
#define SD_FAT_TYPE 3
#define SPI_CLOCK SD_SCK_MHZ(4)
const uint8_t SD_CS_PIN = SS;

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat sd;
File myFile;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 myFile;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile myFile;
#elif SD_FAT_TYPE == 3
SdFat sd;
SdFile myFile;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE



volatile static bool lowPower = false;
volatile static bool flag_Geophone_event = false;
volatile static int16_t int_bufferToSave[GEOPHONE_DATA_SIZE]; // 

float voltage = 0;
bool low_voltage = false;

char geophoneData[16384];  //Array to hold the geophone data. Factor of 512 for easier recording to SD in 512 chunks

// tasks to sample geophones and record the data
TaskHandle_t Sampling_Geophones;
TaskHandle_t Saving_Events;

// to modify threshold, gain
int incoming = 0; // for incoming serial data


TwoWire Wire_0 = TwoWire(0);
TwoWire Wire_1 = TwoWire(1);

// objects for the three ADS1015 ADCs
ADS1015 ADS_48;
ADS1015 ADS_4A;
ADS1015 ADS_49;

// object MAX17048, Lipo battery monitoring and charging
SFE_MAX1704X lipo(MAX1704X_MAX17048);

// object RV8803, Real Time Clock 
RV8803 RTC;

// set LED type and 
LiteLED myLED(LED_TYPE, LED_TYPE_IS_RGBW);  // create the LiteLED object

void setup() {
  Serial.begin(115200);


  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  rtc_wdt_protect_off();    // Turns off the automatic wdt service

  Wire_0.begin(PIN_SDA_0, PIN_SCL_0, I2C_CLOCK_SPEED); // start I2C for RTC and Battery monitoring
  Wire_1.begin(PIN_SDA_1, PIN_SCL_1, I2C_CLOCK_SPEED); // start I2C for ADS1015s

  // Set up the MAX17043 LiPo fuel gauge:
  if (lipo.begin(Wire_0) == false) // Connect to the MAX17043 using the default wire port
  {
    Serial.println(F("MAX17043 not detected. Please check wiring. Freezing."));
    while (1)
      ;
  }

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for " + String(TIME_TO_SLEEP) +
  " Seconds if battery too low");

  lipo.quickStart();
  delay(50);
  voltage = lipo.getVoltage();
  if(voltage <= TOO_LOW_BATTERY_VOLTAGE)
  {
    low_voltage = true;
  }

  if (ADS_48.begin(0x48, Wire_1) == true) {
    Serial.println("ADS1015 with address 0x48 found. I2C connections are good.");
  } else {
    Serial.println("ADS1015 with address 0x48 not found. Check wiring.");
    while (1)
      ;  // stall out forever
  }

  if (ADS_49.begin(0x49, Wire_1) == true) {
    Serial.println("ADS1015 with address 0x49 found. I2C connections are good.");
  } else {
    Serial.println("ADS1015 with address 0x49 not found. Check wiring.");
    while (1)
      ;  // stall out forever
  }

  if (ADS_4A.begin(0x4A, Wire_1) == true) {
    Serial.println("ADS1015 with address 0x4A found. I2C connections are good.");
  } else {
    Serial.println("ADS1015 with address 0x4A not found. Check wiring.");
    while (1)
      ;  // stall out forever
  }




  if (RTC.begin(Wire_0) == false) {
    Serial.println("Device RTC not found. Please check wiring. Freezing.");
    while (1)
      ;
  }
  Serial.println("RTC online!");

  // set the RTC time to compiler time, only to be used one and not leaving in the running code
  /* if (RTC.setToCompilerTime() == false)
    Serial.println("Something went wrong setting the time");
  else
    Serial.println("New time set!");
 */

  RTC.set24Hour();

  ADS_48.setSampleRate(SAMPLING_SPEED);
  ADS_48.setMode(MODE);
  ADS_48.setGain(GAIN);

  ADS_49.setSampleRate(SAMPLING_SPEED);
  ADS_49.setMode(MODE);
  ADS_49.setGain(GAIN);

  ADS_4A.setSampleRate(SAMPLING_SPEED);
  ADS_4A.setMode(MODE);
  ADS_4A.setGain(GAIN);


  myLED.begin(LED_PIN, 1);      // initialze the myLED object. Here we have 1 LED attached to the LED_GPIO pin
  myLED.brightness(LED_BRIGHT);  // set the LED photon intensity level
  myLED.setPixel(0, GREEN, 1);







  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Sampling_Geophones",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Sampling_Geophones,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */

  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Saving_Events",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Saving_Events,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}


void Task1code(void* pvParameters) {
  Serial.print("Sampling_Geophones running on core ");
  Serial.println(xPortGetCoreID());

  // Calling sensors a first time to set what getLastConversionResults() will output
  int16_t sensor_0 = ADS_48.getDifferential();
  int16_t sensor_1 = ADS_49.getDifferential();
  int16_t sensor_2 = ADS_4A.getDifferential();

  int16_t counter = 0;
  int32_t time = micros();
  int32_t timemillis = 0;
  bool full = 0;

  CircularBuffer<int16_t, NUMBER_CHUCKS * 512> buffer;
  
  for (;;) {
    if (PRINT_TIME) {
      counter++;
      if (counter == GEOPHONE_DATA_SIZE) {
        Serial.print("time recorded in millis before full : ");
        Serial.println(micros() - timemillis);
        counter = 0;
        timemillis = micros();
      }
    } 


    sensor_0 = ADS_48.getLastConversionResults();
    sensor_1 = ADS_49.getLastConversionResults();
    sensor_2 = ADS_4A.getLastConversionResults();
    buffer.push(sensor_0);
    buffer.push(sensor_1);
    buffer.push(sensor_2);

    if (( abs(buffer[RECORDING_MOMENT]) > THRESHOLD || 
          abs(buffer[RECORDING_MOMENT+1])> THRESHOLD || 
          abs(buffer[RECORDING_MOMENT+2]) > THRESHOLD) && 
          !flag_Geophone_event) 
    {
      for(int j=0;j<=GEOPHONE_DATA_SIZE-1;j++)
      {
          int_bufferToSave[j] = buffer[j];
      }
      flag_Geophone_event = true;

    }
  }
}

void Task2code(void* pvParameters) {

  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  delay(200);
  Serial.print("Saving_Events running on core ");
  Serial.println(xPortGetCoreID());

  String currentTime = "\0";
  String currentDate = "\0";
  
  char tempData[50];
  uint32_t time = micros();

  // set a timer to record the voltage of the battery every hour
  RTC.disableAllInterrupts();
  RTC.clearAllInterruptFlags();//Clear all flags in case any interrupts have occurred.
  RTC.setCountdownTimerFrequency(COUNTDOWN_TIMER_FREQUENCY_1_HZ);
  RTC.setCountdownTimerClockTicks(VOLTAGE_LOGGING_TIME_SEC);
  RTC.enableHardwareInterrupt(TIMER_INTERRUPT);
  RTC.setCountdownTimerEnable(1); //This will start the timer on the last clock tick of the I2C transaction


  for (;;) {
    // if too low voltage at the startup, will go straignt to sleep
    if(low_voltage)
    {
      if (RTC.updateTime() == true)  //Updates the time variables from RTC
      {
        currentDate = RTC.stringDate();  //Get the current date
        currentTime = RTC.stringTime();  //Get the time
      } 

      myFile.print(currentDate);
      myFile.print(",");
      myFile.print(currentTime);
      myFile.print(",");      
      myFile.println("Voltage is too low to log");
      myFile.close();
      delay(250);
      esp_deep_sleep_start();
    }


    geophoneData[0] = '\0';
    if (flag_Geophone_event) 
    {

      // shows visually with the LED if an event has occured
      myLED.brightness(LED_BRIGHT, 1);  // turn the LED on

      // update RTC time to get time of event
      if (RTC.updateTime() == true)  //Updates the time variables from RTC
      {
        currentDate = RTC.stringDate();  //Get the current date
        currentTime = RTC.stringTime();  //Get the time
      } 
      else 
      {
        Serial.print("RTC read failed");
      }
      
      // converts data to string to make it readable on SD card
      for (uint16_t i = 0; i < GEOPHONE_DATA_SIZE; i++) 
      {
        sprintf(tempData, "%d", int_bufferToSave[i]);
        strcat(geophoneData, tempData);
        strcat(geophoneData, ",");
      }

      // test if can open the SD card and create/add data to it
      if (!myFile.open("geophoneDATAtrial_after.csv", O_RDWR | O_CREAT | O_AT_END)) {
        sd.errorHalt("opening test.txt for write failed");
      }

      // record data, time and data on to SD card
      myFile.print(currentDate);
      myFile.print(",");
      myFile.print(currentTime);
      myFile.print(",");      
      myFile.println(geophoneData);

      myFile.close();

      flag_Geophone_event = false;

    } 
    else 
    {
      myLED.brightness(0, 1);  // turn the LED off
    }

    
    if(RTC.getInterruptFlag(FLAG_TIMER)) // RTC flag activates after set amount of time has passed
    {
      RTC.clearInterruptFlag(FLAG_TIMER); // reset flag
      if (RTC.updateTime() == true)  //Updates the time variables from RTC
      {
        currentDate = RTC.stringDate();  //Get the current date
        currentTime = RTC.stringTime();  //Get the time
      }
      else
      {
        Serial.println("can't update time");
      }

      if (!myFile.open("geophoneDATAtrial_after.csv", O_RDWR | O_CREAT | O_AT_END)) 
      {
        sd.errorHalt("opening test.txt for write failed");
      }
      // record LiPo battery voltage to SD card
      voltage = lipo.getVoltage();

      sprintf(tempData, "%f", voltage);
      myFile.print(currentDate);
      myFile.print(",");
      myFile.print(currentTime);
      myFile.print(",");
      myFile.print("Voltage");
      myFile.print(",");
      myFile.println(tempData);
      myFile.close();

      if(voltage <= TOO_LOW_BATTERY_VOLTAGE)
      {
        Serial.println("Battery Voltage too low, going to sleep");
        Serial.flush(); 

        if (!myFile.open("geophoneDATAtrial_after.csv", O_RDWR | O_CREAT | O_AT_END)) 
        {
          sd.errorHalt("opening test.txt for write failed");
        }
        myFile.println("battery voltage too low, going to sleep");
        myFile.close();
        delay(250);
        esp_deep_sleep_start();
      }

      Serial.print("time: ");
      Serial.print(currentTime);  // Print the battery voltage
      Serial.print("    ");
      Serial.print("Voltage: ");
      Serial.print(voltage);  // Print the battery voltage
      Serial.println(" V");
    }
  } // end task2 loop
} // end task 2

void loop() {
}

