/*****************************************************************************************************************
***                         Author Colin Campbell MM5AGM         mm5agm@outlook.com                            ***                                                                        ***
*** This program is free software: you can redistribute it and/or modify it under the terms of the GNU         ***
*** General Public License as published by the Free Software Foundation, either version 3 of the License,      ***
*** or (at your option) any later version.                                                                     ***
***                                                                                                            ***
*** This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without          ***
*** even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                          ***
*** See the GNU General Public License for more details.                                                       ***
******************************************************************************************************************/
/*****************************************************************************************************************
** This is the multi band WSPR beacon transmitter with different TX bands night and day if required             **
** It's a work in progress and may be changed.                                                                  **
** You will need to change the ssid, password, callsign, locator, si5351 calibration factor, and latitude and   **
** longitude. Select the frequencies to TX. Change DST_OFFSET and TIME_ZONE if you want to display local time   **
** otherwise it's UTC                                                                                           **                                                                                                              **
** Hardware required = ESP32, si5351 square wave generator, Real Time Clock DS3231, SSD1306 0.96inch OLED       **
** and low pass filters for the bands you are using                                                             **
** In Arduino IDE open the option “File/Preferences” and fill in the “Additional boards manager URLs” with      **
**  https://espressif.github.io/arduino-esp32/package_esp32_index.json                                          **
** Unfortunately the 2 libraries I use to manipulate time define time differently.                              **
** The RTC library has days 0 to 6, with 0 = Sunday and 6 = Saturday                                            **
** The Time library, has days 1 to 7, with 1 = Sunday and 7 = Saturday                                          **
** Time library by Paul Stoffregen                                          - install from Arduino IDE          **
** NTP library is https://github.com/SensorsIot/NTPtimeESP                  - download from github and install  **
** Real Time Clock library is Adafruit 2.1.3 for a RTC like DS3231          - install from Arduino IDE          **
** OLED library is Adafruit_SSD1306 which also requires Adafruit_GFX.h      - install both from Arduino IDE     **
** si5351 and WSPR encoding libraries are by Jason Milldrum                 - install both from Arduino IDE     **
** ESP32 - My multi band extension of this sketch uses the library "Sunset" and that requires a 32 bit FPU      **
** hence ESP32. ESP32 comes in 30 and 38 pin varieties. Tested with both varieties                              **
**                                                                                                              **
** si5351 square wave generator Clock0 is used as WSPR output.                                                  **
** si5351, DS3231 real time clock, and OLED SSD1306, conected via I2C bus on GPIO21 (SDA) and pin GPIO22 (SCL)  ** 
** I used sotamaps.org to get locator, latitude and longitude                                                   **
******************************************************************************************************************/
// Hardware info
// ---------------------
// serial debug port baud rate: 115200
// ESP32 - Must be ESP32 due to floating point unit required for library sunset. ESP32 comes in 30 and 38 pin varieties. Mine has 38pins.
// Si5351A is connected via I2C on GPIO21 (SDA) and pin GPIO22 (SCL). Clock0 is used as output.
// DS3231 real time clock via I2C on GPIO21 (SDA) and pin GPIO22 (SCL)
// Si5351 via I2C on GPIO21 (SDA) and pin GPIO22 (SCL).  I've put I2C address in because sometimes it didn't respond without it

/*Important: Some ESP32 boards such as the ESP32-DevKitC have no builtin LED at all ! Either connect an external LED or find another
method of doing whatever you are intending to do.
 Other boards have the built in LED connected to digital pin 13 and normally have a definition for LED_BUILTIN
However, I've recently come across some boards that don't have LED_BUILTIN defined but there is an LED connected to pin2. If you have
a board where LED_BUILTIN isn't defined the following code will define LED_BUILTIN = 2 and allow the program to compile and load.
This means that the LED will come on when WSPR is transmitting. If the led doesn't come on you'll need to try other values or add an external LED
*/

#define DEBUG  // Comment this line to supress debugging to the serial port

#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // I have an led with resistor connected between ground and GPIO23 as my ESP32 doesn't appear to have an inbuilt LED
#endif


/******************************[ Libraries ]***********************************************************************************/
#include <int.h>               // arduino
#include "Wire.h"              // arduino
#include <si5351.h>            // by Jason Mildrum https://github.com/etherkit/Si5351Arduino
#include <JTEncode.h>          // by Jason Mildrum https://github.com/etherkit/JTEncode
#include <TimeLib.h>           //  https://github.com/PaulStoffregen/Time
#include <RTClib.h>            // Adafruit 2.1.3 for RTC like DS3231
#include <Adafruit_GFX.h>      //  Adafruit 1.11.9
#include <Adafruit_SSD1306.h>  //  Adafruit 2.5.9
#include <sunset.h>            // By Peter Buelow - installed from Arduino IDE  https://github.com/buelowp/sunset
#include <NTPtimeESP.h>        //  https://github.com/SensorsIot/NTPtimeESP  // you're only allowed to get the time every 4 seconds from an NTP server or you will be banned

/******************************[ WSPR ]********************************************/
#define TONE_SPACING 146                // ~1.46 Hz
#define WSPR_DELAY 683                  // Delay value for WSPR
#define WSPR_CTC 10672                  // CTC value for WSPR
#define SYMBOL_COUNT WSPR_SYMBOL_COUNT  // txPower is determined by si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA) + any amplification you add

char callsign[7] = "******";  // Your CALLSIGN
char locator[5] = "****";     // Your MAIDENHEAD GRID LOCATOR first 4 characters.
float latitude = 0.0;         // Your latitude. Doesn't need to be accurate. Only used to get sun rise and set times
float longitude = 0.0;        // Your longitude. Doesn't need to be accurate. Only used to get sun rise and set times
int txPower = 0;              // Your actual TX power in dBm. A bare si5351 set at 8ma = 10dBm
int dayTimeSlots = 0;         // Day time number of slots to transmit in each hour. Values 1,2,3,5,6,8,10 and 15
int nightTimeSlots = 0;       // Night time number of slots to transmit in each hour. Values 1,2,3,5,6,8,10 and 15

uint8_t tx_buffer[SYMBOL_COUNT];  // create buffer to hold TX chars
JTEncode jtencode;                //create instance

/*****************************[ Wi-Fi ]*********************************************/
const char* ssid = "*****";            // SSID of your Wifi network
const char* password = "*******";      // Password for your wifi network
int failCount = 20;                      // maximum number of times to attempt to connect to wi-fi. Attempts are 500Ms appart
const char* WiFi_hostname = "ESP_WSPR";  // how it's reported on your router/hub

/******************************[ si5351 ]*******************************************************************************************/
int32_t cal_factor = 0;     //Calibration factor obtained from Calibration arduino program in Examples. You must calbrate first
Si5351 si5351(0x60);        // si5351 instance. I've put I2C address in because sometimes it didn't seem to respond without it

/******************************[ OLED Display ]**************************************************************************************/
int SCREEN_WIDTH = 128;                                            // OLED display width, in pixels
int SCREEN_HEIGHT = 64;                                            // OLED display height, in pixels
int charWidth = 6;                                                 // 21 characters in 128 pixels
int lineSpace = 9;                                                 // gives 6 lines of text with 21 characters a line
int ipLine = 0;                                                    // the line the I.P. address is on
int latLongLine = 1;                                               // latitude and longitude line number
int dayNightLine = 2;                                              // sunrise sunset line
int dateLine = 3;                                                  // date line
int timeLine = 4;                                                  // time line
int bandLine = 5;                                                  // band line
int frequencyLine = 6;                                             // frequency line
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  // create an instance of Adafruit_SSD1306 called display

/************************************************* RTC ************************************************************************/
RTC_DS3231 rtc;
const char* weekDays[] = { "Sun", "Mon", "Tues", "Wed", "Thur", "Fri", "Sat" };                                        // Used for the display only
const char* months[] = { "Jan", "Feb", "March", "April", "May", "June", "July", "Aug", "Sept", "Oct", "Nov", "Dec" };  // Used for the display only

/******************************[ NTP ]*****************************************************************************************/
// If DST_OFFSET is set to 0; the OLED display will show UTC time, which may or may not be the same as your local time.
// In UK, If DST_OFFSET is set to 1, the OLED display will show local time.
#define DST_OFFSET 0                         // 1 for European summer time; 2 for US daylight saving time; 0 for no DST adjustment;  not tested by me but since I'll use UTC only it can stay at 0)
#define TIME_ZONE +0.0f                      // used in NTP time calculation. UTC time zone difference in regards to UTC (floating point number)
const char* NTP_Server = "uk.pool.ntp.org";  // pick an ntp server in your area
NTPtime NTPch(NTP_Server);
strDateTime NTPdateTime;  //strDateTime is declared in NTPtimeESP.h as a type - don't confuse with DateTime declared in RTClib

/******************************[Sunset]***********************************************/
int sunrise;               // how many minutes past midnight relative to the set date that sunrise will happen.
int sunset;                // how many minutes past midnight relative to the set date that sunset will happen.
int minutesAfterMidnight;  // Used to determine if we are at day time or night time
bool isDayTime = true;     // set acording to the sunrise and sunset times. If its after sunrise and before sunset then isDayTime = true, otherwise it's false
SunSet sun;                // create instance

/*****************************[ Other Global Variables]*******************************/
#define BAUDRATE 115200  // Arduino serial monitor
int randomChange = 0;    // 0 to 100.  a random value between -randomChange and +randomChange is applied to the TX frequency random(-100, 100)
unsigned long freq;      // the frequency we will transmit
float freqMHz;           //  the frequency to show on the OLED e.g. 18.106100

unsigned long txFreq;    // the actual frequency being transmitted = freq +- random(randomChange)

// Enumerate bands to make identification for GPIO and Frequency easy
const int band10 = 1;
const int band12 = 2;
const int band15 = 3;
const int band17 = 4;
const int band20 = 5;
const int band30 = 6;
const int band40 = 7;
int band;  // used to tell what band we are meaning. For example band = 4 would give band17

/* Put the bands you want to transmit on in the arrays. This allows the program an easy way to access them.
 Examples
 int dayBands[] = { band10 }; int nightBands[] = {};             //1 band only during the day 
 int dayBands[] = { band10 }; int nightBands[] = { band10  };     //1 band only all the time
 int dayBands[] = {}; int nightBands[] = { band10  };     //1 band only at night
 int dayBands[] = { band10, band12, band15, band17, band20, band30, band40 };  int nightBands[] = { band10, band12, band15, band17, band20, band30, band40 }; // 7 bands all day
 int dayBands[] = { band10, band12, band15, band17 };  int nightBands[] = { band20, band30, band40 }; // different day and night bands 
 int dayBands[] = { band10, band10, band10, band12, band15, band17, band20, band30, band40 };  // 7 bands in the day with band10 having 3 slots in a row
 int dayBands[] = { band10, band12, band10, band15, band10, band17, band10, band20, band10, band30, band10, band40 };  // 7 bands in the day with band10 having every 2nd slot
*/
int dayBands[] = { band10, band12, band15, band17, band20, band30, band40 };  // change this to your day bands
int nightBands[] = { band20, band30, band40 };                                // // change this to your night bands
int numDayBands;                                                              // calculated number of bands during the day. Note that the last example above has 12. It's not the number of distict bands, it's the number of bands in the array
int dayBandIndex = 0;                                                         // start TX at the first day band. The index is the position in the dayBands[] array. Array indexes start at 0
int numNightBands;                                                            // calculated number of bands during the night. It's not the number of distict bands, it's the number of bands in the array
int nightBandIndex = 0;                                                       // start TX at the first night band. The index is the position in the nightBands[] array. Array indexes start at 0

// Give the bands a meaningfull name for display purposes on OLED and Serial monitor.
char strBand10[] = { "10 Meters" };
char strBand12[] = { "12 Meters" };
char strBand15[] = { "15 Meters" };
char strBand17[] = { "17 Meters" };
char strBand20[] = { "20 Meters" };
char strBand30[] = { "30 Meters" };
char strBand40[] = { "40 Meters" };
char unknownBand[] = { "Unknown Band" };
const int band10offset = 0;  // these may be used later if I find I'm off frequency by different amounts on different bands
const int band12offset = 0;
const int band15offset = 0;
const int band17offset = 0;
const int band20offset = 0;
const int band30offset = 0;
const int band40offset = 0;
// The frequencies I tell my si5351 to output
const unsigned long band10freq = 28126100UL + band10offset;
const unsigned long band12freq = 24926100UL + band12offset;
const unsigned long band15freq = 21096100UL + band15offset;
const unsigned long band17freq = 18106100UL + band17offset;
const unsigned long band20freq = 14097100UL + band20offset;
const unsigned long band30freq = 10140200UL + band30offset;
const unsigned long band40freq = 7040100UL + band40offset;
// Relay pins to switch filter depending on band. Colours are my wire colours
const int Band10_GPIO = 12;  //green
const int Band12_GPIO = 12;
const int Band15_GPIO = 12;
const int Band17_GPIO = 14;  //orange
const int Band20_GPIO = 14;
const int Band30_GPIO = 26;  // blue
const int Band40_GPIO = 27;  //brown

/***************************** [ initialiseDisplay ] *****************************************
***                             Initialise the OLED                                        ***
**********************************************************************************************/
void initialiseDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address found in I2C_Scanner.ino = 0x3C
#ifdef DEBUG
    Serial.println("SSD1306 allocation failed");  // if the display isn't found you can't write an error message on it
    Serial.println(" Program will continue");
#endif
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("OLED OK");
    display.display();  // you need display.display() or you don't update the display. So annoying!!
#ifdef DEBUG
    Serial.println("initialiseDisplay finished");
#endif
  }
}

/**************************** [ initialiseWiFi ] ************************************
***                     Initialse and connect to Wi-Fi                            ***
*************************************************************************************/
void initialiseWiFi() {
  int attempts = 0;
  WiFi.mode(WIFI_OFF);  //Prevents reconnection issue (taking too long to connect)
  WiFi.mode(WIFI_STA);  //This line hides the viewing of ESP as wifi hotspot
  delay(1000);          // delay or you won't see the starting messages
  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("Connecting to Wi-Fi");
  display.display();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {  // will be stuck in this loop till connected or timeout at about 10 seconds for failCount = 20
    attempts++;
    display.print("Wi-Fi attempts = ");
    display.println(attempts);
    display.display();
    delay(500);  // short delay before next attempt to connect
#ifdef DEBUG
    Serial.print("Attempting Wi-Fi connection. Number of attempts = ");
    Serial.println(attempts);
#endif
    if (attempts % 7 == 0) {
      display.clearDisplay();
      display.setCursor(0, 10);  // go back to the top of the screen
    }
    if (attempts > failCount) {
#ifdef DEBUG
      Serial.print("Failed to connect to Wi-Fi attempts = ");
      Serial.println(attempts);
      Serial.println("Are you sure you have the correct ssid and password?");
      Serial.print("ssid entered = ");
      Serial.print(ssid);
      Serial.print("  password entered = ");
      Serial.println(password);
      Serial.println("Program Halted");
#endif
      display.clearDisplay();
      display.setCursor(0, 10);
      display.println(" Can't connect Wi-Fi");
      display.println("ssid and password OK?");
      display.print("ssid  ");
      display.println(ssid);
      display.print("pass ");
      display.println(password);
      display.println("Program Halted");
      display.display();
      while (1) {};  // program will stick here to let user read display
    }
    display.display();
  }
}

/********************* [ updateRTC ] **************************************
*** Update RTC from NTP                                                 ***
*** The OLED will show the time continuously unless we are transmitting ***
*** During a TX cycle, I don't want to do anything apart from encode    ***
*** so the OLED clock will not update                                   ***
***************************************************************************/
void updateRTC() {
#ifdef DEBUG
  Serial.println();
  Serial.println("Updating RTC from NTP");
#endif
  do {
    NTPdateTime = NTPch.getNTPtime(TIME_ZONE, DST_OFFSET);  //1 for European summer time; 2 for US daylight saving time; 0 for no DST adjustment; ( not tested by me)
    delay(1);
  } while (!NTPdateTime.valid);  // keep trying till you get a valid time. rtc.adjust comes from the adafruit rtc lib
  rtc.adjust(DateTime(NTPdateTime.year, NTPdateTime.month, NTPdateTime.day, NTPdateTime.hour, NTPdateTime.minute, NTPdateTime.second));
#ifdef DEBUG
  Serial.println();
  Serial.println("RTC updated from NTP");
#endif
}

/***************  [ initialiseRTC ] **************************
*** Initialise the RTC and update from NTP server          ***
**************************************************************/
void initialiseRTC() {
  if (!rtc.begin()) {
#ifdef DEBUG
    Serial.println();
    Serial.println(" Couldn't find RTC.  Program will halt");
#endif
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Couldn't find RTC. Program will halt ");
    display.display();
    while (1) delay(10);  // stay here until rtc found - if it's not found you're here forever
  }
  configTime(0, 0, NTP_Server);
#ifdef DEBUG
  Serial.println("initialise RTC finished");
#endif
  updateRTC();  // now set it to correct time
}

/**************************  [ initialiseSI5351 ] ***********************************
***                Initialse the si5351 square wave generator                     ***
*************************************************************************************/
void initialiseSI5351() {
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  txFreq = freq + random(-randomChange, randomChange);
  freqMHz = txFreq / 1000000.0;                  // used to give the user a meaningful displayed frequency. If you divide with an integer, it only returns the quotient as an integer
  si5351.set_freq((txFreq * 100), SI5351_CLK0);  // Clock 0 used to tx WSPR
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  // If you change the drive strength to another value you will need to change the txPower value
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);  // 2,4,6,8mA  2 mA roughly corresponds to -1.5 dBm output 0.7mW,8 mA is approximately 10 dBm
  si5351.set_clock_pwr(SI5351_CLK0, 0);                  // Disable the clock initially
#ifdef DEBUG
  Serial.println("initialiseSI5351 finished");
#endif
}
/******************************Functions to display Date and Time****************************/

/************************** [ serialPadZero ] ******************************************
*** Print a "0" in front of a single digit number in the Serial Monitor Output       ***
****************************************************************************************/
void serialPadZero(int aNumber) {
  if (aNumber < 10) {
    Serial.print("0");
    Serial.print(aNumber);
  } else {
    Serial.print(aNumber);
  }
}

/**********************************  [ displayPadZero ]  ************************************
*** Print a "0" in front of a single digit number on the OLED display                     ***
*********************************************************************************************/
void displayPadZero(int aNumber) {
  if (aNumber < 10) {
    display.print("0");
    display.print(aNumber);
  } else {
    display.print(aNumber);
  }
  display.display();  // if you dont do display.display() it doesn't display. Very annoying
}
/************************************ [ serialShowTime ] *************************************
***  Only used in debug. Show date and time in serial monitor                              ***
**********************************************************************************************/
void serialShowTime(DateTime now) {
  Serial.print(weekDays[now.dayOfTheWeek()]);
  Serial.print("  ");
  serialPadZero(now.day());
  Serial.print("/");
  serialPadZero(now.month());
  Serial.print("/");
  serialPadZero(now.year());
  Serial.print("  ");
  serialPadZero(now.hour());
  Serial.print(":");
  serialPadZero(now.minute());
  Serial.print(":");
  serialPadZero(now.second());
}

/******************************** [ showDate ] ***************************************************
***          Display the Date on the OLED                                                      ***
***  since I only use the NTP server to update the RTC I've decided only to display RTC data   ***
**************************************************************************************************/
void showDate(DateTime now) {
  display.print(weekDays[now.dayOfTheWeek()]);
  display.print("  ");
  displayPadZero(now.day());
  display.print("/");
  display.print(months[now.month() - 1]);
  display.print("/");
  displayPadZero(now.year());
  display.display();
}

/********************************** [ showTime ] *************************************************
***          Display the Time on the OLED                                                      ***
***  since I only use the NTP server to update the RTC I've decided only to display RTC data   ***
**************************************************************************************************/
void showTime(DateTime now) {
  displayPadZero(now.hour());
  display.print(":");
  displayPadZero(now.minute());
  display.print(":");
  displayPadZero(now.second());
  display.display();
}

/**************************Encoding WSPR and TX functions************************************/

/********************* [ encode] *********************************
***                WSPR encode and transmit                    ***
*** When in this function the time on the OLED will not update ***
******************************************************************/
void encode() {
  uint8_t i;
#ifdef DEBUG
  Serial.println("*** Starting WSPR coding ***");
#endif
  digitalWrite(LED_BUILTIN, HIGH);  // tell user we are now transmitting
  jtencode.wspr_encode(callsign, locator, txPower, tx_buffer);
  for (i = 0; i < SYMBOL_COUNT; i++) {
    si5351.set_freq((txFreq * 100) + (tx_buffer[i] * TONE_SPACING), SI5351_CLK0);
    delay(WSPR_DELAY);
  }
  // si5351.set_clock_pwr(SI5351_CLK0, 0);  // Finished TX so Turn off the output
  digitalWrite(LED_BUILTIN, LOW);  // tx off
#ifdef DEBUG
  Serial.println("*** End of WSPR coding ***");
#endif
  mainScreen();
}
/***************************** [ txOn ] ***********************************************
*** Neither switching TX on a couple of seconds early or having clock1 running at   ***
*** 150MHz when clock0 wasn't transmitting made any difference to my drift problem. ***
*** My drift problems were solved by using another si5351                           ***
*** Frequency is set in the hop function                                            ***
***************************************************************************************/
void txOn() {
  si5351.set_freq((txFreq * 100), SI5351_CLK0);
  si5351.set_clock_pwr(SI5351_CLK0, 1);  // switch on clock0
#ifdef DEBUG
  Serial.print("txFreq = ");
  Serial.print(txFreq, 6);
  Serial.print("  freqMHz = ");
  Serial.println(freqMHz, 6);
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  clearLine(frequencyLine);
  cursorAt(0, frequencyLine);
  display.println("     Transmitting");
  display.display();
}

/************* [ txOff ] ***************
*** Switch clock0 off                ***
****************************************/
void txOff() {
  digitalWrite(LED_BUILTIN, LOW);
  si5351.set_clock_pwr(SI5351_CLK0, 0);  // stop transmitting. Switch off clock 0
#ifdef DEBUG
  Serial.println(" Transmission Stopped");
#endif
}

/*  
  WSPR signals are 6 Hz wide.   
 The WSPR software will decode 100 Hz either side of the Centre Frequency, which gives a total receive bandwidth of only 200 Hz.
 The following dial frequencies are in use for WSPR. We need to add 1500Hz to the dial frequency to get the transmit frequency:
 TX centre Frequency = USB Dial Frequency + 1.5 KHz
 Dial Frequency           TX Frequency
80m: 3.568600 MHz         3,570.100 kHz  +/- 100 Hz
40m: 7.038600 MHz         7,040.100 kHz  +/- 100 Hz
30m: 10.138700 MHz        10,140.200 kHz +/- 100 Hz
20m: 14.095600 MHz        14,097.100 kHz +/- 100 Hz 
17m: 18.104600 MHz        18,106.100 kHz +/- 100 Hz
15m: 21.094600 MHz        21,096.100 kHz +/- 100 Hz
12m: 24.924600 MHz        24,926.100 kHz +/- 100 Hz
10m: 28.124600 MHz        28,126.100 kHz +/- 100 Hz 

The following is taken from my single band WSPR program
WSPR Time Slots - Must start on an EVEN minute so some "Number of slots" can't be used. For example, 4 slots would mean TX every 15 minutes at 0,15,30,45. 7 would mean every 11 minutes
There are 2 distinct periods when you can transmit, 0,4,8,12,16 etc minutes and 2,6,10,14,18 etc minutes. I designated these Odd and Even in the table.
Recomended TX 20% of the time which gives 3 slots in each TX period with a single band. If you have 2 bands, you'll transmit 40% of the time in each band.
1 slot is different because minutes goes from 0 to 59 and we have to choose so that (minute modulus mins between TX) = 0. Any value above 30 will do.

		                                        	Period	1	2	3	4	5	 6  7	   8	9	  10	11	12	13	14	15	16	17	18	19	20	21	22	23	24	25	26	27	28	29	30 	Odd	Even
Num Slots	Mins TX 	%TX	      Mins between TX       	0	2	4	6	8	10	12	14	16	18	20	22	24	26	28	30	32	34	36	38	40	42	44	46	48	50	52	54	56	58		
     1       2	    3.33	     anything > 30                                                                                                     X                    1    0                                                                                                  																						                                            						
     2	     4	    6.67	          30	              X													                        X														                                 	1	   1
     3	     6	    10.00	          20	              X									           	X										                     X									                    	3	   0
     5	    10	    16.67         	12	              X					    X						             X					           	X						            X						              5	   0
     6	    12	    20.00	          10	              X					X				            X					          X					           X					        X			                3    3 
     8	    16	    26.67	           8	              X				X				       X	 			       X	 			      X	 			       X	 			        X	 			         X	    	8    0
    10	    20	    33.33	           6	              X			X			  X			       X			     X		      X			      X			      X			      X			   X			     X    5	   5
    15	    30	    50.00	           4	              X		X   X	 	  X		     X	   	X		     X	 	   X		  X	    	X		   X	    	X		    X	 	     X		   X				15	 0

For single band the allowed values 1,2,3,5,6,8,10 and 15 corresponded to TX 3.33% of the time 6.67%,10%,16.67%,20%,26.67%,33.33%,and 50%
Now that we are transmitting on multiple bands the %TX on a particular band is going to be lower. 
For example, if we are transmitting on 5 bands, if we choose the 15 slots, we will be transmitting for 50.00/5 = 10% of the time on each band.

*/

/***************************** [ txDelay ] ************************************************
*** Return number of minutes between transmissions given the number of slots to be used ***
*******************************************************************************************/
int txDelay(int numSlots) {
  int numMinutes = 10;  // 20% TX time if only 1 band, 10% TX on each band if 2 bands
  switch (numSlots) {
    case 1:             // 60 min cycle
      numMinutes = 38;  // any even number above 30 because usage is      now.minute() % numMinutes == 0
      break;
    case 2:
      numMinutes = 30;
      break;
    case 3:
      numMinutes = 20;
      break;
    case 5:
      numMinutes = 12;
      break;
    case 6:
      numMinutes = 10;
      break;
    case 8:
      numMinutes = 8;
      break;
    case 10:
      numMinutes = 6;
      break;
    case 15:
      numMinutes = 4;
      break;
    default:
      numMinutes = 10;
      break;
  }
  return numMinutes;
}


/***********************************[setFrequency]********************************************
*** Set the frequency and output HIGH to the GPIO connected to the correct low pass filter ***
**********************************************************************************************/
void setFrequency(int aBand) {

#ifdef DEBUG
  Serial.print("*************************Band passed into setFrequency() = ");
  Serial.println(aBand);
#endif
  // Switch all filters off, and then switch the correct one on to match the frequency
  digitalWrite(Band10_GPIO, LOW);
  digitalWrite(Band12_GPIO, LOW);
  digitalWrite(Band15_GPIO, LOW);
  digitalWrite(Band17_GPIO, LOW);
  digitalWrite(Band20_GPIO, LOW);
  digitalWrite(Band30_GPIO, LOW);
  digitalWrite(Band40_GPIO, LOW);
  switch (aBand) {
    case band10:
      digitalWrite(Band10_GPIO, HIGH);
      freq = band10freq;
      band = band10;
      break;
    case band12:
      digitalWrite(Band12_GPIO, HIGH);
      freq = band12freq;
      band = band12;
      break;
    case band15:
      digitalWrite(Band15_GPIO, HIGH);
      freq = band15freq;
      band = band15;
      break;
    case band17:
      digitalWrite(Band17_GPIO, HIGH);
      freq = band17freq;
      band = band17;
      break;

    case band20:
      digitalWrite(Band20_GPIO, HIGH);
      freq = band20freq;
      band = band20;
      break;
    case band30:
      digitalWrite(Band30_GPIO, HIGH);
      freq = band30freq;
      band = band30;
      break;
    case band40:
      digitalWrite(Band40_GPIO, HIGH);
      freq = band40freq;
      band = band40;
      break;
    default:
      freq = band20freq;
      band = band20;
      digitalWrite(Band10_GPIO, LOW);
      digitalWrite(Band12_GPIO, LOW);
      digitalWrite(Band15_GPIO, LOW);
      digitalWrite(Band17_GPIO, LOW);
      digitalWrite(Band20_GPIO, HIGH);
      digitalWrite(Band30_GPIO, LOW);
      digitalWrite(Band40_GPIO, LOW);
      break;
  }
  txFreq = freq + random(-randomChange, randomChange);
  freqMHz = txFreq / 1000000.0;
}

/******************************[displaySunTimes]*****************
*** Display the sunrise and sunset times on the OLED display  ***
*****************************************************************/
void displaySunTimes(int intRise, int intSet) {
  int i;
  display.print("Rise ");
  i = (intRise / 60);
  displayPadZero(i);
  display.print(":");
  i = (intRise % 60);
  displayPadZero(i);
  display.print(" Set ");
  i = intSet / 60;
  displayPadZero(i);
  display.print(":");
  i = intSet % 60;
  displayPadZero(i);
  display.display();
}
/**********************************[getStrBand]*****************************
*** return the descritive band name from the enumerated band integer     ***
*** for example, 17Mtrs is enumerated as band17 = 4, and the description ***
*** of band 4 is "17 Meters"                                             ***
****************************************************************************/
char* getStrBand(int i) {

  switch (i) {
    case band10:
      return strBand10;
      break;
    case band12:
      return strBand12;
      break;
    case band15:
      return strBand15;
      break;
    case band17:
      return strBand17;
      break;
    case band20:
      return strBand20;
      break;
    case band30:
      return strBand30;
      break;
    case band40:
      return strBand40;
      break;
    default:
      return unknownBand;
      break;
  }
}
/***********************[getDayBandFromIndex]*******************************
***  return the descriptive day band from the enumerated band integer    ***
*** for example, 17Mtrs is enumerated as band17 = 4, and the description ***
*** of band 4 is "17 Meters"                                             ***
****************************************************************************/
void getDayBandFromIndex(int anIndex) {
  getStrBand(anIndex);
}
/***********************[getNightBandFromIndex]*******************************
***  return the descriptive night band from the enumerated band integer    ***
*** for example, 17Mtrs is enumerated as band17 = 4, and the description   ***
*** of band 4 is "17 Meters"                                               ***
******************************************************************************/
void getNightBandFromIndex(int anIndex) {
  getStrBand(anIndex);
}

/**********************************[hop]**************************************
*** get the index of the next band in the array dayBands[] or nightBands[] ***
******************************************************************************/
void hop(bool isDayTime) {
  if (isDayTime) {
#ifdef DEBUG
    Serial.println("it is Day Time in hop");
#endif
    if (numDayBands == 0) {
#ifdef DEBUG
      Serial.println("NO DAY BANDS");
#endif
      return;
    }
    dayBandIndex = dayBandIndex + 1;
    if (dayBandIndex >= numDayBands) {
      dayBandIndex = 0;
    }
    band = dayBands[dayBandIndex];
    getDayBandFromIndex(dayBandIndex);
    setFrequency(band);
#ifdef DEBUG
    Serial.print(" band changed to band = ");
    Serial.println(band);
#endif

  } else {  // it's night time
#ifdef DEBUG
    Serial.println("It's night time in hop");
#endif
    if (numNightBands == 0) {
#ifdef DEBUG
      Serial.println("NO NIGHT BANDS");
#endif
      return;
    }
    nightBandIndex = nightBandIndex + 1;
    if (nightBandIndex >= numNightBands) {
      nightBandIndex = 0;
    }
    band = nightBands[nightBandIndex];
#ifdef DEBUG
    Serial.print(" band changed to band = ");
    Serial.println(band);
#endif
    getNightBandFromIndex(nightBandIndex);
    setFrequency(band);
  }
  // display will be updated with new band and frequency in mainScreen()
}

/*********************************[daytime]******************************
***   Sets the variable isDayTime = true if the time is after sunrise ***
*** and before sunset, false otherwise                                ***
*************************************************************************/
void daytime() {
  DateTime now = rtc.now();
  sun.setCurrentDate(now.year(), now.month(), now.day());
  sunrise = static_cast<int>(sun.calcSunrise());
  sunset = static_cast<int>(sun.calcSunset());
  minutesAfterMidnight = now.hour() * 60 + now.minute();
  isDayTime = (minutesAfterMidnight >= sunrise) && (minutesAfterMidnight < sunset);
}
/************************[initialiseGPIOpins]**********************
*** Initialsie the GPIO pins that are used                      ***
*******************************************************************/
void initialiseGPIOpins() {
  pinMode(Band10_GPIO, OUTPUT);
  pinMode(Band12_GPIO, OUTPUT);
  pinMode(Band15_GPIO, OUTPUT);
  pinMode(Band17_GPIO, OUTPUT);
  pinMode(Band20_GPIO, OUTPUT);
  pinMode(Band30_GPIO, OUTPUT);
  pinMode(Band40_GPIO, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}
void initialseSun() {
#ifdef DEBUG
  Serial.println();
  Serial.print("latitude = ");
  Serial.print(latitude, 5);
  Serial.print("  longitude = ");
  Serial.println(longitude, 5);
#endif

  sun.setPosition(latitude, longitude, DST_OFFSET);
  sun.setTZOffset(DST_OFFSET);
  daytime();  //set isDayTime variable
}

/******************[setInitialFrequency]********************
*** Set the initial transmit frequency                   ***
************************************************************/
void setInitialFrequency() {
  if (sizeof(dayBands) > 0) {
    numDayBands = sizeof(dayBands) / sizeof(dayBands[0]);
  } else {
    numDayBands = 0;
  }
  if (sizeof(nightBands) > 0) {
    numNightBands = sizeof(nightBands) / sizeof(nightBands[0]);
  } else {
    numNightBands = 0;
  }
  if ((numDayBands == 0) && ((numNightBands) == 0)) {

#ifdef DEBUG
    Serial.println("************************************************************************");
    Serial.println("************************************************************************");
    Serial.println("**** You need to have at least 1 band in dayBands[] or nightBands[] ****");
    Serial.println("****               Program Stopping                                 ****");
    Serial.println("************************************************************************");
    Serial.println("************************************************************************");
#endif
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println(" No bands in dayBands ");
    display.println("or nightBands");
    display.println("  Program Halted");
    display.display();
    while (true) {};  // this will stop the program at this point
  }
  if (isDayTime && (numDayBands > 0)) {
    band = dayBands[0];  // just set it to something to get it going
#ifdef DEBUG
    Serial.print("******************Initial DAY Band  =");
    Serial.println(band);
    // delay(3000);
#endif
  }
  if (!isDayTime && (numNightBands > 0)) {
    band = nightBands[0];
#ifdef DEBUG
    Serial.print("******************Initial Night Band  =");
    Serial.println(band);
    // delay(3000);
#endif
  }
  setFrequency(band);
#ifdef DEBUG
  Serial.print("NumDayBands = ");
  Serial.println(numDayBands);
  Serial.print("NumNightBands = ");
  Serial.println(numNightBands);
  Serial.print("Initial Frequency = ");
  Serial.println(freq);
  Serial.println("Leaving Setup");
#endif
}

/************************************ [ cursorAt] *************************************************
*** Sets the OLED cursor to the position of the character x chars along line on line lineNumber ***
***************************************************************************************************/
void cursorAt(int charPositionX, int lineNumber) {
  int displayY;                                            // y position to print the number
  displayY = (lineNumber)*lineSpace;                       // display top line = 0
  display.setCursor(charPositionX * charWidth, displayY);  // again, x starts at 0
}


/************************************ [ displayNumberAt ] ************************************
***  OLED display the number on lineNumber at x characters in                              ***
***  doing it this way should stop screen flicker as I will only be changing 1 piece  of   ***
***  data and not refreshing the whole screen Linenumber starts at 0, X  and Y start at 0  ***
**********************************************************************************************/
void displayNumberAt(int charPositionX, int lineNumber, int number) {
  cursorAt(charPositionX, lineNumber);
  display.setTextColor(WHITE, BLACK);  // this allows the spaces to act as blanks
  display.print("  ");                 // blank the old number with 2 spaces
  display.display();
  cursorAt(charPositionX, lineNumber);  // put the curser back 2 chars
  if (number < 10) {
    display.print("0");
    display.display();
  }
  display.print(number);
  display.display();
}
/********************************[ clearLine ]********************
***    OLED clear the line, overprint what's there with spaces ***
***    Linenumbers start at 0 for the top line                 ***
******************************************************************/
void clearLine(int lineNumber) {
  int i;
  cursorAt(0, lineNumber);
  display.setTextColor(WHITE, BLACK);  // this allows the spaces to act as blanks
  for (i = 0; i < 21; i++) {
    display.print(" ");
  }
  display.display();
}
/************************************ [ mainScreen ] ***********************************************
***  This is the data I want to see all the time on the OLED                                     ***
***  1st line I.P address, 2nd Sunset and Sunrise time,3rd date, 4th current time, 5th frequency ***
****************************************************************************************************/
void mainScreen() {

  static bool firstRun = true;  // this will force mainScreen to update everything on first entry
  static int lastSecond = 99;
  static int lastMinute = 99;
  static int lastHour = 99;
  static int lastYear = 9999;
  static int lastMonth = 99;
  static int lastDay = 99;
  static int lastBand = 99;
  static IPAddress lastIP(192, 168, 111, 111);  //WiFi.localIP();
  DateTime now = rtc.now();
  if (firstRun) {
    display.clearDisplay();
    cursorAt(0, latLongLine);
    //  display.print("Lat ");
    display.print(latitude, 5);
    if (latitude > 0.0000001) {
      display.print("N  ");
    } else {
      display.print("S  ");
    }
    //  display.print("  Long ");
    display.print(longitude, 5);
    if (longitude > 0.0000001) {
      display.print("E");
    } else {
      display.print("W");
    }
    cursorAt(0, timeLine);
    if (DST_OFFSET == 0) {
      display.print("UTC ");
    } else {
      display.print("Local ");
    }
    display.print("Time");
    cursorAt(0, ipLine);
    firstRun = false;
  }
  if (!(lastIP == WiFi.localIP())) {
#ifdef DEBUG
    Serial.println("WiFi changing");
#endif
    lastIP = WiFi.localIP();
    cursorAt(0, ipLine);
    display.print(WiFi.localIP());
#ifdef DEBUG
    Serial.print("WiFi I.P. changed to ");
    Serial.println(lastIP);
#endif
  }
  cursorAt(0, dayNightLine);
  if (!(lastDay == now.day())) {  // the day has changed so update sunrise/set and date
    clearLine(dayNightLine);
    cursorAt(0, dayNightLine);
    lastDay = now.day();
    sun.setCurrentDate(now.year(), now.month(), now.day());
    sunrise = static_cast<int>(sun.calcSunrise());
    sunset = static_cast<int>(sun.calcSunset());
    displaySunTimes(sunrise, sunset);
    clearLine(dateLine);
    cursorAt(0, dateLine);
    showDate(now);
  }
  cursorAt(0, timeLine);
  if (!(lastHour == now.hour())) {
    displayNumberAt(11, timeLine, now.hour());
    display.print(":");
    display.display();
    lastHour = now.hour();
  }
  if (!(lastMinute == now.minute())) {
    displayNumberAt(14, timeLine, now.minute());
    display.print(":");
    display.display();
    lastMinute = now.minute();
  }
  if (!(lastSecond == now.second())) {  // only display the main screen if the time has changed otherwise screen flashes
    displayNumberAt(17, timeLine, now.second());
    display.display();
    lastSecond = now.second();
    display.display();
  }
  if (!(lastBand == band)) {
    clearLine(bandLine);
    clearLine(frequencyLine);
    cursorAt(0, bandLine);
    display.print("Band ");
    display.print(getStrBand(band));
    cursorAt(0, frequencyLine);
    display.print("Freq ");
    display.print(freqMHz, 6);
    display.print(" MHz");
    display.display();
    lastBand = band;
  }
}

//**************************************[ SETUP FUNCTION ]*************************************************
void setup() {
  Serial.begin(BAUDRATE);
  Serial.println("Starting Up");
  initialiseGPIOpins();
  digitalWrite(LED_BUILTIN, HIGH);  // Show the TX LED working, if there is one
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  initialiseDisplay();
  initialiseWiFi();  // If we don't get an internet connection we'll be stuck in the trying to connect loop
  initialiseRTC();   // needs wi-fi
  DateTime now = rtc.now();
  initialseSun();  // needs initialiseWiFi() to set day or night time
  delay(200);
  configTime(0, 0, NTP_Server);
  initialiseSI5351();
  now = rtc.now();
  sun.setCurrentDate(now.year(), now.month(), now.day());
  sunrise = static_cast<int>(sun.calcSunrise());
  sunset = static_cast<int>(sun.calcSunset());
#ifdef DEBUG
  Serial.println();
  Serial.print(" Sunrise = ");
  Serial.print(sunrise / 60);
  Serial.print(":");
  Serial.print(sunrise % 60);
  Serial.print("     Sunset = ");
  Serial.print(sunset / 60);
  Serial.print(":");
  Serial.println(sunset % 60);
#endif
  setInitialFrequency();
  mainScreen();
#ifdef DEBUG
  // delay(5000);  // give me a chance to see what mainscreen() is doing
  Serial.println("Leaving Setup");
#endif
}

//*****************************************[ LOOP FUNCTION ]******************************************************
void loop() {
  DateTime now = rtc.now();
  mainScreen();
  minutesAfterMidnight = now.hour() * 60 + now.minute();
  isDayTime = (minutesAfterMidnight >= sunrise) && (minutesAfterMidnight < sunset);
  // Update RTC every hour
  if ((now.second() % 55 == 0) && (now.minute() == 30)) {  //
    updateRTC();
  }
  if (isDayTime) {
    if ((now.minute() % txDelay(dayTimeSlots) == 0) && (now.second() == 0)) {  //start encoding at start of even minute
#ifdef DEBUG
      Serial.println("************************************************* Loop TX ON *******");
#endif
      txOn();
      encode();  // transmit the codes
      txOff();
      hop(isDayTime);
#ifdef DEBUG
      Serial.println("************************************************* Loop TX OFF *******");
#endif
    }
  } else {                                                                       // it's night time
    if ((now.minute() % txDelay(nightTimeSlots) == 0) && (now.second() == 0)) {  //start encoding at start of even minute
                                                                                 // digitalWrite(txLED, HIGH);
#ifdef DEBUG
      Serial.println("************************************************* Loop TX ON *******");
#endif
      txOn();
      encode();  // transmit the codes
      txOff();
      hop(isDayTime);
#ifdef DEBUG
      Serial.println("************************************************* Loop TX OFF *******");
#endif
    }
  }
}
//*************************************[ END OF PROGRAM ]********************************************************
