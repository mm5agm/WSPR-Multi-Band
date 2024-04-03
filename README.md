# WSPR Multi Band
ESPP32 Multi Band WSPR beacon


 ![ESP32_Multi_Band_WSPR](https://github.com/mm5agm/WSPR-Multi-Band/assets/26571503/16636cdf-7bfc-418a-9dc5-b5624069300c)

 
The transistors are 2N2222 and the resistors are all 10k. The ESP32 can be either 30 or 38 pin. I've tested with both. 

Most boards have the built in LED connected to digital pin 13 and normally have a definition for LED_BUILTIN. However, I've recently come across some boards that don't have LED_BUILTIN defined and the LED is connected to pin 2. I've also come across ESP32 boards with no LEDs apart from the power indicator. If you have a board where LED_BUILTIN isn't defined, or there is no LED, the code will define LED_BUILTIN = 2 and allow the program to compile and load. If there is an LED it should come on when WSPR is transmitting. If the LED doesn't come on either there isn't an LED or it's connected to another pin, change LED_BUILTIN to the connected pin. ESP32's marked ESP32_Devkitc_V4 only have a power LED, there is no other LED.

This project is a continuation of my gitHub Single Band WSPR Beacon project published in the May and June editions of Practical Wireless where a step by step approach ended up with a single band WSPR beacon running on an Espressif ESP32. If you haven't used the Arduino IDE before it will be best to go to the other project, WSPR, and go through that first. This project takes the single band code and adds low pass filter switching and different frequencies before and after sunrise. The code has been running for a few months on my ESP32. My setup is for 10,12,15,17,20,30 and 40 meter bands. I have 1 filter that covers the 10,12 and 15 meter bands, another that covers 17 and 20 meters, another for 30 meters and one for 40 meters. If you want more bands you can request that I add them. 

Variables that you need to set up in the WSPR section starting at approximately line 77
- char callsign[7] = "******";  // Your CALLSIGN
- char locator[5] = "****";     // Your MAIDENHEAD GRID LOCATOR first 4 characters.
- float latitude = 0.0;         // Your latitude. Doesn't need to be accurate. Only used to get sun rise and set times
- float longitude = 0.0;        // Your longitude. Doesn't need to be accurate. Only used to get sun rise and set times
- int txPower = 0;              // Your actual TX power in dBm. A bare si5351 set at 8ma = 10dBm
- int dayTimeSlots = 0;         // Day time number of slots to transmit in each hour. Values 1,2,3,5,6,8,10 and 15
- int nightTimeSlots = 0;       // Night time number of slots to transmit in each hour. Values 1,2,3,5,6,8,10 and 15

Variables that you need to set up in the Wi-Fi section starting at approximately line 89
- const char* ssid = "*****";            // SSID of your Wifi network
- const char* password = "*******";      // Password for your wifi network

Variables that you may want to change in the NTP section starting at approximately line 120
- #define DST_OFFSET 0                         // 1 for European summer time; 2 for US daylight saving time; 0 for no DST adjustment. 0 means UTC
- #define TIME_ZONE +0.0f                      // used in NTP time calculation. UTC time zone difference in regards to UTC (floating point number)
- const char* NTP_Server = "uk.pool.ntp.org";  // pick an ntp server in your area

Colin mm5agm@outlook.com

