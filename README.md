# WSPR Multi Band
ESPP32 Multi Band WSPR beacon


 ![ESP32_Multi_Band_WSPR](https://github.com/mm5agm/WSPR-Multi-Band/assets/26571503/16636cdf-7bfc-418a-9dc5-b5624069300c)

 
The transistors are 2N2222 and the resistors are all 10k. The ESP32 can be either 30 or 38 pin. I've tested with both. Most boards have the built in LED connected to digital pin 13 and normally have a definition for LED_BUILTIN. However, I've recently come across some boards that don't have LED_BUILTIN defined and the LED is connected to pin 2. I've also come across ESP32 boards with no LEDs apart from the power indicator. If you have a board where LED_BUILTIN isn't defined, or there is no LED, the code will define LED_BUILTIN = 2 and allow the program to compile and load. If there is an LED it should come on when WSPR is transmitting. If the LED doesn't come on either there isn't an LED or it's connected to another pin, change LED_BUILTIN to the connected pin. ESP32's marked ESP32_Devkitc_V4 only have a power LED, there is no other LED.
This project is a continuation of my Practical Wireless articles where a step by step approach ended up with 
a single band WSPR beacon running on an Espressif ESP32. If you haven't used the Arduino IDE before it will be best 
to go to the other project, WSPR, and go through that first. This project takes the single band code and adds 
low pass filter switching and different frequencies before and after sunrise. The code has been running for 
a few months on my ESP32.
I'VE STILL TO ADD THE CODE TO THE REPOSITORY. SHOULD BE HERE BY MIDDLE OF APRIL. I'M JUST TIDYING IT AT THE MOMENT.
Colin MM5AGM
