# Arduino-ESP Communication Information
    
This readme explains how to control and communicate with an ESP8266 using an Arduino. I hope that this functions as a quick 
intro to help you start developing as fast as possible. The Arduino projects included here are examples of how to communicate
with the Arduino. 

## Hardware Setup

On newer ESP8266, you may need a 10k pull-up resistor on the reset (RST, pin6) and chip select (CHPD, pin 4) pins. 
See the Graphical Datasheet on the Sparkfun Docs link for pins.

Older ESP have 9600 baud setup as default, however more recently made ESPs will have a default value of 115200 baud. Most 
Arduinos do not support 115200 baud so you will have to setup the baudrate without the Arduino. I recommend connecting the ESP 
with a PC using a USB-UART adapter, and then use a terminal emulator software to configure the ESP using AT commands. The emulator 
software provides a terminal-like interface where you just need to type in the command. The command to set baud rate is
```
AT+CIOBAUD=9600\r\n
```
I used TeraTerm as a terminal emulator. To type a command, I would type the command and press Ctrl+M+J to execute the AT command.
You might use different keys depending on emulator settings.

## AT-Commands

The Arduino communicates with the ESP using AT-commands through a Serial connection. The command is trasmitted as a string with the
Serial.print() function. The command string must ALWAYS end with "\r\n" because that signals the ESP the end of the command.

Note from the AT-Command Doc that some commands configure the ESPs flash. This means that whatever settings saved in flash become
default and will always be used when the ESP powers up or resets.

## Helpful Tips
* Connecting to a wireless network takes roughly a minute.
* Reading from a website is also slow, roughly half a minute.
* The command that lists available networks also takes a while, roughly half a minute.
* When you connect the ESP to a PC using the USB-UART cable, you will need the pull-up resistors. With the Arduino, you might or might not.
    
## Usefull Resouces
* [Sparkfun Docs](https://www.sparkfun.com/products/13678) - ESP8266 page on Sparkfun, to go Documentation tab
* [AT-Command Doc](https://www.ctr-electronics.com/downloads/pdf/4A-ESP8266__AT_Instruction_Set__EN_v0.40.pdf) - In-depth AT command listing and documentaton
