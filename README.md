# Yazz_WindDisplay_ESP32
Project:  YAZZ_WindDisplay_ESP32, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  TARGET:   ESP32
  VERSION:  0.01
  Date:     02-10-2020
  Last
  Update:   02-10-2020 V0.01
            Port from Atmega328 to ESP from version 1.13
            
  Achieved: 03-10-2020 Runnable version on ESP32 with Nextion HMI         
  Purpose:  Build an NMEA0183 wind display to replace the old Robertsen wind displays
            supporting following types of tasks:
            - Reading NMEA0183 v2.x data without,
            - Parse the for wind and course and send datat to display
            - Overcome the limitations of the Arduino Nanpo V3, which stalls after ± 15 mintes
  
  NOTES:    
        1)  NMEA encoding conventions in short
            An NMEA sentence consists of a start delimiter, followed by a comma-separated sequence
            of fields, followed by the character '*' (ASCII 42), the checksum and an end-of-line marker.
            i.e. <start delimiter><field 0>,<field 1>,,,<field n>*<checksum><end-of-linemarker>
            The start delimiter is either $ or !. <field 0> contains the tag and the remaining fields
            the values. The tag is normaly a 5 character wide identifier where the 1st 2 characters
            identify the talker ID and the last 3 identify the sentence ID.
            Maximum sentence length, including the $ and <CR><LF> is 82 bytes.

            Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_nmea_0183_physical_protocol_layer


        2)  Rx2(GPIO 16) and TX2(GPIO 17) are reserved for the display communication 115200Bd
            Digital GPIO 22 (and 23) are reserved for NMEA talker via
            SoftSerial on 4800 Bd
  
  Hardware setup:
  The ESP32 has 3 Rx/Tx ports and has to be set to Serial2 
  
  Wiring Diagram (for NMEA0183 to NMEA0183 device):
  ESP32      | NMEA device
     GPIO 22 |  RX +   
     GPIO 23 |  TX + 
  
  Set the pins to the correct one for your development shield or breakout board.
  This program uses these data lines to the Nextion 3,5" Enhanced LCD,
  pin usage as follow:
  Nextion           Rx     | Tx     | 5V        | GND
  ESP32             GPIO16 | GPIO17 | 3.3V      | GND
  NOTE: The above settings works with the USB power to the ESP32

  ! Remember to set the pins to suit your display module! and that the EPS32 GPIO
  ! pins run on 3.3V (and not 5V as the Arduino)


  ---------------
  Terms of use:
  ---------------
  The software is provided "AS IS", without any warranty of any kind, express or implied,
  including but not limited to the warranties of mechantability, fitness for a particular
  purpose and noninfringement. In no event shall the authors or copyright holders be liable
  for any claim, damages or other liability, whether in an action of contract, tort or
  otherwise, arising from, out of or in connection with the software or the use or other
  dealings in the software.

  -----------
  Warning:
  -----------
  Do NOT use this compass in situations involving safety to life
  such as navigation at sea.  
        
  TO DO:    Implement 2-way communication so that incomming NMEA data can be relayed
            to other devices. An NMEA0183 network is typically a daisy chained network

  LIMITATIONS: 
            No relay of NMEA0183 data possible currently. So display needs to be
            implemented as the last node in the daisy chain.
 
  Credit:
