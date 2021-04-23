  Project:  YAZZ_WindDisplay_ESP32, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  TARGET:   ESP32
  VERSION:  1.35
  Date:     22-04-2021
  Last
  Update:   22-04-2021
            Fixed a bug in the depth calculation incase a DBK message is read
            08-11-2020
            Added TWS calculation as defined in Starpath Truewind by DAvid Burch, 2000
            TWS is calculated from AWA and SOG
            04-11-2020
            HMI now shows max SOG during trip. So prevent a false start due to sending fake 
            data to the HMI in hmiCommtest(). It now only sets the status LED to HMI_READY
            03-11-2020
            Modifications made to get it working with the NX8048P070-011R display.
            Component updates are send through their setXXX function which implements
            the recvRetCommandFinished() function. When sendCommand() is used, it needs
            to be succeeded with the recvRetCommandFinished() command
            21-10-2020
            Implemented a serial protocol to overcome the limitations of the
            32 bit register holding the paramaters to display wih a string based data set
            11-10-2020
            Tested on board and ok. This V1.0 as the master release
            02-10-2020 V0.01
            Port from Atmega328 to ESP from version 1.13
            
  Achieved: 11-10-2020 Succesful SAT  with Runnable version on ESP32 with Nextion HMI         
  Purpose:  Build an NMEA0183 wind display to replace the old Robertsen wind displays
            supporting following types of tasks:
            - Reading NMEA0183 v2.x data without,
            - Parse the for wind and course and send datat to display
            - 
  
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
  The ESP32 has 3 Rx/Tx portsand has to be set to Serial2 
  
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
        
  TO DO:    - Connect HMI to 5V from the Buck converter i.s.o. 3.3V pin on ESP32
            -Implement 2-way communication so that incomming NMEA data can be relayed
            to other devices. An NMEA0183 network is typically a daisy chained network

  LIMITATIONS: 
            As per the Arduino Nano V1.13;No relay of NMEA0183 data possible currently. 
            So display needs to be implemented as the last node in the daisy chain.
 
  Credit: 
