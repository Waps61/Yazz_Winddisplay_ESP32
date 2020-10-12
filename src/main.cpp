#include <Arduino.h>

/*
  Project:  YAZZ_WindDisplay_ESP32, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  TARGET:   ESP32
  VERSION:  1.0
  Date:     11-10-2020
  Last
  Update:   11-10-2020
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
*/

#include <HardwareSerial.h>
//*** Include the Nextion Display files here


//*** Since the ESP32 has only one(out of 3) Rx/Tx port free we need SoftwareSerial to
//*** setup the serial communciation with the NMEA0183 network
#include <SoftwareSerial.h>
#include <Nextion.h> //All other Nextion classes come with this libray

//*** Definitions goes here

//For testing  and development purposes only outcomment to disable
//#define WRITE_ENABLED 1
#define NEXTION_ATTACHED 1 //out comment if no display available

#define NMEA_BAUD 4800      //baudrate for NMEA communciation
#define NMEA_BUFFER_SIZE 83 // According NEA0183 specs the max char is 82 + '\0'
#define NMEA_RX 22
#define NMEA_TX 23
#define NEXTION_RX (int8_t) 16
#define NEXTION_TX (int8_t) 17

#define RED 63488  //Nextion color
#define GREEN 2016 //Nextion color

//*** define the oject tags of the Nextion display
#define WINDDISPLAY_STATUS "status"
#define WINDDISPLAY_STATUS_VALUE "status.val"
#define FIELD_BUFFER 15 //nr of char used for displaying info on Nextion

enum displayItems
{
  AWA,
  AWS,
  COG,
  SOG,
  GAUGE,
  WDIR,
  WDSTATUS
};

//*** Global scope variable declaration goes here
NexPicture dispStatus = NexPicture(0, 16, WINDDISPLAY_STATUS);

SoftwareSerial nmeaSerial;

char _AWA[FIELD_BUFFER] = {0};
char _COG[FIELD_BUFFER] = {0};
char _SOG[FIELD_BUFFER] = {0};
char _AWS[FIELD_BUFFER] = {0};
char _DIR[FIELD_BUFFER] = {0};
long _BITVAL = 0L; //32-bit register to communicate with Nextion
long oldVal = 0L;  // holds previos _BITVALUE to check if we need to send

enum nextionStatus
{
  SELFTEST = 3,
  HMI_OK = 4,
  HMI_READY = 5
};

char cvalue[FIELD_BUFFER] = {0};
uint16_t tmpVal = 0;

unsigned int ci = 0; // current index of ',' to iterate through received NMEA data
unsigned int li = 0; // last index of ',' in NMEA buffer
uint16_t cp = 0;     //current index pointer in an array
int field = 0;
bool updateDisplay = false;

const byte numChars = NMEA_BUFFER_SIZE;
char receivedChars[numChars];

bool newData = false;
unsigned long tmr1 = 0;

/*** function check if a string is a number
*/
boolean isNumeric( char *value)
{
  boolean result = true;
  int i = 0;
  while (value[i] != '\0' && result && i<FIELD_BUFFER){
      result= (isDigit( value[i] ) || value[i]=='.' || value[i]=='-');
      i++;
  }
  return result;
}

/* Display wind data onto the nextion HMI
   the 4 parameters aws,sog,awa and cog are encode in a 32bit value
   aws bit 0-5 meaning max value of 63 kts (will you blow of the planet)
   sog bit 6-11 meaning max value of 63 kts (would be world record)
   awa bit 12-20 meaning max value of 512 degrees, only need 360 though
   cog bit 21-29 meanig max valie of 512 degrees, only need 360 though
   2 most significant bits (30-31) are reserved and currently not used
   we shift << the bits from cog,awa,sog and aws respectively so all bits are in place

   The wind angle is typically represented between 0 - 180 degrees Port(-) or Starboard(+) 
   and indicated by a color red(Port) or green(Starboard) and/or in indicator >,<,R,L 
   while the gauge can only display degrees between 0 - 360. The starting point for the
   gauge is at -90 degrees from the normal compass position, so we need to add 90 degrees
   for zero!
   I receive NMEA values either between -179 and +180 degrees where the minus sign indicates 
   the wind coming from Port, depending on your windset.
   Either way we have to convert this to a value between 0-360 where the code in the HMI
   takes care of the wind incdicator; 
   HMI values 0 - 180 = Startboard
   HMI values 181 - 360 = Port
 */

/*** Converts and adjusts the incomming values to usable values for the HMI display 
 * and shifts these integer(!) values into the 32-bit register and sends the
 * 32-bit register to the Nextion HMI in timed intervals of 50ms.
 * This is due the fact that a timer in the HMI checks on new data and refreshes the 
 * display. So no need to send more data than you can chew!
 * A refresh of 20x per second is more then sufficient.
 */
void displayData()
{
  long intValue = 0L;

  // set most significant value; if cog is a number else previous value
  if( isNumeric(_COG)){
    intValue = atoi(_COG);
    // values in degrees can not be bigger than 360
    intValue = (long)(intValue % 360);
  } else intValue = (oldVal>>21) & 511; // mask 9 bit value
  _BITVAL = (long)intValue; // put value cog in register
  _BITVAL = _BITVAL << 9;   // and shift left 9 bits making room for the next 9 bits
   
  //set awa if is a number
  if( isNumeric(_AWA)){
    intValue = atoi(_AWA);
    if (intValue < -180)
      intValue *= -1; // the register has no place for signed integers
    else
      intValue += 360;
    intValue = (long)(intValue % 360); // convert values <0; i.e. -179 -> 181
  } else intValue = (oldVal>>12) & 511;
  _BITVAL ^= (long)intValue;         // XOR add value to register
  _BITVAL = _BITVAL << 6;            // and shift left 6 bits to make room for sog

  //set sog if is a number
  if( isNumeric(_SOG)){
    intValue = atoi(_SOG);
    if (intValue < 0 || intValue > 63) // test for out of range values
      intValue = (oldVal >> 6) & 63;   //0L; // use previos if true to prevent jumping values
  } else intValue = (oldVal >> 6) & 63; 
  _BITVAL ^= (long)intValue;         // XOR the value into register
  _BITVAL = _BITVAL << 6;            // ans shift left 6 bits for final value of aws

  // set aws is is a number
  if(isNumeric(_AWS)){
    intValue = atoi(_AWS);
    if (intValue < 0 || intValue > 63) // test for invalid values
      intValue = oldVal & 63;          //0L; // use previos if true to prevent jumping values
  } else intValue = oldVal & 63; 
  _BITVAL ^= (long)intValue;
  
  //*** Nextion display timer max speed is 50ms
  // so no need to send faster than 50ms otherwise
  // flooding the serialbuffer
  if (millis() - tmr1 > 50)
  {
    tmr1 = millis();
    #ifdef NEXTION_ATTACHED
    if (oldVal != _BITVAL)
    {
      oldVal = _BITVAL;
      sendCommand("code_c");     // clear the previous databuffer if present
      recvRetCommandFinished(5); // always wait for a reply from the HMI!

      nexSerial.print("sys2=");
      nexSerial.print((long)_BITVAL);
      nexSerial.write(0xFF);
      nexSerial.write(0xFF);
      nexSerial.write(0xFF);
      recvRetCommandFinished(5);
    }
    #endif
    newData = false;
  }
}

#ifdef NEXTION_ATTACHED
/*** Test if we can communicate with the HMI by putting the winddisplay in
* steps of 90 degrees starting at t0 and showing some fake results for
* sog, aws, cog and awa and finilize with a green led. Just for fun
*/
void hmiCommtest(uint16_t t0)
{
  int dir = 0;
  for (int i = t0; i < 360; i += 90)
  {
    if (i <= 180)
    {
      dir = i;
    }
    else
      dir = i - 360;
    itoa(i, _COG, FIELD_BUFFER);
    itoa(dir, _AWA, FIELD_BUFFER);
    itoa(i / 3, _SOG, FIELD_BUFFER);
    itoa(i / 3, _AWS, FIELD_BUFFER);
    displayData();
    delay(250);
  }

  
  dispStatus.setPic(HMI_READY);
}
#endif

#ifdef WRITE_ENABLED
/** actualy writes nmea data to digital pin 9 as a additional serial port while
 * inverting the data from TTL-level to RS-232 level
 * 
 * TO DO: find the correct dely time for correct data transmission; still
 * missing the first 3-4 characters
 * */
void nmeaOut(char data)
{
  byte mask;
  //startbit
  digitalWrite(9, HIGH);
  delayMicroseconds(200);
  for (mask = 0x01; mask > 0; mask <<= 1)
  {
    if (data & mask)
    {                       // choose bit
      digitalWrite(9, LOW); // send 1
    }
    else
    {
      digitalWrite(9, HIGH); // send 0
    }
    delayMicroseconds(200);
  }
  //stop bit
  digitalWrite(9, LOW);
  delayMicroseconds(200);
}
#endif

/** reads the softseroal port pin 10 and ckeks for valid nmea data starting with
 * character '$' only (~ and ! can be skipped as start charcter)
*/
void recvNMEAData()
{
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '$';
  char endMarker = '\n';
  char rc;
  
  while (nmeaSerial.available() > 0 && newData == false)
  {
    rc = nmeaSerial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string

        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      receivedChars[ndx] = rc;
      ndx++;
      recvInProgress = true;
    }
  }
}

/* only processes the receivedChars buffer and filters sentence MWV,RM and VWR,
 * which contain the SOG,COG, AWS and AWA parameters, when new data has arrived
*/
void processNMEAData()
{
  
  String sentence="";
  if (newData == true)
  {

    sentence = String(receivedChars);

    //cvalue='\0';
    tmpVal = 0;

    ci = sentence.indexOf(',', 0);
    li = sentence.indexOf(',', ci + 1);
    cp = 0;

    if (sentence.indexOf("MWV", 0) > 0 ||
        sentence.indexOf("RMC",0) > 0 ||
        sentence.indexOf("VWR",0) > 0)
    {
      field = 0; //ignore sentence tag
      while (li < sentence.length() && li < numChars)
      {

        cp = 0;
        while (ci + 1 < li)
        {
          cvalue[cp++] = sentence[ci + 1];
          ci++;
        }
        cvalue[cp] = '\0';
        field++;
        // only check for apparent or relative wind directions and speed
        if ((sentence.indexOf("MWV") > 0 && sentence.indexOf(",R,") > 0) ||
            sentence.indexOf("VWR") > 0)
        {
          if (field == 1)
          {
            memcpy(_AWA, cvalue, FIELD_BUFFER - 1);
          }
          if (field == 2)
          {
            memcpy(_DIR, cvalue, FIELD_BUFFER - 1);
            if (_DIR[0] == 'L' || _DIR[0] == 'T')
            {
              memmove(_AWA + 1, _AWA, FIELD_BUFFER - 2);
              _AWA[0] = '-';
            }
          }
          if (field == 3)
          {
            memcpy(_AWS, cvalue, FIELD_BUFFER - 1);
          }
        }

        if (sentence.indexOf("RMC") > 0)
        {
          if (field == 7)
          {
            memcpy(_SOG, cvalue, FIELD_BUFFER - 1);
          }
          if (field == 8)
          {
            memcpy(_COG, cvalue, FIELD_BUFFER - 1);
          }
        }
        ci = li;
        li = sentence.indexOf(',', ci + 1);
        if (li < 0 || li > numChars)
          li = sentence.length();
      }
    }
  }
}

#ifdef WRITE_ENABLED
// writes nmea data over digital pin as an additional serial port
void relayData()
{
  uint16_t i = 0;
  while (receivedChars[i] != '0')
  {
    nmeaOut(receivedChars[i]);

    i++;
  }
}
#endif

void setup()
{
  
//Initialize the Nextion Display; the display will run a "selftest" and takes
// about 15 seconds to finish
#ifdef WRITE_ENABLED
  pinMode(9, OUTPUT);
#endif
  
#ifdef NEXTION_ATTACHED
  nexInit();

  delay(1500);
  sendCommand("page 1");
  recvRetCommandFinished(100);
  uint32_t displayReady = SELFTEST;
  

  // wait until the display status is OK
  while (displayReady != HMI_OK)
  {
    dispStatus.getPic(&displayReady);
    delay(100);
  }
  hmiCommtest(45);
  // restet the HMI o default 0 values
  memcpy(_AWA, "0", 2);
  memcpy(_COG, "0", 2);
  memcpy(_SOG, "0", 2);
  memcpy(_AWS, "0", 2);
  displayData();
#endif
  //pinMode(10, INPUT_PULLUP);
  
  nmeaSerial.begin(NMEA_BAUD,SWSERIAL_8N1,NMEA_RX, NMEA_TX, true); 
  
}

void loop()
{
  recvNMEAData();
  if( newData ){
    processNMEAData();
    displayData();
  }
#ifdef WRITE_ENABLED
  relayData();
#endif
}
