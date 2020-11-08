#include <Arduino.h>

/*
  Project:  YAZZ_WindDisplay_ESP32, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  TARGET:   ESP32
  VERSION:  1.3
  Date:     08-11-2020
  Last
  Update:   08-11-2020
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
#define VERSION "1.3"
#define NEXTION_ATTACHED 1 //out comment if no display available

#define NMEA_BAUD 4800      //baudrate for NMEA communciation
#define NMEA_BUFFER_SIZE 83 // According NEA0183 specs the max char is 82 + '\0'
#define NMEA_RX 22
#define NMEA_TX 23
#define NEXTION_RX (int8_t)16
#define NEXTION_TX (int8_t)17
#define NEXTION_RCV_DELAY 100
#define NEXTION_SND_DELAY 50

#define RED 63488  //Nextion color
#define GREEN 2016 //Nextion color

//*** define the oject tags of the Nextion display
#define WINDDISPLAY_STATUS "status"
#define WINDDISPLAY_STATUS_VALUE "winddisplay.status.val"
#define WINDDISPLAY_NMEA "nmea"
#define FIELD_BUFFER 15 //nr of char used for displaying info on Nextion

#define FTM 0.3048 //conversion from feet to meter

//*** Global scope variable declaration goes here
NexPicture dispStatus = NexPicture(1, 35, WINDDISPLAY_STATUS);
NexText nmeaTxt = NexText(1, 16, WINDDISPLAY_NMEA);
NexText versionTxt = NexText(0,3,"version");
SoftwareSerial nmeaSerial;

char _AWA[FIELD_BUFFER] = {0};
char _COG[FIELD_BUFFER] = {0};
char _SOG[FIELD_BUFFER] = {0};
char _AWS[FIELD_BUFFER] = {0};
char _BAT[FIELD_BUFFER] = {0};
char _DPT[FIELD_BUFFER] = {0};
char _DIR[FIELD_BUFFER] = {0};
char _TWS[FIELD_BUFFER] = {0};
char oldVal[255] = {0}; // holds previos _BITVALUE to check if we need to send

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
boolean isNumeric(char *value)
{
  boolean result = true;
  int i = 0;
  while (value[i] != '\0' && result && i < FIELD_BUFFER)
  {
    result = (isDigit(value[i]) || value[i] == '.' || value[i] == '-');
    i++;
  }
  return result;
}

/* For backwards informational purpose only!!
    Display wind data onto the nextion HMI
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
 * and concatenates these values in one string so it can be send in one command to the 
 * Nextion HMI in timed intervals of 50ms.
 * This is due the fact that a timer in the HMI checks on new data and refreshes the 
 * display. So no need to send more data than you can chew!
 * A refresh of 20x per second is more then sufficient.
 * The string is formatted like:
 * <Sentence ID1>=<Value1>#....<Sentence IDn>=<Value_n>#
 * Sentence ID = 3 chars i.e. SOG, COG etc
 * Value can be an integer or float with 1 decimal and max 5 char long incl. delimiter
 * i.e. SOG=6.4#COG=213.2#BAT=12.5#AWA=37#AWS=15.7#
 * The order is not applicable, so can be random
 */
void displayData()
{
  char _BITVAL[255] = {0};

  // if cog is a number
  if (isNumeric(_COG))
  {
    strcat(_BITVAL, "COG=");
    strcat(_BITVAL, _COG);
    strcat(_BITVAL, "#");
  }

  //set awa if is a number
  if (isNumeric(_AWA))
  {
    strcat(_BITVAL, "AWA=");
    strcat(_BITVAL, _AWA);
    strcat(_BITVAL, "#");
  }

  //set sog if is a number
  if (isNumeric(_SOG))
  {
    strcat(_BITVAL, "SOG=");
    strcat(_BITVAL, _SOG);
    strcat(_BITVAL, "#");
  }

  // set aws is is a number
  if (isNumeric(_AWS))
  {
    strcat(_BITVAL, "AWS=");
    strcat(_BITVAL, _AWS);
    strcat(_BITVAL, "#");
  }

  // set BATT is is a number
  if (isNumeric(_BAT))
  {
    strcat(_BITVAL, "BAT=");
    strcat(_BITVAL, _BAT);
    strcat(_BITVAL, "#");
  }
  // set dpt is is a number
  if (isNumeric(_DPT))
  {
    strcat(_BITVAL, "DPT=");
    strcat(_BITVAL, _DPT);
    strcat(_BITVAL, "#");
  }
  // Calculate TWS from AWA and SOG as described Starpath TrueWind by, David Burch, 2000
  // TWS= SQRT( SOG^2*AWS^2 + (2*SOG*AWA*COS(AWA/180)))
  double sog, awa, aws, tws = 0.0;
  sog = atof(_SOG);
  awa = atof(_AWA);
  aws = atof(_AWS);
  tws= sqrt( sog*sog + aws*aws -(2*sog*aws*cos((double)awa/180)));
  sprintf(_TWS,".1%f" ,tws);
  strcat(_BITVAL,"TWS=");
  strcat(_BITVAL,_TWS);
  strcat(_BITVAL,"#");
  //*** Nextion display timer max speed is 50ms
  // so no need to send faster than 50ms otherwise
  // flooding the serialbuffer
  if (millis() - tmr1 > NEXTION_SND_DELAY)
  {
    tmr1 = millis();
#ifdef NEXTION_ATTACHED

    if (strcmp(oldVal, _BITVAL) != 0)
    {
      /*
      dbSerial.print("Preparing NMEA data: clearing buffer:");
      sendCommand("code_c");                     // clear the previous databuffer if present
      recvRetCommandFinished(NEXTION_RCV_DELAY); // always wait for a reply from the HMI!
*/
      strcpy(oldVal, _BITVAL);

      /*nexSerial.print("winddisplay.nmea.txt=");
      nexSerial.print(_BITVAL);
      nexSerial.write(0xFF);
      nexSerial.write(0xFF);
      nexSerial.write(0xFF);
      */
      dbSerial.print("Sending NMEA data: ");
      nmeaTxt.setText(_BITVAL);
      dbSerial.println(_BITVAL);
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
  
  /* dbSerial.print("Clearing databuffer:");
  sendCommand("code_c");                     // clear the previous databuffer if present
  recvRetCommandFinished(NEXTION_RCV_DELAY);
  */
  dbSerial.print(" Setting HMI to OK:");
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

  String sentence = "";
  if (newData == true)
  {

    sentence = String(receivedChars);

    //cvalue='\0';
    tmpVal = 0;

    ci = sentence.indexOf(',', 0);
    li = sentence.indexOf(',', ci + 1);
    cp = 0;

    if (sentence.indexOf("MWV", 0) > 0 ||
        sentence.indexOf("RMC", 0) > 0 ||
        sentence.indexOf("DBK", 0) > 0 ||
        sentence.indexOf("TOB", 0) > 0 ||
        sentence.indexOf("VWR", 0) > 0 ||
        sentence.indexOf("BAT", 0) > 0 ||
        sentence.indexOf("DBT", 0) > 0 ||
        sentence.indexOf("DPT", 0) > 0)
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
        if (sentence.indexOf("DBK") > 0)
        {
          if (field == 1)
          {
            memcpy(_DPT, cvalue, FIELD_BUFFER - 1);
          }
          if (field == 2 && cvalue[0] == 'f')
          {
            double dpt = atof(_DPT);
            dpt *= FTM;
            sprintf(_DPT, "%.1f", dpt);
          }
        }
        else if (sentence.indexOf("DBT") > 0)
        {
          if (field == 3)
          {
            memcpy(_DPT, cvalue, FIELD_BUFFER - 1);
          }
        }
        else if (sentence.indexOf("DPT") > 0)
        {
          if (field == 1)
          {
            memcpy(_DPT, cvalue, FIELD_BUFFER - 1);
          }
        }
        if (sentence.indexOf("TOB") > 0)
        {
          if (field == 1)
          {
            memcpy(_BAT, cvalue, FIELD_BUFFER - 1);
          }
        }
        else if (sentence.indexOf("BAT") > 0)
        {
          if (field == 2)
          {
            memcpy(_BAT, cvalue, FIELD_BUFFER - 1);
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
//Serial.begin(115200);
//Initialize the Nextion Display; the display will run a "selftest" and takes
// about 15 seconds to finish
#ifdef WRITE_ENABLED
  pinMode(9, OUTPUT);
#endif

#ifdef NEXTION_ATTACHED
  if (nexInit())
  {
    dbSerial.println("Initialisation succesful....");
  }
  else
  {
    dbSerial.println("Initialisation failed...");
    dbSerial.println("Resetting Nextion...");
    sendCommand("rest");
    delay(3000);
  }

  delay(150);
  dbSerial.print(" Writing version to splash: ");
  versionTxt.setText(VERSION);
  delay(5000);
  dbSerial.print("Switcing to page 1: ");
  sendCommand("page 1");
  recvRetCommandFinished(NEXTION_RCV_DELAY);

  uint32_t displayReady = SELFTEST;
  delay(2500);
  // wait until the display status is OK
  dbSerial.print("Getting HMI status:");
  while (displayReady < HMI_OK)
  {
    dbSerial.print(".");
    dispStatus.getPic(&displayReady);

    delay(100);
  }
  hmiCommtest(45);
  // restet the HMI o default 0 values
  memcpy(_AWA, "--.-", 5);
  memcpy(_COG, "---.-", 6);
  memcpy(_SOG, "--.-", 5);
  memcpy(_AWS, "--.-", 5);
  memcpy(_DPT, "--.-", 5);
  memcpy(_BAT, "--.-", 5);
  displayData();
#endif
  //pinMode(10, INPUT_PULLUP);

  nmeaSerial.begin(NMEA_BAUD, SWSERIAL_8N1, NMEA_RX, NMEA_TX, true);
}

void loop()
{
  recvNMEAData();
  if (newData)
  {
    processNMEAData();
    displayData();
  }
#ifdef WRITE_ENABLED
  relayData();
#endif
}
