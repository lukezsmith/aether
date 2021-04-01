/*
HAB Payload V1.0.0
Written For Aruino 1.0.3
Ugi 2013
MIT License
Based on helpful information from UKHABS.org.uk

***** Designed to run a 3.3v 8MHz ******

Functions required:
Interrupt driven RTTY engine                   Y
General buffer switching and string handling   y
Set GPS mode & confirm                         y
Read GPS data - i2c                            y
Parse GPS data                                 y - maybe some more validtion required
Read Temp Internal                             Y - actually seems stable now!
Read Temp External                             y
String handling of temperature                 y
Read Pressure                                  y
String handling of pressure                    y
Read voltage divider                           y 
String handling for voltage                    y
Checksum handling                              y
Write log to SD card                           y - could expand to store other stuff
Validation / Error control                     y - some errors caught.  GPS mode handling done.
Status LED handling                            y - GPS lock / data so far

Possible - 
Ascent/Descent detection                       ? // difficult to validate
burst detection                                ?
landing detection                              ?
payload finding (light/sound) after landing    ?
power saving modes                             N
*/

#include <string.h>
#include <util/crc16.h>
#include <WSWire.h>
#include <TinyGPS_UBX.h>
//#include <avr/pgmspace.h>
#include <SdFat.h>
SdFat sd;

#define error(s) sd.errorHalt_P(PSTR(s))


// General Variables:
const int GPS=0x42;
const boolean debug = false; // true to print serial debug messages.
const int stringLength=100;
int freeMem=0xFFFF; // global store of free memory
unsigned long landingTime=0;
boolean boredNow=false;
const unsigned long boredTime= 1*60*1000; // delay in ms before "bored" action - light etc
unsigned long flashTimer=millis();
const int flashDelay=6000;
const int flashLength=200;
unsigned long beepTimer=millis();
const int beepDelay=5000;
const int beepLength=400;
unsigned long GPSlockLostTimer=0;

// Telemetry variables:
byte hours=24, minutes=59, seconds=58;
long latitude=999999999, longitude=-888888888;
long altit=0;
long maxAltit=0; 
int altSpeed;
int inTemp=9999;
int outTemp=8888;
long pressure=999999;
unsigned int voltage=999;
char dataString[2][stringLength];
char headerString[18];
unsigned int sendCounter=0;
unsigned int satallites=0;
byte errors=0; // Bits: 0 - GPS fix; 1 - SD Card; 2- Pressure sensor; 3- MiniCam Off 4 - IntTemp off; 5 OutTemp off; 6 - Temp error; 7- GPS mode error.
boolean fix=false;
boolean volatile counterSent=false;
byte mode; // GPS transmssion mode
char missionMode='p'; // pre-launch mode.  Then Ascent, Burst, Descent, and Landing
unsigned long lastResponse=millis();  // Recover from GPS timeout
boolean lockError = false; // this checks for lock errors in flight
unsigned long lockErrorTimer = millis();


// Transmission Variables:
byte volatile buffer=0; // This will control which buffer we read and write to.
byte transByte=0; // which byte in the string are we transmitting
byte transBit=0; // which bit in the byte are we transmitting?
char currentByte='$'; // working copy of actual byte 
byte stopBit=0; // How many stop bits left?
boolean startBit=true; // are we sending start bit?
boolean dataBits=false; // are we sending data bits?
boolean newByte=true; // do we need a new byte from the string?
byte nullByte=0; // Are we transmitting leading "null" bytes?

// Pin Definitions
// D0-D1 used for UART
// D2-D3 spare.  Interrupt enabled if necessary
# define beepPin    2 // to control a siren for after landing
# define flashPin   3 // to control a transistor with 1W LED to flash after landing
# define NTXctrl    4 // this is probably redundant - not likely to use NTX enable control unless need power saving.
# define NTXdata    5 // we use direct port access for this so can't just change too easily.
# define LEDpin1    6 // Orange status LED - using for GPS lock
# define LEDpin2    7 // Blue status LED - using to parallel data pin
# define extTempPin 8 // External DS18B20
# define intTempPin 9 // Internal DS18B20
# define chipSelect 10 // SD card chip select
// D11, D12, D13 used for SPI communication with SD card
# define voltPin  A0
# define cameraPowerPin A1 // This will cut power to minautre video camera if any GPS issues.  
// Use A2 as Gnd connection for remote camera control.
// A2-A3 Spare Analogue/Digital
// A4-A5 i2c data pins

// SD Card FileName String
const char name[] = "HAB.TXT"; // maybe hard-code this to save SRAM?
boolean SDworking = true; // flag to help us to carry on if SD card fails.


// Temperature variables
unsigned long senseTimer=(millis()+500);// Temp & Presure timer - start this at 500ms offset from GPS 
const int tempDelay=1000;// one temp/press reading per second-ish
byte inTempErrors=0; // allow a few errors and then give up on temp reading.
byte outTempErrors=0;

// Pressure variables

const int Honey=0x28; // 12c address of HoneyWell Pressure sensor

// Voltage variables
// voltage divider Batt - 47K - D0 - 6.8k - Gnd
// gives 6.8/53.8 = 0.12639 volts at D0 for every 1V at Batt
// 1023 maps to 8.7029 volts
// or 117.546 units per volt.
// Multiply by 51 & divide by 60? = division by 1.176 to give units of approx 0.01V.
// takes max value to 63488 - which fits in an unsigned int at 65536.


void setup() {
  // Show we are running:
  pinMode(LEDpin1, OUTPUT);
  digitalWrite(LEDpin1, HIGH);
  delay(300);
  digitalWrite(LEDpin1,LOW);
  
  // Throughout there are many print statements conditional on "debug"
  //if(debug){
  //Serial.begin(9600);
  //  Serial.println(F("Ugi's HAB Payload - Debug mode"));
  //}
  
  delay(500); // let GPS interrogate i2c bus for EEPROM etc.
  Wire.begin();
  GPSsetup();
  pinMode(chipSelect, OUTPUT);
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)){
    //Serial.println(F("SD initialisation error.. Continuing without SD card")); 
    SDworking=false; 
    errors|=B10;
  }
  
  // Turn on NTX control - we are not actively using this ATM  
  pinMode(NTXctrl, OUTPUT);
  digitalWrite(NTXctrl, HIGH);
  pinMode(NTXdata, OUTPUT);
  
  // Setup header for first string
  sprintf(headerString,"$$Test1,%d",sendCounter);
  updateString(); // Fill first buffer
  buffer=!buffer; // switch buffers
  sendCounter++;  // Increment counter
  updateString(); // Fill second buffer
  
  // Setup interrupt for RTTY
  cli();//stop interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;//initialize counter value to 0;
    // Tweak this timing if data rate not quite right...
    OCR1A = 19999;// = ((8,000,000 / 8) / 50) - 1 (must be <65536) - 20000-1 for 50 hz on 8 MHz clock
    TCCR1B |= (1 << WGM12);// set CTC mode
    TCCR1B |= (1 << CS11);//| (1 << CS10); //8 pre-scaler uncomment CS10 for 64 scaler to debug. 
    TIMSK1 |= (1 << OCIE1A); // set interrupt
  sei();//allow interrupts

  tempSetup(intTempPin);
  tempSetup(extTempPin);
  voltSetup();
  pinMode(flashPin, OUTPUT);
  digitalWrite(flashPin, HIGH); // test flash
  delay(100);
  digitalWrite(flashPin, LOW);
  
  pinMode(beepPin, OUTPUT);
  digitalWrite(beepPin, HIGH);
  delay(300);
  digitalWrite(beepPin,LOW);
  
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
  
  pinMode(cameraPowerPin, OUTPUT);
  digitalWrite(cameraPowerPin, LOW); // turn on minature camera power

  pinMode(LEDpin2, OUTPUT);
  digitalWrite(LEDpin2, HIGH);
  delay(300);
  digitalWrite(LEDpin2,LOW);
    
}
 
// Transmission Interrupt handler:
  ISR(TIMER1_COMPA_vect){//Interrupt Handling Routine - send RTTY 50HZ
       if(newByte){ // check for and send second leading null-byte
         newByte=false;
         currentByte=dataString[buffer][transByte];
         transByte++;
         if (currentByte=='\0' || transByte>(stringLength-1)){  // don't overrun string or buffer
           buffer=!buffer;
           counterSent=true; // we can update the counter now.
           transByte=0; // Start at beginning of string next time
           currentByte=0x00; // set currentByte so that we transmit a first null byte before each new string for ease of decode.
           nullByte=true;
         }
         if(nullByte){ // send a second null byte at start of string.
           currentByte=0x00;
           transByte=0;
           nullByte=false;
         }
         transBit=0;  
         startBit=true; 
         dataBits=false;
       }
     if(!dataBits){ // start or stop bit
       if(startBit){ // send start bit (0) and get ready for data next time
         //if(debug) Serial.println("start bit");
         PORTD=PIND&B01011111;
         startBit=false;
         dataBits=true;
         return;  // we've sent a "0" startbit - time to quit!
       }
       else{ // not a start, must be a stop - send stop bit (1) twice then get ready for new byte.
         PORTD=PIND|B10100000;
         stopBit--;
         if(!stopBit){
           stopBit=2;
           newByte=true;
         }
         return; // we've sent a "1" stopbit - time to quit.
       }
     } 
     else{// we are sending actual data
       if(currentByte&(1<<transBit)){ // High bit
         PORTD=PIND|B10100000;
       }
       else PORTD=PIND&B01011111; // Low bit
       transBit++;
       if(transBit==7){ // sent 7 bits already time for a stopbit...?
         transBit=0;
         dataBits=false; // will divert to stop/start bits
       }
       
     }
  } // End of interrupt routine

// Main Action - sort out transmission counter, poll GPS, check and update temp & press, read voltage.
void loop() {
 if (counterSent){ // if we have switched buffers then update counter.
   sprintf(headerString,"$$Test1,%d",sendCounter);
   sendCounter++;
   counterSent=false;
   if(SDworking){ // Write to SD the string that has just been sent...
     errors&=B11111101; // clear SD error
     ofstream sdout(name, ios::out | ios::app);
     if (!sdout){SDworking = false; errors|=B10;} // Serial.println(F("open failed"))
     sdout << dataString[!buffer];
     //if (debug){int free=freeRam();}// Serial.print(F("Writing SD. FreeRam="));Serial.println(free, HEX);
     sdout.close();
     if (!sdout){SDworking = false; errors|=B10;}// Serial.println(F("append data failed"));
     // print to the serial port too:
     //if(debug){Serial.print("SD card:");Serial.println(dataString[buffer]);}
     //if(debug)Serial.println(headerString);  
   }
   else errors|=B10;
   //@@@@
   delay(100);// Let SD update before changing string...   
   updateString();  // get a new string ready, just in case.
 }
 
 // Read GPS
 if (GPScheck()){  // Reads GPS returns true if new data
   updateString(); // Update new string with new GSP data and current sensor data
 }
 
 // If we are too slow (esp with SD card write), GPS can time out.
 // This should rescue us and stop us waiting for ever for GPS data
 if(millis()-lastResponse>2500){ // 2.5 seconds since last GPS update?
   char GPSstring[17]="$PUBX,00*33\r\n";
   GPSsend (GPSstring, ((sizeof(GPSstring))-1)); // Poll GPS again
   lastResponse=millis();
  }
 
// Check sensors every second...
 if(millis()-senseTimer>tempDelay){
   errors&=B10111111; // clear temperature errors
   if(inTempErrors<6){// allow up to 5 consecutive retries before giving up on temp sensor
     int temp=tempGet(intTempPin);
     if(temp != -27400){inTemp=temp;inTempErrors=0;errors&=B11101111;}
     else {inTempErrors++; inTemp=0; errors|=B00010000;}
   }
    
   if(outTempErrors<6){
     int temp=tempGet(extTempPin);
     if(temp != -27400){outTemp=temp;outTempErrors=0;errors&=B11011111;}
     else {outTempErrors++; outTemp=0;errors|=B00100000;}
   }
   readPress();
   senseTimer=millis();
   
   
 // Test of Mode routine.. use 1-sec calling of this loop for timing
 //@@@@@
 ///if(random(100)>98)missionMode='l';
 // End test
   
 // Test of Bored routine.. use 1-sec calling of this loop for timing
 // if(random(100)>98)boredNow=true;
 // End test
 } // end sensor routine
 
 voltage=voltGet();
 
 // Is this right?
 if(missionMode=='l' && (millis()-landingTime)>(boredTime)){ boredNow=true; beepTimer=millis(); flashTimer=millis();} // if we've landed an hour ago then start beeps, flashes etc.
 
 // 
 if(boredNow){
   if((millis()-beepTimer)<beepLength){
     digitalWrite(beepPin, HIGH);
   }
   else{
     digitalWrite(beepPin, LOW);
     if((millis()-beepTimer)>beepDelay) beepTimer=millis();
   }
   if((millis()-flashTimer)<flashLength){
     digitalWrite(flashPin, HIGH);
   }
   else{
     digitalWrite(flashPin, LOW);
     if((millis()-flashTimer)>flashDelay) flashTimer=millis();
   }
 }
 
 if(maxAltit>100000 && (errors&1)){// if have taken off and GPS not locked
   if(lockError){
     
     if((millis()-lockErrorTimer)>(2*60*1000));{ // if we are in flight and have a lock error for, turn off power to video.
       pinMode(cameraPowerPin, INPUT);
       errors|=B1000;// Set camera off error code
     }
     
   }
   else{
     lockError=true;
     lockErrorTimer=millis();
   }
 
 }
 if(!(errors&B1)) lockError=false; // if we regain lock before timeout (5 mins) then can allow camera to run
 
 
} // end of main loop
 
 
// this will add all of the parts of the string together into the buffer
// Get the data a piece at a time into a temp string first
// Clear interrupts while we actually update
// to avoid part-updated transmissions
// Could use some error-checking and/or reporting in here?
void updateString(){
  boolean neg=false;
  char tempData[stringLength];
  char temp[12];
  
  // We will put each part of the string into temp and then add each one to the end of tempData in turn
   
  
  sprintf(tempData,"%s,%02d:%02d:%02d,", headerString, hours, minutes,seconds); // max (header) + 2+1+2+1+2+1 = header + 9 = 25-ish
  //if(debug){ Serial.print(hours);Serial.print(":");Serial.print(minutes);Serial.print(":");Serial.println(seconds);Serial.println(tempData);}
  
  if(latitude<0){strcat(tempData, "-"); latitude=-latitude; neg=true;}
  sprintf(temp, "%ld.%05ld,", (latitude/100000), (latitude%100000)); // max 1+2+1+5+1 = 10 char
  strcat(tempData, temp);
  if(neg){latitude=-latitude; neg=false;}
  //if(debug) Serial.println(latitude);
  //if(debug) Serial.println(temp);
  //if(debug) Serial.println(tempData);
  
  if(longitude<0){strcat(tempData, "-"); longitude=-longitude; neg=true;} // max 1+3+1+5+1 = 11 char
  sprintf(temp, "%ld.%05ld,", (longitude/100000), (longitude%100000));
  strcat(tempData, temp);
  if(neg){longitude=-longitude; neg=false;}
  //if(debug) Serial.println(temp);
  //if(debug) Serial.println(tempData);
  
  if(altit<0)altit=0; // don't care about -ve altitude! - max 5+1=6 char
  //sprintf(temp,"%ld.%02ld,",(altit/100),(altit%100));
  sprintf(temp,"%ld,",(altit/100)); // not much point in altitude to less than 1m
  strcat(tempData, temp);
  //if(debug) Serial.println(temp);
  //if(debug) Serial.println(tempData);
  
  sprintf(temp,"%d,",satallites); // max 2+1 = 3 char
  strcat(tempData, temp);
  //if(debug) Serial.println(temp);
  //if(debug) Serial.println(tempData);
  
  if(inTemp<0){strcat(tempData, "-"); inTemp=-inTemp; neg=true;} // max 1+3+1+2+1 = 8 char
  sprintf(temp, "%d.%02d,", (inTemp/100),(inTemp%100));
  strcat(tempData, temp);
  if(neg){inTemp=-inTemp; neg=false;}
  //if(debug) Serial.println(temp);
  //if(debug) Serial.println(tempData);
  
  if(outTemp<0){strcat(tempData, "-"); outTemp=-outTemp; neg=true;} // max 1+3+1+2+1 = 8 char
  sprintf(temp, "%d.%02d,", (outTemp/100),(outTemp%100));
  strcat(tempData, temp);
  if(neg){outTemp=-outTemp; neg=false;}
  //if(debug) Serial.println(temp);
  //if(debug) Serial.println(tempData);
  
  if(pressure<0){strcat(tempData, "-"); pressure=-pressure; neg=true;} // allow for -ve pressure - max 1+4+1 = 6 char
  sprintf(temp, "%ld,", (pressure/100)); // Held to 0.01 but decimal not meaningful.
  strcat(tempData, temp);
  if(neg){pressure=-pressure; neg=false;}
  //if(debug) Serial.println(temp);
  //if(debug) Serial.println(tempData);
  
  sprintf(temp, "%d.%02d,", (voltage/100),(voltage%100)); // max 2+1+2+1 = 6 char
  strcat(tempData, temp);
  //if(debug) Serial.println(temp);
  //if(debug) Serial.println(tempData);
  
  sprintf(temp, "%02X%c%04X",errors,missionMode,freeMem); // max 2+1+4+1 = 8 char
  strcat(tempData, temp);
  
  unsigned int CHECKSUM = gps_CRC16_checksum(tempData);  // Calculates the checksum for this dataString max 5 char
  sprintf(temp, "*%04X\n", CHECKSUM);
  strcat(tempData,temp);

   
    //if(debug) Serial.println(temp);
  //@@@@
 //Serial.println(tempData);
 //Serial.println(buffer, HEX);
  
  // TOTAL = 96 characters maximum. Should never over-run 100 char buffers
  
  cli();
  sprintf(dataString[!buffer], tempData); // move the data to the real send-buffer
  sei();
  
  freeMem=freeRam(); // Reset to free RAM now.  It may go down
   
  // if(debug){Serial.print(F("String updated. FreeRam="));Serial.println(freeMem, HEX);}
  // if(debug) Serial.println(dataString[!buffer]);
}
 
// Checksum code from UKHAS Web site
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
  int free=freeRam();
  if (free<freeMem)freeMem=free;
  return crc;
}

void GPSsend(char* string, byte length){
 Wire.beginTransmission(GPS);
 for(byte i=0; i<length; i++){
   Wire.write(string[i]);
   //Serial.print(string[i]);
   if (i&B11111==31){ // if we try to send more than 32 bytes we'll overrun the buffer so restart
     Wire.endTransmission();
     Wire.beginTransmission(GPS);
   }
 }
 byte unsuccess=Wire.endTransmission();
 if(unsuccess){errors|=B10000000;}
 //if(unsuccess) {Serial.print(F("error sending: ")); Serial.println(unsuccess, DEC);}
 //if (debug){int free=freeRam();Serial.print(F("GPS sent. FreeRam="));Serial.println(free, HEX);}
 int free=freeRam();
 if (free<freeMem)freeMem=free;
}

void GPSsetup(){
  //Serial.println("setting upGPS");
  char PUBX[45]={  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, // Set to flight mode 6 - we want an ack  for this!
                   0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 
                   0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                   0x00, 0x00, 0x16, 0xDC};
                  
 GPSsend(PUBX, 44);
 int avail=0;
 for(byte i=0; i<200; i++){
    Wire.beginTransmission(GPS);
    Wire.write(0xfd);
    Wire.endTransmission();
    Wire.requestFrom(GPS,2);
    avail = (Wire.read()<<8)+Wire.read();
    if(avail>=10) break;
    delay(5); // wait up to 1s for response
  }
  if(avail){ // Read back ack - check for key bytes.
    for (byte i=0; i<10; i++){
     Wire.requestFrom(GPS,1);
     byte temp=Wire.read();
     //Serial.print(temp,HEX);Serial.print(",");
     if(i==6 && temp!=0x06)errors|=B10000000; // discount read if Ack is not correct
     if(i==7 && temp!=0x24)errors|=B10000000; // Error code should trigger re-run of setup
    }
  }
  else{ // No ack in 1 second - GPS error
    errors|=B10000000;
  }
    
 //if(debug){ Serial.println();byte temp=GPSgetMode(); Serial.println(temp, DEC);}
 // turn off other strings - less worried if this doesn't work.
 sprintf(PUBX, "$PUBX,40,GLL,0,0,0,0*5C\r\n");
 GPSsend(PUBX, ((sizeof(PUBX)-1)));
 sprintf(PUBX, "$PUBX,40,ZDA,0,0,0,0*44\r\n");
 GPSsend(PUBX, ((sizeof(PUBX)-1)));
 sprintf(PUBX, "$PUBX,40,VTG,0,0,0,0*5E\r\n");
 GPSsend(PUBX, ((sizeof(PUBX)-1)));
 sprintf(PUBX, "$PUBX,40,GSV,0,0,0,0*59\r\n");
 GPSsend(PUBX, ((sizeof(PUBX)-1)));
 sprintf(PUBX, "$PUBX,40,GSA,0,0,0,0*4E\r\n");
 GPSsend(PUBX, ((sizeof(PUBX)-1)));
 sprintf(PUBX, "$PUBX,40,RMC,0,0,0,0*47\r\n");
 GPSsend(PUBX, ((sizeof(PUBX)-1)));
 sprintf(PUBX, "$PUBX,40,GGA,0,0,0,0*5A\r\n");
 GPSsend(PUBX, ((sizeof(PUBX)-1)));
 
 // Set fast baud rate to avoid hanging around on i2c
 sprintf(PUBX,"$PUBX,41,1,0007,0003,115200,0*13\r\n"); 
 GPSsend(PUBX, ((sizeof(PUBX)-1)));
 
 // Request the string we want:
 sprintf(PUBX,"$PUBX,00*33\r\n");
 GPSsend(PUBX, (sizeof(PUBX)-1));
 int free=freeRam();
 if (free<freeMem)freeMem=free;
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

boolean GPScheck(){
  Wire.beginTransmission(GPS);
  Wire.write(0xfd);
  Wire.endTransmission();
  Wire.requestFrom(GPS,2);
  int avail = (Wire.read()<<8)+Wire.read();
  if(avail){
    //Serial.println("GPS data:");
    TinyGPS decodeGPS;
    for (int i=0; i<avail; i++){
    Wire.requestFrom(GPS,1);
    byte temp=Wire.read();
    while (temp==255){
      //Serial.print("!");
      Wire.requestFrom(GPS,1); 
      temp=Wire.read();
    }
    //Serial.print(char(temp));
    decodeGPS.encode(temp);
    }
    GPSparse(decodeGPS);
   
   // Check mode before we request a new string 
   errors&=B01111111;  // clear flight mode error code.
   mode=GPSgetMode(); // check mode
   //Serial.println(mode, DEC);
   
   // Ped mode test
   //missionMode='l';
   
   if(missionMode=='l'){ // if we have landed, switch UBLOX to pedestrian mode
     if(mode!=3){
       char PUBX[45]={  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, // Set to flight mode 6 - we want an ack  for this!
                   0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 
                   0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                   0x00, 0x00, 0x13, 0x76};
                  
   GPSsend(PUBX, 44);
     }
   }
   else{
    if(mode!=6){  // if not mode 6, check again in case of mis-reading
      mode=GPSgetMode();
    }
    
    // Test code - random mode failure
    //if(random(100)>95) mode=3;
    // End test code
   
    if(mode!=6){  // if still not reading mode 6 re-initialise GPS
      GPSsetup();
      mode=GPSgetMode();
    }
    
  }
  if(mode==6) errors&=B01111111;
  else errors|=B10000000;         // if mode now 6 clear error code.
                                  // This will give an error code after we switch to Ped mode on landing but that's OK 'cos we will know that switch happened
  // Request another PUBX string
  char GPSstring[17]="$PUBX,00*33\r\n";
  GPSsend (GPSstring, ((sizeof(GPSstring))-1));
  lastResponse=millis(); // 
  int free=freeRam();
  if (free<freeMem)freeMem=free;
  return true;
  }
return false;
}

void GPSparse(TinyGPS &gps){
  unsigned long age;
  gps.get_position(&latitude, &longitude, &age);
 // if(debug){Serial.print("Lat/Long(10^-5 deg): "); Serial.print(latitude); Serial.print(", "); Serial.print(longitude); 

  gps.crack_time(&hours, &minutes, &seconds, &age);
  //if(debug){Serial.print("Time: "); Serial.print(static_cast<int>(hours)); Serial.print(":"); Serial.print(static_cast<int>(minutes)); Serial.print(":"); Serial.print(static_cast<int>(seconds));
  
  altit=gps.altitude();
  //if(debug){Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): "); Serial.print(gps.course());}
  
  fix=gps.has_fix();
  if(fix && altit>maxAltit)maxAltit=altit; // increment maxAltit if appropriate

  
  satallites=gps.sats();
  if(fix && altit<100000){errors&=B11111110; digitalWrite(LEDpin1, HIGH);} // not much point displaying LED above 1000m!
  else {
   digitalWrite(LEDpin1, LOW);
   errors|=1;
  }
  if(missionMode=='p' && altit>100000 && gps.vspeed()>100)missionMode='a'; // if we are above 1000 meters and rising 1 m/s or more then we are in ascent mode
  if(missionMode=='a' && altit<(maxAltit-10000) && gps.vspeed()<-1000)missionMode='b'; // if we are falling at -10m/s and are lower than our max by 100m then we have burst
  if(missionMode=='b' &&(maxAltit-altit>500000) && gps.vspeed()<-500)missionMode='d'; //  if we have dropped 5k from max altitude then descending
  if(maxAltit>200000 && altit<100000 && gps.vspeed()>-100){missionMode='l'; landingTime=millis();}// If we have been high, are less than 1K and less than 1m/s velocity - we've landed.
  int free=freeRam();
  if (free<freeMem)freeMem=free;
  //if(debug){Serial.print("Vspeed (cm/s): "); Serial.println(gps.vspeed());
  //Serial.print("Sats: "); Serial.println(gps.sats());
  //Serial.print("has fix: "); Serial.print(gps.has_fix(), DEC); Serial.print("; fix q: "); Serial.println(gps.fix_quality());}

}


byte GPSgetMode(){
 char GPSstring[17]= {0xB5,0x62,0x06,0x24,0x00,0x00,0x2A,0x84};
 GPSsend (GPSstring, ((sizeof(GPSstring))-1));
  byte GPSmode=0;
  int avail=0;
  for(byte i=0; i<200; i++){
    Wire.beginTransmission(GPS);
    Wire.write(0xfd);
    Wire.endTransmission();
    Wire.requestFrom(GPS,2);
    avail = (Wire.read()<<8)+Wire.read();
    if(avail>=54) break;
    delay(5); // wait up to 1s for response
  }
  if(avail){
    for (int i=0; i<54; i++){
     Wire.requestFrom(GPS,1);
     byte temp=Wire.read();
     //Serial.print(temp,HEX);Serial.print(",");
     if(i==8) GPSmode=temp;
     if(i==50 && temp!=0x06)GPSmode=255; // discount read if Ack is not correct
     if(i==51 && temp!=0x24)GPSmode=255;
    }
  }
 int free=freeRam();
 if (free<freeMem)freeMem=free;
 return GPSmode;
}

// DS18D20 Interface Code:

int tempGet(int tempPin){
  byte HighByte, LowByte;
  int TReading; 
  //SignBit, Tc_100, Whole, Fract;
  
    byte OK = OneWireReset(tempPin);
    if(!OK) return -27400;
    OneWireOutByte(tempPin, 0xcc); // skip ROM
    OneWireOutByte(tempPin, 0xbe); // Read Scratchpad

    LowByte = OneWireInByte(tempPin);
    HighByte = OneWireInByte(tempPin);
    TReading = (HighByte << 8) + LowByte;
    byte SignBit = TReading & 0x8000;  // test most sig bit
    if (SignBit) TReading = - TReading; // set +ve for scaling. not sure if I ned this...
    TReading = ( (6 * TReading) + TReading / 4); // convert to 100ths of degrees C
    if(SignBit) TReading =-TReading; // restore sign
    
    // request new reading...
    OK = OneWireReset(tempPin);
    OneWireOutByte(tempPin, 0xcc);
    OneWireOutByte(tempPin, 0x44); // perform temperature conversion
    
    int free=freeRam();
    if (free<freeMem)freeMem=free;
    
    return(TReading);
  
}


boolean OneWireReset(int Pin) // reset.  Should improve to act as a presence pulse
{
     digitalWrite(Pin, LOW);
     pinMode(Pin, OUTPUT); // bring low for 500 us
     delayMicroseconds(500);
     pinMode(Pin, INPUT);
     delayMicroseconds(65);
     byte presence = digitalRead(Pin);
     delayMicroseconds(450);
     if (presence) {
       errors|=B01000000;
       //Serial.println("Reset Presence Error");
       return false;
     }
     return true;
}

void OneWireOutByte(int Pin, byte d) // output byte d (least sig bit first).
{
   byte n;

   for(n=8; n!=0; n--)
   {
      if ((d & 0x01) == 1)  // test least sig bit
      {
         digitalWrite(Pin, LOW);
         pinMode(Pin, OUTPUT);
         delayMicroseconds(3);
         pinMode(Pin, INPUT);
         delayMicroseconds(60);
      }
      else
      {
         digitalWrite(Pin, LOW);
         pinMode(Pin, OUTPUT);
         delayMicroseconds(60);
         pinMode(Pin, INPUT);
      }

      d=d>>1; // now the next bit is in the least sig bit position.
   }

}

byte OneWireInByte(int Pin) // read byte, least sig bit first. Needs interrupts diabled to be reliable
{
    byte d, n, b;

    for (n=0; n<8; n++)
    {
        digitalWrite(Pin, LOW);
        cli();
        pinMode(Pin, OUTPUT);
        delayMicroseconds(4);
        pinMode(Pin, INPUT);
        delayMicroseconds(3);
        b = digitalRead(Pin);
        sei();
        delayMicroseconds(55);
        d = (d >> 1) | (b<<7); // shift d to right and insert b in most sig bit position
    }
    return(d);
}

void tempSetup(int tempPin){
  senseTimer=millis()+500;
  digitalWrite(tempPin, LOW);
  pinMode(tempPin, INPUT);
  OneWireReset(tempPin);
  OneWireOutByte(tempPin, 0xcc);
  OneWireOutByte(tempPin, 0x44); // perform temperature conversion
}

void readPress(){
  int temp=0;
  Wire.requestFrom(Honey,4);
  pressure=(Wire.read()<<8)+Wire.read();
  temp=(Wire.read()<<8)+Wire.read(); // we may be able to derive temperature from this also
  if(pressure & 0xC000)errors|=B100;
  else errors&=B11111011;    
  pressure=(pressure&0x3FFF)-0x0666; // leaves us with a number for 1 bar in range 0 to 13107!
  pressure*=100000; // just fits in long integar
  pressure/=13107; // map to 0 to 100000 - 0.01 mbar
  //if(debug){Serial.print(pressure, DEC);  Serial.println();}
  int free=freeRam();
  if (free<freeMem)freeMem=free;
}
 
 void voltSetup(){
   
   pinMode(voltPin, INPUT);
   analogReference(INTERNAL);
   
 }
 
 unsigned int voltGet(){
   unsigned tempVolt=analogRead(voltPin);
   tempVolt*=51;
   tempVolt/=60;// Scale from 119 units per volt to 100 units per volt (approx)
   return tempVolt;
 }

