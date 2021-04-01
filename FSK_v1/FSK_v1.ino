
#include <util/crc16.h>
#include <string.h>

// Pin Definitions
#define RADIOPIN PB5

// Transmission Variables
const int txStringLength = 80;
//char datastring[2][txStringLength];
char datastring[txStringLength];
byte transByte=0; // which byte in the string are we transmitting
byte transBit=0; // which bit in the byte are we transmitting?
char currentByte='$'; // working copy of actual byte 
byte stopBit=0; // How many stop bits left?
boolean startBit=true; // are we sending start bit?
boolean dataBits=false; // are we sending data bits?
boolean newByte=true; // do we need a new byte from the string?
byte nullByte=0; // Are we transmitting leading "null" bytes?

// Interrupt Counter and Compare Values 
const uint16_t t1Load = 0;
const uint16_t t1Comp = 5000;  // (0.05s * 16MHz) / 64 = 12500 (below Timer1 max val of 65535) =>  0.05s/50ms per interrupt
//const uint16_t t1Comp = 19999;  // (0.05s * 16MHz) / 64 = 12500 (below Timer1 max val of 65535) =>  0.05s/50ms per interrupt

void setup() {

  // Setup Serial monitoring
  Serial.begin(9600);
  
  // Clear all interrupts
  cli();
  
  // Set txPin to be output
  DDRB |= (1 << RADIOPIN);
//  pinMode(RADIOPIN,OUTPUT);
  

  // Reset Timer1 Control Reg A
  TCCR1A = 0;

  // Set CTC
  TCCR1B |= (1 << WGM12);

  // Set to 64 Prescale
  TCCR1B &= ~(1 << CS12); // Clear bit 2
  TCCR1B |= (1 << CS11); // Set bit 1
  TCCR1B |= (1 << CS10); // Set bit 0
  // Set 256 Prescale
//    TCCR1B |= (1 << CS12); // Set bit 2
//    TCCR1B |= (1 << CS11); // Set bit 1
//    TCCR1B |= (1 << CS10); // Set bit 0

  // Reset Timer1 and Set Compare value
  TCNT1 = t1Load; // load timer with zero
  OCR1A = t1Comp; // store timer compare value

  // Enable Timer1 Interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Enable Global Interrupts
  sei();
}

void loop() {
//  sprintf(datastring,"RTTY TEST BEACON RTTY TEST BEACON\n"); // Puts the text in the datastring
  char tempData[txStringLength];
  sprintf(tempData,"THIS IS AN FSK-GENERATED MESSAGE BY LUKE - HELLO GRAMPS\n"); // Puts the text in the datastring
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
//  sprintf(checksum_str, "*%04X\n", CHECKSUM);
//  strcat(tempData,checksum_str);
  cli();
  sprintf(datastring, tempData);
  sei();
//  delay(2000);
}

ISR(TIMER1_COMPA_vect){
  Serial.println(millis()); //Debug output
//  PORTB ^= (1 << RADIOPIN);  // toggle pin output
  if(newByte){  // if we are sending a new byte
    newByte = false;
//    currentByte = datastring[buffer][transByte];
    currentByte = datastring[transByte];
    transByte++;
    if(currentByte == '\0' || transByte > (txStringLength-1)){
//      buffer = !buffer; // switch buffers
      transByte = 0;
      currentByte = 0x00;
      nullByte = true;
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
//         PORTD=PIND&B01011111;
         PORTB &= ~(1 << RADIOPIN);
         startBit=false;
         dataBits=true;
         return;  // we've sent a "0" startbit - time to quit!
       }
       else{ // not a start, must be a stop - send stop bit (1) twice then get ready for new byte.
//         PORTD=PIND|B10100000;
         PORTB |= (1 << RADIOPIN);
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
         PORTB |= (1 << RADIOPIN);
       }
       else PORTB &= ~(1 << RADIOPIN); // Low bit
       transBit++;
       if(transBit==7){ // sent 7 bits already time for a stopbit...?
         transBit=0;
         dataBits=false; // will divert to stop/start bits
       }
       
     }
  
}

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
 
  return crc;
}    
