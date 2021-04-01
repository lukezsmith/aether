
#include <util/crc16.h>
#include <string.h>

// Pin Definitions
#define TXPIN PB5 // Radio TX Pin

// Transmission Variables
const int dataStringLength = 80;  // max length of TX datastring
//char datastring[2][dataStringLength];
char datastring[dataStringLength];  // TX datastring
uint8_t txByte = 0; // datastring current byte index indicator
uint8_t txBit = 0; // txByte current bit indicator
char currentByte = '$'; // current datastring byte
boolean stopBit = false; // Stop bit transmission state indicator
boolean startBit = true; // Start bit transmission state indicator
boolean dataBit = false; // Data bit transmission state indicator
boolean newByte = true; // new byte from datastring indicator

// Interrupt Counter and Compare Values
const uint16_t t1Load = 0;
const uint16_t t1Comp = 5000;  // (0.02s * 16MHz) / 64 = 5000 (below Timer1 max val of 65535) =>  0.02s/20ms per interrupt

void setup() {

  // Setup Serial monitoring
  Serial.begin(9600);

  // Clear all interrupts
  cli();

  // Set txPin to be output
  DDRB |= (1 << TXPIN);

  // Reset Timer1 Control Reg A
  TCCR1A = 0;

  // Set CTC
  TCCR1B |= (1 << WGM12);

  // Set to 64 Prescale
  TCCR1B &= ~(1 << CS12); // Clear bit 2
  TCCR1B |= (1 << CS11); // Set bit 1
  TCCR1B |= (1 << CS10); // Set bit 0

  // Reset Timer1 and Set Compare value
  TCNT1 = t1Load; // load timer with zero
  OCR1A = t1Comp; // store timer compare value

  // Enable Timer1 Interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Enable Global Interrupts
  sei();
}

void loop() {
  //  char tempData[dataStringLength];
  //  sprintf(tempData,"RTTY TEST BEACON RTTY TEST BEACON\n"); // Puts the text in the datastring
  //  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  //  char checksum_str[6];
  //  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  //  strcat(tempData,checksum_str);
  cli();
  sprintf(datastring, "RTTY TEST BEACON RTTY TEST BEACON\n"); // Puts the text in the datastring
  //  sprintf(datastring, tempData);
  sei();
}

ISR(TIMER1_COMPA_vect) {
  Serial.println(millis()); // Debug output
  
  // if we are sending a new byte
  if (newByte) { 
    newByte = false;
    //    currentByte = datastring[buffer][txByte];
    currentByte = datastring[txByte];
    txByte++;
    // if we have reached the end of the string or buffer reset all byte state indicators
    if (currentByte == '\0' || txByte > (dataStringLength - 1)) {  
      //      buffer = !buffer; // switch buffers
      txByte = 0;
      currentByte = 0x00;
//      nullByte = true;
    }
//    if (nullByte) { // send a second null byte at start of string.
//      currentByte = 0x00;
//      txByte = 0;
//      nullByte = false;
//    }
    txBit = 0;
    startBit = true;
    dataBit = false;
  }
  // if we are sending a start or stop bit (i.e no databit)
  if (!dataBit) { 
    if (startBit) {
      PORTB &= ~(1 << TXPIN); // send start bit (0) 
      
      // update state variables to expect data bits on next interrupt
      startBit = false;
      dataBit = true;
      return;  // start bit sent, terminate interrupt
    }
    else { // else must be a stop -  twice then get ready for new byte.
      PORTB |= (1 << TXPIN);  // send stop bit (1)

      // update stopBit state variable to handle the transmission of two stop bits instead of just one
      if (stopBit) {
        stopBit = false;
        newByte = true;
      }
      else stopBit = true;
      
      return; // stop bit sent, terminate interrupt
    }
  }
  // else we are transmitting actual data
  else { 
    // High bit (1)
    if (currentByte & (1 << txBit)) { 
      PORTB |= (1 << TXPIN);
    }
    // Low bit (0)
    else PORTB &= ~(1 << TXPIN); 
    
    txBit++;
    // Check if the character's 7 bits have been sent, update state variables
    if (txBit == 7) { 
      txBit = 0;
      dataBit = false;
    }

  }

}

// function to generate a CRC checksum to be used in the transmission string
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
