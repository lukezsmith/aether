
#include <util/crc16.h>
#include <string.h>

// Pin Definitions
#define RADIOPIN 13

// Transmission Variables
char datastring[80];

// Interrupt Counter and Compare Values 
const uint16_t t1Load = 0;
const uint16_t t1Comp = 12500;  // (0.05s * 16MHz) / 64 = 12500 (below Timer1 max val of 65535) =>  0.05s/50ms per interrupt

void setup() {

  // Setup Serial monitoring
  Serial.begin(9600);
  
  // Clear all interrupts
  cli();
  
  // Set txPin to be output
//  DDRB |= (1 << RADIOPIN);
  pinMode(RADIOPIN,OUTPUT);
  

  // Reset Timer1 Control Reg A
  TCCR1A = 0;

  // Set CTC
  TCCR1B |= (1 << WGM12);

  // Set to 64 Prescale
//  TCCR1B |= (1 << CS12); // Set bit 2
  TCCR1B |= (1 << CS11); // Set bit 1
//  TCCR1B |= (1 << CS10); // Set bit 0

  // Reset Timer1 and Set Compare value
  TCNT1 = t1Load; // load timer with zero
  OCR1A = t1Comp; // store timer compare value

  // Enable Timer1 Interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Enable Global Interrupts
  sei();
}

void loop() {
  sprintf(datastring,"RTTY TEST BEACON RTTY TEST BEACON\n"); // Puts the text in the datastring
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);
//  delay(2000);
}

ISR(TIMER1_COMPA_vect){
  Serial.println(millis()); //Debug output
//  PORTB ^= (1 << RADIOPIN);  // toggle pin output
  rtty_txstring (datastring);
}

void rtty_txstring (char * string)
{
 
  /* Simple function to sent a char at a time to 
     ** rtty_txbyte function. 
    ** NB Each char is one byte (8 Bits)
    */
 
  char c;
 
  c = *string++;
 
  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
}
 
 
void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to 
    ** rtty_txbit function. 
    ** NB The bits are sent Least Significant Bit first
    **
    ** All chars should be preceded with a 0 and 
    ** proceded with a 1. 0 = Start bit; 1 = Stop bit
    **
    */
 
  int i;
 
  rtty_txbit (0); // Start bit
 
  // Send bits for for char LSB first 
 
  for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1); 
 
    else rtty_txbit(0); 
 
    c = c >> 1;
 
  }
 
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    digitalWrite(RADIOPIN, HIGH);
  }
  else
  {
    // low
    digitalWrite(RADIOPIN, LOW);
 
  }
 
  //                  delayMicroseconds(3370); // 300 baud
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
                            // largest value that will produce an accurate delay is 16383
                            // See : http://arduino.cc/en/Reference/DelayMicroseconds
 
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
