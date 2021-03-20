// Pins
const int ledPin = PB5;     // use bit 5 of Port D

// Counter and Compare Values
const uint16_t t1Load = 0;
const uint16_t t1Comp = 31250;  // (0.5s * 16MHz) / 256 = 31250 (below Timer1 max val of 65535) =>  0.5s/500ms per interrupt

void setup() {
  // Init Serial Debug Monitoring
  Serial.begin(9600);
  
  // Setup led pin (5) to be output
  DDRD |= (1 << ledPin);      // set bit 5 in DDR for port D

  // Reset Timer1 Control Reg A
  TCCR1A = 0;

  // Set CTC Mode
  TCCR1B |= (1 << WGM12);

  // Set to 256 Prescale
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);

  // Reset Timer1 and set compare value
  TCNT1 = t1Load;             // load timer with zero
  OCR1A = t1Comp;             // store match value 

  // Enable Timer1 Compare Interrupt
  TIMSK1 = (1 << OCIE1A);     // Set Interrupt Enable bit for Timer1 
  
  // Enable Global Interrupts
  sei();
}

void loop() {
  // Pretend other things are being done
  delay(500);
}

ISR(TIMER1_COMPA_vect) {
  Serial.println(millis()); // Debug output
  PORTD ^= (1 << ledPin);   // toggle ledPin output
}
