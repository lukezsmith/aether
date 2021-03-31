// Pin Definitions
const int txPin = PB5;

// Interrupt Counter and Compare Values 
const uint16_t t1Load = 0;
const uint16_t t1Comp = 12500;  // (0.05s * 16MHz) / 64 = 12500 (below Timer1 max val of 65535) =>  0.05s/50ms per interrupt

void setup() {

  // Setup Serial monitoring
  Serial.begin(9600);
  
  // Clear all interrupts
  cli();
  
  // Set txPin to be output
  DDRB |= (1 << txPin);

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
  // put your main code here, to run repeatedly:
}

ISR(TIMER1_COMPA_vect){
  Serial.println(millis()); //Debug output
  PORTB ^= (1 << txPin);  // toggle pin output
}
