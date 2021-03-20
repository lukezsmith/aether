// Pins
const uint8_t btnPin = 2;
const uint8_t ledPin = 5;

void setup() {
  // Setup button pin (2) to be input with pull-up resistor
  DDRD &= ~(1 << btnPin);   // clear bit 2 in DDR for port D
  PORTD |= (1 << btnPin);   // enable pull up resistor for bit 2 in PORTD

  // Setup led pin (5) to be output
  DDRD |= (1 << ledPin);      // set bit 5 in DDR for port D

  // Falling edge of INTO generates interrupt
  EICRA |= (1 << ISC01);      // set bit 1 in External Interrupt Control Register A
  EICRA &= ~(1 << ISC00);     // Clear bit 0 in External Interrupt Control Register A

  // Enable Interrupts for INT0
  EIMSK |= (1 << INT0);       // Set INT0 bit in the External Interrupt Mask Register

  // Enable Global Interrupts
  sei();
}

void loop() {
  // Pretend other things are being done
  delay(500);
}

ISR(INT0_vect) {
  PORTD ^= (1 << ledPin);   // toggle ledPin output
}
