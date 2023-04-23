#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_PIN1 PD0  // Define the LED pin as PD0
#define LED_PIN2 PD1  // Define the LED pin as PD1

volatile uint8_t pb0_state = 0; volatile uint8_t pb1_state = 0;

ISR(PCINT0_vect) {
  if (PINB & (1 << PB0)) { // Check if PB0 is high
    PORTD &= ~(1 << LED_PIN1); // Set PD0 high
    pb0_state = 1;
  } else {
    if (pb0_state == 1) { // Check if PB0 was previously high
      PORTD |= (1 << LED_PIN1);  _delay_ms(100); //blinking
      PORTD &= ~(1 << LED_PIN1); _delay_ms(100);
      PORTD |= (1 << LED_PIN1);  _delay_ms(100); 
      PORTD &= ~(1 << LED_PIN1); _delay_ms(100);      
      PORTD |= (1 << LED_PIN1); // Turn off the LED again
      pb0_state = 0;
    } else {
      PORTD |= (1 << LED_PIN1); // Set PD0 low
    }
  }
  
  if (PINB & (1 << PB1)) { // Check if PB1 is high
    PORTD &= ~(1 << LED_PIN2); // Set PD1 high
    pb1_state = 1;
  } else {
    if (pb1_state == 1) { // Check if PB1 was previously high
      PORTD |= (1 << LED_PIN2);  _delay_ms(100); //blinking
      PORTD &= ~(1 << LED_PIN2); _delay_ms(100);
      PORTD |= (1 << LED_PIN2);  _delay_ms(100);
      PORTD &= ~(1 << LED_PIN2); _delay_ms(100);
      PORTD |= (1 << LED_PIN2); // Turn off LED2 again
      pb1_state = 0;
    } else {
      PORTD |= (1 << LED_PIN2); // Set PD1 low
    }
  }
}

int main(void) {
  DDRD |= (1 << LED_PIN1); // Set PD0 as output
  DDRD |= (1 << LED_PIN2); // Set PD1 as output

  PORTD = 0xff; // Set all pins on PORTD to low

  PCMSK0 |= (1 << PCINT0); // Enable pin change interrupt on PB0
  PCMSK0 |= (1 << PCINT1); // Enable pin change interrupt on PB1
  PCICR |= (1 << PCIE0); // Enable pin change interrupt 0

  sei(); // Enable interrupts

  while (1) {
    // Do other stuff here
  }

  return 0;
}
