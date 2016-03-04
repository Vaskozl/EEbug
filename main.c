/*
 * EEbug Richard Syms' Group
 * Authors: Vasko, Paul
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define maxOffset 120

void spiral(void);

volatile const uint8_t adc2 = (1 << ADLAR) | 2;
volatile const uint8_t adc3 = (1 << ADLAR) | 3;

void move(float);

void initPWM() {
  DDRB |= (1 << PB0) | (1 << PB1);
  TCCR0A = (1 << COM0A1) | // set OC0A on compare match, clear at TOP
           (1 << COM0B1) | // set OC0B on compare match, clear at TOP
           (1 << WGM00)  |  // fast PWM mode
           (1 << WGM01);

  TCCR0B = (1 << CS00) | (1 << CS01); // prescaler = 64
}

void initADC() {
  ADMUX = (1 << ADLAR) | // left shift result
          (1 << MUX1);   // use ADC2 for input (PB4) (to start)

  ADCSRA = 
           (1 << ADIE)  |  // enable ADC interrupt
           (1 << ADPS0) | // Prescaler = 8
           (1 << ADPS1);  //    - 125KHz with 1MHz clock

  ADCSRB = 0; // free running mode
  sei();
  ADCSRA |= (1 << ADSC); // start conversions
}

ISR(ADC_vect) {
  static uint8_t firstTime = 1;
  static uint8_t val;
  static float current_position;

  val = ADCH;

  //
  // The first time into this routine, the next conversion has already
  // started, so changing the channnel will not be reflected in the next
  // reading. The data sheet says it's easiest to just throw away the second
  // reading, and things will be in sync after that.
  //

  if (firstTime == 1)
    firstTime = 0;

  /*  current_position
   * -1 line is on the left
   *  0 line is in the middle
   *  1 line is on the right
   */

  else if (ADMUX == adc2) {
    ADMUX = adc3;
//    if (val > 21) {
        current_position=(float)val/255;
//      OCR0A = 160;
/*    } else {
      OCR0A = 0;
    }
      */
  }

  else if (ADMUX == adc3) {
      ADMUX = adc2;
//    if (val > 25) {
    if (val > 35) {
      current_position-=(float)val;
//      OCR0B = 160;
/*    } else {
      OCR0B = 0;
    */
    }
  }

    move(current_position);
}

void move(float current_position){
  static int previous_position = 0;
  static int offset;
  static int derivative;
  static int integral = 0;


  derivative  = current_position - previous_position;
  integral += current_position;
  previous_position = current_position;

  offset = (current_position * 120 + integral * 0  + derivative * 0);
//  offset = (current_position * 120);

  if (offset > maxOffset ) offset = maxOffset;
  if (offset < 0) offset = 0;

  OCR0A = 120 + offset;
  OCR0B = 120 - offset;
}

/*
void tune(){
  for(int8_t i = 0;i<5;i++){
  OCR0A = 120;
  OCR0B = 120;
  _delay_ms(200);

  OCR0A = 0;
  OCR0B = 0;
  _delay_ms(200);
  }
  TCCR0B = (1 << CS00) | (1 << CS01);  // prescaler = 64
}
*/

void spiral() {}

int main() {
  initPWM();
  //    tune (); // play a tune

  initADC();

  for (;;) {
    /*
    if (OCR0B == 0 && OCR0A == 0) {
    ADCSRA = 0; //disable adc
      OCR0A = 160;
      for (int i = 160; i > 0; i--) {
        OCR0B = i;
        _delay_ms(100);
      }
      OCR0A = 0;
    }
      */
  }
}
