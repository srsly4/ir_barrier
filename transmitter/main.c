#include "main.h"


#include <avr/io.h> // Standard include for AVR
#include <avr/sleep.h>

#define F_CPU 12000000UL // Crystal frequency required for delay functions

#include <util/delay.h> // Delay functions

#define sbi(x, y) x |= _BV(y)                // set bit - using bitwise OR operator
#define cbi(x, y) x &= ~(_BV(y))             // clear bit - using bitwise AND operator
#define tbi(x, y) x ^= _BV(y)                // toggle bit - using bitwise XOR operator
#define is_high(x, y) (x & _BV(y) == _BV(y)) // check if the y'th bit of register 'x' is high

/* _BV(a) is a macro which returns the value corresponding to 2 to the power 'a'.
 * Thus _BV(PX3) would be 0x08 or 0b00001000. */

#define IRLED PB1

#define ADDRESS 0xAB
#define COMMAND 0x1C

#define ADC_POWER_PIN PA6
#define ADC_POWER_PORT PORTA
#define ADC_POWER_DDR DDRA


__attribute__((always_inline))
inline void pwm_en()  {
    TCCR1A |= _BV(COM1A1);
}
__attribute__((always_inline))
inline void pwm_dis() {
    TCCR1A &= ~(_BV(COM1A1));
}

void sendNEC(unsigned long data, int nbits)
{
    pwm_en();
    _delay_us(NEC_HDR_MARK);

    pwm_dis();
    _delay_us(NEC_HDR_SPACE);

    for (int i = 0; i < nbits; i++) {
        if (data & 0x01) {
            pwm_en();
            _delay_us(NEC_BIT_MARK);
            pwm_dis();
            _delay_us(NEC_ONE_SPACE);
        } else {
            pwm_en();
            _delay_us(NEC_BIT_MARK);
            pwm_dis();
            _delay_us(NEC_ZERO_SPACE);
        }
        data >>= 1;
    }
    pwm_en();
    _delay_us(NEC_BIT_MARK);
    pwm_dis();
}

void enableIROut() {
  // Enables IR output.  The khz value controls the modulation frequency in kilohertz.
  // The IR output will be on pin 6 (OC0B).
  // This routine is designed for 36-40KHz; if you use it for other values, it's up to you
  // to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
  // TIMER0 is used in phase-correct PWM mode, with OCR0A controlling the frequency and OCR0B
  // controlling the duty cycle.
  // There is no prescaling, so the output frequency is F_CPU/ (2 * OCR0A)
  
  DDRB |= _BV(IRLED); // Set as output

  PORTB &= ~(_BV(IRLED)); // When not sending PWM, we want it low

  // Normal port operation, OC0A/OC0B disconnected  
  // COM0A = 00: disconnect OC0A
  // COM0B = 00: disconnect OC0B; to send signal set to 10: OC0B non-inverted    
  // WGM0 = 101: phase-correct PWM with OCR0A as top
  // CS0 = 000: no prescaling
  TCCR1A = (1 << COM1A1) | (1 << PWM1A);
  TCCR1B = (1 << CS11) | (1 << CTC1); // prescaler x2
  // The top value for the timer.  The modulation frequency will be SYSCLOCK / 2 / OCR0A.
  OCR1C = 157;
  OCR1A = 157 * (100 - 10) / 100UL;
}

void ADC_init(void)
{
    // Select Vref=AVcc, and set the input channel to PA6 (ADC)
    ADMUX = (1 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX2) | (0 << MUX1) | (1 << MUX0);

    // Enable ADC
    ADCSR = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // Start the first conversion
    ADCSR |= (1 << ADSC);
}

uint16_t ADC_read(void)
{
    // Start a conversion
    ADCSR |= (1 << ADSC);

    // Wait for conversion to complete
    while (!(ADCSR & (1 << ADIF)));

    // Return the ADC value
    return ADC;
}

int main()
{
    // prepare ADC on PA6 for power supply monitoring
    ADC_init();

    enableIROut();

    uint16_t adc_val = ADC_read();

    // 2.56V internal ref but empirically correct with a factor
    float power = ((float)adc_val * (2.56 * 1.13)) * 2 / 1024;

    sendNEC(0x00001221 | ((uint32_t)(power * 100) << 16), 32);

    MCUSR = 0;

    // disable and clear watchdog settings
    WDTCR = (1 << WDCE) | (1 << WDE);
    WDTCR = 0;

    // set up watchdog reset
    WDTCR = (1 << WDE) | (1 << WDP2); // 0.27s

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    while(1) // Infinite loop
    {
        sleep_mode();
    }

    return 0;
}