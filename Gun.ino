/**
* Standalone arduino-boot based project
* Emulate a "gun" shot via speaker and blink the led when user press the button.
* Added powersave code to increase battery life
*
* Author: Andy Karpov <andy.karpov@gmail.com>
* Created at someday
* Modified at December 17, 2011 23:00
*/

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

const int beepPin = 11; // buzzer pin
const int ledPin = 2; // led pin
const int btnPin = 3; // button pin (!!! changed from A2 to D3 - from avr pin 25 to 5)
const int wakePin = 3; // wake up pin
int sleepStatus = 0; // sleep status
#define DEBUG 1

#define SAMPLE_RATE 8000
#include "sounddata.h"

volatile uint16_t sample;
byte lastSample;

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
    if (sample >= GUNSHOT_length) {
        if (sample == GUNSHOT_length + lastSample) {
            stopPlayback();
        }
        else {
            // Ramp down to zero to reduce the click at the end of playback.
            OCR2A = GUNSHOT_length + lastSample - sample;
        }
    }
    else {
        OCR2A = pgm_read_byte(&GUNSHOT_data[sample]);
    }

    ++sample;
}

void startPlayback()
{
    pinMode(beepPin, OUTPUT);

    // Set up Timer 2 to do pulse width modulation on the speaker
    // pin.

    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

    // Do non-inverting PWM on pin OC2A (p.155)
    // On the Arduino this is pin 11.
    TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
    TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));

    // No prescaler (p.158)
    TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set initial pulse width to the first sample.
    OCR2A = pgm_read_byte(&GUNSHOT_data[0]);


    // Set up Timer 1 to send a sample every interrupt.

    cli();

    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);

    lastSample = pgm_read_byte(&GUNSHOT_data[GUNSHOT_length-1]);
    sample = 0;
    sei();
}

void stopPlayback()
{
    // Disable playback per-sample interrupt.
    TIMSK1 &= ~_BV(OCIE1A);

    // Disable the per-sample timer completely.
    TCCR1B &= ~_BV(CS10);

    // Disable the PWM timer.
    TCCR2B &= ~_BV(CS10);

    digitalWrite(beepPin, LOW);
}

void wakeUpNow() {
}

void sleepNow() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(1, wakeUpNow, LOW);
  sleep_mode();
  sleep_disable();
  detachInterrupt(1);
}

void setup() {
    pinMode(btnPin, INPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(beepPin, OUTPUT);
    digitalWrite(btnPin, HIGH);    
    attachInterrupt(1, wakeUpNow, LOW);
}

void loop() {
  
   int btnState = digitalRead(btnPin);
   if (btnState == LOW) {
     playSound();
     int state = LOW;
     for (int i=0; i<20; i++) {
       state = (state == HIGH) ? LOW : HIGH;       
       digitalWrite(ledPin, state);
       delay(100);
     }
     sleepNow();
   }
   delay(100);
   sleepNow();
}

void playSound() {  
  startPlayback();
}
