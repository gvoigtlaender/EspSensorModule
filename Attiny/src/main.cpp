/* Copyright 2018 Georg Voigtlaender gvoigtlaender@googlemail.com */
#include <Arduino.h>


#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdint.h>

/*
    EspSensorModule Attiny firmware
      enable module power (via RELAIS_PIN -> Tranistor > ReedRelay)
      wait for ESP8266 notifies done (IN_PIN == HIGH)
      disable module power
      sleep for [20min] with periodic flashing of LED_BUILTIN
*/

//! sleep 20min
const int64_t c_nSleepPeriodMs = 1200000;  // 20*60*1000;   // 600000;
//! correction time, obviously sleep is not accurate
//!  note: even that does not help, need to rework later
const int16_t c_nSleepPeriodCorrMs = 54000;


// Routines to set and claer bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BODCR _SFR_IO8(0x30)
#if defined ARDUINO_AVR_ATTINY13
  #define BODSE 0
  #define BODS 1
#endif


#define VERISON 2

#if defined VERSION == 1
  #define RELAIS_PIN  PB2
  #define LED_BUILTIN  PB3
  #define IN_PIN PB4
#else
  #define RELAIS_PIN  PB3
  #define LED_BUILTIN  PB2
  #define IN_PIN PB4
#endif

enum E_DELAY {
    e16MS   = 16,
    e32MS   = 32,
    e64MS   = 64,
    e125MS  = 125,
    e250MS  = 250,
    e500MS  = 500,
    e1S     = 1000,
    e2S     = 2000,
    e4S     = 4000,
    e8S     = 8000,
};

const E_DELAY c_eDelay = e4S;

// Variables for the Sleep/power down modes:
volatile boolean f_wdt = 1;

void blink(int nTimes, int nDelay) {
  for ( int n = 0; n < nTimes; n++ ) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    delay(nDelay);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
    if ( n < (nTimes-1) )
      delay(nDelay);                       // wait for a second
  }
}

// set system into the sleep state
// system wakes up when wtchdog is timed out
void system_sleep() {
  // set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();  // System continues execution here when watchdog timed out
}


void system_sleep_s(int64_t nMilliSecs, bool bBlink) {
  while ( nMilliSecs > c_eDelay ) {
    system_sleep();
    nMilliSecs -= c_eDelay;
    if ( bBlink ) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(5);
      digitalWrite(LED_BUILTIN, LOW);
      nMilliSecs -= 5;
    }
  }
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt = 1;  // set global flag
}


void setup_timer(E_DELAY eDelay) {
  // prescale timer to 8s so we can measure current
  byte bb;
  switch (eDelay) {
    case e16MS:
      bb |= (0 << WDP3)|(0 << WDP2)|(0 << WDP1)|(0 << WDP0); break;
    case e32MS:
      bb |= (0 << WDP3)|(0 << WDP2)|(0 << WDP1)|(1 << WDP0); break;
    case e64MS:
      bb |= (0 << WDP3)|(0 << WDP2)|(1 << WDP1)|(0 << WDP0); break;
    case e125MS:
      bb |= (0 << WDP3)|(0 << WDP2)|(1 << WDP1)|(1 << WDP0); break;
    case e250MS:
      bb |= (0 << WDP3)|(1 << WDP2)|(0 << WDP1)|(0 << WDP0); break;
    case e500MS:
      bb |= (0 << WDP3)|(1 << WDP2)|(0 << WDP1)|(1 << WDP0); break;
    case e1S:
      bb |= (0 << WDP3)|(1 << WDP2)|(1 << WDP1)|(0 << WDP0); break;
    case e2S:
      bb |= (0 << WDP3)|(1 << WDP2)|(1 << WDP1)|(1 << WDP0); break;
    case e4S:
      bb |= (1 << WDP3)|(0 << WDP2)|(0 << WDP1)|(0 << WDP0); break;
    case e8S:
      bb |= (1 << WDP3)|(0 << WDP2)|(0 << WDP1)|(1 << WDP0); break;
  }
  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCR = bb;
#if defined ARDUINO_AVR_ATTINY13
  WDTCR |= _BV(WDTIE);
#else
  WDTCR |= _BV(WDIE);
#endif
  // Use the Power Down sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

// cppcheck-suppress unusedFunction
// the setup function runs once when you press reset or power the board
void setup() {
  // Set up FAST PWM
  // Set control register A for Timer 0
  TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
  // Set control register B for Timer 0
  TCCR0B = 0 << WGM02 | 1 << CS00;

  ADCSRA &= ~(1 << ADEN);  // Disable ADC
  ACSR = (1 << ACD);  // Disable the analog comparator

  // Disable Brown Out Detector Control Register
  BODCR = (1 << BODS)|(1 << BODSE);
  BODCR = (1 << BODS);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(RELAIS_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN_PIN, INPUT);

  setup_timer(c_eDelay);

  blink(5, 50);                      // wait for a second
}

// cppcheck-suppress unusedFunction
// the loop function runs over and over again forever
void loop() {
  int64_t ulMillisStart = millis();
  int64_t ulTimeToSleep = c_nSleepPeriodMs;
  ulTimeToSleep -= c_nSleepPeriodCorrMs;

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
  digitalWrite(RELAIS_PIN, HIGH);   // turn the LED on

  system_sleep_s(1000, false);

  //! wait for PIN to go high
  while ( true ) {
    if ( digitalRead(IN_PIN) == LOW ) {
      delay(200);
    } else {
      // blink(1, 50);

      break;
    }
  }

  //! wait 2s
  // system_sleep();
  delay(500);

  //! wait if PIN is still high
  while ( true ) {
    if ( digitalRead(IN_PIN) == LOW ) {
      delay(200);
    } else {
      // blink(1, 50);
      break;
    }
  }

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on
  digitalWrite(RELAIS_PIN, LOW);   // turn the LED on
  // blink(3, 25);                      // wait for a second

  // delay(20000);
  // pinMode(RELAIS_PIN, INPUT);
  // pinMode(LED_BUILTIN, INPUT);
  ulTimeToSleep -= millis();
  ulTimeToSleep += ulMillisStart;
  system_sleep_s(ulTimeToSleep, true);
}
