/*
 * This emulator is written to replace Intersil CA3162E in some applications,
 * in particular in Solomon SL-30 series soldering stations.
 *
 * Differences with CA2162E:
 *  - Doesn't have differential input (but it can be done using two ADC pins).
 *  - Accepts only positive voltages, i.e. works only in configurations where
 *    CA3162E pin 10 is connected to ground. But negative voltage can be done
 *    using external level shift circuitry.
 *  - Doesn't have undervoltage or overvoltage detection.
 *  - ...
 *
 * Should work on ATmega8 and ATmega328. It uses Timer2 for BCD output protocol.
 */

#include <avr/interrupt.h>
#include <Arduino.h>

#define DEBUG

#ifndef __AVR
#error "Only for AVR arduinos"
#endif

#if defined(__AVR_ATmega8__)
#define AREF_INTERNAL (2.56)
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define AREF_INTERNAL (1.1)
#else
#error "Parameters are not defined for this chip"
#endif

// CA3162E updates 3-digit display at about 67 Hz, this gives 5 ms/digit.
#define DISPLAY_REFRESH_RATE  67
#define DISPLAY_DIGITS        3
#define TIMER_PRESCALER       1024  // CS22 = 1, CS21 = 1, CS20 = 1
// Calculate number of timer cycles, needed to get 5 ms between digits.
#define DIGIT_REFRESH_TIMER_CYCLES ( (F_CPU) / TIMER_PRESCALER / DISPLAY_REFRESH_RATE / DISPLAY_DIGITS )

#if defined(__AVR_ATmega8__)
// For compatibility with ATmega328, from Tone.cpp
#define TCCR2A TCCR2
#define TCCR2B TCCR2
#define COM2A1 COM21
#define COM2A0 COM20
#define OCR2A OCR2
#define TIMSK2 TIMSK
#define OCIE2A OCIE2
#define TIMER2_COMPA_vect TIMER2_COMP_vect
#endif

static byte input = A0;
/* LSD, NSD, MSD */
static byte digit_drivers[] = { 11, 12, 13 };
/* LSB to MSB, 4 bits */
static byte bcd_outputs[] = { 7, 8, 9, 10 };

/* Tuning, we should get reading in millivolts at the BCD output. */
static double scale_factor = 1.0;
static double scale_addition = 0.0;

static int outval = 0;

#define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))

void setup()
{
  int ii;

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("CA3162E emulator - http://github.com/lorf/ca3162e-emulator");
#endif

  for (ii = 0; ii < ARRAY_SIZE(digit_drivers); ii++)
    pinMode(digit_drivers[ii], OUTPUT);

  for (ii = 0; ii < ARRAY_SIZE(bcd_outputs); ii++)
    pinMode(bcd_outputs[ii], OUTPUT);

  // Note that this is 2.56V on ATmega8 and 1.1V on ATmega328. One can also use
  // EXTERNAL reference of about 1V on ATmega8 for better resolution.
  analogReference(INTERNAL);

  // Setup Timer2

  // Normal mode
  bitWrite(TCCR2A, WGM21, 0);
  bitWrite(TCCR2A, WGM20, 0);
#ifdef WGM22
  bitWrite(TCCR2B, WGM22, 0);
#endif

#ifdef USE_TIMER_OVERFLOW_INTERRUPT
  // CA3162E updates 3-digit display at about 67 Hz, this gives 5 ms/digit. Set
  // Timer2 prescaler to 128, gives about 244 Hz overflow interrupt at 8MHz CPU
  // clock, i.e. 4.1 ms/digit.
  bitWrite(TCCR2B, CS20, 1);
  bitWrite(TCCR2B, CS21, 0);
  bitWrite(TCCR2B, CS22, 1);

  // Enable Timer2 overflow interrupt
  bitWrite(TIMSK2, TOIE2, 1);
#else
  // CA3162E updates 3-digit display at about 67 Hz, this gives 5 ms/digit. Set
  // Timer2 prescaler to 1024 and enable compare interrupt each
  // DIGIT_REFRESH_TIMER_CYCLES clock cycles to get ~ 200 Hz compare interrupt
  // (each 5 ms).
  bitWrite(TCCR2B, CS20, 1);
  bitWrite(TCCR2B, CS21, 1);
  bitWrite(TCCR2B, CS22, 1);
  OCR2A = DIGIT_REFRESH_TIMER_CYCLES - 1;

  // Enable Timer2 compare interrupt A
  bitWrite(TIMSK2, OCIE2A, 1);
#endif
}

void loop()
{
  int reading;
  double dv;
  int val;

  reading = analogRead(input);

  /* Convert to millivolts */
  dv = (((double)reading) + scale_addition) * (1024.0/AREF_INTERNAL) * scale_factor;
  val = (int)dv;

  noInterrupts();
  outval = val;
  interrupts();

#ifdef DEBUG
  Serial.print(reading);
  Serial.print("\t");
  Serial.println(outval);
#endif

  delay(200);
}

void output_one_digit(void)
{
  static int stored_val;
  static int current_digit = 0;
  int digit, ii;

  /* Take a whole value at the start */
  if (current_digit == 0)
    stored_val = outval;
  /* Take a least significant digit */
  digit = stored_val % 10;
  /* Store store remaining digits for later use */
  stored_val /= 10;

  /* Select a digit position */
  for (ii = 0; ii < ARRAY_SIZE(digit_drivers); ii++)
    digitalWrite(digit_drivers[ii], ii == current_digit ? LOW : HIGH);

  /* Output the digit in binary form */
  for (ii = 0; ii < ARRAY_SIZE(bcd_outputs); ii++)
    digitalWrite(bcd_outputs[ii], !((digit >> ii) & 0x01));

  current_digit++;
  if (current_digit >= ARRAY_SIZE(digit_drivers) - 1)
    current_digit = 0;
}

#ifdef USE_TIMER_OVERFLOW_INTERRUPT
ISR(TIMER2_OVF_vect)
{
  output_one_digit();
}
#else
ISR(TIMER2_COMPA_vect)
{
  OCR2A += DIGIT_REFRESH_TIMER_CYCLES;
  output_one_digit();
}
#endif
