// Please see README.md

#include <avr/interrupt.h>
#include <limits.h>
#include <Arduino.h>

// Serial output in debug mode causes flickering on the display
//#define DEBUG
#define DEMO
#define DEMO_DELAY  100

#if defined(__AVR_ATmega8__)
#define AREF_INTERNAL (2.56)
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define AREF_INTERNAL (1.1)
#else
#error "Parameters are not defined for this chip"
#endif

// Use oversampling to reduce noise
#define ADC_RESOLUTION    10 // adc_bits
#define OVERSAMPLE_BITS 5
#define OVERSAMPLE_SAMPLES (1UL << (OVERSAMPLE_BITS * 2))
#define OVERSAMPLE_MAX_VALUE (1UL << (ADC_RESOLUTION + OVERSAMPLE_BITS))

// CA3162E updates 3-digit display at about 67 Hz, this gives 5 ms/digit.
#define DIGIT_REFRESH_TIME    5000  // In microseconds

#define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))

// CA3161 accepts following codes as special symbols
#define L_DASH  10
#define L_E     11
#define L_H     12
#define L_L     13
#define L_P     14
#define L_SPACE 15

// See
// http://41.media.tumblr.com/6a7069c75223ee39d701e7c93bd8613b/tumblr_n8t59l7b2L1s5t695o1_1280.png
// for bare ATmega328 based Arduino pinout. See
// http://arduino.cc/en/Hacking/PinMapping for bare ATmega8 Arduino pinout.

// Cable pinout to easy interface with CA3162E socket:
//
// CA3162E  Arduino/DIP28/ATmega
// VCC      VCC
// 2^2      2/pin 4/PD2
// 2^3      3/pin 5/PD3
// 2^1      4/pin 6/PD4
// 2^0      5/pin 11/PD5
// NSD      6/pin 12/PD6
// MSD      7/pin 13/PD7
// LSD      8/pin 14/PB0
// GND      GND
// AIN      A0/pin 23/PC0

// Pinouts
static const byte analog_input = A0; // PC0 / Pin 23 of DIP28
static const byte output_calibrated = 9; // Connect to GND to display uncalibrated values // PB1 / pin 15 of DIP28
// LSD, NSD, MSD, active low
static const byte digit_drivers[] = { 8, 6, 7 };  // PB0, PD6, PD7 / pins 14, 12, 13 of DIP28
// LSB to MSB, 4 bits, active high
static const byte bcd_outputs[] = { 5, 4, 2, 3 }; // PD5, PD4, PD2, PD3 / pins 11, 6, 4, 5 of DIP28

// See README.md for calibration

// Calibration by measured voltage
//static const double calib_mul = 1.02867;
//static const double calib_add = 11.4881;

// Calibration by real measured SL-30 temperature
// See http://www.wolframalpha.com/input/?i=fit+{153%2C216}%2C+{205%2C292}%2C+{299%2C439}%2C+{405%2C640}%2C+{460%2C750}
static const double calib_mul = 1.74182;
static const double calib_add = -62.81;

static byte outdigits[ARRAY_SIZE(digit_drivers)] = { L_SPACE, L_SPACE, L_DASH };

static inline void set_digits(byte d2, byte d1, byte d0)
{
  outdigits[2] = d2;
  outdigits[1] = d1;
  outdigits[0] = d0;
}

// Set pins according to current digit
void output_one_digit(void)
{
  static byte current_digit = 0;
  byte ii;

  // All digits off
  for (ii = 0; ii < ARRAY_SIZE(outdigits); ii++)
    digitalWrite(digit_drivers[ii], HIGH);

  // Output the digit in binary form
  for (ii = 0; ii < ARRAY_SIZE(bcd_outputs); ii++)
    digitalWrite(bcd_outputs[ii], ((outdigits[current_digit] >> ii) & 1) ? HIGH : LOW);

  // Switch current digit on
  digitalWrite(digit_drivers[current_digit], LOW);

  current_digit++;
  if (current_digit >= ARRAY_SIZE(outdigits))
    current_digit = 0;
}

// Show digits for specified time
static inline void show_digits(byte d2, byte d1, byte d0, unsigned long time_ms)
{
  unsigned long start_ms, elapsed_ms;
  
  set_digits(d2, d1, d0);
  start_ms = millis();

  while (1) {
    elapsed_ms = millis() - start_ms;
    if (elapsed_ms > ULONG_MAX / 2) // millis() wrapped around
      elapsed_ms += time_ms;
    if (elapsed_ms >= time_ms)
      break;

    output_one_digit();
    delayMicroseconds(DIGIT_REFRESH_TIME);
    output_one_digit();
    delayMicroseconds(DIGIT_REFRESH_TIME);
    output_one_digit();
    delayMicroseconds(DIGIT_REFRESH_TIME);
  }
}

void setup()
{
  int ii;

  Serial.begin(9600);
  Serial.println("CA3162E emulator - https://github.com/lorf/CA3162E-emulator");

  // Note that this is 2.56V on ATmega8 and 1.1V on ATmega328. One can also use
  // EXTERNAL reference of about 1V on ATmega8 for better resolution.
  analogReference(INTERNAL);

  // Setup pins
  for (ii = 0; ii < ARRAY_SIZE(digit_drivers); ii++) {
    pinMode(digit_drivers[ii], OUTPUT);
    digitalWrite(digit_drivers[ii], HIGH);
  }

  for (ii = 0; ii < ARRAY_SIZE(bcd_outputs); ii++) {
    pinMode(bcd_outputs[ii], OUTPUT);
    digitalWrite(bcd_outputs[ii], LOW);
  }

  pinMode(analog_input, INPUT);

  pinMode(output_calibrated, INPUT);
  // Enable pullup
  digitalWrite(output_calibrated, HIGH);

#ifdef DEMO
  // Output scrolling "HELLO"
  show_digits(L_SPACE, L_SPACE, L_SPACE, DEMO_DELAY);
  show_digits(L_SPACE, L_SPACE, L_H, DEMO_DELAY);
  show_digits(L_SPACE, L_H, L_E, DEMO_DELAY);
  show_digits(L_H, L_E, L_L, DEMO_DELAY);
  show_digits(L_E, L_L, L_L, DEMO_DELAY);
  show_digits(L_L, L_L, 0, DEMO_DELAY);
  show_digits(L_L, 0, L_SPACE, DEMO_DELAY);
  show_digits(0, L_SPACE, L_SPACE, DEMO_DELAY);
  show_digits(L_SPACE, L_SPACE, L_SPACE, DEMO_DELAY);
#endif
}

void loop()
{
  static unsigned long cumul_reading = 0;
  static int num_samples = 0;
  static unsigned long last_us = 0;

  unsigned long elapsed_us;
  double readval, convval;
  int val;

  elapsed_us = micros() - last_us;
  if (elapsed_us > ULONG_MAX / 2) // micros() wrapped around
    elapsed_us += DIGIT_REFRESH_TIME;
  if (elapsed_us >= DIGIT_REFRESH_TIME) {
    output_one_digit();
    last_us += elapsed_us;
  }

  cumul_reading += analogRead(analog_input);
  num_samples++;

  if (num_samples == OVERSAMPLE_SAMPLES) {
    cumul_reading >>= OVERSAMPLE_BITS;

    // Convert reading to millivolts
    readval = (((double)cumul_reading * AREF_INTERNAL) / (double)OVERSAMPLE_MAX_VALUE) * 1000.0;
    convval = readval * calib_mul + calib_add;
    if(digitalRead(output_calibrated)) {
      val = (int)(convval + 0.5);
    } else {
      val = (int)(readval + 0.5);
    }      

    if (val > 999) {
      // Overvoltage
      set_digits(L_E, L_E, L_E);
    } else if (cumul_reading == 0 || val < 0) {
      // Possibly negative voltage
      set_digits(L_DASH, L_DASH, L_DASH);
    } else {
      byte d0, d1, d2;
      d0 = (byte)(val % 10);
      d1 = (byte)((val / 10) % 10);
      d2 = (byte)((val / 100) % 10);
      // Convert zero padding to spaces
      if (d2 == 0) {
        d2 = L_SPACE;
        if (d1 == 0)
          d1 = L_SPACE;
      }
      set_digits(d2, d1, d0);
    }

#ifdef DEBUG
    Serial.print("millis=");
    Serial.print(millis());
    Serial.print(", adc=");
    Serial.print(cumul_reading);
    Serial.print(", read=");
    Serial.print(readval);
    Serial.print(", conv=");
    Serial.print(convval);
    Serial.print(", out=");
    Serial.println(val);
#endif

    cumul_reading = 0;
    num_samples = 0;
  }
}
