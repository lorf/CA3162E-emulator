# CA3162E emulator

This emulator is written to replace Intersil/Harris CA3162E in some applications,
in particular in Solomon SL-30 series soldering stations.

Differences with CA2162E:
 * Emulator don't have differential input (but it can be done using two ADC
   pins).
 * Accepts only positive voltages, i.e. works only in configurations where
   CA3162E pin 10 is connected to ground.
 * Zero/gain ajust done in software.
 * No hold mode, no fast mode (can be easily done in software).

Should work on ATmega8 and ATmega328, may be easily ported to other ATmegas. It
uses Timer2 for BCD output.

Emulator requires minimum external components:
* Decoupling ATmega VCC and AREF pins to GND via 0.1uF ceramic caps is required.
* Interfacing with transistor based digit drivers needs ~ 1K resistors in
  transistor base lines.
* It's recommended to connect RESET pin to VCC via 10K resistor.

## Calibration

* Connect Arduino pin 9 (PB1 of ATmega, pin 15 on DIP28) to ground. This will
  force uncalibrated value to be output on the display;
* Connect your voltmeter to Arduino A0 pin (PC0 pin of ATmega, pin 23 on
  DIP28);
* Write down pair of values: value on the display, millivolt value on the
  voltmeter;
* Repeat previous step 2-3 times, at different input values. For example You
  will get the following table:

  | Display | Voltmeter, mV |
  |---------| --------------|
  |      83 |           100 |
  |     184 |           203 |
  |     307 |           329 |
  |     589 |           620 |
* Find linear approximation for this data. For eample on wolframalpha.com enter
  the data like this:

        linear fit {83, 100}, {184, 203}, {307, 329}, {589, 620}

  See result here:
  http://www.wolframalpha.com/input/?i=linear+fit+{83%2C+100}%2C+{184%2C+203}%2C+{307%2C+329}%2C+{589%2C+620}.
  You will get an expression like this:

        1.02812 x + 14.0734
* Use the first floating point number as `calib_mul` and the second number as
  `calib_add` in the sketch.

To calibrate Solomon SL-30 by temperature use the same technique, but use
measured temperature instead of voltage.
