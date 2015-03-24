#  Emulator to replace Intersil CA3162/CA3162E using Arduino on bare ATmega

This Arduino sketch emulates Intersil CA3162/CA3162E (also NTE2054, ECG2054) in
Solomon SL-30 soldering station. It's written to run on bare ATmega8 or
ATmega328 MCU, but can be easily ported to other Arduinos/MCUs.

Differences from CA2162E:
 * Emulator don't have differential input, accepts only positive voltage, i.e.
   works only in configurations where CA3162E pin 10 is connected to ground.
 * Zero/gain ajust done in software.
 * No hold mode, no fast mode.

Emulator requires minimum external components:
* Decoupling ATmega VCC and AREF pins to GND via 0.1uF ceramic caps is required.
* Interfacing with transistor based digit drivers needs ~ 1K resistors in
  transistor base lines.
* It's recommended to connect RESET pin to VCC via 10K resistor.

## Calibration

To calibrate the emulator by voltage:

* Connect Arduino pin 9 (PB1 of ATmega, pin 15 on DIP28) to ground. This will
  force uncalibrated ADC value to be output on the display;
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
* Find linear approximation for this data. For eample on
  http://www.wolframalpha.com/ enter the data like this:
  [linear fit {83, 100}, {184, 203}, {307, 329}, {589, 620}](http://www.wolframalpha.com/input/?i=linear+fit+{83%2C+100}%2C+{184%2C+203}%2C+{307%2C+329}%2C+{589%2C+620}).
  You will get an expression like this: `1.02812 x + 14.0734`
* Use the multiplier from that expression as `calib_mul` and the addend as
  `calib_add` in the sketch.

To calibrate Solomon SL-30 by temperature use the same technique, but use
measured temperature instead of voltage.

## Relevant information
* [CA3162 datasheet](http://www.intersil.com/content/dam/Intersil/documents/ca31/ca3162.pdf)
* [CA3161 datasheet](http://www.intersil.com/content/dam/Intersil/documents/ca31/ca3161.pdf)
* Solomon soldering stations [schematics and repair
  manual](http://www.remserv.ru/cgi/download/Solomon_part1.pdf), [tuning
  manual](http://www.remserv.ru/cgi/download/Solomon_part3.pdf) - in Russian
* Using ATmega8 with internal 8MHz oscillator as minimal Arduino [tutorial](http://www.neonile.net/articles/atmega8-arduino-bootloader-optiboot) and [another one](http://todbot.com/blog/2009/05/26/minimal-arduino-with-atmega8/).
