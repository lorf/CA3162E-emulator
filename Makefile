# Makefile for Arduino IDE >= 1.5
ARDUINO		?= arduino
PKG			?= arduino-bare
ARCH		?= avr
BOARD		?= atmega8o_8mhz_noxtal
PORT		?= /dev/ttyUSB0

SRC			?= CA3162E-emulator.ino
RESULT		?= ${BUILD_DIR}/CA3162E-emulator.cpp.hex

BUILD_DIR	?= build-${PKG}-${ARCH}-${BOARD}

all: ${RESULT}

upload: ${RESULT}
	${ARDUINO} --pref build.path=${BUILD_DIR} --verbose --upload --port ${PORT} ${SRC}

${BUILD_DIR}/%.cpp.hex: %.ino
	${ARDUINO} --pref build.path=${BUILD_DIR} --verbose --verify $<

clean:
	rm -rf ${BUILD_DIR}
