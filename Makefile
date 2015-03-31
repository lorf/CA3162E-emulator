# Makefile for Arduino IDE >= 1.5.8
ARDUINO		?= arduino
PKG			?= arduino-bare
ARCH		?= avr
BOARD		?= atmega8o_8mhz_noxtal
CPU			?= 
PORT		?= /dev/ttyUSB0

SRC			?= $(wildcard *.ino *.pde)
RESULT		?= ${BUILD_DIR}/$(patsubst %.ino,%.cpp.hex,$(patsubst %.pde,%.cpp.hex,$(firstword $(SRC))))

ifneq (${CPU},)
BOARD_TAG	?= ${PKG}-${ARCH}-${BOARD}-${CPU}
BOARD_SPEC	?= ${PKG}:${ARCH}:${BOARD}:cpu=${CPU}
else
BOARD_TAG	?= ${PKG}-${ARCH}-${BOARD}
BOARD_SPEC	?= ${PKG}:${ARCH}:${BOARD}
endif

BUILD_DIR	?= build-${BOARD_TAG}

all: ${RESULT}

upload: ${RESULT}
	${ARDUINO} --pref build.path=${BUILD_DIR} --board ${BOARD_SPEC} --verbose --upload --port ${PORT} ${SRC}

${BUILD_DIR}/%.cpp.hex: %.ino
	${ARDUINO} --pref build.path=${BUILD_DIR} --board ${BOARD_SPEC} --verbose --verify $<

clean:
	rm -rf ${BUILD_DIR}
