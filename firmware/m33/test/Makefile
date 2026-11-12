# Host build of the M33 logic: tests and the M33 simulator used by the Linux app's tests.
CC ?= gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -I../src
SRC = ../src/protocol.c ../src/control.c

all: test_firmware m33_sim

test_firmware: test_firmware.c $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

m33_sim: m33_sim.c $(SRC)
	$(CC) $(CFLAGS) -D_DEFAULT_SOURCE -o $@ $^

test: test_firmware
	./test_firmware

clean:
	rm -f test_firmware m33_sim

.PHONY: all test clean
