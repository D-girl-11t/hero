# Host build of the M33 logic: tests and the fake M33 used by the Linux app's tests.
CC ?= gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -I../src
SRC = ../src/protocol.c ../src/control.c

all: test_firmware fake_m33

test_firmware: test_firmware.c $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

fake_m33: fake_m33.c $(SRC)
	$(CC) $(CFLAGS) -D_DEFAULT_SOURCE -o $@ $^

test: test_firmware
	./test_firmware

clean:
	rm -f test_firmware fake_m33

.PHONY: all test clean
