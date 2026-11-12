# Everything that runs without the board.
test:
	$(MAKE) -C firmware/m33/test all
	./firmware/m33/test/test_firmware
	python3 -m unittest discover -s linux/tests -v

fake-m33:
	$(MAKE) -C firmware/m33/test fake_m33
	./firmware/m33/test/fake_m33 --pty -v

clean:
	$(MAKE) -C firmware/m33/test clean

.PHONY: test fake-m33 clean
