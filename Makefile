# Everything that runs without the board.
test:
	$(MAKE) -C firmware/m33/test all
	./firmware/m33/test/test_firmware
	python3 -m unittest discover -s linux/tests -v

m33-sim:
	$(MAKE) -C firmware/m33/test m33_sim
	./firmware/m33/test/m33_sim --pty -v

clean:
	$(MAKE) -C firmware/m33/test clean

.PHONY: test m33-sim clean
