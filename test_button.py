#!/usr/bin/env python
import RPi.GPIO
import time

btnStart = 21

RPi.GPIO.setmode(RPi.GPIO.BCM)
RPi.GPIO.setup(btnStart,RPi.GPIO.IN,pull_up_down = RPi.GPIO.PUD_UP)

try:
	while True:
		if(RPi.GPIO.input(btnStart) == 0):
			print("GPIO low")
		else:
			print("GPIO High")
except KeyboardInterrupt:

	pass

RPi.GPIO.cleanup