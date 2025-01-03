#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

BeepPin = 26    # 26

def setup():
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)        # Numbers pins by physical location
	GPIO.setup(BeepPin, GPIO.OUT)   # Set pin mode as output
	GPIO.output(BeepPin, GPIO.LOW) # Set pin to low(+3.3V) to off the beep

def stop():
	GPIO.output(BeepPin, GPIO.LOW)    # beep off

def loop():
	for i in range(1, 10):
		GPIO.output(BeepPin, GPIO.LOW)
		time.sleep(0.1)
		GPIO.output(BeepPin, GPIO.HIGH)
		time.sleep(0.1)
	stop()

def destroy():
	stop()
	GPIO.cleanup()                     # Release resource

if __name__ == '__main__':     # Program start from here
	print('Press Ctrl+C to end the program...')
	setup()
	try:
		loop()
	except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
		destroy()

