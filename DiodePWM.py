#-*- coding:utf-8 -*-
#!usr/bin/python

import RPi.GPIO as IO         
import time


led_pin = 19
beacon_pin = 13

height = 5

IO.setwarnings(False)           
IO.setmode (IO.BCM)      
IO.setup(led_pin,IO.OUT)
IO.setup(beacon_pin, IO.OUT)


pwm = IO.PWM(led_pin,100)        	 
#pwm.start(0)

try:
	while height:
		if height > 10:
			#La beacon være på. Kobler beacon til BJT-transistoren, så skal vel dette gå.
			if not IO.input(beacon_pin):
				IO.output(beacon_pin,IO.HIGH)
				pwm.stop()	

		else:
			#Sjekker om beaconet er slått av og gjør dette bare én gang.
			if IO.input(beacon_pin):
				IO.output(beacon_pin, IO.LOW)
				pwm.start()
			for x in range(50):
				pwm.changeDutyCycle(x)
				time.sleep(.5)
			for x in range(50):
				pwm.changeDutyCycle(50-x)
				time.sleep(.5)
#except KeyboardInterrupt:
if pwm:
	pwm.stop()
IO.cleanup()

