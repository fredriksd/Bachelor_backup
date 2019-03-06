#-*- coding:utf-8-*-

import spidev
import time
import RPi.GPIO as GPIO

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b00
spi.cshigh = False

search_light = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(search_light, GPIO.OUT)



#Legge den i en egen fil? Litt "ryddigere"
#Begrenser val mellom min_ og max_
def constrain(val, min_, max_):
	return max(min(max_,val),min_)


class PWM():
	'''PWM-klasse for motorutgangene. PWM-utgangen er PD-regulert
	og kan inverteres etter behov.
	'''

	def __init__(self,initial_position = 1500, P_gain = 400, D_gain = 300, inverted = False):
		self.u0 = initial_position
		self.inverted = inverted
		self.firstupdate = True
		self.P_gain = P_gain
		self.D_gain = D_gain
		self.previous_error = 0
		self.position = 0
		self.start = 0
		self.sample = 0

	def update(self, error):
		if self.inverted:
			error *= -1 
		if self.firstupdate == False:
			error_delta = error - self.previous_error
			vel = (self.P_gain * error + error_delta * self.D_gain)/1024
			self.position = self.u0 + vel
			#Begrenser utslagene på servoutgangen mellom 1000us
			#og 2000us.
			self.position = constrain(self.position, 1000, 2000)
			self.sample = time.time() - self.start

			return self.position

		else:
			self.firstupdate = False
			self.start = time.time()
		self.previous_error = error


def bit_to_pixel(bit):
	'''
	Bit verdien som pixy sender ut konverteres til pixel verdi.
	'''
	#Folholdstall som brukes til å konvertere bit til pixel
	x_ratios = 639/319
	y_ratios = 399/199
	#Tar inn bit verdi(0 - 255) multipliseres med pixel-forholdstall
	bit[0] = (bit[0]*256 + bit[1]) * x_ratios
	bit.pop(1) 
	bit[1] = bit[1]*y_ratios

	return bit



def get_Pixy():
	send = []
	i = 0
	resp = spi.xfer([0xaa55,0xaa55])
	
	while resp[0] == 0 and resp[1] == 0 or not resp:
		resp = spi.xfer([0xaa55,0xaa55])
		GPIO.output(search_light, GPIO.LOW)
		return False
	if resp[0] == 170 and resp[1] == 85:
		while i < 7:
			resp = spi.xfer([0xaa55,0xaa55])
			
			if i == 3: #x-posisjon
				send.append(resp[0])
				send.append(resp[1])
			elif i == 4: #y-posisjon
				#send.append(resp[0])
				send.append(resp[1])
			elif i == 5: #bredde
				send.append(resp[1])
			elif i == 6: #høyde
				send.append(resp[1])
			time.sleep(0.01)
			i += 1
		GPIO.output(search_light, GPIO.HIGH)
	return send


