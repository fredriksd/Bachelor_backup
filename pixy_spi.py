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
GPIO.setwarnings(False)
GPIO.setup(search_light, GPIO.OUT)

#Legge den i en egen fil? Litt "ryddigere"
#Begrenser val mellom min_ og max_
def constrain(val, min_, max_):
	'''
	Låser en variable mellom min_ og max_ - verdiene satt.
	'''
	return max(min(max_,val),min_)


class PWM():
	'''PWM-klasse for motorutgangene. PWM-utgangen er PD-regulert
	og kan inverteres etter behov.
	I-ledd skal senere implementeres. 
	Parameterne til PID-regulatoren settes som argumenter til klassen.
	Dersom utgangen må reverseres, settes inverted-variabelen til True. Denne står ellers lav.
	Etter hvert skal det implementeres en throttle - PWM, som settes ved 
	"throttle = True". Denne skal brukes for vertikal navigering.
	'''

	def __init__(self,initial_position = 1500, P_gain = 400, D_gain = 300, I_gain = 0, inverted = False):
		self.u0 = initial_position
		self.inverted = inverted
		self.firstupdate = True
		self.P_gain = P_gain
		self.D_gain = D_gain
		self.I_gain = I_gain
		self.previous_error = 0
		self.position = 0
		self.start = 0
		self.start_stample = 0
		self.sample = 0
		self.stample = 0
		self.previous_sum = 0

		'''
		Implementer en Height PID Attenuation (HPA) for å redusere
		PID-verdiene ved landing (ca siste 10m). Ulineært system.
		'''
	def I_PID(self, error, sample):
		if self.firstupdate:
			self.previous_sum = self.I_gain * sample * error
			return self.previous_sum
		else:
			self.previous_sum = self.I_gain * sample * error + self.previous_sum
			return self.previous_sum
			#return  self.I_PID(self.previous_error, self.sample)

	def update(self, error):
		if self.inverted:
			error *= -1 
		if self.firstupdate == False:
			error_delta = error - self.previous_error
			self.sample = time.time() - self.start
			#Kanskje med P_gain = 10, I_gain = 1, D_gain = 5
			vel = (self.P_gain * error + error_delta * self.D_gain)
			#vel = (self.P_gain * error + error_delta * self.D_gain)/1024
			self.stample = time.time() - self.start_stample
			self.start = time.time()
			self.position = self.u0 + vel
			#Begrenser utslagene på servoutgangen mellom 1000us
			#og 2000us.
			self.position = constrain(self.position, 1000, 2000)

			return self.position

		else:
			self.firstupdate = False
			self.start = time.time()
			self.start_stample = time.time()
		self.previous_error = error


def bit_to_pixel(bit):
	'''
	Bitverdien som pixy sender ut konverteres til pixelverdi.
	'''
	#Folholdstall som brukes til å konvertere bit til pixel
	x_ratios = 639/319
	y_ratios = 399/199
	#Tar inn bit verdi(0 - 255) multipliseres med pixel-forholdstall
	bit[0] = (bit[0]*256 + bit[1]) * x_ratios
	bit.pop(1) 
	bit[1] = bit[1]*y_ratios	
	return bit

def indikering(t = 0.1, i = 0, constant = False):
	'''
	Blinking av grønt lys for indikering av status.
	t = tid, i = antall ganger
	'''
	if constant:
		GPIO.output(search_light, GPIO.HIGH)
	elif constant and i == -1:
		GPIO.output(search_light, GPIO.LOW)
	else:
		x = 0
		while x < i:
			GPIO.output(search_light,GPIO.HIGH)
			time.sleep(t)
			GPIO.output(search_light,GPIO.LOW)
			time.sleep(t)
			x += 1
'''
#Lag som klasse? Kanskje enklest
#Prøver å gjenskape koden til http://www.cmucam.org/projects/cmucam5/wiki/Porting_Guide
def getByte():
	c = spi.xfer([0])
	return c

def getWord():
	c = getByte()
	w = getByte()
	w <<= 8
	w += c
	return w
def getStart():
	lastw = 0xffff #Bare noe data uten mening
	w = getWord()
	while True:
		if w == 0 and lastw == 0: #Ingen frame
			return False 
		elif w == 0xaa55 and lastw == 0xaa55: #Nytt frame
			return True
		elif w == 0xaa55: #Resync
			getByte()
		lastw = w


#Kode for å sjekke antall frames i sekunder (Skal være 50)
i = 0
while True:
	curr = getStart()
	if curr and prev:
		i += 1
		print i
	prev = curr

skipStart = False
def getBlocks(max_blocks = 2):
	global skipStart
	blocks = []
	
	if not skipStart:
		if getStart() == False:
			return 0
		else:
			skipStart = False
	while len(blocks) < max_blocks:
		checksum = getWord()
		if checksum == 0xaa55: #pixy-start-word
			skipStart = True
			return len(blocks)
		elif checksum == 0:
			return len(blocks)
		block = []
		for i in range(5): #Block length = 5
			block.append(getWord())
		sum_ = sum(block)

		if checksum == sum_:
			blocks.append(block)
		w = getWord()
		if w != 0xaa55:
			return len(blocks)

'''

def get_Pixy():
	'''
	Leser av bildedataen fra Pixycam. 
	Dataen returneres i form av et array på 5 elementer:
	Element 1: x-posisjon
	Element 2: y-posisjon
	Element 3: Boksens bredde
	Element 4: Boksens høyde
	'''
	send = []
	i = 0
	resp = spi.xfer([0x5a,0])
	
	while resp[0] == 0 and resp[1] == 0 or not resp:
		resp = spi.xfer([0,0])
		GPIO.output(search_light, GPIO.LOW)
		return False
	if resp[0] == 170 and resp[1] == 85:
		while i < 7:
			resp = spi.xfer([0,0])
			#resp = spi.xfer([0])
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


