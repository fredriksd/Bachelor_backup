#-*- coding:utf-8-*-

import spidev
import time
import numpy as np
import binascii

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b00
spi.cshigh = False


x_max = 319
x_min = 0
y_max = 199
y_min = 0

x_center = (x_max - x_min)/2
y_center = (y_max - y_min)/2


#send = []
#buff = ["sig:","x koordinat:","y koordinat:","høyde:","bredde:"]

#Legge den i en egen fil? Litt "ryddigere"
#Begrenser val mellom min_ og max_
def constrain(val, min_, max_):
	return max(min(max_,val),min_)

class PWM():
	def __init__(self,initial_position = 1500, P_gain = 400, D_gain = 300):
		self.position = initial_position
		self.firstupdate = True
		self.P_gain = P_gain
		self.D_gain = D_gain
		self.previous_error = 0

	def update(self, error):
		if self.firstupdate == False:
			error_delta = error - self.previous_error
			vel = (self.P_gain * error + error_delta * self.D_gain)/1024

			self.position += vel
			#Begrenser utslagene på servoutgangen mellom 1000us
			#og 2000us.
			self.position = constrain(self.position, 1000, 2000)
			return self.position
			
		else:
			self.firstupdate = False
		self.previous_error = error


def get_Pixy():
	send = []
	i = 0
	resp = spi.xfer([0xaa55,0xaa55])
	
	while resp[0] == 0 and resp[1] == 0 or not resp:
		print 'Searching...'
		time.sleep(0.1)
		resp = spi.xfer([0xaa55,0xaa55])
	if resp[0] == 170 and resp[1] == 85:
		while i < 7:
			resp = spi.xfer([0xaa55,0xaa55])
			#a = resp[0]
			#b = resp[1]
			if i == 3: #x-posisjon
				send.append(resp[1])
			elif i == 4: #y-posisjon
				send.append(resp[1])
			elif i == 5: #bredde
				send.append(resp[1])
			elif i == 6: #høyde
				send.append(resp[1])
			#send.append(a)
			#send.append(b)
			time.sleep(0.1)
			i += 1
	return send
			#print "resp: ", str(resp), "i: ", str(i)

pwm_roll, pwm_pitch = PWM(), PWM()

#Legger dette inn i hovedfila
#Eksporterer denne inn i mappa til hovedfila.
while True:
	send = get_Pixy()
	while not send:
		send = get_Pixy()
	print send

	error_x = x_center - send[0]
	error_y = y_center - send[1]
	print error_x

	
	pwm_roll.update(error_x)
	pwm_pitch.update(error_y)
	













	#buf = spi.writebytes([0xaa55])
	#spi.cshigh = True 
	#spi.cshigh = False
	#resp = spi.readbytes(2)
	#string =  ''.join(["0x%02X " % x for x in resp]).strip()
	#string2 = ''.join([chr(int(''.join(c), 16)) for c in zip(string[0::2],string[1::2])])
	#spi.cshigh = False


#	i=0
#	buf = spi.writebytes([0xaa,0x55])
#	while i<15:
#		#spi.cshigh = False
#		resp = spi.xfer2([0x5b])
#		send = np.append(send, resp)
#		i+=1
		#spi.cshigh = True
	#resp = spi.xfer([0xaa55])
	#string = "".join(map(chr, resp))
	#buff = "string = %d\n" % resp
	#string = ''.join(["0x%02X " % x for x in send]).strip()
#	string2 = ''.join([chr(int(''.join(c), 16)) for c in zip(string[0::2],string[1::2])])
#	time.sleep(0.1)
#	print a 
#	print send

