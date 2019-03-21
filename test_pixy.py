#-*- coding:utf-8 -*-
import spidev
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
#cspin = 26

#GPIO.setup(cspin, GPIO.OUT)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b00
spi.cshigh = False


#Prøver å gjenskape koden til http://www.cmucam.org/projects/cmucam5/wiki/Porting_Guide
def getByte():
	#GPIO.output(cspin, GPIO.LOW)
	c = spi.xfer([0xaa55,0xaa55]) #sync_byte_data
	#GPIO.output(cspin, GPIO.HIGH)
	return c

def getWord():
	w = getByte()
	c = getByte()
	#w <<= 8
	print w[0]
	w[0] = w[0] << 8
	w[0] += c[0]
	return w[0]


def getStart():
	lastw = 0xffff #Bare noe data uten mening
	w = spi.xfer([0x00, 0x00])
	w = (w[0]<<8) + w[1]
	while True:
		if w == 0 and lastw == 0: #Ingen frame
			return False 
		elif w == 0xaa55 and lastw == 0xaa55: #Nytt IR-frame
			print "IR-frame"
			return True
		'''elif w == 0xaa55: #Resync
			print "resync"
			getByte()
		'''
		lastw = w




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

def get_Pixy(maxBlocks = 2):
	send = [[]]
	#send1 = []
	lastw = [0xff, 0xff]
	w = spi.xfer([0x5a]) #sync
	i = 0 
	block = 0

	while True:
		w = spi.xfer([0,0])
		print w
		if (w[0] == 0 and lastw[0] == 0) and (w[1] == 0 and lastw[1] == 0): #Ingen frame
			return False 
		elif (w[0] == 0xaa and lastw[0] == 0xaa) and (w[1] == 0x55 and lastw[1] == 0x55): #Nytt IR-frame
			print "IR-frame"
			w = spi.xfer([0,0])
			for block in range(maxBlocks):
				while i < 5:
					w = spi.xfer([0,0])
					if i == 1: #x-posisjon
						send[block].append(w[0])
						send[block].append(w[1])
					elif i == 2: #y-posisjon
						#send.append(resp[0])
						send[block].append(w[1])
					elif i == 3: #bredde
						send[block].append(w[1])
					elif i == 4: #høyde
						send[block].append(w[1])
					time.sleep(0.01)
					i += 1
				w = spi.xfer([0,0])
				print "Blokk 1: " + str(send)
			
				if w[0] == 170 and w[1] == 85:
					continue
				else:
					break

					'''w = spi.xfer([0,0])
					print "w = " + str(w)
					if w[0] != 170 and w[1] != 85:
						i=0
						while i < 6:
							w = spi.xfer([0,0])
							send1.append(w[0])
							send1.append(w[1])
							i += 1
						#print blokk2
						print "Blokk 2: " + str(send1)
				#if w[0] == 170 and w[1] == 85 and i == 6: #Nytt objekt i samme frame 
					'''
			return True
		lastw = w
	'''
	
	while resp[0] == 0 and resp[1] == 0 or not resp:
		resp = spi.xfer([0xaa55,0xaa55])
		#GPIO.output(search_light, GPIO.LOW)
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
		#GPIO.output(search_light, GPIO.HIGH)
	return send
	'''
i = 0
while True:
	#send = get_Pixy()
	#print send
	get_Pixy()


'''
if __name__ == "__main__":	
	#Kode for å sjekke antall frames i sekunder (Skal være 50)
	i = 0
	prev = 0
	while True:
		curr = getStart()
		if curr and prev:
			i += 1
			print i
		prev = curr
'''