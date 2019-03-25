#-*- coding:utf-8-*-

import spidev
import time
import RPi.GPIO as GPIO

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1953000
spi.mode = 0b00
spi.cshigh = False

search_light = 21

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(search_light, GPIO.OUT)


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


lastw = [0xff, 0xff]
def get_Pixy(maxBlocks = 2):
    '''
    Leser av bildedataen fra Pixycam.
    Dataen returneres i form av et todimensjonalt array på 5 elementer:
    Arrayets størrelse (dvs. antall rader) avhenger av verdien til maxBlocks
    Element 1 og 2: x-posisjon
    Element 3: y-posisjon
    Element 4: Boksens bredde
    Element : Boksens høyde
    '''
    #TODO Lag et filter for uønsket data slik som LiDAR-sensoren
    send = [[] for j in range(maxBlocks)]
    global lastw
    block = 0
    while True:
        w = spi.xfer([0,0])
        print w
        if (w[0] == 0 and lastw[0] == 0) and (w[1] == 0 and lastw[1] == 0): #Ingen frame
			#print "LYS AV"
			#GPIO.output(search_light, GPIO.LOW) #Hold indikatorlys av
			return False
        elif (w[0] == 0xaa and lastw[0] == 0xaa) and (w[1] == 0x55 and lastw[1] == 0x55): #Nytt IR-frame
            print "IR-frame"
            i = 0
            while block < maxBlocks:
                while i < 6:
                    w = spi.xfer([0,0])
                    if i == 2: #x-posisjon
                        send[block].append(w[0])
                        send[block].append(w[1])
                    elif i == 3: #y-posisjon
                        #send.append(resp[0])
                        send[block].append(w[1])
                    elif i == 4: #bredde
                        send[block].append(w[1])
                    elif i == 5: #høyde
                        send[block].append(w[1])
                    #time.sleep(0.01)
                    i += 1
 
                w = spi.xfer([0,0])
                #print "Blokk 1: " + str(send)
           
                if w[0] == 170 and w[1] == 85: #Potensielt nytt blokk-objekt i frame.
                    w = spi.xfer([0,0])
                    if w[0] != 170 and w[1] != 85: #Nytt blokk-objekt i frame: Begynn ny iterasjon.
                        #Lur ting detta, å sjekke neste linje. "Problemet" her e at i ikke blir lengre synka.
                        #Kan fikses lett ved å bare la i resettes til 1 i stedet for 0 i andre omgang.
                        i = 1
                        block += 1
                   
                else: #Slutt på objekter i frame: Slå på indikatorlys og send ut array.
                    GPIO.output(search_light, GPIO.HIGH) #Hold indikatorlys på
                    time.sleep(0.05)
                    lastw = w 
                    return send
        lastw = w

'''
def get_Pixy():
	
	#Leser av bildedataen fra Pixycam. 
	#Dataen returneres i form av et array på 5 elementer:
	#Element 1: x-posisjon
	#Element 2: y-posisjon
	#Element 3: Boksens bredde
	#Element 4: Boksens høyde
	
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

#while True:
#	send = get_Pixy()
#	print send
'''
