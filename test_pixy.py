#-*- coding:utf-8 -*-
import spidev
import RPi.GPIO as GPIO
import time


spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b00
spi.cshigh = False

search_light = 21

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(search_light, GPIO.OUT)

 
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
    lastw = [0xff, 0xff]
    global lastw
    block = 0
    while True:
        w = spi.xfer([0,0])
        if (w[0] == 0 and lastw[0] == 0) and (w[1] == 0 and lastw[1] == 0): #Ingen frame
            GPIO.output(search_light, GPIO.LOW) #Hold indikatorlys av
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
                    time.sleep(0.01)
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
                    #lastw = [0xff,0xff] #Resetter lastw til en meningsløs verdi
                    return send
        lastw = w
while True:
	#send = get_Pixy()
	#print send
	send = get_Pixy()
	print send

