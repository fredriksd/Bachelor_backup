#-*- coding:utf-8 -*-
#! usr/bin/env python


#Koden henter ut GPS-data fra bluetooth-linken.
#Skal vi legge ved "N" og "E" på sendinga av GPS-posisjonen?

#Må finne en metode for å gjøre GPS-posisjonen generisk, dvs, vite om den er "sør" eller "øst"
#Kan hende dette må ordnes for pixhawken, men er litt usikker. 

import time 
import serial
import sys

#data_in = "$GPRMC,123519,A,4807.03845,N,01131.23520,E,022.4,084.4,230394,003.1,W*6A"
data_out = {"lat": "46.0344", "long": "113.52"}

def read_and_process(data):
        data_to_read = {}
        for i in range(0,len(data)):
                if i == 3:
                        data_to_read['lat'] = data[i]
                elif i == 5:
                        data_to_read['long'] = data[i]
                else:
                        pass
        lat = data_to_read['lat']
        lat = lat[:2] + '.' + str(int(round(float(lat[2:4] + lat[5:9])/60.0)))

        longi = data_to_read['long']
        longi = str(int(longi[:3])) + '.' + str(int(round(float(longi[3:5] + longi[6:10])/60.0)))


        data_to_read['lat'] = lat
        data_to_read['long'] = longi

        return data_to_read


def gps_send(data):
        #global ser 

	data = data["lat"] + ',' +  data["long"] + ' ' + '\n'
 	ser.write(data)
        ser.write(data.encode('utf-8'))
	print "Success"
#	ser.close()

ser = serial.Serial(
        port = '/dev/serial0',
        baudrate = 9600,
        timeout = 15
        )


invalid_counter = 0
while True:
	start_time = time.time()
        data_in = ser.readline()
	print data_in
        if data_in[3:6] == b"RMC":
                data_in = str(data_in).split(",")
                gps_send(data_out)

                if data_in[2] == 'A':
                        #print "Valid data"
                        read_data = read_and_process(data_in)
                        print read_data
                        time.sleep(1)

                else:
                	print "Invalid data"
			stop_time = time.time()

			print "Tid: " + str(stop_time - start_time)

                	invalid_counter += 1
                	time.sleep(1)	
                	if invalid_counter == 5:
                		break
                                ser.close()
                		sys.exit(0)
