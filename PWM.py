#-*- coding:utf-8 -*-

import time

def constrain(val, min_, max_):    
    """
    Låser en variable mellom min_ og max_ - verdiene satt.
    """
    return max(min(max_,val),min_)
    
class PWM:
    """
    PWM-klasse for motorutgangene. PWM-utgangen er PD-regulert og kan inverteres etter behov.
	I-ledd skal senere implementeres. 
	Parameterne til PID-regulatoren settes som argumenter til klassen.
	Dersom utgangen må reverseres, settes inverted-variabelen til True. Denne står ellers lav.
	Etter hvert skal det implementeres en throttle - PWM.
	Inneholder følgende metoder:
	I_PID: Beregner I-leddets pådrag
	descend (bare for throttle-kanal): Beregner pådrag for PD-regulator som styrer
	throttle-kanalen ved hjelp av en ønsket nedstigningsrate og feil.
	update: Beregner pådraget for en PID-regulator til en PWM-kanal.
    """
    def __init__(self,initial_position = 1500, P_gain = 1.0, D_gain = 1.0, I_gain = 0, inverted = False):
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
    


    def update(self, error):
        def I_PID(error, sample):
		    if self.firstupdate:
			    self.previous_sum = self.I_gain * sample * error
			    return self.previous_sum
		    else:
			    self.previous_sum = self.I_gain * sample * error + self.previous_sum
			    return self.previous_sum
        if self.inverted:
            error *= -1 
        if self.firstupdate == False:
            error_delta = error - self.previous_error
            self.sample = time.time() - self.start
            if (error >= 0 and self.previous_error <= 0) or (error <= 0 and self.previous_error >= 0):
                self.previous_sum = 0
            
            self.ui = constrain(I_PID(error, self.sample),-100, 100)
            
            print "ui = " + str(self.ui)
            #Kanskje med P_gain = 10, I_gain = 1, D_gain = 5?
            vel = (self.P_gain * error + self.ui + error_delta * self.D_gain)
            #vel = (self.P_gain * error + error_delta * self.D_gain)/1024
            self.stample = time.time() - self.start_stample
            self.start = time.time()
            self.position = self.u0 + vel
            #Begrenser utslagene på servoutgangen mellom 1000us
            #og 2000us.
            self.previous_error = error
            self.position = int(constrain(self.position, 1000, 2000))
            return self.position
        else:
            self.firstupdate = False
            self.start = time.time()
            self.start_stample = time.time()
    def gain_schedule(self, h, gains):
        if h >= 20: 
            self.P_gain = gains[4][0]
            self.I_gain = gains[4][1]
            self.D_gain = gains[4][2]
        elif h >= 15 and h < 20:
            self.P_gain = gains[3][0]
            self.I_gain = gains[3][1]
            self.D_gain = gains[3][2]
        elif h >= 10 and h < 15:
            self.P_gain = gains[2][0]
            self.I_gain = gains[2][1]
            self.D_gain = gains[2][2]
        elif h >= 5 and h < 10:
            self.P_gain = gains[1][0]
            self.I_gain = gains[1][1]
            self.D_gain = gains[1][2]
        elif h >= 0 and h < 5:
            self.P_gain = gains[0][0]
            self.I_gain = gains[0][1]
            self.D_gain = gains[0][2]

    '''if self.throttle:
        def descend(self, error_rate):
            if self.firstupdate == False:
                error_delta = error_rate - self.previous_error 
                u = self.P_gain * error_rate + error_delta * self.D_gain
                self.position = self.u0 + u
                self.position = constrain(self.position, 1000, 2000)
            else:
                self.firstupdate = True
                self.previous_error = error_rate
    '''     


