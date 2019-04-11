#-*- coding:utf-8 -*-
"""
FILNAVN: PWM.py
Inneholder et PWM-objekt som brukes for styring av de fire ulike 
kanalene til UAV. PWM-klassen inneholder følgende metoder:
.update: Beregner pådraget for en PID-regulator til en PWM-kanal.
.I_PID: Beregner I-leddets pådrag
.gain_schedule: Setter PID-parametre alt etter høyden h satt som argument.
"""
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
	Dersom utgangen må reverseres, settes inverted-variabelen til True. Denne står lavt av default.

    """
    def __init__(self, initial_position = 1500, P_gain = 1.0, D_gain = 1.0, I_gain = 0, inverted = False):
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
        self.ui = 0


    def update(self, error):
        '''
        Beregner pådraget for en PID-regulator til en PWM-kanal.
        '''
        def I_PID(error, sample):
		    if self.firstupdate:
			    self.previous_sum = self.I_gain * sample * error
		    else:
			    self.previous_sum = self.I_gain * sample * error + self.previous_sum
            return self.previous_sum

        if self.inverted:
            error *= -1 
        if self.firstupdate == False:
            error_delta = error - self.previous_error
            self.sample = time.time() - self.start
            
            #Anti Windup
           # if (error >= 0 and self.previous_error <= 0) or (error <= 0 and self.previous_error >= 0):
           #    self.previous_sum = 0
            self.ui = constrain(I_PID(error, self.sample),-50, 50)
            
            vel = (self.P_gain * error + self.ui + error_delta * self.D_gain)
            self.stample = time.time() - self.start_stample
            self.start = time.time()
            self.position = self.u0 + vel
            #Begrenser utslagene på servoutgangen mellom 1000us
            #og 2000us.
            self.previous_error = error
            self.position = int(constrain(self.position, 1250, 1750))
            return self.position
        else:
            self.firstupdate = False
            self.start = time.time()
            self.start_stample = time.time()
            
    def gain_schedule(self, h, gains):
        '''
        Velger et sett med PID - parametre til PWM-objekt 
        dersom høyden er innenfor et gitt intervall.
        h: Integer i cm
        '''
        if h >= 2000: 
            self.P_gain = gains[4][0]
            self.I_gain = gains[4][1]
            self.D_gain = gains[4][2]
        elif h >= 1500 and h < 2000:
            self.P_gain = gains[3][0]
            self.I_gain = gains[3][1]
            self.D_gain = gains[3][2]
        elif h >= 1000 and h < 1500:
            self.P_gain = gains[2][0]
            self.I_gain = gains[2][1]
            self.D_gain = gains[2][2]
        elif h >= 500 and h < 1000:
            self.P_gain = gains[1][0]
            self.I_gain = gains[1][1]
            self.D_gain = gains[1][2]
        elif h >= 0 and h < 500:
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


