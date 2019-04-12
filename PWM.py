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
		self.set_speed = 0
		self.actual_speed = 0
		self.previous_speed_error = 0
		self.actual_speed = 0
		self.lostflag = False

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

		def dist_to_speed(dist, error_delta, sample, P_gain = 1, D_gain = 1.0, speed_constrain = 250):
			'''
			Converts horizontal distance to a velocity.
			Maximal velocity is constrained between speed_constrain
			'''
			speed = P_gain * dist + D_gain * error_delta/sample 
			speed = constrain(speed, -speed_constrain, speed_constrain)
			return speed

		if self.inverted:
			error *= -1 
		if self.firstupdate == False:
			#error_delta = error - self.previous_error
			self.sample = time.time() - self.start
			error_delta = error - self.previous_error
			#actual_speed = faktisk hastighet mot punkt
			self.actual_speed = (self.previous_error - error)/self.sample
			#Settpunktet beregnes ved å multiplisere feil med Kp (P = 1)
			self.set_speed = dist_to_speed(error, error_delta, self.sample)
			speed_error = self.set_speed - self.actual_speed 
			speed_error_delta = (speed_error - self.previous_speed_error)/self.sample

			#Anti Windup
		   # if (error >= 0 and self.previous_error <= 0) or (error <= 0 and self.previous_error >= 0):
		   #    self.previous_sum = 0

			self.ui = constrain(I_PID(speed_error, self.sample),-50, 50)
			vel = (self.P_gain * speed_error + self.ui) # + speed_error_delta * self.D_gain)
			self.stample = time.time() - self.start_stample
			self.start = time.time()
			self.position = self.u0 + vel
			#Begrenser utslagene på servoutgangen mellom 1250us
			#og 1750us.
			self.previous_error = error
			self.previous_speed_error = speed_error
			self.position = int(constrain(self.position, 1250, 1750))
			return self.position

		elif self.lostflag:
			self.lostflag = False
			self.start = time.time()
		else:
			self.firstupdate = False
			#self.previous_error = error
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

