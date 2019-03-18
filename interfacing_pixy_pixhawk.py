#-*- coding:utf-8 -*-
#!/usr/bin/python

import RPi.GPIO as GPIO
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import datetime
import argparse
import exceptions
import socket
from pixy_spi import PWM, get_Pixy, bit_to_pixel, indikering
from write_to_file import write_to_file
import sys
import os

#Parametre 
lost_counter = 0
searching = True

first_check = True

###GPIO-SETUP###
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


####RC####
ROLL = '2' #HITL: 2 #SITL: 1
PITCH = '3' #HITL: 3 #SITL: 2
THROTTLE = '1' #HITL: 1 #SITL: 3
NAV_MODE = "STABILIZE" #NAV_MODE = "ALT_HOLD"
RTL_MODE = "RTL"
MANUAL_ANGLE = 4500
NAV_ANGLE = 3000 #Foreløpig verdi for NAV_ANGLE = 1000

pwm_roll = PWM(P_gain = 700, D_gain = 250, inverted = True) 
pwm_pitch = PWM(P_gain = 700, D_gain = 250, inverted = True) #INVERTERT BARE I SITL
#####

####PIXEL-SETTPUNKTER####
x_max = 640
x_min = 0
y_max = 400
y_min = 0

x_center = (x_max - x_min)/2
y_center = (y_max - y_min)/2
####

#SETUP 
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='udp:127.0.0.1:14550',
	help = 'Connect to vehicle on ip address given. Default set to udp:127.0.0.1:14550')
args = parser.parse_args()

# Koble til fartøy
try:
  print'Connecting to vehicle on: %s' % args.connect
  vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)

  # Dårlig TCP-forbindelse
except socket.error:
  print 'No server exists!'
  print "Reconnecting to vehicle on: %s" %args.connect
  vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)

# Dårlig TTY-forbindelse
except exceptions.OSError as e:
  print 'No serial exists!'
  print "Reconnecting to vehicle on: %s" %args.connect
  vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)


# API-error
except APIException:
  print 'Timeout!'
  print "Reconnecting to vehicle on: %s" %args.connect
  vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)


last_rangefinder_distance=0
'''
@vehicle.on_attribute('rangefinder')
def rangefinder_callback(self,attr_name):
    #attr_name not used here.
    global last_rangefinder_distance
    if last_rangefinder_distance == round(self.rangefinder.distance, 1):
        return
    last_rangefinder_distance = round(self.rangefinder.distance, 1)
    print " Rangefinder (metres): %s" % last_rangefinder_distance
'''

@vehicle.on_attribute('mode')   
def decorated_mode_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.
    print "CALLBACK: Mode changed to", value

def RTL_failsafe():
  if vehicle.mode.name == RTL_MODE:
    return True

def takeover():
  '''Funksjon for å sjekke for takeover fra radio eller GCS.'''
  if not searching or vehicle.channels['7'] < 1750: #(ROLL and PITCH in vehicle.channels.overrides and not vehicle.mode.name == NAV_MODE) or not searching:
    return True
  elif 1750 <= vehicle.channels['7'] <= 2012 or (1750 <= vehicle.channels['7'] <= 2012 and searching):
    return False

def manual_flight():
  '''
  Funksjon for manuell flyging. Programmet leter enda etter lyspunkt mens 
  dronen er kontrollert manuelt. 
  '''
  global searching
  
  time.sleep(0.05)
  potential_counter = 0
  print "Manual flight..."
  vehicle.parameters["ANGLE_MAX"] = MANUAL_ANGLE
  while not searching:
    #print vehicle.channels
    print "Searching..."
    time.sleep(0.05)
    potential = get_Pixy()
    if potential and len(potential) == 5:
      potential_counter += 1
    if potential_counter == 3:
      print "Found point! Activate NAV mode"
      time.sleep(0.05)
      searching = True

def set_home():
  while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print"Waiting for home location..."
  
  print "Home location set"
  time.sleep(1)
  print "Home location set at: {}".format(vehicle.home_location)


# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
  print"Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print "Waiting for vehicle to initialise..."
    time.sleep(1)

    while vehicle.gps_0.fix_type < 2:
        print "Waiting for GPS", vehicle.gps_0.fix_type
        time.sleep(1)
    
  print "Arming motors"
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)

  print "Taking off!"
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print "Altitude: ", vehicle.location.global_relative_frame.alt
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print "Reached target altitude"
      break
    time.sleep(1)

def analyze():
  '''Plotter prosessutgangene for pitch og roll samt 
    deres feilverdier etter at fartøyet har landet. 
    Disse plottene lagres som egne *.png - filer.
  '''
  import matplotlib 
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  i = 1
  for filename in ('y_utgang.txt', 'x_utgang.txt', 'x_feil.txt','y_feil.txt'):
    if os.path.isfile(filename):
      with open(filename, 'r') as file:
        file_input = file.read().split('\n')
        output = [] 
        data = []
        dataTime = []
        for row in file_input:
            output.append(row.split(','))

        for row in output:
          if len(row) != 1:
            data.append(row[0])
            dataTime.append(row[1])
        date_name = datetime.datetime.now()

        plt.figure(i)
        plt.plot(dataTime, data, markersize = 1, linestyle = 'solid')
        plt.xlabel('Tid [s]')
        if i == 1:
          plt.title('Y-pixel: Prosessutgang')
          plt.ylabel('Prosessutgang [Pixel')
          plt.savefig('./figurer/yfig ' + date_name.strftime("%d%m%Y-%H:%M:%S") + '.png')
        elif i == 2:
          plt.title('X-pixel: Prosessutgang')
          plt.ylabel('Prosessutgang [Pixel]')
          plt.savefig('./figurer/xfig ' + date_name.strftime("%d%m%Y-%H:%M:%S") + '.png')
        elif i == 3:
          plt.title('X-pixel: Feil')
          plt.ylabel('Feil [Pixel]')
          plt.savefig('./figurer/xfeil ' + date_name.strftime("%d%m%Y-%H:%M:%S") + '.png')
        elif i == 4:
          plt.title('Y-pixel: Feil')
          plt.ylabel('Feil [Pixel]')
          plt.savefig('./figurer/yfeil ' + date_name.strftime("%d%m%Y-%H:%M:%S") + '.png')
    else:
      pass
      
    i += 1  
def after_landing():
  '''
  After landing - liste som analyserer dataen fra pixycam, renser overrides og gir 
  piloten muligheten til å restarte programmet.
  '''
  print "After landing..."
  vehicle.channels.overrides = {}
  analyze()
  indikering(constant = True)
  while not vehicle.armed:
    if vehicle.channels['7'] > 1750:
      indikering(1, 5)
      GPIO.cleanup()
      vehicle.close()
      os.execv(sys.executable, ['python'] + sys.argv)
    else:
      pass  

#MAIN PROGRAM
if __name__ == "__main__":

  try:
    '''
    ###BARE FOR SITL###
    arm_and_takeoff(10)
    vehicle.mode = VehicleMode(NAV_MODE)
    vehicle.channels.overrides[THROTTLE] = 1500 
    time.sleep(0.5)
    '''
    indikering(0.5, 2)
    indikering(0.1, 10)

    #Mens dronen står på bakken disarmert, venter koden.
    #Hvis SF-bryteren blir høy, stopper programmet, 
    #og pi'en slås av.
    while not vehicle.armed:
      print "Waiting..."
      if vehicle.channels['7'] > 1750:
        print "Shutting down"
        indikering(0.1, 10)
        os.system("sudo shutdown -h now")
      else:
        pass

    indikering(0.05, 20)
    while vehicle.armed: #While True:
      send = get_Pixy()
    
      if not takeover():
        #Dersom gjenoppretting av lyspunkt:
        #Skift tilbake til NAV_MODE og redusere vinkelutslag
        if vehicle.mode.name != NAV_MODE:
          vehicle.mode = VehicleMode(NAV_MODE)
          time.sleep(0.05)
          vehicle.parameters["ANGLE_MAX"] = NAV_ANGLE
          time.sleep(0.05)
        
        if RTL_failsafe():
          vehicle.channels.overrides = {}
          while True:
            pass         

        while not send:
          send = get_Pixy()
          lost_counter += 1
          time.sleep(0.05)

          if lost_counter == 5: #if not pixy_search():
            print 'Lost track of point...'

            #Resetter Firstupdate-variabelen for å resette telling. 
            pwm_pitch.firstupdate = True
            pwm_roll.firstupdate = True
            time.sleep(0.05)
            searching = False
            break

        #Begynn ny iterasjon dersom søkinga har sviktet. 
        if not searching:
          continue
          
        else:          
          send = bit_to_pixel(send)
          print send

          #Beregner feil for PD-regulering
          error_x = x_center - send[0]
          error_y = y_center - send[1]
          #Setter denne feilen inn i PD-regulatoren og beregner PWM-posisjon
          pwm_roll.update(error_x)
          pwm_pitch.update(error_y)

        
          print "Roll: ", pwm_roll.position
          print "Pitch: ", pwm_pitch.position
          
          write_to_file(send[0], pwm_roll.stample, 'x_utgang', first_check)
          write_to_file(send[1], pwm_pitch.stample, 'y_utgang', first_check)
          write_to_file(error_x, pwm_roll.stample, 'x_feil', first_check)
          write_to_file(error_y, pwm_pitch.stample, 'y_feil', first_check)

          if first_check == True:
            first_check = False

          ###THROTTLE-INPUT ER BARE FOR SITL###
          vehicle.channels.overrides = {ROLL: pwm_roll.position ,PITCH: pwm_pitch.position}
          time.sleep(0.05)
           
      else:
        '''
        Ved takeover fra senderen (som merkes i form av mode-bytte) renses kanaloverskrivelsene
        slik at en kan gjenopprette kontrollen.
        '''
        
        time.sleep(0.05)
        vehicle.channels.overrides = {}
        #vehicle.channels.overrides = {THROTTLE: 1500} #BARE FOR SITL
        manual_flight()
        

  # Close vehicle object when testing
  except KeyboardInterrupt:
    vehicle.channels.overrides = {} #HITL
    #vehicle.channels.overrides = {THROTTLE: 1500} #BARE FOR SITL
    vehicle.mode = VehicleMode('LOITER')
    time.sleep(0.3)
    vehicle.close()
    GPIO.cleanup()
    analyze() 
  
#Analyserer og starter eventuelt programmet på nytt om ønsket.
  after_landing()
  