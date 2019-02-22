#-*- coding:utf-8 -*-
#!/usr/bin/python


from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import argparse
import exceptions
import socket
from pixy_spi import PWM, get_Pixy, bit_to_pixel

#Parametre 
lost = False
lost_counter = 0
searching = True

####RC####
ROLL = '1'
PITCH = '2'
THROTTLE = '3'
NAV_MODE = "ALT_HOLD"
MANUAL_ANGLE = 4500
NAV_ANGLE = 1000
pwm_roll = PWM(P_gain = 200, D_gain = 200, inverted = True) 
pwm_pitch = PWM(P_gain = 200, D_gain = 200, inverted = True) #INVERTERT BARE I SITL
#####

####PIXEL-SETTPUNKTER####
x_max = 640
x_min = 0
y_max = 400
y_min = 0

x_center = (x_max - x_min)/2
y_center = (y_max - y_min)/2
####

is_takeover = False

#SETUP 
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/serial0',
	help = 'Connect to vehicle on ip address given. Default set to 127.0.0.1:14550')
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

def takeover():
  '''Funksjon for å sjekke for takeover fra radio eller GCS.'''
  if not searching: #(ROLL and PITCH in vehicle.channels.overrides and not vehicle.mode.name == NAV_MODE) or not searching:
    return True
  else:
    return False

def manual_flight():
  global searching
  '''
  Funksjon for manuell flyging. Programmet står i ro mens 
  dronen er kontrollert manuelt. Når vehicle.mode.name == NAV_MODE gå tilbake til pixy_search
  '''
  vehicle.mode = VehicleMode('LOITER')
  time.sleep(0.3)
  potential_counter = 0

  vehicle.parameters["ANGLE_MAX"] = MANUAL_ANGLE
  while vehicle.mode.name != NAV_MODE:
    print "Manual flight..."
    time.sleep(0.3)
    potential = get_Pixy()
    if potential and len(potential) == 5:
      potential_counter += 1
    if potential_counter == 3:
      print "Found point! Returning to %s" % NAV_MODE
      vehicle.mode = VehicleMode(NAV_MODE)
      time.sleep(0.3)
      vehicle.parameters["ANGLE_MAX"] = NAV_ANGLE
      searching = True
      break


def set_home():
  while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print"Waiting for home location..."
  
  print "Home location set"
  time.sleep(1)
  print "Home location set at: {}".format(vehicle.home_location)

    
def arm():
  print "Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
#  while not vehicle.is_armable:
#    print " Waiting for vehicle to initialise..."
#    time.sleep(1)

#    while vehicle.gps_0.fix_type < 2:
#        print "Waiting for GPS", vehicle.gps_0.fix_type
#        time.sleep(1)
    
  print "Arming motors"
  # Copter should arm in GUIDED mode
  #vehicle.mode    = VehicleMode("GUIDED")
  vehicle.mode = VehicleMode("STABILIZE")
  vehicle.armed   = True

  while not vehicle.armed:
    print "Waiting for arming..."
    time.sleep(1)


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

def simpleGoto(lat, longi):
	loc = LocationGlobalRelative(lat, longi)
	print "moving to ({}, {})".format(lat, longi)
	vehicle.simple_goto(loc)


'''
Lage en pixy_search funksjon som søker etter pixy OG sjekker swith på kontroller. 
Hvis vericle.mode.name != NAV_MODE, gå inn i manual_flight funksjon. Når pixy ser objekt, gå videre i "while true" løkka
'''

def pixy_search():
  '''
  Funksjon for å fortsette å lete etter lyspunkt.
  Etter en gitt tid returneres 'False' og kontrollen overføres til manuell.
  '''
  global lost_counter 
  print 'Searching...'
  lost_counter += 1
  time.sleep(0.05)

  if ROLL and PITCH in vehicle.channels.overrides and lost_counter == 40:
    return False
  



#MAIN PROGRAM
if __name__ == "__main__":
  try:
    
    '''
    Foreløpig for SITL-bruk: Armere og ta av. Deretter navigere inn mot punktet. 
    '''
    arm_and_takeoff(10)
    vehicle.mode = VehicleMode(NAV_MODE)
    vehicle.channels.overrides[THROTTLE] = 1500 #BARE FOR SITL
    time.sleep(0.5)

    if vehicle.parameters["ANGLE_MAX"] == MANUAL_ANGLE:
      vehicle.parameters["ANGLE_MAX"] = NAV_ANGLE

    
    while True:
      if not takeover():
        #Dersom gjenoppretting av lyspunkt:
        #Skift tilbake til NAV_MODE og redusere vinkelutslag
        '''if lost:
          vehicle.mode = VehicleMode(NAV_MODE)
          lost_counter = 0
          vehicle.parameters["ANGLE_MAX"] = NAV_ANGLE '''

        send = get_Pixy()
        while not send:
          #lag en pixy_search funksjon
          pixy_search()

          if not pixy_search():
            print 'Lost track of point...'
            time.sleep(0.5)
            searching = False
            break
            #manual_flight()

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

          print vehicle.channels
        
          print "Roll: ", pwm_roll.position
          print "Pitch: ", pwm_pitch.position
          
          print "Error_x: %d" % error_x
          print "Error_y %d" % error_y

          vehicle.channels.overrides = {THROTTLE: 1500, ROLL: pwm_roll.position ,PITCH: pwm_pitch.position}
          time.sleep(0.1)
          
      else:
        '''
        Ved takeover fra senderen (som merkes i form av mode-bytte) renses kanaloverskrivelsene
        slik at en kan gjenopprette kontrollen.
        '''
        print "Tx takeover"
        time.sleep(0.1)
        vehicle.channels.overrides = {THROTTLE: 1500} #BARE FOR SITL
        manual_flight()
        


  # Close vehicle object
  except KeyboardInterrupt:
    vehicle.channels.overrides = {THROTTLE: 1500} #BARE FOR SITL
    vehicle.mode = VehicleMode('LOITER')
    time.sleep(0.3)
    vehicle.close()
  
  vehicle.close()