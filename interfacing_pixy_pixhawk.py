#-*- coding:utf-8 -*-
#!/usr/bin/python

'''
FILNAVN:OLS1
Inneholder følgende funksjoner:
RTL_Failsafe:
takeover: Sjekker for takeover-signal fra sender (Tx)
manual_flight: Gir manuell kontroll tilbake til Tx. Leter samtidig etter IR-lys
analyze: Plotter opp dataene fra de målingene tatt under presisjonslanding 
after_landing: Samlefunksjon som kjører analyze() og gir mulighet til å restarte programmet for nytt testflyvning.
descend_check: Sjekker om dronen er innenfor visse kriterier for initiering av vertikal navigasjon
landing_check (IKKE FERDIG IMPLEMENTERT): Sjekker om dronen har landet. Sjekkes i sammeheng med vertikal navigasjon. 
... 

'''

import RPi.GPIO as GPIO
from dronekit import connect, VehicleMode, APIException
from pymavlink import mavutil
import time
import datetime
import argparse
import exceptions
import socket
from pixy_spi import get_Pixy, bit_to_pixel, indikering
from PWM import PWM
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

###GAIN SCHEDULE###
pitch_gain_schedule = [[0.6,0.9,1.4], #0m --> 5m
[0.6, 0.9, 1.4], #5m --> 10m
[0.6, 0.9, 1.4], #10m --> 15m
[0.35, 0, 0.3], #15m --> 20m
[0.2,0,0.2]] # 20m --> 
roll_gain_schedule = [[0.6,0.9,1.4], #0m --> 5m
[0.6, 0.9, 1.4], #5m --> 10m
[0.8, 0.9, 1.4], #10m --> 15m
[0.5, 0, 0.3], #15m --> 20m
[0.5,0,0.3]] # 20m --> 


####RC####
ROLL = '2' #HITL: 2 #SITL: 1
PITCH = '3' #HITL: 3 #SITL: 2
THROTTLE = '1' #HITL: 1 #SITL: 3
NAV_MODE = "STABILIZE" #NAV_MODE = "ALT_HOLD" HARDWARE "STABILIZE"
RTL_MODE = "RTL"
MANUAL_ANGLE = 4500
NAV_ANGLE = 2500 #Foreløpig verdi for NAV_ANGLE = 1000. Må muligens nedjusteres.
desired_rate = -10 #cm/s

pwm_roll = PWM(P_gain = 1.2, D_gain = 0.0, I_gain = 0.5, inverted = True) #P_gain = 700, D_gain = 250
pwm_pitch = PWM(P_gain = 0.8, D_gain = 0.0, I_gain = 0.5, inverted = True) #P_gain = 700, D_gain = 250
#pwm_throttle = PWM(P_gain = 2, D_gain = 3)
#####

####PIXEL-SETTPUNKTER####
x_max = 640
x_min = 0
y_max = 400
y_min = 0
####PIXYCAM####
sensor_height = 0.2430
sensor_width = 0.3888
focal = 0.36936


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

'''
last_rangefinder_distance = 0
@vehicle.on_attribute('rangefinder')
def rangefinder_callback(self,attr_name):
    #attr_name not used here.
    global last_rangefinder_distance
    if last_rangefinder_distance == round(self.rangefinder.distance, 1):
        return
    last_rangefinder_distance = round(self.rangefinder.distance, 1)
    #print " Rangefinder (metres): %s" % last_rangefinder_distance
'''

def RTL_failsafe():
  ''' Sjekker om RTL-failsafe er aktivert. Hvis dette skjer, nullstilles overrides og overgir 
  kontroll tilbake til Tx
  '''
  if vehicle.mode.name == RTL_MODE:
    return True

def takeover():
  '''Boolsk funksjon for å sjekke for takeover fra radio eller GCS.'''
  if not searching or vehicle.channels['7'] < 1750:
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
    print "Searching..."
    time.sleep(0.05)
    potential = get_Pixy()
    if potential:
      print len(potential)
    if (potential and len(potential)) == 5 or (potential and len(potential) == 2):
      potential_counter += 1
    if potential_counter == 3:
      print "Found point! Activate NAV mode"
      time.sleep(0.05)
      searching = True

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
    Plottene blir lagret i /figurer/
  '''
  lostcount = 0
  print "Analyze.."
  import matplotlib 
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  i = 1
  for filename in ('y_utgang.txt', 'x_utgang.txt', 'x_feil.txt','y_feil.txt'):
    if not os.path.isfile(filename):
      lostcount += 1
      if lostcount == 4:
        print "No files to write. "
        return 
      print lostcount
      continue
    else:
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
        #date_name = datetime.datetime.now()

        plt.figure(i)
        plt.plot(dataTime, data, markersize = 1, linestyle = 'solid')
        plt.xlabel('Tid [s]')
        if i == 1:
          plt.title('Y-pixel: Prosessutgang')
          plt.ylabel('Prosessutgang [pixel]')
          plt.plot([0, dataTime[-1]],[200,200], color='green', linestyle='--')
          plt.savefig('/home/pi/Bachelor_backup/figurer/yfig_Kp='+str(pwm_pitch.P_gain)+'_Kd='+str(pwm_pitch.D_gain)+'_Ki='+str(pwm_pitch.I_gain)+'.png')
          #plt.savefig('/home/pi/Bachelor_backup/figurer/yfig_' + date_name.strftime("%d%m%Y-%H:%M:%S") + '.png')
        elif i == 2:
          plt.title('X-pixel: Prosessutgang')
          plt.ylabel('Prosessutgang [pixel]')
          plt.plot([0, dataTime[-1]],[320, 320],color='green', linestyle='--')
          plt.savefig('/home/pi/Bachelor_backup/figurer/xfig_Kp='+str(pwm_roll.P_gain)+'_Kd='+str(pwm_roll.D_gain)+'_Ki='+str(pwm_roll.I_gain)+'.png')
          #plt.savefig('/home/pi/Bachelor_backup/figurer/xfig_' + date_name.strftime("%d%m%Y-%H:%M:%S") + '.png')
        elif i == 3:
          plt.title('X-cm: Feil')
          plt.plot([0, dataTime[-1]],[0,0], color='green', linestyle='--')
          plt.ylabel('Feil [cm]')
          plt.savefig('/home/pi/Bachelor_backup/figurer/xfeil_Kp='+str(pwm_roll.P_gain)+'_Kd='+str(pwm_roll.D_gain)+'_Ki='+str(pwm_roll.I_gain)+'.png')
          #plt.savefig('/home/pi/Bachelor_backup/figurer/xfeil_' + date_name.strftime("%d%m%Y-%H:%M:%S") + '.png')
        elif i == 4:
          plt.title('Y-cm: Feil')
          plt.plot([0, dataTime[-1]],[0,0],color='green', linestyle='--')
          plt.ylabel('Feil [cm]')
          plt.savefig('/home/pi/Bachelor_backup/figurer/yfeil_Kp='+str(pwm_pitch.P_gain)+'_Kd='+str(pwm_pitch.D_gain)+'_Ki='+str(pwm_pitch.I_gain)+'.png')
          #plt.savefig('/home/pi/Bachelor_backup/figurer/yfeil_' + date_name.strftime("%d%m%Y-%H:%M:%S") + '.png')
    i += 1  
  print "Graphs made." #Check /figurer/ with dates " + date_name.strftime("%d%m%Y")

def landing_check(h, error_rate, throttle_pwm):
  '''
  Sjekker om fartøyet har landet. 
  #TODO Finn ut hva som skal til for at fartøyet skal gjenkjenne at det har landet
  '''
  if h < 20 and -10 < error_rate < 10 and 1000 <=throttle_pwm <= 1100:
    return True
  else:
    return False

def descend_check(error_x, error_y, boundary_x = 20, boundary_y = 20):
  '''
  Sjekker om fartøyet er innenfor ønskede grenser til 
  å kunne begynne vertikal navigering.
  Grensene er satt som en boks som i default er kvadratisk 20cm*20cm
  Boksens størrelse og form kan justeres ved å endre på boundary_x og boundary_y.
  '''
  if -boundary_x <= error_x <= boundary_x and -boundary_y <= error_y <= boundary_y:
    return True
  else:
    return False

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
    elif vehicle.channels[ROLL] < 1000:
      indikering(0.1, 10)
      GPIO.cleanup()
      vehicle.close()
      os.system("sudo shutdown -h now")
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

    
    #KOMMENTER INN FOR HARDWARE IN THE LOOP
    while not vehicle.armed:
      print "Waiting..."
      if vehicle.channels['7'] > 1750:
        print "Shutting down"
        indikering(0.1, 10) 
        os.system("sudo shutdown -h now")
      else:
        pass
    
    indikering(0.05, 20)
    #arm_and_takeoff(10) #BARE FOR SITL
    #vehicle.parameters["ANGLE_MAX"] = NAV_ANGLE
    #vehicle.mode = VehicleMode(NAV_MODE) #KUN FOR SITL
    
    while vehicle.armed:
      send = get_Pixy()
    
      if not takeover(): #HARDWARE
        #Dersom gjenoppretting av lyspunkt:
        #Skift tilbake til NAV_MODE og redusere vinkelutslag
        if vehicle.mode.name != NAV_MODE or vehicle.parameters["ANGLE_MAX"] == MANUAL_ANGLE:
          vehicle.mode = VehicleMode(NAV_MODE)
          #time.sleep(0.05)
          vehicle.parameters["ANGLE_MAX"] = NAV_ANGLE
          #time.sleep(0.05)
        #Fjerner alle overrides og setter programmet i en pass-loop
        if RTL_failsafe():
          vehicle.channels.overrides = {}
          while True:
            pass         

        #Sjekker for tapt data fra Pixy-kameraet.
        while not send or (not send[0] and not send[1]) or (send[0][0]*256+send[0][1]) > 65535:
          send = get_Pixy()
          lost_counter += 1
          #time.sleep(0.05)
          
          #Søker 10 ganger før den gir opp søket.
          if lost_counter == 10: 
            print 'Lost track of point...'

            #Resetter Firstupdate-variabelen for å resette telling. 
            #pwm_pitch.firstupdate = True
            #pwm_roll.firstupdate = True
            #pwm_roll.sample = 0
            #pwm_roll.start = 0
            #pwm_pitch.sample = 0
            pwm_roll.previous_sum = 0
            pwm_pitch.previous_sum = 0
            #For å ikke resette den totale telleren, brukes dette flagget til å resette 
            #integrator-telleren.
            pwm_roll.lostflag = True
            pwm_pitch.lostflag = True
            lost_counter = 0
            searching = False
            break

        #Begynn ny iterasjon dersom søkinga har sviktet. 
        if not searching:
          continue
          
        else:
          h = round(vehicle.rangefinder.distance * 100, 2)
          #Gain Scheduling
          #pwm_roll.gain_schedule(h, roll_gain_schedule)
          #pwm_pitch.gain_schedule(h, pitch_gain_schedule)
          #For første gjennomkjøring vil det ikke være noe forandring i feil.
          if first_check:
            start_time = time.time()
            previous_h = h
          else:
            loop_time = time.time() - start_time
            desc_rate = (h - previous_h)/loop_time
            error_rate = desired_rate - desc_rate
            previous_h = h
            start_time = time.time()
            
          
          #Passiv filtrering av objekter. Velger objekter med størst areal.
          '''
          if len(send) == 2 and len(send[1]) != 0:
            areal1 = send[0][3] * send[0][4]
            areal2 = send[1][3] * send[1][4]
            if areal1 > areal2:
              send = send[0]
            elif areal2 > areal1:
              send = send[1]
            send = bit_to_pixel(send)
          else:
            send = bit_to_pixel(send[0]) #Egentlig send[0]
          '''
          send = bit_to_pixel(send[0])
          #print send

          #GSD = Ground Sampling Distance
          gsd_h = (h*sensor_height)/(focal*y_max)
          gsd_w = (h*sensor_width)/(focal*x_max)
          gsd = max(gsd_h, gsd_w)
          #Beregner feil for PID-regulering
          error_x = (x_center - send[0]) * gsd
          error_y = (y_center - send[1]) * gsd

          #Setter denne feilen inn i PD-regulatoren og beregner PWM-posisjon
          pwm_roll.update(error_x)
          pwm_pitch.update(error_y)
          '''
          if descend_check(error_x,error_y):
            pwm_throttle.update(error_rate)
            vehicle.channels.overrides = {THROTTLE: pwm_throttle.position, ROLL: pwm_roll.position ,PITCH: pwm_pitch.position}
          else:
            vehicle.channels.overrides = {ROLL: pwm_roll.position ,PITCH: pwm_pitch.position} ###THROTTLE-INPUT ER BARE FOR SITL###
          '''
          vehicle.channels.overrides = {ROLL: pwm_roll.position ,PITCH: pwm_pitch.position} 
          #vehicle.channels.overrides = {PITCH: pwm_pitch.position}
          #time.sleep(0.005) 
          print "Send = " + str(send)
          print "h = " + str(h)
          print "Error_x = " + str(error_x)
          print "Previous_error x = " + str(pwm_roll.previous_error)
          print "Error_y = " + str(error_y)
          print "Roll: ", pwm_roll.position
          print "Pitch: ", pwm_pitch.position
          print "ui_roll = " + str(pwm_roll.ui)
          print "ui_pitch = " + str(pwm_pitch.ui)
          print "sample = " + str(pwm_roll.sample)
          print "Actual speed y = " + str(pwm_pitch.actual_speed)
          print "Actual speed x = " + str(pwm_roll.actual_speed)
          write_to_file(send[0], pwm_roll.stample, 'x_utgang', first_check)
          write_to_file(send[1], pwm_pitch.stample, 'y_utgang', first_check)
          write_to_file(round(error_x,2), pwm_roll.stample, 'x_feil', first_check)
          write_to_file(round(error_y,2), pwm_pitch.stample, 'y_feil', first_check)

          if first_check:
            first_check = False

      else:
        '''
        Ved takeover fra senderen (som merkes i form av mode-bytte) renses kanaloverskrivelsene
        slik at en kan gjenopprette kontrollen.
        '''
        #time.sleep(0.05)
        vehicle.channels.overrides = {} #HARDWARE IN THE LOOP
        #vehicle.channels.overrides = {THROTTLE: 1500} #BARE FOR SITL
        manual_flight()
        

  # Close vehicle object when testing
  except KeyboardInterrupt:
    vehicle.channels.overrides = {} #HITL
    vehicle.mode = VehicleMode('STABILIZE')
    time.sleep(0.3)
    indikering(constant = True, i=-1)
    vehicle.close()
    analyze() #KOMMENTER INN FOR HARDWARE
    sys.exit(0)
  
#Analyserer og starter eventuelt programmet på nytt om ønsket.
  after_landing()
  
