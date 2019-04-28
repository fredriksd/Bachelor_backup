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
from pixy_spi import * #get_Pixy, bit_to_pixel, indicating
from PWM import PWM
from write_to_file import write_to_file
import sys
import os
import matplotlib
#Program runs on headless host. Must therefore use the Agg - backend for plotting purposes. 
matplotlib.use('Agg')
import matplotlib.pyplot as plt

#Parameters
lost_counter = 0
searching = True
first_check = True
do_descend_counter = 0
stage = 1 #Stage counter for descend og landing. 

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
MANUAL_ANGLE = 4500 #Angle for manual control. 
NAV_ANGLE = 2500 #Preliminary value for NAV_ANGLE.
desired_height = 1000 #Desired height for stage 1. This variable is changed depending on which stage the navigation is in.


#Initialization of PWM classes
pwm_roll = PWM(inner_P_gain = 1.2, inner_I_gain = 0.5, outer_P_gain = 1.0, outer_D_gain = 1.0, inverted = True) 
pwm_pitch = PWM(inner_P_gain = 0.8, inner_I_gain = 0.5, outer_P_gain = 1.0, outer_D_gain = 1.0, inverted = True) 
pwm_throttle = PWM(inner_P_gain = 0.5, inner_I_gain = 0.0, outer_P_gain = 1.0, outer_D_gain = 1.0, vertical = True)


####PIXEL SETPOINTS####
x_max = 640
x_min = 0
y_max = 400
y_min = 0
####PIXYCAM####
sensor_height = 0.2430
sensor_width = 0.3888
focal = 0.36936

####HORISONTALE SETTPUNKTER####
x_center = (x_max - x_min)/2
y_center = (y_max - y_min)/2
####

#SETUP 
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='udp:127.0.0.1:14550',
	help = 'Connect to vehicle on ip address given. Default set to udp:127.0.0.1:14550')
args = parser.parse_args()

#Vehicle connection
try:
  print'Connecting to vehicle on: %s' % args.connect
  vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)

# Bad TCP connection
except socket.error:
  print 'No server exists!'
  print "Reconnecting to vehicle on: %s" %args.connect
  vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)

# Bad TTY connection
except exceptions.OSError as e:
  print 'No serial exists!'
  print "Reconnecting to vehicle on: %s" %args.connect
  vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)

# API error
except APIException:
  print 'Timeout!'
  print "Reconnecting to vehicle on: %s" %args.connect
  vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)

def RTL_failsafe():
  '''
  Checks for RTL - failsafe. If activated, overrides are reset and control is given to Tx.
  '''
  if vehicle.mode.name == RTL_MODE:
    return True
  else:
    return False

def takeover():
  '''Boolean function to check for takeover by Tx.'''
  if not searching or vehicle.channels['7'] < 1750:
    return True
  elif 1750 <= vehicle.channels['7'] <= 2012 or (1750 <= vehicle.channels['7'] <= 2012 and searching):
    return False

def manual_flight():
  ''' Function for manual manual flight. Program still searches for IR-light while manual flight. '''
  global searching
  time.sleep(0.05)
  potential_counter = 0
  print "Manual flight..."
  vehicle.parameters["ANGLE_MAX"] = MANUAL_ANGLE
  while not searching:
    #Potensiell fiks??
    if not vehicle.armed:
      break 
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
  #Don't let the user try to arm until autopilot is ready
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
  '''
    Plots process and error value for pitch and roll in their own *.png files.
    Names will consist of inner PD-values for relevant axis. Check /figurer/. 
  '''
  lostcount = 0
  print "Analyze.."
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

        plt.figure(i)
        plt.plot(dataTime, data, markersize = 1, linestyle = 'solid')
        plt.xlabel('Tid [s]')
        if i == 1:
          plt.title('Y-pixel: Prosessutgang')
          plt.ylabel('Prosessutgang [pixel]')
          plt.plot([0, dataTime[-1]],[200,200], color='green', linestyle='--')
          plt.savefig('/home/pi/Bachelor_backup/figurer/yfig_Kp='+str(pwm_pitch.inner_P_gain)+'_Kd='+str(pwm_pitch.outer_D_gain)+'_Ki='+str(pwm_pitch.inner_I_gain)+'.png')
        elif i == 2:
          plt.title('X-pixel: Prosessutgang')
          plt.ylabel('Prosessutgang [pixel]')
          plt.plot([0, dataTime[-1]],[320, 320],color='green', linestyle='--')
          plt.savefig('/home/pi/Bachelor_backup/figurer/xfig_Kp='+str(pwm_roll.inner_P_gain)+'_Kd='+str(pwm_roll.outer_D_gain)+'_Ki='+str(pwm_roll.inner_I_gain)+'.png')
        elif i == 3:
          plt.title('X-cm: Feil')
          plt.plot([0, dataTime[-1]],[0,0], color='green', linestyle='--')
          plt.ylabel('Feil [cm]')
          plt.savefig('/home/pi/Bachelor_backup/figurer/xfeil_Kp='+str(pwm_roll.inner_P_gain)+'_Kd='+str(pwm_roll.outer_D_gain)+'_Ki='+str(pwm_roll.inner_I_gain)+'.png')
        elif i == 4:
          plt.title('Y-cm: Feil')
          plt.plot([0, dataTime[-1]],[0,0],color='green', linestyle='--')
          plt.ylabel('Feil [cm]')
          plt.savefig('/home/pi/Bachelor_backup/figurer/yfeil_Kp='+str(pwm_pitch.inner_P_gain)+'_Kd='+str(pwm_pitch.outer_D_gain)+'_Ki='+str(pwm_pitch.inner_I_gain)+'.png')
    i += 1  
  print "Graphs made."

def landing_check(h, error_rate, throttle_pwm):
  '''
  Checks for landing. 
  #TODO Finn ut hva som skal til for at fartøyet skal gjenkjenne at det har landet
  '''
  if h < 20 and -10 < error_rate < 10 and 1250 <= throttle_pwm <= 1300:
    return True
  else:
    return False

def descend_check(error_x, error_y, stage):
  '''
  Checks if vehicle is within certain boundaries for descent.
  The boundaries' size is dependent on which stage the aircraft is in:
  The higher the stage number, the smaller the box gets.
  '''
  if stage == 1:
    boundary_x, boundary_y = 30, 40
  elif stage == 2:
    boundary_x, boundary_y = 20, 40
  elif stage == 3:
    boundary_x, boundary_y = 20, 40

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
  indicating(constant = True)
  while not vehicle.armed:
    if vehicle.channels['7'] > 1750:
      indicating(1, 5)
      GPIO.cleanup()
      spi.close()
      vehicle.close()
      os.execv(sys.executable, ['python'] + sys.argv)
    elif vehicle.channels[ROLL] < 1000:
      indicating(0.1, 10)
      GPIO.cleanup()
      vehicle.close()
      spi.close()
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
    indicating(0.5, 2)
    indicating(0.1, 10)

    #While disarmed, the program rests.
    #If SF switch is high, the Pi shuts down (if disarmed). 

    
    #KOMMENTER INN FOR HARDWARE IN THE LOOP
    while not vehicle.armed:
      print "Waiting..."
      if vehicle.channels['7'] > 1750:
        print "Shutting down"
        indicating(0.1, 10) 
        os.system("sudo shutdown -h now")
      else:
        pass
    
    indicating(0.05, 20)
    #arm_and_takeoff(10) #BARE FOR SITL
    #vehicle.parameters["ANGLE_MAX"] = NAV_ANGLE
    #vehicle.mode = VehicleMode(NAV_MODE) #KUN FOR SITL
    
    while vehicle.armed:
      send = get_Pixy()
    
      if not takeover(): #HARDWARE
        #By re - discovery of light:
        #Enter NAV_MODE and reduce angle movement. 
        if vehicle.mode.name != NAV_MODE or vehicle.parameters["ANGLE_MAX"] == MANUAL_ANGLE:
          vehicle.mode = VehicleMode(NAV_MODE)
          vehicle.parameters["ANGLE_MAX"] = NAV_ANGLE

        #See RTL_failsafe documentation.
        if RTL_failsafe():
          vehicle.channels.overrides = {}
          while True:
            pass         

        #Controls Pixy data for lost or corrupt data, sort of..
        while not send or (not send[0] and not send[1]) or (send[0][0]*256+send[0][1]) > 65535:
          send = get_Pixy()
          lost_counter += 1
          
          #Searches 10 times before giving up search.
          if lost_counter == 10: 
            print 'Lost track of point...'

            #Resets I term if light is lost. 
            pwm_roll.previous_sum = 0
            pwm_pitch.previous_sum = 0
            #To not reset the total counter, a lost flag is set high. See PWM.py for further function.
            pwm_roll.lostflag = True
            pwm_pitch.lostflag = True
            lost_counter = 0
            do_descend_counter = 0
            searching = False
            break

        #Begin new iteration if search has failed.  
        if not searching:
          continue
          
        else:
          h = round(vehicle.rangefinder.distance * 100, 2)
            
          #Passive filtering of image objects. Picks the one with largest area. Turns out to be a bad idea.. 
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

          #GSD = Ground Sampling Distance
          
          gsd_h = (h*sensor_height)/(focal*y_max)
          gsd_w = (h*sensor_width)/(focal*x_max)
          gsd = max(gsd_h, gsd_w)
          #Calculates error for PD-PI controller. 
          error_x = (x_center - send[0]) * gsd
          error_y = (y_center - send[1]) * gsd

          #Puts error into the PD-PI controller. 
          pwm_roll.update(error_x)
          pwm_pitch.update(error_y)
          
          #TODO: 
          # Tilpass programmet til tap av punkt. Skal den stige til en fast høyde? 
          # Eller skal den fortsette på nåværende høyde (ikke så lurt tror jeg).
          print "Descend check?"
          if descend_check(error_x, error_y, stage):
            if do_descend_counter == 13: #(1sek/0.075 ≈ 13 run throughs)
              print "Descend True!"
              if h > 1000:
                stage = 1
                desired_height = 250
                pwm_throttle.vertical_speed = 30
              elif 1000 > h > 100:
                stage = 2
                desired_height = 250
                pwm_throttle.vertical_speed = 20
              elif h < 100:
                desired_height = 250
                stage = 3
                pwm_throttle.vertical_speed = 15
                
              error_height = desired_height - h
              print "Error_h = " + str(error_height)
              pwm_throttle.update(error_height)
              print "Set_speed = " + str(pwm_throttle.set_speed)
              vehicle.channels.overrides[THROTTLE] = pwm_throttle.position 
              if stage == 3:
                pass
                #if landing_check(h, pwm_throttle.actual_speed, pwm_throttle.position):
                #  vehicle.armed = False #Disarms aicraft if landed. 
                #  time.sleep(1.0)
                #  continue
            else:
              do_descend_counter += 1
          else:
            #If the drone's error boundaries are within the wanted borders, throttle override is popped. 
            vehicle.channels.overrides.pop(THROTTLE, None)
            do_descend_counter = 0

          #vehicle.channels.overrides = {ROLL: pwm_roll.position, PITCH: pwm_pitch.position} ###THROTTLE-INPUT ER BARE FOR SITL###
          #Syntaksen under er kanskje noe bedre med tanke på throttle-implementering.
          vehicle.channels.overrides[ROLL] = pwm_roll.position
          vehicle.channels.overrides[PITCH] = pwm_pitch.position
          
          print "Send = " + str(send)
          print "h = " + str(h)
          print "Error_x = " + str(error_x)
          print "Error_y = " + str(error_y)
          print "Roll: ", pwm_roll.position
          print "Pitch: ", pwm_pitch.position
          print "Throttle: ", pwm_throttle.position
          print "Stage: ", stage
          print "ui_roll = " + str(pwm_roll.ui)
          print "ui_pitch = " + str(pwm_pitch.ui)
          print "sample = " + str(pwm_roll.sample)
          print "Actual speed y = " + str(pwm_pitch.actual_speed)
          print "Actual speed x = " + str(pwm_roll.actual_speed)
          print "Vertical speed = " + str(pwm_throttle.actual_speed)
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
        By takeover from Tx overwrites are cleared so control is restored by manual. 
        '''
        vehicle.channels.overrides = {} #HARDWARE IN THE LOOP
        #vehicle.channels.overrides = {THROTTLE: 1500} #BARE FOR SITL
        manual_flight()
        

  # Close vehicle object when testing
  except KeyboardInterrupt:
    vehicle.channels.overrides = {} #HITL
    vehicle.mode = VehicleMode('STABILIZE')
    time.sleep(0.3)
    indicating(constant = True, i=-1)
    spi.close()
    vehicle.close()
    analyze() #KOMMENTER INN FOR HARDWARE
    sys.exit(0)
  
#Analyserer og starter eventuelt programmet på nytt om ønsket.
  after_landing()
  
