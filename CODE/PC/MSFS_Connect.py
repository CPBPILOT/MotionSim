# -*- coding: utf-8 -*-
"""
Created on Sat Oct 26 18:48:44 2024

@author: micha
"""

from SimConnect import SimConnect, AircraftRequests
import time
from pprint import pprint
#import serial
#import serial.tools.list_ports
import math


port = "COM8"  # Update this to your Arduino's serial port
baud_rate = 9600
#ser=serial.Serial(port, baud_rate, timeout=1)
time.sleep(2)

vector = [0, 0]
  
sm = SimConnect()
# Create a request object to pull data from MSFS
aq = AircraftRequests(sm, _time=10)
accel_mode = 1
    
while True:
    try:
        # Retrieve pitch and roll data
        pitchRad = aq.get("PLANE_PITCH_DEGREES")
        rollRad = aq.get("PLANE_BANK_DEGREES")
        
        if accel_mode == 1: 
            gravVectX = 32.174 * (math.cos(pitchRad) * math.sin(rollRad)) 
            gravVectY = 32.174 * (math.cos(pitchRad) * math.cos(rollRad)) 
            gravVectZ = 32.174 * (math.sin(pitchRad)) #* math.cos(rollRad)) 
            
            accel_x = aq.get("ACCELERATION_BODY_X") + gravVectX #roll
            accel_y = aq.get("ACCELERATION_BODY_Y") + gravVectY  #vertival
            accel_z = -aq.get("ACCELERATION_BODY_Z") + gravVectZ #pitch
            
            #result of math.atan is rads which is the same unit as
            pitch = math.atan2(accel_z, accel_y)
            roll = math.atan2(accel_x, accel_y)
            # Display pitch and roll
            print(f"Accel X: {accel_x:.6f} FPSS, Accel Y: {accel_y:.6f} FPSS, Accel Z: {accel_z:.6f} FPSS ||| Pitch: {pitch:.6f} rad, Roll: {roll:.2f} rad ||| Pitch Plane: {pitchRad:.2f} rad, Roll Plane: {rollRad:.2f} rad")
        else:  
            pitch = pitchRad
            roll = rollRad
            # Display pitch and roll
            print(f"Pitch: {pitch:.2f} rad, Roll: {roll:.2f} rad")

        # Pause for a short interval to reduce load
        #time.sleep(.1)
        # Convert to degrees and flip sign to match hardware orientation
        vector[0] = -math.degrees(roll)
        vector[1] = -math.degrees(pitch)
        vector_str = f"{vector[0]},{vector[1]}\n"
        #ser.write(vector_str.encode('utf-8'))
        #print(vector)
        

    except KeyboardInterrupt:
        #ser.close()
        print('Closing')
        break
        
    #except: 
    #    next
    except Exception as e:
        print("Error:", e)
        continue