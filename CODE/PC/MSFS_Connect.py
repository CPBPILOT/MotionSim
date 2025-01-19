# -*- coding: utf-8 -*-
"""
Created on Sat Oct 26 18:48:44 2024

@author: micha
"""

from SimConnect import SimConnect, AircraftRequests
import time
from pprint import pprint
import serial
import serial.tools.list_ports
import math


port = "COM8"  # Update this to your Arduino's serial port
baud_rate = 9600
ser=serial.Serial(port, baud_rate, timeout=1)
time.sleep(2)

vector = [0, 0]
  
sm = SimConnect()
# Create a request object to pull data from MSFS
aq = AircraftRequests(sm, _time=10)
    
while True:
    try:
        # Retrieve pitch and roll data
        
        pitch = aq.get("PLANE_PITCH_DEGREES")
        roll = aq.get("PLANE_BANK_DEGREES")
        
        # Display pitch and roll
        #print(f"Pitch: {pitch:.2f} degrees, Roll: {roll:.2f} degrees")

        # Pause for a short interval to reduce load
        #time.sleep(0.1)
        vector[0]=roll*(-180/math.pi)
        vector[1]=pitch*(-180/math.pi)
        vector_str = f"{vector[0]},{vector[1]}\n"
        ser.write(vector_str.encode('utf-8'))
        print(vector)
        

    except KeyboardInterrupt:
        ser.close()
        print('Closing')
        break
        
    except: 
        next