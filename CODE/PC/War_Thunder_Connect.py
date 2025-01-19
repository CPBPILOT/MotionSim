# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 00:17:30 2024

@author: micha
"""

from WarThunder import telemetry
from WarThunder import mapinfo
from pprint import pprint
import serial
import serial.tools.list_ports
import time

port = "COM14"  # Update this to your Arduino's serial port
baud_rate = 9600
ser=serial.Serial(port, baud_rate, timeout=1)
time.sleep(2)

vector = [0, 0]
  
while True:
    try:
        telem = telemetry.TelemInterface()
        
        while not telem.get_telemetry():
            pass
        
        dat=telem.basic_telemetry
        pitch_ang=dat['pitch']
        roll_ang=dat['roll']
        
        vector[0]=roll_ang
        vector[1]=pitch_ang
        vector_str = f"{vector[0]},{vector[1]}\n"
        ser.write(vector_str.encode('utf-8'))
        print(vector)
        

    except KeyboardInterrupt:
        print('Closing')
        break
        