# -*- coding: utf-8 -*-
"""
Created on Tue Feb  6 16:56:49 2024

@author: micha
"""

from pymavlink import mavutil
import struct
import math
import serial
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()


import serial
import time
port = "COM5"  # Update this to your Arduino's serial port
baud_rate = 9600
ser=serial.Serial(port, baud_rate, timeout=1)
time.sleep(2)

vector = [0, 0]

def float_to_bytes(f):
    return struct.pack('f', f)

master = mavutil.mavlink_connection('udpin:localhost:14445')

timeout = 5 # maximum seconds to wait for a message
if not master.wait_heartbeat(timeout=timeout):
    ser.close()
    master.close()
    raise TimeoutError(f"No heartbeat within {timeout=}A seconds!")

while True:
    msg = master.recv_match(type='ATTITUDE', blocking=True).to_dict()
    #roll_bytes = float_to_bytes(msg['roll']*(180/math.pi))
    #pitch_bytes = float_to_bytes(msg['pitch']*(180/math.pi))
    #heave_bytes = float_to_bytes(0)
    #yaw_bytes = float_to_bytes(0)
    #sway_bytes = float_to_bytes(0)
    #surge_bytes = float_to_bytes(0)
    #data_send = roll_bytes + pitch_bytes+heave_bytes+yaw_bytes+sway_bytes+surge_bytes
    vector[0]=msg['roll']*(180/math.pi)
    vector[1]=msg['pitch']*(180/math.pi)
    vector_str = f"{vector[0]},{vector[1]}\n"
    ser.write(vector_str.encode('utf-8'))
    print(vector)
    
    
    #ser.write(data_send)
    #print(data_send)
    
    #ser.flushOutput()

    #ser.write(bytearray(str(2),'ascii'))

   


# Read line   

# from pymavlink import mavutil

# # Start a connection listening on a UDP port
# the_connection = mavutil.mavlink_connection('udpin:localhost:14540')

# # Wait for the first heartbeat 
# #   This sets the system and component ID of remote system for the link
# the_connection.wait_heartbeat()
# print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))