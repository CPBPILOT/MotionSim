# -*- coding: utf-8 -*-
"""
Created on Sat Oct 26 18:48:44 2024

@author: micha
"""

# Import required libraries
from SimConnect import SimConnect, AircraftRequests  # MSFS SimConnect API to get flight data
import time  # For sleep/delay
import serial  # For communication with Arduino 
import serial.tools.list_ports  # For detecting available serial ports 
import math  # For trigonometric and math functions

# Define serial port and baud rate for Arduino communication
port = "COM8"  # Update this to your Arduino's serial port
baud_rate = 9600
ser=serial.Serial(port, baud_rate, timeout=1)  # Initialize serial connection
time.sleep(2)  # Wait for the serial connection or system to stabilize

# Initialize the output vector to hold pitch and roll values
vector = [0, 0]

# Create SimConnect object to communicate with MSFS
sm = SimConnect()
# Create an AircraftRequests object to access aircraft data
aq = AircraftRequests(sm, _time=10)

# Set the acceleration mode to 1 (enables acceleration-based motion)
accel_mode = 1

# Main loop to continuously read and process flight datay
while True:
    try:
        # Retrieve aircraft pitch and roll from MSFS in radians
        pitchRad = aq.get("PLANE_PITCH_DEGREES")
        rollRad = aq.get("PLANE_BANK_DEGREES")
        
        if accel_mode == 1:
            # Compute gravity vector components in the aircraft's body axis
            # 32.174 ft/s² is standard gravity
            gravVectX = 32.174 * (math.cos(pitchRad) * math.sin(rollRad))  # Lateral component
            gravVectY = 32.174 * (math.cos(pitchRad) * math.cos(rollRad))  # Vertical component
            gravVectZ = 32.174 * (math.sin(pitchRad))  # Longitudinal (forward/backward) component
            
            # Combine measured acceleration with gravitational effects
            accel_x = aq.get("ACCELERATION_BODY_X") + gravVectX  # Roll axis (lateral)
            accel_y = aq.get("ACCELERATION_BODY_Y") + gravVectY  # Vertical axis
            accel_z = -aq.get("ACCELERATION_BODY_Z") + gravVectZ  # Pitch axis (longitudinal); negative is intentional

            # Calculated pitch and roll using arctangent of acceleration components
            pitch = math.atan2(accel_z, accel_y) # Positive pitch = tilt forward, negitive = tilt backwards 
            roll = math.atan2(accel_x, accel_y) # Positive roll = tilt left, negitive = tilt right 

            # Print acceleration and calculated orientation
            #print(f"Accel X: {accel_x:.6f} FPSS, Accel Y: {accel_y:.6f} FPSS, Accel Z: {accel_z:.6f} FPSS ||| Pitch: {pitch:.6f} rad, Roll: {roll:.2f} rad ||| Pitch Plane: {pitchRad:.2f} rad, Roll Plane: {rollRad:.2f} rad")
        else:
            # Use raw pitch and roll from simulator if accel_mode is not enabled
            pitch = pitchRad
            roll = rollRad
            #print(f"Pitch: {pitch:.2f} rad, Roll: {roll:.2f} rad")

        # Optional delay to reduce system load
        #time.sleep(.1)

        # Convert radians to degrees and invert signs to match physical orientation of the chair motion
        vector[0] = -math.degrees(roll)
        vector[1] = -math.degrees(pitch)

        # Format output string for Arduino
        vector_str = f"{vector[0]},{vector[1]}\n"

        # Send data to Arduino via serial
        ser.write(vector_str.encode('utf-8'))

        # Debug print of vector (optional)
        print(vector)

    except KeyboardInterrupt:
        # Graceful exit on Ctrl+C
        ser.close()  # Close serial port
        print('Closing')
        break

    except Exception as e:
        # Print unexpected errors without crashing the loop
        print("Error:", e)
        continue
