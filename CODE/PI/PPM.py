#!/usr/bin/env python

# PPM.py
# Author: Michael Rechtin 2025

import time
import pigpio # http://abyz.me.uk/rpi/pigpio/python.html
import pygame
import serial
#import threading
pygame.init()
pygame.joystick.init()
NUM_CHANNELS = 8
CHANNEL_MIN = 1.0  # Minimum pulse width in milliseconds
CHANNEL_MAX = 2.0  # Maximum pulse width in milliseconds
FRAME_LENGTH = 22.5  # Total frame length in milliseconds
SYNC_WIDTH = 0.3  # Sync pulse width in milliseconds
GPIO_PIN = 18  # GPIO pin for PPM output
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 9600

float_array = [0.00,0.00,0.00]


Joystick_Name='Mad Catz Saitek Pro Flight X-56 Rhino Stick'
Throttle_Name='Mad Catz Saitek Pro Flight X-56 Rhino Throttle'
Rudder_Name='Thrustmaster T-Rudder'
#channel_lock = threading.Lock()

ax_1=1500
ax_2=1500
ax_3=1000
ax_4=1500
ax_5=1000
ax_6=1000
ax_7=1000
ax_8=1000
class X:

   GAP=300
   WAVES=3

   def __init__(self, pi, gpio, channels=NUM_CHANNELS, frame_ms=FRAME_LENGTH):
      self.pi = pi
      self.gpio = gpio

      if frame_ms < 5:
         frame_ms = 5
         channels = 2
      elif frame_ms > 100:
         frame_ms = 100

      self.frame_ms = frame_ms

      self._frame_us = int(frame_ms * 1000)
      self._frame_secs = frame_ms / 1000.0

      if channels < 1:
         channels = 1
      elif channels > (frame_ms // 2):
         channels = int(frame_ms // 2)

      self.channels = channels

      self._widths = [1000] * channels # set each channel to minimum pulse width

      self._wid = [None]*self.WAVES
      self._next_wid = 0

      pi.write(gpio, pigpio.LOW)

      self._update_time = time.time()

   def _update(self):
      wf =[]
      micros = 0
      for i in self._widths:
         wf.append(pigpio.pulse(1<<self.gpio, 0, self.GAP))
         wf.append(pigpio.pulse(0, 1<<self.gpio, i-self.GAP))
         micros += i
      # off for the remaining frame period
      wf.append(pigpio.pulse(1<<self.gpio, 0, self.GAP))
      micros += self.GAP
      wf.append(pigpio.pulse(0, 1<<self.gpio, self._frame_us-micros))

      self.pi.wave_add_generic(wf)
      wid = self.pi.wave_create()
      self.pi.wave_send_using_mode(wid, pigpio.WAVE_MODE_REPEAT_SYNC)
      self._wid[self._next_wid] = wid

      self._next_wid += 1
      if self._next_wid >= self.WAVES:
         self._next_wid = 0

      
      remaining = self._update_time + self._frame_secs - time.time()
      if remaining > 0:
         time.sleep(remaining)
      self._update_time = time.time()

      wid = self._wid[self._next_wid]
      if wid is not None:
         self.pi.wave_delete(wid)
         self._wid[self._next_wid] = None

   def update_channel(self, channel, width):
      self._widths[channel] = width
      self._update()

   def update_channels(self, widths):
      self._widths[0:len(widths)] = widths[0:self.channels]
      self._update()

   def cancel(self):
      self.pi.wave_tx_stop()
      for i in self._wid:
         if i is not None:
            self.pi.wave_delete(i)
def initialize_devices():
    # Initialize pygame
    pygame.init()

    # Check for connected joysticks
    if pygame.joystick.get_count() == 0:
        print("No joystick connected.")
        return None

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(get_joystick_id(Joystick_Name))
    joystick.init()
    throttle = pygame.joystick.Joystick(get_joystick_id(Throttle_Name))
    throttle.init()
    pedals = pygame.joystick.Joystick(get_joystick_id(Rudder_Name))
    pedals.init()
    print(f"Initialized joystick: {joystick.get_name()}")
    print(f"Initialized joystick: {throttle.get_name()}")
    print(f"Initialized joystick: {pedals.get_name()}")
    return joystick, throttle, pedals

def get_joystick_id(name):
    
    count=pygame.joystick.get_count()
    if count == 0:
        print("No joystick connected.")
        return None
    
    for i in range(count):
        if (pygame.joystick.Joystick(i).get_name()==name):
            return i
        
def read_joystick(joystick):
    # Poll joystick events
    pygame.event.pump()

    # Read axis values
    num_axes = joystick.get_numaxes()
    axes = [joystick.get_axis(i) for i in range(num_axes)]
    
    # Read button states
    num_buttons = joystick.get_numbuttons()
    buttons = [joystick.get_button(i) for i in range(num_buttons)]
    
    # Read hat (D-pad) states
    num_hats = joystick.get_numhats()
    hats = [joystick.get_hat(i) for i in range(num_hats)]

    return axes, buttons, hats

def read_throttle(throttle):
    # Poll joystick events
    pygame.event.pump()

    # Read axis values  
    num_axes = throttle.get_numaxes()
    axes = [throttle.get_axis(i) for i in range(num_axes)]
    # Read button states
    num_buttons = throttle.get_numbuttons()
    buttons = [throttle.get_button(i) for i in range(num_buttons)]
    
    # Read hat (D-pad) states
    num_hats = throttle.get_numhats()
    hats = [throttle.get_hat(i) for i in range(num_hats)]

    return axes, buttons, hats
        
def read_pedals(pedals):
    # Poll joystick events
    pygame.event.pump()

    # Read axis values  
    num_axes = pedals.get_numaxes()
    axes = [pedals.get_axis(i) for i in range(num_axes)]
    # Read button states
    num_buttons = pedals.get_numbuttons()
    buttons = [pedals.get_button(i) for i in range(num_buttons)]
    
    # Read hat (D-pad) states
    num_hats = pedals.get_numhats()
    hats = [pedals.get_hat(i) for i in range(num_hats)]

    return axes, buttons, hats

if __name__ == "__main__":

    import time
    import PPM
    import pigpio
    
    joystick, throttle, pedals = initialize_devices()
    
    pi = pigpio.pi()

    if not pi.connected:
        exit(0)
    pi.wave_tx_stop() # Start with a clean slate.

    ppm = PPM.X(pi, GPIO_PIN, frame_ms=FRAME_LENGTH)


    try:
        while True:
            axes_joystick, buttons_joystick, hats_joystick = read_joystick(joystick)
            axes_throttle, buttons_throttle, hats_throttle = read_throttle(throttle)
            axes_pedals, buttons_pedals, hats_pedals = read_pedals(pedals)
            ax_1=int((axes_joystick[0]/2+1.5)*1000)
            ax_2=int(((axes_joystick[1]*-1/2)+1.5)*1000)
            ax_3=int(((axes_throttle[0]*-1/2)+1.5)*1000)
            ax_4=int((axes_pedals[2]/2+1.5)*1000)
            ax_5=int((buttons_throttle[32]+1)*1000)
            
            if (buttons_throttle[33]):
                ax_6=1000
            elif (buttons_throttle[34]):
                ax_6=1500
            else:
                ax_6=2000
            #ax_6=int((buttons_joystick[1]/2+1.5)*1000)
            ax_7=int((buttons_joystick[0]+1)*1000)
            ax_8=int((buttons_joystick[1]+1)*1000)
            ppm.update_channels([ax_1,ax_2,ax_3,ax_4,ax_5,ax_6,ax_7,ax_8])
            print([ax_1,ax_2,ax_3,ax_4,ax_5,ax_6,ax_7,ax_8])
            with serial.Serial(SERIAL_PORT,BAUD_RATE,timeout=1) as ser:
                data_string=f"{ax_1},{ax_2},{ax_3}\n"
                ser.write(data_string.encode('utf-8'))
            #print(buttons_throttle)
    except KeyboardInterrupt:
        print("Joystick handling stopped.")
    finally:
        pygame.quit()
        ppm.cancel()
        pi.stop()
#while True:
    #ppm.update_channels([1000, 2000, 1000, 2000, 1000, 2000, 1000, 2000])

    time.sleep(2)

    ppm.cancel()

    pi.stop()

