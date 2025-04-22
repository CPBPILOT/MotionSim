#!/usr/bin/env python

# PPM.py
# Author: Michael Rechtin 2025

# Import necessary libraries
import time
import pigpio  # For generating precise GPIO waveforms
import pygame  # For joystick/game controller input
import serial  # For serial communication
# import threading  # Commented out but can be used for concurrency

# Initialize pygame
pygame.init()
pygame.joystick.init()

# Constants
NUM_CHANNELS = 8  # Number of PPM channels
CHANNEL_MIN = 1.0  # Min pulse width in ms
CHANNEL_MAX = 2.0  # Max pulse width in ms
FRAME_LENGTH = 22.5  # Total frame duration in ms
SYNC_WIDTH = 0.3  # Width of sync pulse
GPIO_PIN = 18  # GPIO pin used for outputting PPM signal
SERIAL_PORT = "/dev/serial0"  # Serial port for sending data
BAUD_RATE = 9600  # Baud rate for serial communication

# Placeholder array to send via serial
float_array = [0.00, 0.00, 0.00]

# Joystick device names to identify specific hardware
Joystick_Name = 'Mad Catz Saitek Pro Flight X-56 Rhino Stick'
Throttle_Name = 'Mad Catz Saitek Pro Flight X-56 Rhino Throttle'
Rudder_Name = 'Thrustmaster T-Rudder'

# Initial default values for each PPM channel
ax_1 = 1500
ax_2 = 1500
ax_3 = 1000
ax_4 = 1500
ax_5 = 1000
ax_6 = 1000
ax_7 = 1000
ax_8 = 1000

# PPM Signal Generator Class
class X:

    GAP = 300  # Time gap between pulses in microseconds
    WAVES = 3  # Buffer size for pigpio wave IDs

    def __init__(self, pi, gpio, channels=NUM_CHANNELS, frame_ms=FRAME_LENGTH):
        self.pi = pi
        self.gpio = gpio

        # Clamp the frame length within bounds
        if frame_ms < 5:
            frame_ms = 5
            channels = 2
        elif frame_ms > 100:
            frame_ms = 100

        self.frame_ms = frame_ms
        self._frame_us = int(frame_ms * 1000)  # Convert to microseconds
        self._frame_secs = frame_ms / 1000.0  # Convert to seconds

        # Clamp channel count
        if channels < 1:
            channels = 1
        elif channels > (frame_ms // 2):
            channels = int(frame_ms // 2)

        self.channels = channels
        self._widths = [1000] * channels  # Initialize all channels to min width
        self._wid = [None] * self.WAVES  # Waveform ID buffer
        self._next_wid = 0  # Index of next wave ID to reuse

        pi.write(gpio, pigpio.LOW)  # Ensure GPIO starts low
        self._update_time = time.time()

    def _update(self):
        # Create pigpio waveform for the current set of channel pulse widths
        wf = []
        micros = 0
        for i in self._widths:
            wf.append(pigpio.pulse(1 << self.gpio, 0, self.GAP))  # Pulse on
            wf.append(pigpio.pulse(0, 1 << self.gpio, i - self.GAP))  # Pulse off
            micros += i

        # Add sync pulse to finish the frame
        wf.append(pigpio.pulse(1 << self.gpio, 0, self.GAP))
        micros += self.GAP
        wf.append(pigpio.pulse(0, 1 << self.gpio, self._frame_us - micros))  # Fill remaining frame time

        # Create and send waveform
        self.pi.wave_add_generic(wf)
        wid = self.pi.wave_create()
        self.pi.wave_send_using_mode(wid, pigpio.WAVE_MODE_REPEAT_SYNC)
        self._wid[self._next_wid] = wid

        # Rotate buffer index
        self._next_wid += 1
        if self._next_wid >= self.WAVES:
            self._next_wid = 0

        # Sleep to maintain consistent frame rate
        remaining = self._update_time + self._frame_secs - time.time()
        if remaining > 0:
            time.sleep(remaining)
        self._update_time = time.time()

        # Delete the previous waveform
        wid = self._wid[self._next_wid]
        if wid is not None:
            self.pi.wave_delete(wid)
            self._wid[self._next_wid] = None

    def update_channel(self, channel, width):
        # Update a single channel pulse width
        self._widths[channel] = width
        self._update()

    def update_channels(self, widths):
        # Update all channel pulse widths
        self._widths[0:len(widths)] = widths[0:self.channels]
        self._update()

    def cancel(self):
        # Stop and clean up waveforms
        self.pi.wave_tx_stop()
        for i in self._wid:
            if i is not None:
                self.pi.wave_delete(i)

# Initialize connected input devices
def initialize_devices():
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick connected.")
        return None

    # Initialize each device by matching name
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

# Find device ID based on name
def get_joystick_id(name):
    count = pygame.joystick.get_count()
    if count == 0:
        print("No joystick connected.")
        return None
    for i in range(count):
        if pygame.joystick.Joystick(i).get_name() == name:
            return i

# Read joystick data (axes, buttons, hats)
def read_joystick(joystick):
    pygame.event.pump()
    num_axes = joystick.get_numaxes()
    axes = [joystick.get_axis(i) for i in range(num_axes)]
    num_buttons = joystick.get_numbuttons()
    buttons = [joystick.get_button(i) for i in range(num_buttons)]
    num_hats = joystick.get_numhats()
    hats = [joystick.get_hat(i) for i in range(num_hats)]
    return axes, buttons, hats

# Read throttle data
def read_throttle(throttle):
    pygame.event.pump()
    num_axes = throttle.get_numaxes()
    axes = [throttle.get_axis(i) for i in range(num_axes)]
    num_buttons = throttle.get_numbuttons()
    buttons = [throttle.get_button(i) for i in range(num_buttons)]
    num_hats = throttle.get_numhats()
    hats = [throttle.get_hat(i) for i in range(num_hats)]
    return axes, buttons, hats

# Read rudder pedal data
def read_pedals(pedals):
    pygame.event.pump()
    num_axes = pedals.get_numaxes()
    axes = [pedals.get_axis(i) for i in range(num_axes)]
    num_buttons = pedals.get_numbuttons()
    buttons = [pedals.get_button(i) for i in range(num_buttons)]
    num_hats = pedals.get_numhats()
    hats = [pedals.get_hat(i) for i in range(num_hats)]
    return axes, buttons, hats

# Main execution block
if __name__ == "__main__":
    import time
    import PPM  # Self-import for running PPM class
    import pigpio

    # Initialize devices
    joystick, throttle, pedals = initialize_devices()

    # Connect to pigpio daemon
    pi = pigpio.pi()
    if not pi.connected:
        exit(0)
    pi.wave_tx_stop()  # Clear any previous waveform activity

    # Create PPM generator instance
    ppm = PPM.X(pi, GPIO_PIN, frame_ms=FRAME_LENGTH)

    try:
        while True:
            # Read input from devices
            axes_joystick, buttons_joystick, hats_joystick = read_joystick(joystick)
            axes_throttle, buttons_throttle, hats_throttle = read_throttle(throttle)
            axes_pedals, buttons_pedals, hats_pedals = read_pedals(pedals)

            # Map joystick axes to PPM channels (range conversion)
            ax_1 = int((axes_joystick[0] / 2 + 1.5) * 1000)
            ax_2 = int(((axes_joystick[1] * -1 / 2) + 1.5) * 1000)
            ax_3 = int(((axes_throttle[0] * -1 / 2) + 1.5) * 1000)
            ax_4 = int((axes_pedals[2] / 2 + 1.5) * 1000)

            # Map buttons to fixed values
            ax_5 = int((buttons_throttle[32] + 1) * 1000)
            if buttons_throttle[33]:
                ax_6 = 1000
            elif buttons_throttle[34]:
                ax_6 = 1500
            else:
                ax_6 = 2000
            ax_7 = int((buttons_joystick[0] + 1) * 1000)
            ax_8 = int((buttons_joystick[1] + 1) * 1000)

            # Send values to PPM generator
            ppm.update_channels([ax_1, ax_2, ax_3, ax_4, ax_5, ax_6, ax_7, ax_8])

            # Print the current channel values
            print([ax_1, ax_2, ax_3, ax_4, ax_5, ax_6, ax_7, ax_8])

            # Send first three values via serial (e.g., to Arduino or telemetry system)
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                data_string = f"{ax_1},{ax_2},{ax_3}\n"
                ser.write(data_string.encode('utf-8'))

    except KeyboardInterrupt:
        # Handle graceful shutdown
        print("Joystick handling stopped.")
    finally:
        pygame.quit()
        ppm.cancel()
        pi.stop()

# Optional test block (commented out)
# while True:
#     ppm.update_channels([1000, 2000, 1000, 2000, 1000, 2000, 1000, 2000])
#     time.sleep(2)
#     ppm.cancel()
#     pi.stop()
