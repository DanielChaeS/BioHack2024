from pyfirmata2 import Arduino
import time  # Ensure time module is imported

# Initialize the board
board = Arduino('/dev/cu.usbmodem101')  # Replace with your port

# Define the pins for each servo
thumb = board.get_pin('d:13:s')  # Servo on pin 13
index = board.get_pin('d:11:s')  # Servo on pin 11
middle = board.get_pin('d:9:s')  # Servo on pin 9
ring = board.get_pin('d:7:s')  # Servo on pin 7
pinky = board.get_pin('d:5:s')  # Servo on pin 5


# Function to close the grip (move fingers to a specified angle)
def slow_move(start_angle, end_angle, speed):
    """
    Move multiple servos slowly and simultaneously.

    Args:
        start_angle (int): Starting angle (0-180).
        end_angle (int): Ending angle (0-180).
        speed (float): Delay in seconds between each step.
    """
    if start_angle < end_angle:
        step = 1
    else:
        step = -1

    for angle in range(start_angle, end_angle + step, step):
        thumb.write(angle)
        index.write(angle)
        middle.write(angle)
        ring.write(angle)
        pinky.write(angle)
        time.sleep(speed)


slow_move(180, 0, 0.007)  # Grip (Counter-Clockwise rotation)
time.sleep(1)            # Pause
slow_move(180, 0, 0.009)  # Release (Clockwise rotation)
