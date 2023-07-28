import RPi.GPIO as GPIO
import time
import threading

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

GPIO.cleanup()

# Use the i2c-8 bus
i2c_bus = busio.I2C(SCL, SDA)

pca_output = PCA9685(i2c_bus)

# Set the PWM frequency to 60Hz
pca_output.frequency = 1000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


switch_lift_up = 21
switch_level_down = 20


GPIO.setup(switch_lift_up, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(switch_level_down, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Use event objects to control the threads
lift_up_event = threading.Event()
level_up_event = threading.Event()
lift_down_event = threading.Event()
level_down_event = threading.Event()

def lift_up():
    print("Lift: Moving Up")
    pca_output.channels[2].duty_cycle = int(100 / 100 * 0xFFFF)
    pca_output.channels[3].duty_cycle = int(0 / 100 * 0xFFFF)

    while not lift_up_event.is_set():
        if GPIO.input(switch_lift_up) == False:
            print("Lift: Stop")
            pca_output.channels[2].duty_cycle = int(0 / 100 * 0xFFFF)
            pca_output.channels[3].duty_cycle = int(0 / 100 * 0xFFFF)
            break
        time.sleep(0.1)

def lift_down():
    print("Lift: Moving Down")
    pca_output.channels[2].duty_cycle = int(0 / 100 * 0xFFFF)
    pca_output.channels[3].duty_cycle = int(100 / 100 * 0xFFFF)

    while not lift_down_event.is_set():
        # Add your condition to stop lift down here
        # For now, I am using a dummy condition (False)
        if False:
            print("Lift: Stop")
            pca_output.channels[2].duty_cycle = int(0 / 100 * 0xFFFF)
            pca_output.channels[3].duty_cycle = int(0 / 100 * 0xFFFF)
            break
        time.sleep(0.1)

def level_up():
    print("Level: Moving Up")
    pca_output.channels[3].duty_cycle = int(0 / 100 * 0xFFFF)
    pca_output.channels[4].duty_cycle = int(100 / 100 * 0xFFFF)

    while not level_up_event.is_set():
        if GPIO.input(switch_level_down) == False:
            print("Level: Stop")
            pca_output.channels[3].duty_cycle = int(0 / 100 * 0xFFFF)
            pca_output.channels[4].duty_cycle = int(0 / 100 * 0xFFFF)
            break
        time.sleep(0.1)

def level_down():
    print("Level: Moving Down")
    pca_output.channels[3].duty_cycle = int(100 / 100 * 0xFFFF)
    pca_output.channels[4].duty_cycle = int(0 / 100 * 0xFFFF)

    while not level_down_event.is_set():
        # Add your condition to stop level down here
        # For now, I am using a dummy condition (False)
        if False:
            print("Level: Stop")
            pca_output.channels[3].duty_cycle = int(0 / 100 * 0xFFFF)
            pca_output.channels[4].duty_cycle = int(0 / 100 * 0xFFFF)
            break
        time.sleep(0.1)

def move_up():
    lift_up_thread = threading.Thread(target=lift_up, name="Lift Up Thread")
    level_up_thread = threading.Thread(target=level_up, name="Level Up Thread")

    lift_up_thread.start()
    level_up_thread.start()

    while lift_up_thread.is_alive() or level_up_thread.is_alive():
        time.sleep(0.1)

    lift_up_event.set()
    level_up_event.set()

def move_down():
    lift_down_thread = threading.Thread(target=lift_down, name="Lift Down Thread")
    level_down_thread = threading.Thread(target=level_down, name="Level Down Thread")

    lift_down_thread.start()
    level_down_thread.start()

    while lift_down_thread.is_alive() or level_down_thread.is_alive():
        time.sleep(0.1)

    lift_down_event.set()
    level_down_event.set()

lift_up()
# lift_down()
# level_up()
# level_down()
# move_up()
# move_down()
# try:
#     move_down()
# except:
#     print('Error!')

