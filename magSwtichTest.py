import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(37, GPIO.IN, GPIO.PUD_UP)  # Add pull-up resistor

# Debounce time in seconds
debounce_time = 0.1
# Threshold for consecutive readings
consecutive_threshold = 6

prev_input_state = True
consecutive_count = 0

while True:
    current_input_state = GPIO.input(37)

    if prev_input_state == current_input_state:
        consecutive_count += 1
    else:
        consecutive_count = 0

    if consecutive_count >= consecutive_threshold:
        if not current_input_state:
            print("Triggered")
        else:
            print("NO")
        consecutive_count = 0

    prev_input_state = current_input_state
    time.sleep(debounce_time)
