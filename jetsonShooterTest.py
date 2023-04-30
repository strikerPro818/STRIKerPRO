import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)

shooter = GPIO.PWM(32, 100)
shooter.start(0)
speed = 0
while True:
    shooter.ChangeDutyCycle(speed)
    print('Duty cycle:',speed)
    # print("Current duty cycle: {:.2f}".format(duty_cycle))
    time.sleep(0.1)
# pwm.ChangeDutyCycle(0)
shooter.Stop()
GPIO.cleanup()