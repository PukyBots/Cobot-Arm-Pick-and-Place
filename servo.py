import RPi.GPIO as GPIO
import time

servo_pin = 22  # choose your servo pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# Set PWM at 50Hz
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    duty = 2 + (angle / 18)   # convert angle to duty cycle
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)

    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    # Move to 90 degrees
    set_angle(90)
    time.sleep(2)

    # Move back to 0 degrees
    set_angle(0)
    time.sleep(2)

finally:
    pwm.stop()
    GPIO.cleanup()
