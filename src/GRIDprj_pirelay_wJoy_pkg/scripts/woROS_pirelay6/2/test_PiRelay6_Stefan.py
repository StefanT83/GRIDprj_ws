import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(5,GPIO.OUT)

GPIO.output(5,GPIO.LOW)
time.sleep(1)

GPIO.output(5, GPIO.HIGH)
time.sleep(1)

GPIO.output(5, GPIO.LOW)
time.sleep(1)

GPIO.setup(6,GPIO.OUT)

GPIO.output(6,GPIO.LOW)
time.sleep(1)

GPIO.output(6, GPIO.HIGH)
time.sleep(1)

GPIO.output(6, GPIO.LOW)
time.sleep(1)
