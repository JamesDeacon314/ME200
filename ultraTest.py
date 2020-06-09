import time
import numpy as np
import RPi.GPIO as GPIO

NUM_ULTRA = 5
TRIG = 33
ECHOS = [35, 36, 37, 38, 40]
			
			
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
for ECHO in ECHOS:
	GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(TRIG, GPIO.OUT)

GPIO.output(TRIG, False)
time.sleep(2)

# Prep the sensors
begin = time.time()
for ECHO in ECHOS:
	time.sleep(0.01)
	GPIO.output(TRIG, True)
	time.sleep(0.00001)
	GPIO.output(TRIG, False)
	
	while GPIO.input(ECHO) == 0:
		pulse_start = time.time()
		start = time.time()
	while GPIO.input(ECHO) == 1:
		pulse_end = time.time()
	
	print((pulse_end - pulse_start) * 17150)

print(time.time() - begin)
print((time.time() - begin) * 1000)
