import time
import numpy as np
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt

NUM_ULTRA = 5
TRIG = 33
ECHOS = [35, 36, 37, 38, 40]	
			
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
for ECHO in ECHOS:
	GPIO.setup(ECHO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(TRIG, GPIO.OUT)

GPIO.output(TRIG, False)
time.sleep(2)

class Test:
	def __init__(self):
		self.lastUltra = np.zeros(NUM_ULTRA)
		
	def ultra_back_cb(self, channel):
		global back, start_back, done
		if (GPIO.input(ECHOS[0])):
			start_back = time.time()
		else:
			back = time.time()
			done += 1
		
	def ultra_left_cb(self, channel):
		global left, start_left, done
		if (GPIO.input(ECHOS[1])):
			start_left = time.time()
		else:
			left = time.time()
			done += 1
		
	def ultra_right_cb(self, channel):
		global right, start_right, done
		if (GPIO.input(ECHOS[2])):
			start_right = time.time()
		else:
			right = time.time()
			done += 1
		
	def ultra_frontLeft_cb(self, channel):
		global frontLeft, start_frontLeft, done
		if (GPIO.input(ECHOS[3])):
			start_frontLeft = time.time()
		else:
			frontLeft = time.time()
			done += 1
		
	def ultra_frontRight_cb(self, channel):
		global frontRight, start_frontRight, done
		if (GPIO.input(ECHOS[4])):
			start_frontRight = time.time()
		else:
			frontRight = time.time()
			done += 1
			
tst = Test()
	
GPIO.add_event_detect(ECHOS[0], GPIO.BOTH, callback=tst.ultra_back_cb)
GPIO.add_event_detect(ECHOS[1], GPIO.BOTH, callback=tst.ultra_left_cb)
GPIO.add_event_detect(ECHOS[2], GPIO.BOTH, callback=tst.ultra_right_cb)
GPIO.add_event_detect(ECHOS[3], GPIO.BOTH, callback=tst.ultra_frontLeft_cb)
GPIO.add_event_detect(ECHOS[4], GPIO.BOTH, callback=tst.ultra_frontRight_cb)

start = time.time()
back = start
left = start
right = start
frontLeft = start
frontRight = start

done = 0

GPIO.output(TRIG, True)
time.sleep(0.00001)
GPIO.output(TRIG, False)

data = []
try:
	while True:
		time.sleep(0.03)
		
		GPIO.output(TRIG, True)
		time.sleep(0.00001)
		GPIO.output(TRIG, False)
		while (done < NUM_ULTRA):
			time.sleep(0.000001)
		done = 0
		# print("NEW")
		# print((back - start_back) * 17150)
		print((left - start_left) * 17150)
		data.append((left - start_left) * 17150)
		# print((right - start_right) * 17150)
		# print((frontLeft - start_frontLeft) * 17150)
		# print((frontRight - start_frontRight) * 17150)
except (KeyboardInterrupt, SystemExit):
	plt.plot(data)
	plt.show()
	print("Cleaning up GPIO and Exiting")
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.remove_event_detect(ECHOS[0])
	GPIO.remove_event_detect(ECHOS[1])
	GPIO.remove_event_detect(ECHOS[2])
	GPIO.remove_event_detect(ECHOS[3])
	GPIO.remove_event_detect(ECHOS[4])
	GPIO.cleanup()
	
'''
# Prep the sensors
begin = time.time()
for ECHO in ECHOS:
	time.sleep(0.01)
	start = time.time()
	GPIO.output(TRIG, True)
	time.sleep(0.00001)
	GPIO.output(TRIG, False)
	
	while GPIO.input(ECHO) == 0:
		pulse_start = time.time()
		ready = time.time()
	while GPIO.input(ECHO) == 1:
		pulse_end = time.time()
	
	# print("ECHO {0}".format(ECHO))
	print((ready - start) * 1000)
	# print((pulse_end - pulse_start) * 1000)
	# print((pulse_end - pulse_start) * 17150)

# print(time.time() - begin)
# print((time.time() - begin) * 1000)

'''
