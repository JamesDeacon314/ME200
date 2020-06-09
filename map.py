import cv2
import sys
import time
import pickle
import threading
import numpy as np
import RPi.GPIO as GPIO
import concurrent.futures as cf
from goprocam import GoProCamera, constants

NUM_ULTRA = 5
TRIG = 33
ECHOS = [35, 36, 37, 38, 40]

class GaussianMap:
	def __init__(self):
		self.frame = None
		self.map = None
		self.position = (None, None)
		self.orientation = None
		self.lastUltra = np.zeros(NUM_ULTRA)
		self.saveUltra = []
		self.key = ""
		self._lock = threading.Lock()
		'''
		self._lock is not currently being used.  Use it as follows:
			with self._lock:
				<CODE>
		Once the with loop ends the lock is dropped
		'''
		
	# TODO : Stop all threads on 'q'
	# TODO : Actually process the frames
			# TODO : Fix Distortion
	# TDOO : Don't allow ultrasonic data to hang
	
	def stream(self):
		goproCamera = GoProCamera.GoPro()
		goproCamera.stream("udp://127.0.0.1:10000")
		
	def readStream(self):
		gpCam = GoProCamera.GoPro()
		cap = cv2.VideoCapture("udp://127.0.0.1:10000")
		while True:
			ret, frame = cap.read()
			if ret == True:
				self.frame = frame
				
	def processStream(self):
		while True:
			with self._lock:
				img = self.frame
			if (img is not None):
				#cv2.imshow("GoPto OpenCV", img)
				#cv2.waitKey(1)
				continue
			
		cap.release()
		cv2.destroyAllWindows()
		
	def queryUltra(self):
		while True:
			for i, ECHO in enumerate(ECHOS):
				time.sleep(0.01)
				GPIO.output(TRIG, True)
				time.sleep(0.00001)
				GPIO.output(TRIG, False)
				
				while GPIO.input(ECHO) == 0:
					pulse_start = time.time()
					start = time.time()
				while GPIO.input(ECHO) == 1:
					pulse_end = time.time()
				
				self.lastUltra[i] = (pulse_end - pulse_start) * 17150
			with self._lock:
				self.saveUltra.append(self.lastUltra)
			
	def drive(self, drive, reverse, speed, s, steer):
		start = time.time()
		stop = False
		while (True):
			ch = sys.stdin.read(1)
			if stop == True:
				s.ChangeDutyCycle(0)
				s.stop()
				reverse.ChangeDutyCycle(0)
				drive.ChangeDutyCycle(0)
				GPIO.setup(13,GPIO.IN)
				drive.stop()
				reverse.stop()
				break
			if ch == "a":
				with self._lock:
					steer=min(90, steer + 5)
					time.sleep(0.1)
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					time.sleep(0.1)
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					time.sleep(0.1)
					print(steer)
			elif ch == "d":
				with self._lock:
					steer=max(50, steer - 5)
					time.sleep(0.1)
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					time.sleep(0.1)
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					time.sleep(0.1)
					print(steer)
			elif ch == 'p':
					stop = True
			elif ch == "w":
				print("forward")
				reverse.ChangeDutyCycle(0)
				drive.ChangeDutyCycle(speed)
			elif ch == "s":
				print("backward")
				drive.ChangeDutyCycle(0)
				reverse.ChangeDutyCycle(speed)
			elif ch == "f":
				print("stop")
				reverse.ChangeDutyCycle(0)
				drive.ChangeDutyCycle(0)
			
# Configure GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Setup Ultrasonic GPIO
for ECHO in ECHOS:
	GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(TRIG, GPIO.OUT)

GPIO.output(TRIG, False)
time.sleep(2)

# Setup Driving GPIO
GPIO.setup(13,GPIO.OUT)

print("w/s: acceleration")
print("a/d: steering")
print("esc: exit")

s=GPIO.PWM(13,500)
s.start(0)
s.start(70)
time.sleep(0.1)

steer=70
s.ChangeDutyCycle(steer)
time.sleep(0.1)
s.ChangeDutyCycle(0)
stop = False

LPWM = 16
RPWM = 18
GPIO.setup(LPWM, GPIO.OUT)
GPIO.setup(RPWM, GPIO.OUT)

drive = GPIO.PWM(LPWM, 1000)
drive.start(0)

reverse = GPIO.PWM(RPWM, 1000)
reverse.start(0)
speed = 20

# Create the class object
gMap = GaussianMap()

print("Starting Threads\n")
workers = 5
try:
	with cf.ThreadPoolExecutor(max_workers=workers) as e:
		e.submit(gMap.stream)
		e.submit(gMap.readStream)
		e.submit(gMap.processStream)
		e.submit(gMap.queryUltra)
		e.submit(gMap.drive, drive, reverse, speed, s, steer)
except (KeyboardInterrupt, SystemExit):
	print("\nThreads are Complete")
	print("Saving Class Object...")
	with open("map.pkl", "wb") as f:
		pickle.dump(gMap.saveUltra, f, -1)
	print("Cleaning up GPIO and Exiting")
	GPIO.cleanup()
	
