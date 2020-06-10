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
OBJ_TO_FIND = 3

class GaussianMap:
	def __init__(self):
		self.frame = None
		self.map = None
		self.position = (None, None)
		self.orientation = None
		self.lastUltra = np.zeros(NUM_ULTRA)
		self.ultraState = np.ones(NUM_ULTRA) * -1
		self.hasUltra = np.zeros(NUM_ULTRA)
		self.start_back = 0
		self.start_left = 0
		self.start_right = 0
		self.start_frontLeft = 0
		self.start_frontRight = 0
		self.back = 0
		self.left = 0
		self.right = 0
		self.frontLeft = 0
		self.frontRight = 0
		self.done = 0
		self._lock = threading.Lock()
		'''
		self._lock is not currently being used.  Use it as follows:
			with self._lock:
				<CODE>
		Once the with loop ends the lock is dropped
		'''
		
	# TODO : Put Ultrasonic Info somewhere
	# TODO : Use the ultrasonic info plus center position to make a map
		# TODO : Calculat orientation as well
	# TODO : Save Ultrasonic Info so graphs can be made
	
	def ultra_back_cb(self, channel):
		if (GPIO.input(ECHOS[0])):
			self.start_back = time.time()
			self.ultraState[0] = 0
		else:
			self.back = time.time()
			if (self.ultraState[0] == 0):
				self.ultraState[0] = 1
			else:
				self.ultraState[0] = -1
			if (self.hasUltra[0] == 0):
				self.done += 1
				self.hasUltra[0] = 1
		
	def ultra_left_cb(self, channel):
		if (GPIO.input(ECHOS[1])):
			self.start_left = time.time()
			self.ultraState[1] = 0
		else:
			self.left = time.time()
			if (self.ultraState[1] == 0):
				self.ultraState[1] = 1
			else:
				self.ultraState[1] = -1
			if (self.hasUltra[1] == 0):
				self.done += 1
				self.hasUltra[1] = 1
		
	def ultra_right_cb(self, channel):
		if (GPIO.input(ECHOS[2])):
			self.start_right = time.time()
			self.ultraState[2] = 0
		else:
			self.right = time.time()
			if (self.ultraState[2] == 0):
				self.ultraState[2] = 1
			else:
				self.ultraState[2] = -1
			if (self.hasUltra[2] == 0):
				self.done += 1
				self.hasUltra[2] = 1
		
	def ultra_frontLeft_cb(self, channel):
		if (GPIO.input(ECHOS[3])):
			self.start_frontLeft = time.time()
			self.ultraState[3] = 0
		else:
			self.frontLeft = time.time()
			if (self.ultraState[3] == 0):
				self.ultraState[3] = 1
			else:
				self.ultraState[3] = -1
			if (self.hasUltra[3] == 0):
				self.done += 1
				self.hasUltra[3] = 1
		
	def ultra_frontRight_cb(self, channel):
		if (GPIO.input(ECHOS[4])):
			self.start_frontRight = time.time()
			self.ultraState[4] = 0
		else:
			self.frontRight = time.time()
			if (self.ultraState[4] == 0):
				self.ultraState[4] = 1
			else:
				self.ultraState[4] = -1
			if (self.hasUltra[4] == 0):
				self.done += 1
				self.hasUltra[4] = 1
	
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
				print("PROCESSING THE STREAMMMMMMMMM")
				GPIO.output(TRIG, True)
				time.sleep(0.00001)
				GPIO.output(TRIG, False)
				
				self.detect_spheres(img)
			
		cap.release()
		cv2.destroyAllWindows()
		
	def extract_position(self, maskr, image):
		output = []

		# get contours
		contours, hierarchy = cv2.findContours(maskr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = sorted(contours, key=cv2.contourArea, reverse=True)
		for i in range(min(len(contours), OBJ_TO_FIND)):
			cv2.drawContours(image, contours, i, (0, 255, 0), -1)

		x = []
		y = []
		for i, con in enumerate(contours):
			if (i >= OBJ_TO_FIND):
				break
			M = cv2.moments(con)
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
			x.append(cX)
			y.append(cY)

		if (len(x) > 0):
			self.position = (sum(x) / len(x), sum(y) / len(y))

	def detect_spheres(self, I):
		# Format the image
		image = cv2.cvtColor(I, cv2.COLOR_RGB2BGR)
		I = cv2.cvtColor(I, cv2.COLOR_RGB2BGR)
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		# Hue thresholds
		lower_orange = np.array([0, 150, 150])
		upper_orange = np.array([25, 255, 255])

		# Mask generation
		maskr = cv2.inRange(hsv, lower_orange, upper_orange)

		# Mask filtering
		kernele = np.ones((3,3),np.uint8)
		kernel = np.ones((2,2), np.uint8)
		kerneld = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5, 5))
		maskr = cv2.erode(maskr,kernel,iterations=1)
		maskr = cv2.morphologyEx(maskr, cv2.MORPH_CLOSE, kerneld, iterations=1)
		maskr = cv2.dilate(cv2.erode(maskr,kernel,iterations=1),kernele,iterations=3)

		#print(image.shape)
		#cv2.imshow('image', image)
		result = cv2.bitwise_and(image,image,mask = maskr)
		#result = cv2.resize(result, (960, 720), interpolation=cv2.INTER_AREA)
		#image = cv2.resize(image,(960, 720), interpolation=cv2.INTER_AREA)
		#maskr = cv2.resize(maskr, (960, 720), interpolation=cv2.INTER_AREA)
		self.extract_position(maskr, image)
		
	def queryUltra(self):
		while True:
			while (self.done < NUM_ULTRA):
				time.sleep(0.000001)

			with self._lock:
				self.done = 0
				self.hasUltra = np.zeros(NUM_ULTRA)
				print(self.ultraState)
				print((self.back - self.start_back) * 17150)
				print((self.left - self.start_left) * 17150)
				print((self.right - self.start_right) * 17150)
				print((self.frontLeft - self.start_frontLeft) * 17150)
				print((self.frontRight - self.start_frontRight) * 17150)
			
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
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					print(steer)
			elif ch == "d":
				with self._lock:
					steer=max(50, steer - 5)
					time.sleep(0.1)
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(steer)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
					time.sleep(0.1)
					s.ChangeDutyCycle(0)
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
time.sleep(0.1)
s.start(70)
time.sleep(0.1)

steer=70
s.ChangeDutyCycle(steer)
time.sleep(0.1)
s.ChangeDutyCycle(steer)
time.sleep(0.1)
s.ChangeDutyCycle(steer)
time.sleep(0.1)
s.ChangeDutyCycle(0)
time.sleep(0.1)
s.ChangeDutyCycle(0)
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

GPIO.add_event_detect(ECHOS[0], GPIO.BOTH, callback=gMap.ultra_back_cb)
GPIO.add_event_detect(ECHOS[1], GPIO.BOTH, callback=gMap.ultra_left_cb)
GPIO.add_event_detect(ECHOS[2], GPIO.BOTH, callback=gMap.ultra_right_cb)
GPIO.add_event_detect(ECHOS[3], GPIO.BOTH, callback=gMap.ultra_frontLeft_cb)
GPIO.add_event_detect(ECHOS[4], GPIO.BOTH, callback=gMap.ultra_frontRight_cb)

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
	'''
	print("Saving Class Object...")
	with open("map.pkl", "wb") as f:
		pickle.dump(gMap.saveUltra, f, -1)
	'''
	print("Cleaning up GPIO and Exiting")
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.remove_event_detect(ECHOS[0])
	GPIO.remove_event_detect(ECHOS[1])
	GPIO.remove_event_detect(ECHOS[2])
	GPIO.remove_event_detect(ECHOS[3])
	GPIO.remove_event_detect(ECHOS[4])
	GPIO.cleanup()
