import cv2
import sys
import time
import pickle
import threading
from multiprocessing import Process, Queue, Array
import numpy as np
import RPi.GPIO as GPIO
import concurrent.futures as cf
from goprocam import GoProCamera, constants

NUM_ULTRA = 5
TRIG = 33
ECHOS = [35, 36, 37, 38, 40]
OBJ_TO_FIND = 3
img = None
	
def stream():
	goproCamera = GoProCamera.GoPro()
	goproCamera.stream("udp://127.0.0.1:10000")
			
def processStream(q, a):
	gpCam = GoProCamera.GoPro()
	cap = cv2.VideoCapture("udp://127.0.0.1:10000")
	img = np.zeros((480,640,3))
	x = 280
	y = 280
	while True:
		for i in range(4):
			ret, frame = cap.read()
			
		if ret == True:
			pos = detect_spheres(frame)
			
			'''
			# Hack
			x += 1
			pos = (x, y)
			'''
			
			if (pos == (None, None)):
				continue
			
			x_b = int(pos[0] - a[0] * 1.4)
			y_b = int(pos[1])
			
			x_l = int(pos[0])
			y_l = int(pos[1] - a[1] * 1.4)
			
			x_r = int(pos[0])
			y_r = int(pos[1] + a[2] * 1.4)
			
			x_fl = int(pos[0] + a[3] * 1.4)
			y_fl = int(pos[1])
			
			x_fr = int(pos[0] + a[4] * 1.4)
			y_fr = int(pos[1])
			
			cv2.circle(img, (x_b, y_b), 3, (0, 255, 0), thickness=-1)
			cv2.circle(img, (x_l, y_l), 3, (0, 255, 0), thickness=-1)
			cv2.circle(img, (x_r, y_r), 3, (0, 255, 0), thickness=-1)
			cv2.circle(img, (x_fl, y_fl), 3, (0, 255, 0), thickness=-1)
			cv2.circle(img, (x_fr, y_fr), 3, (0, 255, 0), thickness=-1)
			
			q.put(img)
			# (480, 640, 3) -> ~138 pixels per meter
			# Or 1.4 pixels per cm
		
	cap.release()
	cv2.destroyAllWindows()
	
def extract_position(maskr, image):
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

	position = (None, None)
	if (len(x) > 0):
		position = (sum(x) / len(x), sum(y) / len(y))
		
	return position

def detect_spheres(I):
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
	return extract_position(maskr, image)
	
def queryUltra(a):
	while True:
		# distances = []
		for i in range(NUM_ULTRA):
			start = time.time()
			pulse_start = start
			
			GPIO.output(TRIG, True)
			time.sleep(0.00001)
			GPIO.output(TRIG, False)
			
			while GPIO.input(ECHOS[i]) == 0:
				pulse_start = time.time()
				if pulse_start - start > 0.01:
					break

			while GPIO.input(ECHOS[i]) == 1:
				pulse_end = time.time()
				if pulse_end - pulse_start > 0.01:
					break

			duration = pulse_end - pulse_start
			distance = duration * 17150
			# distances.append(distance)
			if (distance > 0):
				a[i] = distance
			for j  in range(NUM_ULTRA):
				if (j != i):
					while GPIO.input(ECHOS[j]) == 1:
						continue
						
		# q.put(distances)
		
def driveCmd(drive, reverse, speed, s, steer):
	stop = False
	ch = sys.stdin.read(1)
	if ch == "a":
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
		s.ChangeDutyCycle(0)
		s.stop()
		reverse.ChangeDutyCycle(0)
		drive.ChangeDutyCycle(0)
		GPIO.setup(13,GPIO.IN)
		drive.stop()
		reverse.stop()
	elif ch == "w":
		print("forward")
		reverse.ChangeDutyCycle(0)
		drive.ChangeDutyCycle(speed)
	elif ch == "s":
		print("backward")
		drive.ChangeDutyCycle(0)
		reverse.ChangeDutyCycle(speed)
	elif ch == "f":
		stop = True
		reverse.ChangeDutyCycle(0)
		drive.ChangeDutyCycle(0)
		
	return steer, stop
			
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

print("Starting Threads\n")
try:
	q = Queue()
	a = Array('d', range(NUM_ULTRA))
	p_ultra = Process(target=queryUltra, args = (a, ))
	p_ultra.start()
	p_stream= Process(target=stream)
	p_stream.start()
	p_process = Process(target=processStream, args = (q, a))
	p_process.start()
	while True:
		while not q.empty():
			img = q.get()
		steer, stop = driveCmd(drive,reverse,speed,s,steer)
		if stop:
			 raise SystemExit('Driving Complete')
except (KeyboardInterrupt, SystemExit):
	print("\nThreads are Complete")
	'''
	results = []
	while not q.empty():
		res = q.get()
		results.append(res)
	'''
	
	if img is not None:
		cv2.imshow("environment", img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	print("Saving Class Object...")
	with open("map.pkl", "wb") as f:
		# pickle.dump(results, f, -1)
		pickle.dump(img, f, -1)

	print("Cleaning up GPIO and Exiting")
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.remove_event_detect(ECHOS[0])
	GPIO.remove_event_detect(ECHOS[1])
	GPIO.remove_event_detect(ECHOS[2])
	GPIO.remove_event_detect(ECHOS[3])
	GPIO.remove_event_detect(ECHOS[4])
	GPIO.cleanup()
	
	p_ultra.join()
	p_stream.join()
	p_process.join()
