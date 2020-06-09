import cv2
import time
import pickle
import threading
import numpy as np
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import concurrent.futures as cf
from goprocam import GoProCamera, constants

data = pickle.load(open("map.pkl", "rb", -1))

ultra_1 = []
ultra_2 = []
ultra_3 = []
ultra_4 = []
ultra_5 = []

for i in range(len(data)):
	ultra_1.append(data[i][0])
	ultra_2.append(data[i][1])
	ultra_3.append(data[i][2])
	ultra_4.append(data[i][3])
	ultra_5.append(data[i][4])

plt.plot(ultra_1,label="Back")
plt.plot(ultra_2,label="Left")
plt.plot(ultra_3,label="Right")
plt.plot(ultra_4,label="Front Left")
plt.plot(ultra_5,label="Front Right")
plt.legend()
plt.show()
