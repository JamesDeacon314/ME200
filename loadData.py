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

print(len(data))
print(data[0])
print(data[0][0])
print(data[0][0][0])
print(data[0][1])
print(data[0][1][0])

pos_x = []
pos_y = []
ultra_1 = []
ultra_2 = []
ultra_3 = []
ultra_4 = []
ultra_5 = []

for i in range(len(data)):
	x = data[i][0][0]
	y = data[i][0][1]
	u1 = data[i][1][0]
	u2 = data[i][1][1]
	u3 = data[i][1][2]
	u4 = data[i][1][3]
	u5 = data[i][1][4]
	if (x is not None):
		pos_x.append(x)
	if (y is not None):
		pos_y.append(y)
	if (u1 != -1):
		ultra_1.append(data[i][1][0])
	if (u2 != -1):
		ultra_2.append(data[i][1][1])
	if (u3 != -1):
		ultra_3.append(data[i][1][2])
	if (u4 != -1):
		ultra_4.append(data[i][1][3])
	if (u5 != -1):
		ultra_5.append(data[i][1][4])
		
plt.figure(0)
plt.plot(pos_x, pos_y, label='position')
plt.legend()
plt.show()

plt.figure(1)
# plt.plot(ultra_1,label="Back")
plt.plot(ultra_2,label="Left")
# plt.plot(ultra_3,label="Right")
# plt.plot(ultra_4,label="Front Left")
# plt.plot(ultra_5,label="Front Right")
plt.legend()
plt.show()
