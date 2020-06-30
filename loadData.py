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

print(data)

cv2.imshow('graph', data)
cv2.waitKey(0)
cv2.destroyAllWindows()

# pos_x = []
# pos_y = []
ultra_1 = []
ultra_2 = []
ultra_3 = []
ultra_4 = []
ultra_5 = []

for i in range(len(data)):
	# x = data[i][0][0]
	# y = data[i][0][1]
	u1 = data[i][0]
	u2 = data[i][1]
	u3 = data[i][2]
	u4 = data[i][3]
	u5 = data[i][4]
	'''
	if (x is not None):
		pos_x.append(x)
	if (y is not None):
		pos_y.append(y)
	'''
	if (u1 != -1):
		ultra_1.append(u1)
	if (u2 != -1):
		ultra_2.append(u2)
	if (u3 != -1):
		ultra_3.append(u3)
	if (u4 != -1):
		ultra_4.append(u4)
	if (u5 != -1):
		ultra_5.append(u5)

'''		
plt.figure(0)
plt.plot(pos_x, pos_y, label='position')
plt.legend()
plt.show()
'''
x = 400
y = 400
grid = np.zeros((800,800,3))
cv2.circle(grid, (int(x), int(y)), 5, (0, 255, 0), thickness=-1)
for u in ultra_1:
	cv2.circle(grid, (int(x - u), int(y)), 5, (255, 0, 0), thickness=-1)
for u in ultra_2:
	cv2.circle(grid, (int(x), int(y - u)), 5, (255, 0, 0), thickness=-1)
for u in ultra_3:
	cv2.circle(grid, (int(x), int(y + u)), 5, (255, 0, 0), thickness=-1)
for u in ultra_4:
	cv2.circle(grid, (int(x + u), int(y)), 5, (255, 0, 0), thickness=-1)
for u in ultra_5:
	cv2.circle(grid, (int(x + u), int(y)), 5, (255, 0, 0), thickness=-1)
cv2.imshow("environment", grid)
cv2.waitKey(0)
cv2.destroyAllWindows()

plt.figure(1)
plt.plot(ultra_1,label="Back")
plt.legend()
plt.show()

plt.figure(2)
plt.plot(ultra_2,label="Left")
plt.legend()
plt.show()

plt.figure(3)
plt.plot(ultra_3,label="Right")
plt.legend()
plt.show()

plt.figure(4)
plt.plot(ultra_4,label="Front Left")
plt.legend()
plt.show()

plt.figure(5)
plt.plot(ultra_5,label="Front Right")
plt.legend()
plt.show()
