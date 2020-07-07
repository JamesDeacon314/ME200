import numpy as np
import cv2
import time
import random

def updateImage(img, values, pos, dist, direction):
	sine = 0.130526
	py, px = pos
	py = int(py)
	px = int(px)
	start = time.time()
	dy, dx = direction
	if (dx == 0):
		sx = int(max(0, px - sine * dist))
		ex = int(min(img.shape[1] - 1, px + sine * dist + 1))
	elif (dx > 0):
		sx = int(max(0, px))
		ex = int(min(img.shape[1] - 1, px + 21 + dx * dist))
	else:
		sx = int(max(0, px - 20 + dx * dist))
		ex = int(min(img.shape[1] - 1, px))
		
	if (dy == 0):
		sy = int(max(0, py - sine * dist))
		ey = int(min(img.shape[0] - 1, py + sine * dist + 1))
	elif (dy > 0):
		sy = int(max(0, py))
		ey = int(min(img.shape[0] - 1, py + 21 + dy * dist))
	else:
		sy = int(max(0, py - 20 + dy * dist))
		ey = int(min(img.shape[0] - 1, py))
		
	for y in range(sy, ey):
		for x in range(sx, ex):
			distance = np.sqrt((y - py) ** 2 + (x - px) ** 2)
			if (distance > dist + 20):
				continue
			elif (distance < dist - 40):
				if (dx == 0) and (abs(x - px) / distance < sine):
					values[y, x] = (-1 + values[y, x] ** 2 * np.sign(values[y, x])) / \
									(1 + abs(values[y, x]))
					img[y, x, 0] = max(0, -1 * values[y, x])
					img[y, x, 2] = max(0, values[y, x])
				elif (dy == 0) and (abs(y - py) / distance < sine):
					values[y, x] = (-1 + values[y, x] ** 2 * np.sign(values[y, x])) / \
									(1 + abs(values[y, x]))
					img[y, x, 0] = max(0, -1 * values[y, x])
					img[y, x, 2] = max(0, values[y, x])
					
			elif (distance < dist - 20):
				dif = (dist - 20) - distance
				if (dx == 0) and (abs(x - px) / distance < sine):
					values[y, x] = (-1 * (dif / 20) ** 2 + values[y, x] ** 2 * \
									np.sign(values[y, x])) / ((dif / 20) + abs(values[y, x]))
					img[y, x, 0] = max(0, -1 * values[y, x])
					img[y, x, 2] = max(0, values[y, x])
				elif (dy == 0) and (abs(y - py) / distance < sine):
					values[y, x] = (-1 * (dif / 20) ** 2 + values[y, x] ** 2 * \
									np.sign(values[y, x])) / ((dif / 20) + abs(values[y, x]))
					img[y, x, 0] = max(0, -1 * values[y, x])
					img[y, x, 2] = max(0, values[y, x])
					
			else:
				dif = abs(distance - dist)
				if (dx == 0) and (abs(x - px) / distance < sine):
					values[y, x] = (((20 - dif) / 20 * (sine - abs(x - px) / distance / 3) / sine) ** 2 + \
									values[y, x] ** 2 * \
									np.sign(values[y, x])) / \
									(((20 - dif) / 20 * (sine - abs(x - px) / distance / 3) / sine) + \
									abs(values[y, x]))
					img[y, x, 0] = max(0, -1 * values[y, x])
					img[y, x, 2] = max(0, values[y, x])
				elif (dy == 0) and (abs(y - py) / distance < sine):
					values[y, x] = (((20 - dif) / 20 * (sine - abs(y - py) / distance / 3) / sine) ** 2 + \
									values[y, x] ** 2 * \
									np.sign(values[y, x])) / \
									(((20 - dif) / 20 * (sine - abs(y - py) / distance / 3) / sine) + \
									abs(values[y, x]))
					img[y, x, 0] = max(0, -1 * values[y, x])
					img[y, x, 2] = max(0, values[y, x])
	print(time.time() - start)
	img[py, px, 0] = 0
	img[py, px, 1] = 255
	img[py, px, 2] = 0
	
	return(img, values)


# Everything is y, x
img = np.zeros((480,640,3))
values = np.zeros((480, 640))
li = [150, 150, 150, 150, 150, 120, 120, 120, 80, 40]
ri = [80, 80, 80, 90, 100, 110, 120, 140, 140, 140]
bi = [40, 50, 60, 70, 80, 90, 100, 110, 120, 130]
fi = [170, 160, 150, 140, 130, 120, 110, 100, 90, 80]
l = []
r = []
b = []
f = []
for i in range(len(li)):
	for j in range(10):
		if random.random() < 0.9:
			l.append(li[i] + random.random() * 5)
		else:
			l.append(li[i] + random.random() * 20)
		
		if random.random() < 0.9:
			r.append(ri[i] + random.random() * 5)
		else:
			r.append(ri[i] + random.random() * 20)
			
		if random.random() < 0.9:
			b.append(bi[i] + random.random() * 5)
		else:
			b.append(bi[i] + random.random() * 20)
			
		if random.random() < 0.9:
			f.append(fi[i] + random.random() * 5)
		else:
			f.append(fi[i] + random.random() * 20)
		
y = 200
x = 300
for i in range(len(l)):
	img, values = updateImage(img, values, (y, x), l[i], (-1, 0))
	img, values = updateImage(img, values, (y, x), r[i], (1, 0))
	img, values = updateImage(img, values, (y, x), b[i], (0, -1))
	img, values = updateImage(img, values, (y, x), f[i], (0, 1))
	x += 1 + (random.random() - 0.5) * 4
	y + (random.random() - 0.5) * 4
cv2.imshow("detection", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
