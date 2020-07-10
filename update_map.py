import numpy as np
import cv2
import time
import random
import math

def rotate_image(image, angle, center):
	rot_mat = cv2.getRotationMatrix2D(center, angle, 1.0)
	result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
	return result

def updateImage2(img, values, pos, dist, direction, orientation):
	sine = 0.130526
	cosine = 0.991445
	py, px = pos
	py = int(py)
	px = int(px)
	dy, dx = direction
	oy, ox = orientation
	angle = math.degrees(math.atan2(oy, ox))
	img = rotate_image(img, -angle, (px, py))
	values= rotate_image(values, -angle, (px, py))
	'''
	cv2.imshow('image', img)
	cv2.imshow('rotate', image)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	'''
	
	if not dy:
		if dx == -1:
			sx = max(0, px - dist)
			ex = px
		elif dx == 1:
			sx = px
			ex = min(img.shape[1], px + dist)
		sy = max(0, py - 20 - sine * dist)
		ey = min(img.shape[0], py + 21 + sine * dist)
		
	elif not dx:
		if dy == -1:
			sy = max(0, py - dist)
			ey = py
		elif dy == 1:
			sy = py
			ey = min(img.shape[0], py + dist)
		sx = max(0, px - 20 - sine * dist)
		ex = min(img.shape[1], px + 21 + sine * dist)
		
	sx = int(sx)
	sy = int(sy)
	ex = int(ex)
	ey = int(ey)
		
	for y in range(sy, ey):
		for x in range(sx, ex):
			distance = np.sqrt((y - py) ** 2 + (x - px) ** 2)
			norm = math.sqrt((y - py) ** 2 + (x - px) ** 2) * math.sqrt(dy*dy+dx*dx)
			if norm != 0:
				acos = max(-1, min(1, (((y - py) * dy) + ((x - px) * dx)) / norm))
				if acos > cosine:
					if (distance > dist + 20):
						continue
					elif (distance < dist - 40):
						values[y, x] = (-1 + values[y, x] ** 2 * np.sign(values[y, x])) / \
								(1 + abs(values[y, x]))
				
					elif (distance < dist - 20):
						dif = (dist - 20) - distance
						values[y, x] = (-1 * (dif / 20) ** 2 + values[y, x] ** 2 * \
											np.sign(values[y, x])) / ((dif / 20) + abs(values[y, x]))
				
					else:
						dif = abs(distance - dist)
						values[y, x] = (((20 - dif) / 20) ** 2 + \
									values[y, x] ** 2 * \
									np.sign(values[y, x])) / \
									(((20 - dif) / 20) + \
									abs(values[y, x])) * ((acos - cosine) / (1 - cosine) / 3 + 2/3)
					img[y, x, 0] = max(0, -1 * values[y, x])
					img[y, x, 2] = max(0, values[y, x])
					
	# img[py, px, 0] = 0
	# img[py, px, 1] = 255
	# img[py, px, 2] = 0

	return (img, values)

def updateImage(img, values, pos, dist, direction):
	sine = 0.130526
	cosine = 0.991445
	py, px = pos
	py = int(py)
	px = int(px)
	dy, dx = direction
	'''
	U dot V = 0
	u1*v1 + u2*v2 = 0
	v1 = -u2/u1 * v2
	v2 = -u1/u2 * v1
	'''
	ox = 0
	oy = 0
	if (dy == 0):
		oy = 1
	elif (dx == 0):
		ox = 1
	else:
		ox = 1
		oy = -dx / dy
		ox = 1 / (np.sqrt(oy * oy + 1))
		oy = oy / (np.sqrt(oy * oy + 1))
	
	sx = int(max(0, min(px - (abs(ox) * sine * dist * 1.2), px + dx * dist * 1,2 - (abs(ox) * sine * dist * 1.2))))
	ex = int(min(img.shape[1] - 1, max(px + (abs(ox) * sine * dist * 1.2), px + dx * dist * 1.2+ (abs(ox) * sine * dist * 1.2))))
	sy = int(max(0, min(py - (abs(oy) * sine * dist * 1.2), py + dy * dist * 1.2 - (abs(oy) * sine * dist * 1.2))))
	ey = int(min(img.shape[0] - 1, max(py + (abs(oy) * sine * dist * 1.2), py + dy * dist * 1.2 + (abs(oy) * sine * dist * 1.2))))
	
	for y in range(sy, ey):
		for x in range(sx, ex):
			distance = np.sqrt((y - py) ** 2 + (x - px) ** 2)
			norm = math.sqrt((y - py) ** 2 + (x - px) ** 2) * math.sqrt(dy*dy+dx*dx)
			if norm != 0:
				acos = max(-1, min(1, (((y - py) * dy) + ((x - px) * dx)) / norm))
				if acos > cosine:
					if (distance > dist + 20):
						continue
					elif (distance < dist - 40):
						values[y, x] = (-1 + values[y, x] ** 2 * np.sign(values[y, x])) / \
								(1 + abs(values[y, x]))
				
					elif (distance < dist - 20):
						dif = (dist - 20) - distance
						values[y, x] = (-1 * (dif / 20) ** 2 + values[y, x] ** 2 * \
											np.sign(values[y, x])) / ((dif / 20) + abs(values[y, x]))
				
					else:
						dif = abs(distance - dist)
						values[y, x] = (((20 - dif) / 20) ** 2 + \
									values[y, x] ** 2 * \
									np.sign(values[y, x])) / \
									(((20 - dif) / 20) + \
									abs(values[y, x])) * ((acos - cosine) / (1 - cosine) / 3 + 2/3)
					img[y, x, 0] = max(0, -1 * values[y, x])
					img[y, x, 2] = max(0, values[y, x])
		
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
angle = (0.707, 0.707)
angle2 = angle
for i in range(len(l)):
	start = time.time()
	# img, values = updateImage(img, values, (y, x), l[i], (-0.707, -0.707))
	# img, values = updateImage(img, values, (y, x), r[i], (.707, .707))
	# img, values = updateImage(img, values, (y, x), b[i], (.707, -.707))
	# img, values = updateImage(img, values, (y, x), f[i], (-.707, .707))
	img, values = updateImage2(img, values, (y, x), l[i], (-1, 0), angle)
	angle = (0, 0)
	img, values = updateImage2(img, values, (y, x), r[i], (1, 0), angle)
	img, values = updateImage2(img, values, (y, x), b[i], (0, -1), angle)
	img, values = updateImage2(img, values, (y, x), f[i], (0, 1), angle)
	print(time.time() - start)
	x += .707 + (random.random() - 0.5) * 4
	y += -.707 + (random.random() - 0.5) * 4
	oy, ox = angle2
angle = math.degrees(math.atan2(oy, ox))
img = rotate_image(img, angle, (x, y))
values = rotate_image(values, angle, (x, y))
cv2.imshow("detection", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
