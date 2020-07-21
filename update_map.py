import numpy as np
import cv2
import time
import random
import math
import pickle

def initializeTemplate():
	dist = 100
	py = 240
	px = 320
	dy = 0
	dx = 1
	sine = 0.130526
	cosine = 0.991445
	template = np.zeros((480,640,3))
	values = np.zeros((480,640))
	for y in range(240 - (int(sine*dist) + 1), 240 + (int(sine*dist) + 1) + 1):
		for x in range(320, 320 + dist + 20 + 1):
			distance = np.sqrt((y - py) ** 2 + (x - px) ** 2)
			norm = math.sqrt((y - py) ** 2 + (x - px) ** 2) * math.sqrt(dy*dy+dx*dx)
			if distance != 0:
				acos = max(-1, min(1, (((y - py) * dy) + ((x - px) * dx)) / norm))
				if acos > cosine:
					if (distance > dist + 20):
						continue
					elif (distance < dist - 40):
						values[y,x] = (-1 + values[y,x] ** 2 * np.sign(values[y,x])) / \
							(1 + abs(values[y,x]))
					elif (distance < dist - 20):
						dif = (dist - 20) - distance
						values[y,x] = (-1 * (dif / 20) ** 2 + values[y,x] ** 2 * \
							np.sign(values[y,x])) / ((dif / 20) + abs(values[y,x]))
					else:
						dif = abs(distance - dist)
						values[y,x] = (((20 - dif) / 20) ** 2 + values[y,x] ** 2 * \
							np.sign(values[y,x])) / (((20 - dif) / 20) + abs(values[y,x])) * \
							((acos - cosine) / (1 - cosine) / 2 + 1 / 2)
					
					template[y,x,0] = max(0, -1 * values[y, x])
					template[y,x,2] = max(0, values[y,x])
					
	return template, values
	
def rotate_image(image, angle, center):
	rot_mat = cv2.getRotationMatrix2D(center, angle, 1.0)
	result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
	return result
	
def createTemplate(template, values, scale, angle):
	template = cv2.resize(template, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
	values = cv2.resize(values, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
	cy = int(template.shape[0] / 2)
	cx = int(template.shape[1] / 2)
	cy2 = template.shape[0] - cy
	cx2 = template.shape[1] - cx
	
	if scale > 1:
		template = template[cy - 240:cy + 240, cx - 320:cx + 320]
		values = values[cy - 240:cy + 240, cx - 320:cx + 320]
	else:
		new_template = np.zeros((480,640,3))
		new_template[240 - cy:240 + cy2, 320 - cx:320 + cx2] = template
		template = new_template
		
		new_values = np.zeros((480,640))
		new_values[240 - cy:240 + cy2, 320 - cx:320 + cx2] = values
		values = new_values
		
	template = rotate_image(template, angle, (320, 240))
	values = rotate_image(values, angle, (320, 240))
	
	return template, values
	
def addMeasurement(img, pos, dist, direction, template, values):
	sine = 0.130526
	cosine = 0.991445
	py, px = pos
	py = int(py)
	px = int(px)
	dy, dx = direction
	ty = py - 240
	tx = px - 320
	angle = math.degrees(math.atan2(dy, dx))
	test = time.time()
	template, values = createTemplate(template, values, dist / 100, angle)
	M = np.float32([[1,0,tx],[0,1,ty]])
	template = cv2.warpAffine(template,M,(640,480))
	values = cv2.warpAffine(values,M,(640,480))
	
	absMap = abs(img)
	squaredMap = np.multiply(img, absMap)
	
	absDetection = abs(values)
	squaredDetection = np.multiply(values, absDetection)
	
	norm = absMap + absDetection
	
	numerator = squaredMap + squaredDetection
	
	img = np.divide(numerator, norm, out=np.zeros_like(numerator), where=norm!=0)
	
	return img
	

data = pickle.load(open("map.pkl", "rb", -1))

print(data.shape)
l = []
r = []
b = []
f = []
for x in range(data.shape[1]):
	top = -1
	bottom = data.shape[0]
	for y in range(data.shape[0]):
		if top == -1 and y < 240:
			if data[y,x,1] != 0:
				top = y
		else:
			if data[y,x,1] != 0:
				bottom = y
	if top != -1:
		l.append(240 - top)
	if bottom != data.shape[0]:
		r.append(bottom - 240)
		
# print(l)
# print(r)
for i in range(len(l)):
	b.append(30)
	f.append(225)


'''
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
'''
		
y = 250
x = 250
img = np.zeros((480,640))
display = np.zeros((480,640,3))
template, values = initializeTemplate()
for i in range(len(l)):
	start = time.time()
	# img = addMeasurement(img, (y, x), l[i], (1, 0), template, values)
	# img = addMeasurement(img, (y, x), r[i], (-1, 0), template, values)
	# img = addMeasurement(img, (y, x), b[i], (0, -1), template, values)
	# img = addMeasurement(img, (y, x), f[i], (0, 1), template, values)
	img = addMeasurement(img, (y, x), l[i], (0.707, -0.707), template, values)
	img = addMeasurement(img, (y, x), r[i], (-0.707, 0.707), template, values)
	img = addMeasurement(img, (y, x), b[i], (-0.707, -0.707), template, values)
	img = addMeasurement(img, (y, x), f[i], (0.707, 0.707), template, values)
	print(time.time() - start)
	# x += 1
	x += .707
	y += -.707
	

for y in range(480):
	for x in range(640):
		display[y,x,0] = max(0, -1 * img[y, x])
		display[y,x,2] = max(0, img[y,x])
cv2.imshow("detection", display)
cv2.waitKey(0)
cv2.destroyAllWindows()
