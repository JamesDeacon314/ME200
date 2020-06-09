import cv2
import time
import numpy as np
from goprocam import GoProCamera, constants

gpCam = GoProCamera.GoPro()
cap = cv2.VideoCapture("udp://127.0.0.1:10000")
while True:
	start = time.time()
	ret, frame = cap.read()
	if ret == True:
		cv2.imshow("GoPto OpenCV", frame)
		cv2.waitKey(1)
	else:
		break
	print(time.time() - start)
	
cap.release()
cv2.destroyAllWindows()
