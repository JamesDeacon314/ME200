from goprocam import GoProCamera, constants
import time

'''
goproCamera = GoProCamera.GoPro()

# Took almost 4 seconds per pic
for i in range(3):
	start = time.time()
	goproCamera.take_photo(timer=0)
	goproCamera.downloadLastMedia()
	print(time.time() - start)
	start = time.time()
'''

goproCamera = GoProCamera.GoPro()
goproCamera.stream("udp://127.0.0.1:10000")


