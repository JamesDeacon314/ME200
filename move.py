import RPi.GPIO as GPIO
import time
import pygame
from pygame.locals import *

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(13,GPIO.OUT)

pygame.init()
screen = pygame.display.set_mode((240, 240))
pygame.display.set_caption('Pi Car')

print("w/s: acceleration")
print("a/d: steering")
print("esc: exit")

s=GPIO.PWM(13,500)
s.start(0)
s.start(70)
time.sleep(0.1)

steer=70
s.ChangeDutyCycle(steer)
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

print("Starting")
while (True):
    if stop == True:
            break
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == K_a:
                    steer=min(90, steer + 5)
                    s.ChangeDutyCycle(steer)
                    time.sleep(0.1)
                    s.ChangeDutyCycle(0)
                    time.sleep(0.1)
                    print(steer)
            elif event.key == K_d:
                    steer=max(50, steer - 5)
                    s.ChangeDutyCycle(steer)
                    time.sleep(0.1)
                    s.ChangeDutyCycle(0)
                    time.sleep(0.1)
                    print(steer)
            elif event.key == K_ESCAPE:
                    stop = True
            elif event.key == K_w:
                print("forward")
                reverse.ChangeDutyCycle(0)
                drive.ChangeDutyCycle(speed)
            elif event.key == K_s:
                print("backward")
                drive.ChangeDutyCycle(0)
                reverse.ChangeDutyCycle(speed)
            elif event.key == K_f:
                print("stop")
                reverse.ChangeDutyCycle(0)
                drive.ChangeDutyCycle(0)
         
print("DONE")
s.ChangeDutyCycle(0)
s.stop()
GPIO.setup(13,GPIO.IN)
drive.stop()
GPIO.cleanup()
