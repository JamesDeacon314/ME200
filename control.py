import RPi.GPIO as GPIO

import time
import os, sys
import pygame
from pygame.locals import *
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import numpy as np
from PIL import Image

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

ena = 16
in1 = 18
in2 = 22
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.HIGH)
drive = GPIO.PWM(ena, 1000)
drive.start(0)
speed = 80

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
                    steer=max(50, steer + 5)
                    s.ChangeDutyCycle(steer)
                    time.sleep(0.1)
                    s.ChangeDutyCycle(0)
                    time.sleep(0.1)
                    print(steer)
            elif event.key == K_ESCAPE:
                    stop = True
            elif event.key == K_w:
                speed = min(100, speed + 5)
                print(speed)
                drive.ChangeDutyCycle(speed)
            elif event.key == K_s:
                speed = max(0, speed - 5)
                drive.ChangeDutyCycle(speed)
                print(speed)
         
s.ChangeDutyCycle(0)
s.stop()
GPIO.setup(13,GPIO.IN)
drive.stop()
GPIO.cleanup()
