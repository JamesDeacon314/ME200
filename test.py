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
GPIO.setup(11,GPIO.IN)

pygame.init()
screen = pygame.display.set_mode((240, 240))
pygame.display.set_caption('Pi Car')

print("w/s: acceleration")
print("a/d: steering")
print("esc: exit")

s=GPIO.PWM(11,50)
