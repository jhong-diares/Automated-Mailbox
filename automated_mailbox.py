import RPi.GPIO as GPIO
from datetime import datetime
import cv2
from PIL import Image
import serial
import time, sys
from pytesseract import *
from time import sleep
from picamera.array import PiRGBArray
import io
from picamera import PiCamera
import numpy as np
from matplotlib import pyplot as plt
SERIAL_PORT="/dev/serial0"
ser=serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 5)
IMAGE_FILE = 'cam_footage.jpg'

LOOP_COUNT = 0

#SERVO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3, GPIO.OUT)
pwm=GPIO.PWM(3, 50)
pwm.start(0)
#IR
sensor = 37
LED = 35

GPIO.setmode(GPIO.BOARD)
GPIO.setup(sensor,GPIO.IN)
GPIO.setup(LED,GPIO.OUT)
GPIO.output(LED,False)

def SetAngle(angle):
	duty = angle / 18 + 2
	GPIO.output(3, True)
	pwm.ChangeDutyCycle(duty)
	sleep(3)
	GPIO.output(3, False)
	pwm.ChangeDutyCycle(0)
def GSM_FUNCTION(msg):
    wr ="AT+CMGF=1\r"
    ser.write(wr.encode())
    time.sleep(3)
    #OWNER SMS NUMBER
    num = 'AT+CMGS="09514419800"\r'
    ser.write(num.encode())
    time.sleep(3)
    tr ="msg+chr(26)"
    ser.write(str.encode(msg+chr(26)))
def IR_ACTIVATE():
    while True:
      if GPIO.input(sensor):
          GPIO.output(LED,True)
          print ("IR READY...")
          while GPIO.input(sensor):
              print ("...")
              time.sleep(0.1)
          else:
              print("DETECTED")
              time.sleep(4)
              SetAngle(80)
              break
      else:
          GPIO.output(LED,False)

# ==> PROCESS START
while True:
    LOOP_COUNT = LOOP_COUNT +1
    print (LOOP_COUNT)
    # save image from picam
    img = cv2.VideoCapture(0).read()[1]
    cv2.imwrite(IMAGE_FILE, img)
    # load image
    img = Image.open(IMAGE_FILE)
    
    # detect words in image
    words = image_to_string(img).strip()
    # words detected
    result_name = 'ARTURO INDIONGCO'
    result = '' in words or '' in words or  result_name in words or '' in words
        
    result1_name = ''
    result1 =  result1_name in words or '' in words or '' in words
    print(result1)
    
    result2 = '' in words or '' in words
    print(result2)
    
    result3 = '' in words 
    print(result3)
    
    result4_name = '' 
    result4 = '' in words or result4_name in words or '' in words
    print(words)
    
    result5 = '' in words
    print(result5)
    
    result6 = '' in words
    print(result6)
    
    result7 = '' in words
    print(result7)
    
    result8_name = ''
    result8_name1= ''
    result8_name2= ''
    
    result8 = '' in words or result8_name in words or result8_name1 in words or result8_name2 in words
    print(result8)
    
    
    print(result)
    
    if result == True:
        print('Water Bill has arrived')
        #SMS FUNCTION
        msg="Water Bill has arrived1"
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    elif result1 == True:
        print('Electricity Bill has arrived')
        msg="Electricity Bill has arrived"
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    elif result2 == True:
        print('Trial Court Mail has arrived')
        msg="Trial Court Mail has arrived"
        #SMS FUNCTION
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    elif result3 == True:
        print('Bank of Commerce')
        msg="Bank of Commerce has arrived"
        #SMS FUNCTION
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    elif result4 == True:
        print('PLDT Home has arrived')
        msg="PLDT Home has arrived"
        #SMS FUNCTION
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    elif result5 == True:
        print('SOCIAL SECURITY SYSTEM has arrived')
        msg="SOCIAL SECURITY SYSTEM has arrived"
        #SMS FUNCTION
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    elif result6 == True:
        print('Sun Life has arrived')
        msg="Sun Life has arrived"
        #SMS FUNCTION
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    elif result7 == True:
        print('ABENSON has arrived')
        msg="ABENSON has arrived"
        #SMS FUNCTION
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    elif result8 == True:
        print('BDO has arrived')
        msg="BDO has arrived"
        #SMS FUNCTION
        GSM_FUNCTION(msg)
        #SERVO FUNCTION
        SetAngle(0)
        IR_ACTIVATE()
    else:
        print('NOT DETECTED')
