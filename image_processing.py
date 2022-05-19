from email.message import Message
from typing import Dict
from webbrowser import get
from intersectionType import *
import socket
import time
import select

#values for decrease brightness function for each maze. This parameter must be changed as the different have different brightnesses in the camera
#Maze 1: 230
#Maze 3: 210




BUFFER_SIZE = 5
MESSAGE = ""
beginFlag = False
myDict = dict()

#Julian: 111
#Steven: 120
#the IP address must be configured to match the Heltec board depending on what WiFi network its connected to. The IP address can be found in the serial monitor of the Arduino IDE

SERVER_HOST = '192.168.2.111'
SERVER_PORT = 8000

sock = socket.socket()
sock.connect((SERVER_HOST, SERVER_PORT))
sock.settimeout(1)



#Currently follows the right wall algorithm, but can be changed to follow the left wall instead
def send_message(type_of_intersec):
    global MESSAGE
    if type_of_intersec == "Left_and_Forward":
        MESSAGE = "F" #L
        print(MESSAGE)
        sock.send(MESSAGE.encode())
        #sock.close()
    elif type_of_intersec == "Right_and_Forward":
        MESSAGE = "R" #F
        print(MESSAGE)
        sock.send(MESSAGE.encode())
        #sock.close()
    elif type_of_intersec == "T":
        MESSAGE = "R" #L
        print(MESSAGE)
        sock.send(MESSAGE.encode())
        #sock.close()
    elif type_of_intersec == "Left":
        MESSAGE = "L"
        print(MESSAGE)
        sock.send(MESSAGE.encode())
        #sock.close()
    elif type_of_intersec == "Right":
        MESSAGE = "R"
        print(MESSAGE)
        sock.send(MESSAGE.encode())
        #sock.close()
    elif type_of_intersec == "Three_Way":
        MESSAGE = "R" #L
        print(MESSAGE)
        sock.send(MESSAGE.encode())
        #sock.close()
    elif type_of_intersec == "Dead_End":
        MESSAGE = "D"
        print(MESSAGE)
        sock.send(MESSAGE.encode())
        #sock.close()
    elif type_of_intersec == "Middle_of_Maze":
        MESSAGE = "W"
        print(MESSAGE)
        sock.send(MESSAGE.encode())
        #sock.close()


def find_type_of_intersection(img):
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    #blur = cv.GaussianBlur(gray_img, (5, 5), 0)
    ret2, thresh_mask = cv.threshold(gray_img, 70, 255, cv.THRESH_BINARY)
    #cv.imshow('binary thresh feed', thresh_mask)


    top_crop = thresh_mask[0:40, :] #maybe get more rows.
    left_crop = thresh_mask[:, 110:160]
    right_crop = thresh_mask[:, 480:530]

    print(top_crop.sum())
    print(left_crop.sum())
    print(right_crop.sum())
    print(thresh_mask.sum())


    top_crop_sum = top_crop.sum()
    left_crop_sum = left_crop.sum()
    right_crop_sum = right_crop.sum()
    mask_sum = thresh_mask.sum()

    topThreshold = 200000
    LRThreshold = 300000
    winThreshold = 30000000

    if mask_sum > winThreshold: # WE ARE AT MIDDLE
        return (intersectionType.Middle_of_Maze)
    elif top_crop_sum > topThreshold and left_crop_sum > LRThreshold and right_crop_sum > LRThreshold: #Top Left and Right
        return (intersectionType.Three_Way)
    elif left_crop_sum > LRThreshold and right_crop_sum > LRThreshold: #Left and Right
        return (intersectionType.T)
    elif top_crop_sum > topThreshold and left_crop_sum > LRThreshold: #Top and Left
        return (intersectionType.Left_and_Forward)
    elif top_crop_sum > topThreshold and right_crop_sum > LRThreshold: #Top and Right
        return (intersectionType.Right_and_Forward)
    elif left_crop_sum > LRThreshold: # Left
        return (intersectionType.Left)
    elif right_crop_sum > LRThreshold: # Right
        return (intersectionType.Right)
    else:
        return (intersectionType.Dead_End) #It is a dead end






cap = cv.VideoCapture(0)
while True:

    ret, img = cap.read()  #
    #cv.imshow('pure feed', img)  # debug
    img = img[0:380, :]  # debug
    newimg = decrease_brightness(img, 230)  # debug
    cv.imshow('pure feed with brightness turned down', newimg)  # debug

    # GET THE WIFI MESSAGE

    dat = []

    ready = select.select([sock], [], [], .03)
    if ready[0]:
        dat = sock.recv(5)


    dat = bytes(dat)
    dat = dat.decode()
    if dat == "Begin":
        print("Good receive")
        beginFlag = True


    if(beginFlag == True):
        # print(type(img)) #This is a <class 'numpy.ndarray'>
        # print(img.shape) # img is a numpy matrix 480 x 640 x 3
        acc = 0
        setDict(myDict)

        while acc < 30:
            ret, img = cap.read()
            img = img[0:380, :]
            newimg = decrease_brightness(img, 230)
            #cv.imshow('pure feed with brightness turned down in loop', newimg)
            type_of_inter = find_type_of_intersection(newimg).name # For debug
            print(type_of_inter) # For debug

            myDict[type_of_inter] += 1
            acc += 1

        type_of_inter = max(myDict, key = myDict.get)
        print('FINAL -> ' + type_of_inter)


        #NOW WE NEED TO UPDATE MAZE STRUCTURE AND SEND COMMAND BACK TO ESP32

        send_message(type_of_inter)
        beginFlag = False





