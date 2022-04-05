from enum import Enum
import cv2 as cv
import numpy as np


class intersectionType(Enum):
    Left_and_Forward = 1
    Right_and_Forward = 2
    T = 3
    Left = 4
    Right = 5
    Three_Way = 6
    Dead_End = 7
    Middle_of_Maze = 8


def setDict(dict):
    dict.clear
    dict[intersectionType.Left_and_Forward.name] = 0
    dict[intersectionType.Right_and_Forward.name] = 0
    dict[intersectionType.T.name] = 0
    dict[intersectionType.Left.name] = 0
    dict[intersectionType.Right.name] = 0
    dict[intersectionType.Three_Way.name] = 0
    dict[intersectionType.Dead_End.name] = 0
    dict[intersectionType.Middle_of_Maze.name] = 0
    

def decrease_brightness(img, value):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)

    lim = value
    v[v < lim] = 0
    v[v >= lim] -= value
    #v = v - lim
    v = (v.astype(np.float32) * 255/(255 - lim)).astype(np.uint8)    
    #v =  v * 2

    final_hsv = cv.merge((h, s, v))
    img = cv.cvtColor(final_hsv, cv.COLOR_HSV2BGR)
    return img
