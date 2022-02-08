import cv2
from matplotlib import pyplot as plt
import numpy as np

vid = cv2.VideoCapture(0)
ret, frame = vid.read()

lower_threshold = np.array([0, 0, 215])
upper_threshold = np.array([180, 15, 255])

def black_white(img):
    #convert to BW
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, lower_threshold, upper_threshold)

def mask(img):
    height, width = img.shape
    triangle = np.array([
                       [(0, height), (340, 255), (width, height)]#CHANGE LATER
                       ])
    mask = np.zeros_like(img)
    mask = cv2.fillPoly(mask, triangle, 255)
    mask = cv2.bitwise_and(img, mask)
    return mask

def canny(img):
    edges = cv2.Canny(img,50,150)
    return edges

img_bw = black_white(frame)
masked_img = mask(img_bw)

img_edges = canny(masked_img)

edges_fig = plt.figure(1)
plt.imshow(img_edges)

lines_fig = plt.figure(2)
plt.imshow(masked_img)
plt.show()
