import cv2
from matplotlib import pyplot as plt
import numpy as np

image = cv2.imread("/Users/julian/Desktop/img1.jpg")

lower_threshold = np.array([0, 0, 215])
upper_threshold = np.array([180, 15, 255])

def black_white(img):
    #convert to BW
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, lower_threshold, upper_threshold)

def canny_edges(img):
    #Use canny to detect edges
    img_edge = cv2.Canny(img,100,200)

image_bw = black_white(image)
image_edge = canny_edges(image_bw)

plt.imshow(image_bw)
plt.show()
