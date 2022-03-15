import numpy as np
import cv2

# Define Region of Interest for line detection
x1 = 250
x2 = 350
y1 = 300
y2 = 450

# Define parameters for intersection region of interest
lr_offset = 10
lr_length = 100
lr_height = 30
ud_offset = 20
ud_length = 10
ud_height = 50


# Define upper and lower threshold for detecting white on black
lower_threshold = np.array([0, 0, 215])
upper_threshold = np.array([180, 15, 255])


# Functions
def black_white_thresh(img):
    # convert to BW
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, lower_threshold, upper_threshold)


def forward_query(x1, x2, y1, y2, img):
    roi = img[y1:y2, x1:x2]

    length = x2 - x1
    height = y2 - y1
    totalpixels = length * height

    whitecount = np.sum(roi == 255)

    if whitecount / totalpixels > .3:
        return 1
    else:
        return 0


def draw_roi(img, x1, x2, y1, y2, f_flag):
    start = (x1, y1)
    end = (x2, y2)
    color = (255, 0, 0)

    new_img = cv2.rectangle(img, start, end, color, 10)
    if f_flag == 1:
        new_img = cv2.putText(new_img, 'Move Forward!', start, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    else:
        new_img = cv2.putText(new_img, 'Stop!', end, cv2.FONT_HERSHEY_SIMPLEX, 5, color, 5, cv2.LINE_AA)

    return new_img


def draw_intersect_roi(img, lr_offset, lr_length, lr_height, ud_offset, ud_length, ud_height):
    height, width = img.size
    print(img.size)


def main():
    cap = cv2.VideoCapture(1)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        img_thresh = black_white_thresh(frame)

        forwardflag = forward_query(x1, x2, y1, y2, img_thresh)

        frame = draw_roi(img_thresh, x1, x2, y1, y2, forwardflag)

        frame = draw_intersect_roi(frame, lr_offset, lr_length, lr_height, ud_offset, ud_length, ud_height)


        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


main()
