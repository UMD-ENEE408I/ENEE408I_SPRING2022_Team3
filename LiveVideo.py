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
lr_bottom = 50
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


def draw_intersect_roi(img, lr_offset, lr_length, lr_height, lr_bottom, ud_offset, ud_length, ud_height):
    height, width = img.shape
    print(img.shape)

    color = (255, 0, 0)

    # v_midpoint = height/2
    h_midpoint = width / 2

    l_box.x1 = h_midpoint + lr_offset
    l_box.x2 = h_midpoint + lr_offset + lr_length

    l_box.y1 = lr_bottom
    l_box.y2 = lr_bottom + lr_height

    r_box.x1 = h_midpoint - lr_offset
    r_box.x2 = h_midpoint - lr_offset - lr_length

    r_box.y1 = lr_bottom
    r_box.y2 = lr_bottom + lr_height

    f_box.x1 = h_midpoint + ud_offset
    f_box.x2 = h_midpoint - ud_offset

    f_box.y1 = ud_length
    f_box.y2 = ud_length + ud_height

    # Define coord start/end for cv2.rectangle
    l_box_start = (l_box.x1, l_box.y1)
    l_box_end = (l_box.x2, l_box.y2)

    r_box_start = (r_box.x1, r_box.y1)
    r_box_end = (r_box.x2, r_box.y2)

    f_box_start = (f_box.x1, f_box.y1)
    f_box_end = (f_box.x2, f_box.y2)

    # Create boxes
    new_img = cv2.rectangle(img, l_box_start, l_box_end, color, 10)
    new_img = cv2.rectangle(new_img, r_box_start, r_box_end, color, 10)
    new_img = cv2.rectangle(new_img, f_box_start, f_box_end, color, 10)

    return new_img


def send_command(command):
    # Send command


def main():
    cap = cv2.VideoCapture(1)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        img_thresh = black_white_thresh(frame)

        forwardflag = forward_query(x1, x2, y1, y2, img_thresh)

        frame = draw_roi(img_thresh, x1, x2, y1, y2, forwardflag)

        frame = draw_intersect_roi(frame, lr_offset, lr_length, lr_height, lr_bottom, ud_offset, ud_length, ud_height)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


main()
