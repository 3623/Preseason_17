import numpy as np
import cv2
from matplotlib import pyplot as plt

#img = cv2.imread('ceiling1.jpg',0)

cap = cv2.VideoCapture('drop_tile.mp4')

while (cap.isOpened()):
    ret, frame = cap.read()

    resize = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    img = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    # Initiate STAR detector
    orb = cv2.ORB_create()

    # find the keypoints with ORB
    kp = orb.detect(img,None)

    # compute the descriptors with ORB
    kp, des = orb.compute(img, kp)

    # draw only keypoints location,not size and orientation
    img2 = cv2.drawKeypoints(img,kp,img,color=(0,255,0), flags=0)
    cv2.imshow("Image", img2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()