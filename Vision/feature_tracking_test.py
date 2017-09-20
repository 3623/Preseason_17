import sys
import cv2
import numpy as np
import math

if __name__ == '__main__':
    src = cv2.imread("ceiling1.jpg", 0)
    cv2.imshow("Source", src)
    cv2.waitKey(500)
    height, width = src.shape[:2]
    minDistance = (width+height)/2/25
    goodFeatures = cv2.goodFeaturesToTrack(src, 15, 0.2, minDistance)
    # print goodFeatures
    draw = src
    for center in goodFeatures:
        cv2.circle(draw, (center[0][0], center[0][1]), 4, (0, 0, 255), -1)
    cv2.imshow("Good Features", draw)
    cv2.waitKey(0)


    cap = cv2.VideoCapture('drop_tile.mp4')

    while (cap.isOpened()):
        ret, frame = cap.read()

        resize = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
        gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
        draw = resize
        height, width = resize.shape[:2]
        minDistance = (width + height) / 2 / 5
        goodFeatures = cv2.goodFeaturesToTrack(gray, 15, 0.2, minDistance)
        for center in goodFeatures:
            cv2.circle(draw, (center[0][0], center[0][1]), 4, (0, 0, 255), -1)
        cv2.imshow('frame', draw)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
