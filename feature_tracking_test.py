import sys
import cv2
import rospy
import numpy as np
import math

if __name__ == '__main__':
    src = cv2.imread("ceiling1.jpg", 0)
    cv2.imshow("Source", src)
    cv2.waitKey(500)
    height, width = src.shape[:2]
    minDistance = (width+height)/2/25
    goodFeatures = cv2.goodFeaturesToTrack(src, 15, 0.2, minDistance)
    print goodFeatures
    draw = src
    for center in goodFeatures:
        cv2.circle(draw, (center[0][0], center[0][1]), 4, (0, 0, 255), -1)
    cv2.imshow("Good Features", draw)
    cv2.waitKey(0)