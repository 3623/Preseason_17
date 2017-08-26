import sys
import cv2
import numpy as np
import math

if __name__ == '__main__':
    # src = cv2.imread("ceiling1.jpg", 0)
    # cv2.imshow("Source", src)
    # cv2.waitKey(500)
    # height, width = src.shape[:2]
    # minDistance = (width+height)/2/25
    # goodFeatures = cv2.goodFeaturesToTrack(src, 15, 0.2, minDistance)
    # # print goodFeatures
    # draw = src
    # for center in goodFeatures:
    #     cv2.circle(draw, (center[0][0], center[0][1]), 4, (0, 0, 255), -1)
    # cv2.imshow("Good Features", draw)
    # cv2.waitKey(0)

    cap = cv2.VideoCapture('drop_tile.mp4')

    # params for ShiTomasi corner detection
    feature_params = dict(maxCorners=100,
                          qualityLevel=0.3,
                          minDistance=7,
                          blockSize=7)
    # Parameters for lucas kanade optical flow
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    # Create some random colors
    color = np.random.randint(0, 255, (100, 3))
    # Take first frame and find corners in it
    ret, old_frame = cap.read()
    resize = cv2.resize(old_frame, (0, 0), fx=0.5, fy=0.5)
    old_gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
    # Create a mask image for drawing purposes
    mask = np.zeros_like(resize)
    count = 0;
    while (count < 300):
        ret, frame = cap.read()
        count += 1
    while (1):
        ret, frame = cap.read()
        resize = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        frame_gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
        # calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        # Select good points
        if st[0] == 1:
            good_new = p1[st == 1]
        good_old = p0[st == 1]
        # draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (a, b), (c, d), color[i].tolist(), 2)
            frame = cv2.circle(frame, (a, b), 5, color[i].tolist(), -1)
        img = cv2.add(cv2.resize(frame, (0, 0), fx=0.5, fy=0.5), mask)
        cv2.imshow('frame', img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
