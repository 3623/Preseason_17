import sys
import cv2
import numpy as np
import math

if __name__ == '__main__':

    cap = cv2.VideoCapture('drop_tile.mp4')

    # Create some random colors
    color = np.random.randint(0, 255, (100, 3))
    # Take first frame and find corners in it
    ret, old_frame = cap.read()
    resize = cv2.resize(old_frame, (0, 0), fx=0.5, fy=0.5)
    height, width = resize.shape[:2]
    minDistanceP = (width + height) / 2 / 30
    reset = 3
    maxDif = ((height*height) + (width*width))/1000
    # params for ShiTomasi corner detection
    feature_params = dict(maxCorners=30,
                          qualityLevel=0.6,
                          minDistance =minDistanceP,
                          blockSize=7)
    # Parameters for lucas kanade optical flow
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    old_gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
    # Create a mask image for drawing purposes
    mask = np.zeros_like(resize)

    count = 0
    totalX = 0
    totalY = 0
    while (1):
        ret, frame = cap.read()
        if frame is None:
            break
        resize = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        frame_gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)

        # calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        # Select good points
        try:
            good_new = p1[st == 1]
        except:
            print "Tracking Failed"
            try:
                p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
                good_new = p1[st == 1]
            except:
                print "2nd attempt failed"
        good_old = p0[st == 1]

        sumX = 0
        sumY = 0
        terms = 0
        count += 1
        if count == 60:
            totalX = 0
            totalY = 0
        # draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            difY = d - b
            difX = c - a
            # print (difY*difY) + (difX*difX)
            if (difY*difY) + (difX*difX) < maxDif:
                sumX += difX
                sumY += difY
                terms += 1
                mask = cv2.line(mask, (a, b), (c, d), color[i].tolist(), 2)
                resize = cv2.circle(resize, (a, b), 5, color[i].tolist(), -1)
        error = False
        try:
            totalX += sumX/(terms)
            totalY += sumY/(terms)
            #print "%.2f, %.2f" % (round(totalX,2), round(totalY,2))
        except:
            print "No terms ", count
            error = True

        old_gray = frame_gray.copy()
        if count%reset == 0 or error == True:
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
        else:
            p0 = good_new.reshape(-1, 1, 2)

        img = cv2.add(resize, mask)
        cv2.imshow("Optical Flow", img)
        # Now update the previous frame and previous points
        if cv2.waitKey(2) & 0xFF == ord('q'):
            break

    print "Max Length: %.2f,  Point Reset: %.f" %(maxDif, reset)
    print "Displacement X: %.2f,  Displacement Y: %.2f" % (round(totalX, 2), round(totalY, 2))
    cv2.imshow("Optical Flow", img)
    cv2.waitKey(0)
    cap.release()
    cv2.destroyAllWindows()
