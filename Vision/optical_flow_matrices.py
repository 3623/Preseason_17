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
    reset = 1
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

    # Function applied to points to make a corrected 3 column matrix
    def transCoord(a):
        return([a[0]-(0*width/2), (0*height/2)-a[1], 1])

    count = 0
    totalR = 0
    totalX = 0
    totalY = 0
    while (1):
        #Counter- resets at a certain count
        count += 1
        if count == 60:
            None
        else:
            None

        # Read Video
        ret, frame = cap.read()
        if frame is None:
            break
        resize = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        frame_gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)

        # calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        # Select good points
        try:### IS THIS USELESS???
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
        ### TODO Need a way to maintain points if tracking fails

        x_prime = np.matrix(np.apply_along_axis(transCoord, 1, good_new))
        x = np.matrix(np.apply_along_axis(transCoord, 1, good_old))
        # print x_prime
        # print x
        x_inv = np.linalg.pinv(x)
        t = x_inv * x_prime
        transpose = np.asarray(np.transpose(t))
#        print "========================="
#        print transpose.round(4)
        # print x_inv.shape, x_prime.shape
        # print t.round(4)
        # print "========================="
        # check = x * t
        # print check.round(3)

        opfl_trans_error = False
        try:
            if np.abs(transpose[2][2]-1) > 0.05:
                print "Optical Flow Error, corner value of matrix is: %.5f. Count: %.f" \
                      %(transpose[2][2].round(5), count)
        except:
            print "Transformation Matrix Error, Count: ", count
            opfl_trans_error = True

        # draw the tracks
        terms = 0
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (a, b), (c, d), color[i].tolist(), 2)
            resize = cv2.circle(resize, (a, b), 5, color[i].tolist(), -1)

        no_terms_error = False

        old_gray = frame_gray.copy()
        if count%reset == 0 or error == True:
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
        else:
            p0 = good_new.reshape(-1, 1, 2)

        img = cv2.add(resize, mask)
        cv2.imshow("Optical Flow", img)
        # Now update the previous frame and previous points
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print "Max Length: %.2f,  Point Reset: %.f" %(maxDif, reset)
    print "Displacement X: %.2f,  Displacement Y: %.2f" % (round(totalX, 2), round(totalY, 2))
    cv2.imshow("Optical Flow", img)
    cv2.waitKey(0)
    cap.release()
    cv2.destroyAllWindows()
